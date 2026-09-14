"""Windows-side logger for the BLE connection parameters actually in force.

WHY THIS EXISTS: the firmware banner's `UPi` field only ever shows the FIRST connection-parameter update of
a connection (ExoBLE.cpp refreshes the banner when exo_ble_cu_status changes, and it stays 1 after the first
success), and the GUI reads that banner once, ~2.5 s after connecting. On 2026-09-13 it read `UPi12` (15 ms)
while the trial CSV shows the link locked at 30 ms for the whole trial - Windows changes the interval on its
own and nothing on our side could see it. See "Modification log with claude/Nano-Hang-Watchdog-And-Breadcrumbs.md".

This asks WINDOWS instead, for the whole connection:
  - BluetoothLEDevice.ConnectionParametersChanged -> logs every change the moment it happens   [event]
  - BluetoothLEDevice.ConnectionStatusChanged     -> logs the value when the link comes up/drops [status]
  - a slow heartbeat                               -> cross-checks that the events really fire [heartbeat]
  - one read as soon as the device object exists   -> the value at attach time               [attach]

Log line, in the device-manager log:
    CONN_PARAMS [event] interval=15.00ms latency=0 timeout=9600ms read=0.04ms CHANGED (was interval=30.00ms ...)
Units are converted from WinRT's: interval in 1.25 ms steps, supervision timeout in 10 ms steps - the same
units as the firmware banner's CPi/UPi/t fields, so `interval=30.00ms timeout=9600ms` == `CPi24_t960`.

SAFETY: these are local queries of Windows' own Bluetooth stack. Nothing is written to the Nano, no GATT
traffic is generated, so it cannot collide with parameter writes the way the 2 s ping could. Every call is
wrapped: on a host without the API (it needs Windows 11, 10.0.22000+) it logs "not available" once and does
nothing else. It must never be able to affect the connection.

Relies on Bleak's PRIVATE attribute `BleakClient._backend._requester` (the WinRT BluetoothLEDevice, bleak 2.1.1).
If a Bleak update renames it, watch_client() logs "CONN_PARAMS off" and the GUI works exactly as before.

TO REMOVE: delete this file and the lines tagged "ConnParamsMonitor" in QtExoDeviceManager.py.
"""
import asyncio
import threading
import time


def _fmt(values):
    interval, latency, timeout = values
    return f"interval={interval * 1.25:.2f}ms latency={latency} timeout={timeout * 10}ms"


class ConnParamsMonitor:
    """Thread-safe: WinRT events arrive on a WinRT thread-pool thread, the heartbeat on the BLE asyncio thread."""

    def __init__(self, logger, clock=time.perf_counter):
        self.logger = logger
        self._clock = clock
        self._lock = threading.RLock()
        self._device = None
        self._tokens = []           # (name of the remove_* method, registration token)
        self._last = None           # last successfully read (interval, latency, timeout), WinRT units
        self._read_failing = False  # so a dead link logs one failure, not one per heartbeat

    @property
    def attached(self) -> bool:
        return self._device is not None

    def is_attached_to(self, device) -> bool:
        return device is not None and self._device is device

    def attach(self, device) -> bool:
        """Subscribe to the device's parameter/status events and log the current value. Never raises."""
        self.detach()
        with self._lock:
            self._device = device
            self._last = None
            self._read_failing = False
        try:
            token = device.add_connection_parameters_changed(lambda sender, args: self.sample("event"))
        except Exception as ex:
            with self._lock:
                self._device = None
            self.logger.warning("CONN_PARAMS not available on this host (needs Windows 11): %s", ex)
            return False
        tokens = [("remove_connection_parameters_changed", token)]
        try:
            tokens.append(("remove_connection_status_changed",
                           device.add_connection_status_changed(lambda sender, args: self.sample("status"))))
        except Exception as ex:
            # Not fatal - the parameter-change event is the one that matters.
            self.logger.warning("CONN_PARAMS status event not available: %s", ex)
        with self._lock:
            self._tokens = tokens
        self.logger.info("CONN_PARAMS monitoring started")
        self.sample("attach")
        return True

    def sample(self, tag: str) -> None:
        """Read the parameters now and log them, flagging a change. No-op when detached. Never raises."""
        with self._lock:
            device = self._device
            if device is None:
                return
            start = self._clock()
            try:
                p = device.get_connection_parameters()
                values = (int(p.connection_interval), int(p.connection_latency), int(p.link_timeout))
            except Exception as ex:
                if not self._read_failing:
                    self._read_failing = True
                    self.logger.warning("CONN_PARAMS [%s] read failed (expected while the link is down): %s", tag, ex)
                return
            read_ms = (self._clock() - start) * 1000.0
            self._read_failing = False
            no_link = values == (0, 0, 0)
            if not no_link:
                previous, self._last = self._last, values
        if no_link:
            # Windows reports all zeros, rather than raising, when there is no live link (seen on the real API
            # with the exo disconnected). Not a parameter change, so it must not become the "was" of the next line.
            self.logger.info("CONN_PARAMS [%s] no active link (Windows reports zeros) read=%.2fms", tag, read_ms)
            return
        line = f"CONN_PARAMS [{tag}] {_fmt(values)} read={read_ms:.2f}ms"
        if previous is not None and previous != values:
            line += f" CHANGED (was {_fmt(previous)})"
        self.logger.info(line)

    def detach(self) -> None:
        """Unsubscribe. Idempotent, safe from any thread, never raises."""
        with self._lock:
            device, tokens = self._device, self._tokens
            self._device, self._tokens = None, []
        if device is None:
            return
        for remove_name, token in tokens:
            try:
                getattr(device, remove_name)(token)
            except Exception:
                pass  # Bleak may already have closed the device object on disconnect; nothing left to undo
        self.logger.info("CONN_PARAMS monitoring stopped")


async def watch_client(client, monitor: ConnParamsMonitor, poll_s: float = 0.05, timeout_s: float = 15.0,
                       heartbeat_s: float = 10.0) -> bool:
    """Attach `monitor` to a BleakClient's WinRT device as soon as it exists, then heartbeat until detached.

    Start this BEFORE `await client.connect()`: Bleak creates the device object at the very start of connect(),
    before GATT service discovery, and the discovery phase is exactly where Windows was seen to change the
    interval. Returns False if monitoring never started (non-WinRT backend, timeout, API missing).
    """
    backend = getattr(client, "_backend", None)
    if not hasattr(backend, "_requester"):
        monitor.logger.info("CONN_PARAMS off: not a WinRT Bleak backend")
        return False
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    device = getattr(backend, "_requester", None)
    while device is None:
        if loop.time() >= deadline:
            monitor.logger.info("CONN_PARAMS off: WinRT device object never appeared within %.0f s", timeout_s)
            return False
        await asyncio.sleep(poll_s)
        device = getattr(backend, "_requester", None)
    if not monitor.attach(device):
        return False
    # Loop on THIS device, not just "attached": a reconnect re-attaches the monitor to a new device, and this
    # older loop must end rather than double the heartbeat.
    while monitor.is_attached_to(device):
        await asyncio.sleep(heartbeat_s)
        if monitor.is_attached_to(device):
            monitor.sample("heartbeat")
    return True
