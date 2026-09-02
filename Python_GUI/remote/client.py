"""Standalone UDP client for the OpenExo GUI remote-control service.

Self-contained: depends only on the Python standard library. It carries its own
copy of the wire format defined authoritatively in remote/service.py (the
PROTOCOL section). If that format changes, update this file to match.

Usage:
    from client import ExoRemote           # if this file is beside your script
    with ExoRemote() as exo:
        exo.wait_for_matrix()
        exo.set_param("Ankle(L)", "spline", "node1_y", 3.0, bilateral=True)
        for frame in exo.stream("rt", timeout=5):
            print(frame)
"""
import json
import socket
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9750
ALL_STREAMS = ("rt", "ack", "matrix", "status")


class RemoteError(Exception):
    """The GUI rejected a command, or no reply arrived in time."""

    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


class ExoRemote:
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=2.0):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)
        self._id = 0
        self._matrix = []
        self._names = []
        self._last_ack = None

    # ---- context manager: subscribe on enter, clean up on exit ----
    def __enter__(self):
        self.subscribe(ALL_STREAMS)
        return self

    def __exit__(self, *exc):
        try:
            self.unsubscribe()
        finally:
            self._sock.close()

    # ---- low-level ----
    def _send(self, obj):
        self._sock.sendto(json.dumps(obj).encode("utf-8"), self._addr)

    def _recv(self):
        data, _ = self._sock.recvfrom(65535)
        return json.loads(data.decode("utf-8"))

    def _next_id(self):
        self._id += 1
        return self._id

    def _absorb(self, msg):
        """Update cached state from a stream frame."""
        stream = msg.get("stream")
        if stream == "matrix":
            if "matrix" in msg:
                self._matrix = msg["matrix"]
            if "names" in msg:
                self._names = msg["names"]
        elif stream == "ack":
            # Also reached from _command's wait loop, so a fast ack can be captured
            # here rather than by a later stream("ack"). See last_ack().
            self._last_ack = msg

    def _command(self, obj):
        """Send a command and wait for its ok/error reply, absorbing any stream
        frames that arrive in the meantime. Raises RemoteError on rejection/timeout."""
        mid = self._next_id()
        obj = dict(obj)
        obj["id"] = mid
        self._send(obj)
        timeout = self._sock.gettimeout() or 2.0
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self._recv()
            except socket.timeout:
                break
            if "stream" in msg:
                self._absorb(msg)
                continue
            if msg.get("id") == mid:
                if not msg.get("ok"):
                    raise RemoteError(msg.get("error", "rejected"), msg.get("code"))
                return msg
        raise RemoteError("timeout waiting for reply to %r" % obj.get("cmd"))

    # ---- commands ----
    def subscribe(self, streams=ALL_STREAMS):
        return self._command({"cmd": "subscribe", "streams": list(streams)})

    def unsubscribe(self):
        try:
            return self._command({"cmd": "unsubscribe"})
        except RemoteError:
            return None

    def ping(self):
        return self._command({"cmd": "ping"})

    def set_param(self, joint, controller, param, value, bilateral=False):
        return self._command({
            "cmd": "set_param", "joint": joint, "controller": controller,
            "param": param, "value": value, "bilateral": bilateral,
        })

    def get_matrix(self):
        reply = self._command({"cmd": "get_matrix"})
        if "matrix" in reply:
            self._matrix = reply["matrix"]
        if "names" in reply:
            self._names = reply["names"]
        return self._matrix

    def wait_for_matrix(self, timeout=5.0):
        """Request the matrix and wait until it arrives (or timeout). Returns the matrix."""
        self.get_matrix()
        if self._matrix:
            return self._matrix
        deadline = time.time() + timeout
        while not self._matrix and time.time() < deadline:
            try:
                self._absorb(self._recv())
            except socket.timeout:
                break
        return self._matrix

    def controllers(self, joint):
        """Controller names available on a joint (name or numeric id)."""
        want = str(joint).strip().lower()
        out = []
        for row in self._matrix:
            if len(row) < 4:
                continue
            display = str(row[0])
            name = display.rsplit(" (", 1)[0].strip().lower()
            if name == want or str(row[1]) == str(joint):
                out.append(row[2])
        return out

    def last_ack(self):
        """Most recent firmware ack seen on any recv, or None.

        Check this BEFORE falling back to `stream("ack")`. `_command` absorbs
        stream frames while waiting for its own ok/error reply, so an ack that
        arrives inside that window lands here and is never re-delivered by
        `stream`. A `stream("ack")` loop started after `set_param` can therefore
        block until timeout on an update the exo already acknowledged.

        Snapshot before the set, then check both sources:

            before = exo.last_ack()
            exo.set_param(...)                       # raises on bad names
            ack = exo.last_ack() if exo.last_ack() is not before else None
            if ack is None:
                for ack in exo.stream("ack", timeout=5):
                    break

        Interpreting the result:

        * `ack is None` means no ack arrived at all - the BLE write never
          landed. Silence is the only failure signal; a dropped write produces
          no negative ack. The GUI itself treats 5s of silence as failure.
        * The `ok: true` reply to set_param only means the GUI queued a BLE
          write. It is returned even when no exo is connected. Only the ack
          means the firmware actually stored the value.
        * Bilateral updates ack twice, once per side, and either side can be
          rejected independently.
        * Acks carry no request id and do not echo the value - only
          joint_id / controller_id / param_index / accepted / reason. Match
          concurrent updates yourself on that id triple.
        """
        return self._last_ack

    def stream(self, which, timeout=None):
        """Yield frames of one stream ('rt'|'ack'|'matrix'|'status'). RT frames are
        labeled dicts (param name -> value). Stops after `timeout` seconds of silence."""
        if timeout is not None:
            self._sock.settimeout(timeout)
        while True:
            try:
                msg = self._recv()
            except socket.timeout:
                return
            if msg.get("stream") != which:
                self._absorb(msg)
                continue
            if which == "rt" and "names" in msg:
                yield dict(zip(msg["names"], msg["values"]))
            else:
                yield msg
