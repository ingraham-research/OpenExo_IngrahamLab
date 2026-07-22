"""UDP remote-control service for the OpenExo GUI.

AUTHORITY for the wire format. `remote/client.py` carries its own mirror of the
PROTOCOL section below and must be updated by hand when this changes.

Wire format: one UDP datagram == one JSON object, UTF-8, localhost only.
"""
import json
import logging
import time
from typing import Optional

try:
    from PySide6 import QtCore, QtNetwork
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

logger = logging.getLogger("OpenExo.RemoteControl")

# ============================ PROTOCOL (authority) ============================
PROTOCOL_VERSION = 1

# Firmware param-update rejection reasons. Mirrors MainWindow._PARAM_UPDATE_REASONS
# (which maps 1..6) plus 0 = accepted.
ACK_REASONS = {
    0: "accepted",
    1: "invalid message",
    2: "side or joint mismatch",
    3: "controller mismatch",
    4: "invalid parameter index",
    5: "value out of bounds",
    6: "value must be an integer",
}

VALID_STREAMS = ("rt", "ack", "matrix", "status")


class ProtocolError(Exception):
    """Datagram was not valid JSON, or not a JSON object."""


class CommandError(Exception):
    """A well-formed command that cannot be honored. Carries a machine-readable code."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def encode(obj: dict) -> bytes:
    """Serialize a message dict to a UTF-8 JSON datagram."""
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")


def decode(data: bytes) -> dict:
    """Parse a datagram into a message dict. Raises ProtocolError on bad input."""
    try:
        obj = json.loads(bytes(data).decode("utf-8"))
    except Exception as e:
        raise ProtocolError(f"invalid JSON: {e}") from e
    if not isinstance(obj, dict):
        raise ProtocolError("datagram must be a JSON object")
    return obj


def _joint_display_name(row) -> str:
    """'Ankle(L) (68)' -> 'Ankle(L)'. Strips a trailing ' (<id>)' if present."""
    disp = str(row[0])
    cut = disp.rfind(" (")
    return disp[:cut] if cut > 0 else disp


def _is_int(v) -> bool:
    # bool is a subclass of int; treat True/False as NOT an id.
    return isinstance(v, int) and not isinstance(v, bool)


def _resolve_joint(j, matrix) -> int:
    if _is_int(j):
        return int(j)
    if isinstance(j, str):
        target = j.strip().lower()
        names = []
        for row in matrix:
            if len(row) < 2:
                continue
            name = _joint_display_name(row)
            names.append(name)
            if name.strip().lower() == target or str(row[1]) == j.strip():
                return int(row[1])
        raise CommandError("bad_name", f"unknown joint {j!r}; options: {sorted(set(names))}")
    raise CommandError("malformed", f"joint must be a name or integer id, got {j!r}")


def _resolve_controller_row(c, joint_rows):
    if _is_int(c):
        for row in joint_rows:
            if len(row) >= 4 and int(row[3]) == int(c):
                return row
        raise CommandError("bad_name", f"controller id {c} not present on this joint")
    if isinstance(c, str):
        target = c.strip().lower()
        names = [str(row[2]) for row in joint_rows if len(row) >= 3]
        for row in joint_rows:
            if len(row) >= 4 and str(row[2]).strip().lower() == target:
                return row
        raise CommandError("bad_name", f"unknown controller {c!r}; options: {names}")
    raise CommandError("malformed", f"controller must be a name or integer id, got {c!r}")


def _resolve_param_index(p, row) -> int:
    params = row[4:] if len(row) > 4 else []
    if _is_int(p):
        if 0 <= int(p) < len(params):
            return int(p)
        raise CommandError("invalid_index",
                           f"param index {p} out of range 0..{len(params) - 1}")
    if isinstance(p, str):
        target = p.strip().lower()
        for i, name in enumerate(params):
            if str(name).strip().lower() == target:
                return i
        raise CommandError("bad_name", f"unknown param {p!r}; options: {list(params)}")
    raise CommandError("malformed", f"param must be a name or integer index, got {p!r}")


def resolve_set_param(msg: dict, matrix: list) -> list:
    """Resolve a set_param message to [bilateral, joint_id, controller_id, param_index, value].

    Names are resolved against `matrix`; integer ids pass through and work with no matrix.
    Raises CommandError (codes: malformed, no_matrix, bad_name, invalid_index).
    """
    for field in ("joint", "controller", "param", "value"):
        if field not in msg:
            raise CommandError("malformed", f"set_param requires field {field!r}")

    bilateral = bool(msg.get("bilateral", False))
    try:
        value = float(msg["value"])
    except (TypeError, ValueError):
        raise CommandError("malformed", f"value must be a number, got {msg['value']!r}")

    j, c, p = msg["joint"], msg["controller"], msg["param"]

    # Raw-id fast path: all three integer ids, no matrix needed.
    if _is_int(j) and _is_int(c) and _is_int(p):
        return [bilateral, int(j), int(c), int(p), value]

    # Any name requires the handshake matrix.
    if not matrix:
        raise CommandError("no_matrix",
                           "no controller matrix yet (device not connected / no handshake); "
                           "use integer ids to send anyway")

    joint_id = _resolve_joint(j, matrix)
    joint_rows = [row for row in matrix if len(row) >= 4 and str(row[1]) == str(joint_id)]
    if not joint_rows:
        raise CommandError("bad_name", f"joint id {joint_id} has no controllers in the matrix")
    controller_row = _resolve_controller_row(c, joint_rows)
    controller_id = int(controller_row[3])
    param_index = _resolve_param_index(p, controller_row)
    return [bilateral, joint_id, controller_id, param_index, value]
# ========================== END PROTOCOL (authority) =========================


class RemoteControlService(QtCore.QObject):
    """Localhost UDP listener that turns JSON commands into GUI signal emissions,
    and fans GUI/BLE signals out to subscribers. Owns the wire format (above)."""

    setParamRequested = QtCore.Signal(list)  # [bilateral, joint_id, controller_id, param_index, value]

    def __init__(self, host: Optional[str] = None, port: Optional[int] = None,
                 max_send_failures: Optional[int] = None, parent=None):
        super().__init__(parent)
        from utils import RemoteConfig
        self._host = RemoteConfig.HOST if host is None else host
        self._port = RemoteConfig.PORT if port is None else port
        self._max_failures = (RemoteConfig.MAX_SEND_FAILURES
                              if max_send_failures is None else max_send_failures)

        self._matrix: list = []
        self._param_names: list = []
        self._subscribers: dict = {}  # (host, port) -> {"streams": set, "failures": int}

        self._socket = QtNetwork.QUdpSocket(self)
        self._bound = self._socket.bind(QtNetwork.QHostAddress(self._host), self._port)
        if self._bound:
            self._port = int(self._socket.localPort())
            self._socket.readyRead.connect(self._read_pending)
            logger.info("Remote control bound to %s:%s", self._host, self._port)
        else:
            logger.error("Remote control FAILED to bind %s:%s (%s)",
                         self._host, self._port, self._socket.errorString())

    # ---- lifecycle / introspection ----
    def is_bound(self) -> bool:
        return self._bound

    def address(self) -> tuple:
        return (self._host, self._port)

    @QtCore.Slot()
    def _read_pending(self):
        while self._socket.hasPendingDatagrams():
            size = self._socket.pendingDatagramSize()
            data, sender, sender_port = self._socket.readDatagram(size)
            host = sender.toString()
            # Normalize IPv4-mapped IPv6 form (e.g. "::ffff:127.0.0.1") to dotted quad.
            if host.startswith("::ffff:"):
                host = host[len("::ffff:"):]
            self.handle_datagram(bytes(data), host, int(sender_port))

    # ---- inbound dispatch (testable without real UDP) ----
    def handle_datagram(self, data: bytes, host: str, port: int) -> None:
        try:
            msg = decode(data)
        except ProtocolError as e:
            self._send({"ok": False, "error": str(e), "code": "bad_json"}, host, port)
            return

        cmd = msg.get("cmd")
        mid = msg.get("id")
        try:
            if cmd == "set_param":
                payload = resolve_set_param(msg, self._matrix)
                self.setParamRequested.emit(payload)
                self._reply_ok(mid, host, port)
            elif cmd == "subscribe":
                streams = msg.get("streams") or list(VALID_STREAMS)
                bad = [s for s in streams if s not in VALID_STREAMS]
                if bad:
                    raise CommandError("bad_stream",
                                       f"unknown streams {bad}; valid: {list(VALID_STREAMS)}")
                sub = self._subscribers.setdefault((host, port),
                                                   {"streams": set(), "failures": 0})
                sub["streams"].update(streams)
                sub["failures"] = 0
                self._reply_ok(mid, host, port, streams=sorted(sub["streams"]))
            elif cmd == "unsubscribe":
                self._subscribers.pop((host, port), None)
                self._reply_ok(mid, host, port)
            elif cmd == "get_matrix":
                # Matrix travels IN the reply (not a separate stream frame) so the
                # client gets it atomically, independent of datagram ordering.
                self._reply_ok(mid, host, port,
                               matrix=self._matrix, names=self._param_names)
            elif cmd == "ping":
                self._reply_ok(mid, host, port, pong=True)
            else:
                raise CommandError("bad_cmd", f"unknown cmd {cmd!r}")
        except CommandError as e:
            reply = {"ok": False, "error": e.message, "code": e.code}
            if mid is not None:
                reply["id"] = mid
            self._send(reply, host, port)

    def _reply_ok(self, mid, host, port, **extra):
        reply = {"ok": True}
        if mid is not None:
            reply["id"] = mid
        reply.update(extra)
        self._send(reply, host, port)

    # ---- send seam (overridden/monkeypatched in tests) ----
    def _send(self, obj: dict, host: str, port: int) -> bool:
        try:
            n = self._socket.writeDatagram(encode(obj), QtNetwork.QHostAddress(host), port)
            return n != -1
        except Exception as e:  # never let a dead peer crash the GUI thread
            logger.debug("remote send to %s:%s failed: %s", host, port, e)
            return False

    # ---- outbound fan-out ----
    def _broadcast(self, stream: str, payload: dict) -> None:
        frame = dict(payload)
        frame["stream"] = stream
        dead = []
        for key, sub in self._subscribers.items():
            if stream not in sub["streams"]:
                continue
            if self._send(frame, key[0], key[1]):
                sub["failures"] = 0
            else:
                sub["failures"] += 1
                if sub["failures"] >= self._max_failures:
                    dead.append(key)
        for key in dead:
            self._subscribers.pop(key, None)
            logger.info("remote: dropped unresponsive subscriber %s", key)

    @QtCore.Slot(list)
    def publish_rt(self, values: list) -> None:
        values = list(values)
        frame = {"t": time.time(), "values": values}
        if self._param_names:
            frame["names"] = self._param_names[:len(values)]
        self._broadcast("rt", frame)

    @QtCore.Slot(dict)
    def publish_ack(self, ack: dict) -> None:
        try:
            reason = int(ack.get("reason", 0))
        except (TypeError, ValueError):
            reason = 0
        self._broadcast("ack", {
            "joint_id": ack.get("joint_id"),
            "controller_id": ack.get("controller_id"),
            "param_index": ack.get("param_index"),
            "accepted": bool(ack.get("accepted")),
            "reason_code": reason,
            "reason": ACK_REASONS.get(reason, f"reason {reason}"),
        })

    @QtCore.Slot(list)
    def publish_values(self, rows: list) -> None:
        self._broadcast("matrix", {"values": [list(r) for r in rows]})

    def publish_status(self, event: str, **extra) -> None:
        self._broadcast("status", {"event": event, **extra})

    @QtCore.Slot(list)
    def set_controller_matrix(self, matrix: list) -> None:
        self._matrix = [list(r) for r in matrix]
        self._broadcast("matrix", {"matrix": self._matrix, "names": self._param_names})

    @QtCore.Slot(list)
    def set_param_names(self, names: list) -> None:
        self._param_names = list(names)
