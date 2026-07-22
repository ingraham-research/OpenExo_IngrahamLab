# UDP Remote Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an external script on the same PC change controller parameters at run time and receive every stream the GUI gets over BLE, by feeding the GUI's existing signal paths through a localhost UDP listener.

**Architecture:** A new `RemoteControlService` (QObject owning a `QUdpSocket` bound to 127.0.0.1) parses inbound JSON datagrams, resolves controller/parameter names against the handshake matrix, and emits `setParamRequested([bilateral, joint_id, controller_id, param_index, value])` — the same 5-element shape `ActiveTrialSettingsPage.applyRequested` already emits. `MainWindow` connects that to a new navigation-free `_apply_param_update`, and fans existing `RtBridge`/device-manager signals out to subscribers. The service owns the wire format; a standalone `client.py` carries its own copy.

**Tech Stack:** Python 3, PySide6 (`QtNetwork.QUdpSocket`, `QtCore`), stdlib `json`/`socket`, pytest.

## Global Constraints

- **No firmware changes.** Nothing in `ExoCode/` is touched.
- **Do not modify** `services/RtBridge.py` or `services/QtExoDeviceManager.py`. Read-only consumers of their existing signals.
- **Localhost only.** Bind address is `127.0.0.1`, taken from `RemoteConfig.HOST`. Never `0.0.0.0`.
- **Port 9750** (`RemoteConfig.PORT`). Tests use port `0` (ephemeral) and read the bound port back.
- **Default on.** `RemoteConfig.ENABLED = True`. A startup banner is printed from `MainWindow.__init__` **after** the socket binds, reporting the actual bound address; also logged at INFO. If the bind fails, print the failure just as loudly and continue — the GUI must still start without the service.
- **Banner uses `print()`, not `logger.info`** — `GUI.py` pins the console log handler at `WARNING`, so an info log never reaches the terminal.
- **No shared protocol module.** `remote/service.py` is the authority (a `PROTOCOL` block of constants + `encode`/`decode` + name resolution). `remote/client.py` carries its own copy of the wire format; its one rule is that it does not import a shared protocol-definition file. It may import stdlib freely.
- **Send scope is parameter updates only.** Receive streams are `rt`, `ack`, `matrix`, `status`. No motor/trial control commands.
- **Payload shape** is `[bilateral: bool, joint_id: int, controller_id: int, param_index: int, value: float]`. Bilateral mirroring happens downstream in `QtExoDeviceManager.build_parameter_updates`; the service does not mirror.
- **Commits are the user's responsibility (standing instruction — see memory `user-workflow-and-safety`).** Do NOT run `git commit`. Each task ends with a **review checkpoint**: stage the changed files with `git add`, summarize what changed, and let the user review and commit.
- **No motors move in any task.** All tests are headless (no BLE, no device). Bench validation with the exo is a separate step requiring explicit user consent.

---

## File Structure

| File | Responsibility |
|---|---|
| `Python_GUI/remote/__init__.py` | Empty package marker. Must NOT import `service` (would pull Qt into any importer). |
| `Python_GUI/remote/service.py` | Authority. Protocol codec + constants, name resolution, and the `RemoteControlService` QObject (socket, dispatch, subscribers, fan-out). |
| `Python_GUI/remote/client.py` | Standalone `ExoRemote` reference client. Own copy of the wire format. |
| `Python_GUI/utils/config.py` | Add `RemoteConfig`. |
| `Python_GUI/utils/__init__.py` | Export `RemoteConfig`. |
| `Python_GUI/MainWindow.py` | Wire the service to existing signals; split `_on_apply_settings`; print banner. |
| `Python_GUI/examples/sweep_example.py` | Worked example using `client.py`. |
| `Python_GUI/tests/conftest.py` | Offscreen `qapp` fixture + sys.path setup. |
| `Python_GUI/tests/test_remote_protocol.py` | Codec + name-resolution tests. |
| `Python_GUI/tests/test_remote_service.py` | Service inbound/outbound tests. |
| `Python_GUI/tests/test_remote_loopback.py` | End-to-end service + real client (drift guard). |
| `Python_GUI/tests/test_mainwindow_remote.py` | Offscreen smoke test of the wiring + navigation split. |
| `Modification log with claude/Remote-Control-UDP.md` | Feature write-up. |

---

## Task 1: Config + protocol codec

**Files:**
- Create: `Python_GUI/remote/__init__.py`
- Create: `Python_GUI/remote/service.py`
- Modify: `Python_GUI/utils/config.py` (append `RemoteConfig`)
- Modify: `Python_GUI/utils/__init__.py` (export `RemoteConfig`)
- Create: `Python_GUI/tests/conftest.py`
- Test: `Python_GUI/tests/test_remote_protocol.py`

**Interfaces:**
- Produces: `remote.service.encode(obj: dict) -> bytes`, `remote.service.decode(data: bytes) -> dict` (raises `ProtocolError`), exceptions `ProtocolError`, `CommandError(code: str, message: str)`, and module constants `PROTOCOL_VERSION`, `ACK_REASONS: dict[int,str]`, `VALID_STREAMS: tuple[str,...]`.
- Produces: `utils.RemoteConfig` with `ENABLED`, `HOST`, `PORT`, `MAX_SEND_FAILURES`.

- [ ] **Step 1: Write the failing test**

Create `Python_GUI/tests/conftest.py`:

```python
import os
import sys

# Headless Qt for all tests in this suite.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Make Python_GUI importable (remote/, utils/, MainWindow, ...) regardless of cwd.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402
from PySide6 import QtWidgets  # noqa: E402


@pytest.fixture(scope="session")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app
```

Create `Python_GUI/tests/test_remote_protocol.py`:

```python
import pytest

from remote.service import (
    encode, decode, ProtocolError, ACK_REASONS, VALID_STREAMS,
)


def test_encode_decode_round_trip():
    obj = {"cmd": "set_param", "joint": "Ankle(L)", "value": 3.0, "id": 42}
    assert decode(encode(obj)) == obj


def test_decode_rejects_non_json():
    with pytest.raises(ProtocolError):
        decode(b"not json {{{")


def test_decode_rejects_json_array():
    with pytest.raises(ProtocolError):
        decode(b"[1, 2, 3]")


def test_ack_reasons_cover_firmware_codes():
    # Mirror of MainWindow._PARAM_UPDATE_REASONS (1..6) plus accepted (0).
    for code in range(0, 7):
        assert code in ACK_REASONS


def test_valid_streams():
    assert VALID_STREAMS == ("rt", "ack", "matrix", "status")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_remote_protocol.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'remote'`.

- [ ] **Step 3: Create the package and codec**

Create `Python_GUI/remote/__init__.py` (empty — a leading comment only, so nothing imports Qt on package import):

```python
# OpenExo UDP remote-control package.
# Intentionally empty: do NOT import `service` here (it pulls in PySide6/QtNetwork).
```

Create `Python_GUI/remote/service.py` with the protocol section (the QObject is added in Task 3):

```python
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
# ========================== END PROTOCOL (authority) =========================
```

Append `RemoteConfig` to `Python_GUI/utils/config.py` (after `PlotConfig`):

```python
class RemoteConfig:
    """UDP remote-control listener settings. Localhost only."""

    ENABLED = True
    HOST = "127.0.0.1"      # localhost only; widening this is a deliberate change
    PORT = 9750
    MAX_SEND_FAILURES = 5   # consecutive send failures before dropping a subscriber
```

- [ ] **Step 4: Export `RemoteConfig`**

Modify `Python_GUI/utils/__init__.py`:

```python
from .config import UIConfig, JointConfig, PlotConfig, RemoteConfig
```

and add `'RemoteConfig',` to the `__all__` list.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_remote_protocol.py -v`
Expected: PASS (5 passed).

Also confirm the config import: `python -c "from utils import RemoteConfig; print(RemoteConfig.PORT)"`
Expected: `9750`.

- [ ] **Step 6: Review checkpoint (user commits)**

```bash
git add Python_GUI/remote/__init__.py Python_GUI/remote/service.py \
        Python_GUI/utils/config.py Python_GUI/utils/__init__.py \
        Python_GUI/tests/conftest.py Python_GUI/tests/test_remote_protocol.py
```
Summarize the change and hand off to the user to review and commit. Do not run `git commit`.

---

## Task 2: Name resolution

**Files:**
- Modify: `Python_GUI/remote/service.py` (add resolution functions inside the PROTOCOL section)
- Test: `Python_GUI/tests/test_remote_protocol.py` (append)

**Interfaces:**
- Consumes: `CommandError` from Task 1.
- Produces: `remote.service.resolve_set_param(msg: dict, matrix: list) -> list` returning `[bool, int, int, int, float]`. Raises `CommandError` with codes `malformed`, `no_matrix`, `bad_name`, `invalid_index`.
- Matrix row shape (from `RtBridge`): `[display_name, joint_id_str, controller_name, controller_id_str, param0_name, param1_name, ...]`, e.g. `["Ankle(L) (68)", "68", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"]`.

- [ ] **Step 1: Write the failing test**

Append to `Python_GUI/tests/test_remote_protocol.py`:

```python
from remote.service import resolve_set_param, CommandError

MATRIX = [
    ["Ankle(L) (68)", "68", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"],
    ["Ankle(L) (68)", "68", "spline", "1", "node1_x", "node1_y", "use_pid"],
    ["Ankle(R) (36)", "36", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"],
]


def test_resolve_by_name():
    assert resolve_set_param(
        {"joint": "Ankle(L)", "controller": "spline", "param": "node1_y", "value": 3.0},
        MATRIX,
    ) == [False, 68, 1, 1, 3.0]


def test_resolve_is_case_insensitive():
    assert resolve_set_param(
        {"joint": "ankle(l)", "controller": "SPLINE", "param": "USE_PID", "value": 1},
        MATRIX,
    ) == [False, 68, 1, 2, 1.0]


def test_resolve_bilateral_flag_passes_through():
    out = resolve_set_param(
        {"joint": "Ankle(L)", "controller": "zeroTorque", "param": "p_gain",
         "value": 2.5, "bilateral": True},
        MATRIX,
    )
    assert out == [True, 68, 0, 1, 2.5]


def test_resolve_raw_ids_pass_through_without_matrix():
    assert resolve_set_param(
        {"joint": 68, "controller": 0, "param": 1, "value": 4.0}, []
    ) == [False, 68, 0, 1, 4.0]


def test_resolve_names_without_matrix_raises_no_matrix():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline",
                           "param": "node1_y", "value": 3.0}, [])
    assert ei.value.code == "no_matrix"


def test_resolve_unknown_controller_raises_bad_name():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "nope",
                           "param": "p_gain", "value": 1.0}, MATRIX)
    assert ei.value.code == "bad_name"


def test_resolve_param_index_out_of_range_raises_invalid_index():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline",
                           "param": 99, "value": 1.0}, MATRIX)
    assert ei.value.code == "invalid_index"


def test_resolve_missing_field_raises_malformed():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline"}, MATRIX)
    assert ei.value.code == "malformed"


def test_resolve_non_numeric_value_raises_malformed():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": 68, "controller": 0, "param": 1,
                           "value": "high"}, [])
    assert ei.value.code == "malformed"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_remote_protocol.py -k resolve -v`
Expected: FAIL — `ImportError: cannot import name 'resolve_set_param'`.

- [ ] **Step 3: Implement the resolution functions**

Insert into `Python_GUI/remote/service.py`, just above the `# ===== END PROTOCOL` line:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_remote_protocol.py -v`
Expected: PASS (all protocol + resolution tests).

- [ ] **Step 5: Review checkpoint (user commits)**

```bash
git add Python_GUI/remote/service.py Python_GUI/tests/test_remote_protocol.py
```
Summarize and hand off. Do not run `git commit`.

---

## Task 3: Service inbound — socket, dispatch, replies

**Files:**
- Modify: `Python_GUI/remote/service.py` (add the `RemoteControlService` QObject)
- Test: `Python_GUI/tests/test_remote_service.py`

**Interfaces:**
- Consumes: `encode`, `decode`, `resolve_set_param`, `ProtocolError`, `CommandError`, `VALID_STREAMS`, `RemoteConfig`.
- Produces: `remote.service.RemoteControlService(QtCore.QObject)` with:
  - signal `setParamRequested = QtCore.Signal(list)`
  - `__init__(self, host: Optional[str] = None, port: Optional[int] = None, max_send_failures: Optional[int] = None, parent=None)`
  - `is_bound() -> bool`, `address() -> tuple[str, int]`
  - `handle_datagram(data: bytes, host: str, port: int) -> None` (testable core)
  - `_send(self, obj: dict, host: str, port: int) -> bool` (send seam; monkeypatched in tests)
  - subscriber registry `self._subscribers: dict[tuple[str,int], dict]` with `{"streams": set, "failures": int}`
  - state `self._matrix: list`, `self._param_names: list`

- [ ] **Step 1: Write the failing test**

Create `Python_GUI/tests/test_remote_service.py`:

```python
import pytest

from remote.service import RemoteControlService, encode


@pytest.fixture
def svc(qapp):
    s = RemoteControlService(host="127.0.0.1", port=0)
    assert s.is_bound()
    return s


@pytest.fixture
def sent(svc, monkeypatch):
    """Capture outbound datagrams instead of putting them on the wire."""
    box = []
    monkeypatch.setattr(svc, "_send",
                        lambda obj, host, port: box.append((obj, host, port)) or True)
    return box


def test_bind_reports_address(svc):
    host, port = svc.address()
    assert host == "127.0.0.1"
    assert port != 0


def test_set_param_emits_payload_and_acks(svc, sent):
    # Set matrix state directly; the public setter arrives in Task 4.
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y", "use_pid"]]
    emitted = []
    svc.setParamRequested.connect(lambda p: emitted.append(p))

    svc.handle_datagram(
        encode({"cmd": "set_param", "joint": "Ankle(L)", "controller": "spline",
                "param": "node1_y", "value": 3.0, "id": 7}),
        "127.0.0.1", 55555,
    )

    assert emitted == [[False, 68, 1, 0, 3.0]]
    assert ({"ok": True, "id": 7}, "127.0.0.1", 55555) in sent


def test_bad_json_gets_error_reply(svc, sent):
    svc.handle_datagram(b"garbage{{{", "127.0.0.1", 55555)
    obj, host, port = sent[-1]
    assert obj["ok"] is False
    assert obj["code"] == "bad_json"


def test_unknown_name_error_reply_has_code(svc, sent):
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    svc.handle_datagram(
        encode({"cmd": "set_param", "joint": "Ankle(L)", "controller": "nope",
                "param": "node1_y", "value": 1.0, "id": 9}),
        "127.0.0.1", 55555,
    )
    obj, _, _ = sent[-1]
    assert obj == {"ok": False, "id": 9, "error": obj["error"], "code": "bad_name"}


def test_subscribe_registers_sender(svc, sent):
    svc.handle_datagram(
        encode({"cmd": "subscribe", "streams": ["rt", "ack"], "id": 1}),
        "127.0.0.1", 44444,
    )
    assert ("127.0.0.1", 44444) in svc._subscribers
    assert svc._subscribers[("127.0.0.1", 44444)]["streams"] == {"rt", "ack"}
    obj, _, _ = sent[-1]
    assert obj["ok"] is True


def test_subscribe_unknown_stream_rejected(svc, sent):
    svc.handle_datagram(
        encode({"cmd": "subscribe", "streams": ["bogus"], "id": 2}),
        "127.0.0.1", 44444,
    )
    obj, _, _ = sent[-1]
    assert obj["ok"] is False and obj["code"] == "bad_stream"
    assert ("127.0.0.1", 44444) not in svc._subscribers


def test_unsubscribe_removes_sender(svc, sent):
    svc.handle_datagram(encode({"cmd": "subscribe"}), "127.0.0.1", 44444)
    svc.handle_datagram(encode({"cmd": "unsubscribe"}), "127.0.0.1", 44444)
    assert ("127.0.0.1", 44444) not in svc._subscribers


def test_get_matrix_replies_with_cached_matrix(svc, sent):
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    svc._param_names = ["percent_gait", "torque_l"]
    svc.handle_datagram(encode({"cmd": "get_matrix", "id": 3}), "127.0.0.1", 44444)
    # ok reply, then a matrix stream frame.
    assert sent[0][0] == {"ok": True, "id": 3}
    frame = sent[1][0]
    assert frame["stream"] == "matrix"
    assert frame["matrix"] == [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    assert frame["names"] == ["percent_gait", "torque_l"]


def test_unknown_cmd_rejected(svc, sent):
    svc.handle_datagram(encode({"cmd": "explode", "id": 5}), "127.0.0.1", 44444)
    obj, _, _ = sent[-1]
    assert obj["ok"] is False and obj["code"] == "bad_cmd"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_remote_service.py -v`
Expected: FAIL — `ImportError: cannot import name 'RemoteControlService'`.

- [ ] **Step 3: Implement the QObject (inbound half)**

Append to `Python_GUI/remote/service.py`:

```python
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
                self._reply_ok(mid, host, port)
                self._send({"stream": "matrix", "matrix": self._matrix,
                            "names": self._param_names}, host, port)
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_remote_service.py -v`
Expected: PASS (all inbound tests).

- [ ] **Step 5: Review checkpoint (user commits)**

```bash
git add Python_GUI/remote/service.py Python_GUI/tests/test_remote_service.py
```
Summarize and hand off. Do not run `git commit`.

---

## Task 4: Service outbound — stream fan-out + subscriber eviction

**Files:**
- Modify: `Python_GUI/remote/service.py` (add publish/broadcast slots)
- Test: `Python_GUI/tests/test_remote_service.py` (append)

**Interfaces:**
- Consumes: everything from Task 3 (`_send`, `_subscribers`, `_matrix`, `_param_names`, `ACK_REASONS`).
- Produces on `RemoteControlService`:
  - `_broadcast(self, stream: str, payload: dict) -> None`
  - `publish_rt(self, values: list)` (Slot(list))
  - `publish_ack(self, ack: dict)` (Slot(dict))
  - `publish_values(self, rows: list)` (Slot(list))
  - `publish_status(self, event: str, **extra)`
  - `set_controller_matrix(self, matrix: list)` (Slot(list))
  - `set_param_names(self, names: list)` (Slot(list))
- Ack dict shape consumed (from `RtBridge.paramUpdateAckReceived`): `{"joint_id", "controller_id", "param_index", "accepted", "reason"}`.

- [ ] **Step 1: Write the failing test**

Append to `Python_GUI/tests/test_remote_service.py`:

```python
def _subscribe(svc, host, port, streams):
    svc._subscribers[(host, port)] = {"streams": set(streams), "failures": 0}


def test_publish_rt_reaches_only_rt_subscribers(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    _subscribe(svc, "127.0.0.1", 2222, ["ack"])
    svc.set_param_names(["percent_gait", "torque_l", "torque_r"])
    sent.clear()

    svc.publish_rt([10.0, 1.5, -1.5])

    targets = {(port, obj["stream"]) for obj, host, port in sent}
    assert (1111, "rt") in targets
    assert all(port != 2222 for _, _, port in sent)  # ack-only sub gets nothing
    rt_frame = next(obj for obj, _, port in sent if port == 1111)
    assert rt_frame["values"] == [10.0, 1.5, -1.5]
    assert rt_frame["names"] == ["percent_gait", "torque_l", "torque_r"]


def test_publish_ack_maps_reason_text(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["ack"])
    sent.clear()
    svc.publish_ack({"joint_id": 68, "controller_id": 0, "param_index": 1,
                     "accepted": False, "reason": 5})
    frame = sent[-1][0]
    assert frame["stream"] == "ack"
    assert frame["accepted"] is False
    assert frame["reason_code"] == 5
    assert frame["reason"] == "value out of bounds"


def test_set_controller_matrix_broadcasts_to_matrix_subscribers(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["matrix"])
    sent.clear()
    svc.set_controller_matrix([["Ankle(L) (68)", "68", "spline", "1", "node1_y"]])
    frame = sent[-1][0]
    assert frame["stream"] == "matrix"
    assert frame["matrix"][0][2] == "spline"


def test_subscriber_evicted_after_max_failures(svc):
    # _send returns False (failure) every time.
    svc._send = lambda obj, host, port: False
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    for _ in range(svc._max_failures):
        svc.publish_rt([1.0])
    assert ("127.0.0.1", 1111) not in svc._subscribers


def test_successful_send_resets_failure_count(svc):
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    sub = svc._subscribers[("127.0.0.1", 1111)]
    # Fail a few, but not enough to evict.
    svc._send = lambda obj, host, port: False
    for _ in range(svc._max_failures - 1):
        svc.publish_rt([1.0])
    assert sub["failures"] == svc._max_failures - 1
    # One success resets.
    svc._send = lambda obj, host, port: True
    svc.publish_rt([1.0])
    assert sub["failures"] == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_remote_service.py -k "publish or evict or matrix or failure" -v`
Expected: FAIL — `AttributeError: 'RemoteControlService' object has no attribute 'publish_rt'`.

- [ ] **Step 3: Implement the outbound half**

Append to the `RemoteControlService` class in `Python_GUI/remote/service.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_remote_service.py -v`
Expected: PASS (inbound + outbound).

- [ ] **Step 5: Review checkpoint (user commits)**

```bash
git add Python_GUI/remote/service.py Python_GUI/tests/test_remote_service.py
```
Summarize and hand off. Do not run `git commit`.

---

## Task 5: MainWindow wiring + navigation split + banner

**Files:**
- Modify: `Python_GUI/MainWindow.py`
- Test: `Python_GUI/tests/test_mainwindow_remote.py`

**Interfaces:**
- Consumes: `RemoteControlService` (Task 3/4), `RemoteConfig`, existing `RtBridge` signals (`rtDataUpdated`, `paramUpdateAckReceived`, `controllerMatrixReceived`, `controllerValuesReceived`, `parameterNamesReceived`, `shutdownProgressReceived`) and device-manager signals (`connected(str,str)`, `disconnected()`, `deviceErrorReceived(str)`).
- Produces: `MainWindow._apply_param_update(self, payload) -> bool` (no navigation) and a `self.remote` attribute (a `RemoteControlService` or `None`).

- [ ] **Step 1: Write the failing test**

Create `Python_GUI/tests/test_mainwindow_remote.py`:

```python
import pytest


@pytest.fixture
def window(qapp, monkeypatch):
    # Bind an ephemeral port so repeated window construction can't collide on 9750,
    # and so a lingering socket from a prior test never fails the next bind.
    from utils import RemoteConfig
    monkeypatch.setattr(RemoteConfig, "PORT", 0)
    from MainWindow import MainWindow
    w = MainWindow()
    yield w
    # Release the UDP socket promptly rather than waiting on GC.
    if getattr(w, "remote", None) is not None:
        try:
            w.remote._socket.close()
        except Exception:
            pass
    w.close()


def test_remote_service_is_bound(window):
    assert window.remote is not None
    assert window.remote.is_bound()


def test_apply_param_update_does_not_navigate(window):
    # Land on the settings page, then apply a remote-style update.
    window.stack.setCurrentWidget(window.settings_page)
    before = window.stack.currentWidget()
    window._apply_param_update([False, 68, 0, 1, 3.0])  # not connected -> no BLE, but must not navigate
    assert window.stack.currentWidget() is before


def test_on_apply_settings_navigates_to_trial(window):
    window.stack.setCurrentWidget(window.settings_page)
    window._on_apply_settings([False, 68, 0, 1, 3.0])
    assert window.stack.currentWidget() is window.trial_page


def test_setparam_signal_is_wired_to_apply(window):
    # Qt binds a slot at connect() time, so monkeypatching the attribute afterward
    # would not redirect the signal. Assert the connection directly: disconnect()
    # succeeds if wired, and raises if not.
    try:
        window.remote.setParamRequested.disconnect(window._apply_param_update)
    except (RuntimeError, TypeError):
        pytest.fail("setParamRequested is not connected to _apply_param_update")
    finally:
        # Best-effort reconnect so the window is left functional.
        try:
            window.remote.setParamRequested.connect(window._apply_param_update)
        except Exception:
            pass
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_mainwindow_remote.py -v`
Expected: FAIL — `AttributeError: 'MainWindow' object has no attribute 'remote'` (and no `_apply_param_update`).

- [ ] **Step 3: Split `_on_apply_settings`**

In `Python_GUI/MainWindow.py`, replace the existing `_on_apply_settings` method (currently around lines 850-874) with the split version:

```python
    def _apply_param_update(self, payload) -> bool:
        """Core parameter-update path shared by the GUI Apply button and the UDP
        remote. Validates, queues the pending ack, and sends over BLE. Does NOT
        navigate — callers that are GUI buttons handle navigation themselves."""
        self.logger.info(f"Applying param update: {payload}")
        try:
            updates = QtExoDeviceManager.build_parameter_updates(payload)
            self._queue_pending_param_updates(updates)
            self._show_param_update_status("", warning=False)
        except Exception as e:
            self.logger.error(f"Invalid parameter update request: {e}")
            self.logger.debug(traceback.format_exc())
            self._show_param_update_status(f"Controller update not sent: {e}", warning=True)
            return False

        try:
            self.qt_dev.updateTorqueValues(payload)
        except Exception as e:
            self.logger.error(f"Failed to update torque values: {e}")
            self.logger.debug(traceback.format_exc())
            return False
        return True

    @QtCore.Slot(list)
    def _on_apply_settings(self, payload):
        # payload: [isBilateral, joint, controller, parameter, value]
        self._apply_param_update(payload)
        # Return to trial page (GUI-button behavior only).
        try:
            self.stack.setCurrentWidget(self.trial_page)
        except Exception as e:
            self.logger.error(f"Failed to navigate to trial page after settings: {e}")
            self.logger.debug(traceback.format_exc())
```

- [ ] **Step 4: Add the service setup + banner to `__init__`**

In `Python_GUI/MainWindow.py`, add the import near the top (with the other `from ... import` lines, after `from services import QtExoDeviceManager, RtBridge`):

```python
from remote.service import RemoteControlService
from utils import RemoteConfig
```

Then, at the end of `MainWindow.__init__` (after the existing `log_path = self.qt_dev.get_log_file_path()` block), add:

```python
        # ----- UDP remote control (localhost only; default on) -----
        self.remote = None
        if RemoteConfig.ENABLED:
            try:
                self.remote = RemoteControlService(parent=self)
            except Exception as e:
                self.logger.error(f"Failed to construct remote control service: {e}")
                self.logger.debug(traceback.format_exc())
                self.remote = None

        if self.remote is not None and self.remote.is_bound():
            host, port = self.remote.address()
            # Inbound: remote commands drive the navigation-free apply path.
            self.remote.setParamRequested.connect(self._apply_param_update)
            # Outbound: fan existing BLE-derived signals to subscribers.
            self.rt_bridge.rtDataUpdated.connect(self.remote.publish_rt)
            self.rt_bridge.paramUpdateAckReceived.connect(self.remote.publish_ack)
            self.rt_bridge.controllerMatrixReceived.connect(self.remote.set_controller_matrix)
            self.rt_bridge.controllerValuesReceived.connect(self.remote.publish_values)
            self.rt_bridge.parameterNamesReceived.connect(self.remote.set_param_names)
            self.rt_bridge.shutdownProgressReceived.connect(
                lambda step: self.remote.publish_status("shutdown_progress", step=int(step)))
            self.qt_dev.connected.connect(
                lambda name, addr: self.remote.publish_status("connected", name=name, address=addr))
            self.qt_dev.disconnected.connect(
                lambda: self.remote.publish_status("disconnected"))
            self.qt_dev.deviceErrorReceived.connect(
                lambda msg: self.remote.publish_status("device_error", message=msg))

            banner = (
                "\n" + "=" * 60 + "\n"
                f" REMOTE CONTROL ACTIVE - listening on udp://{host}:{port}\n"
                " External scripts can change controller parameters.\n"
                " Localhost only. Disable: utils/config.py RemoteConfig.ENABLED\n"
                + "=" * 60
            )
            print(banner)
            self.logger.info("Remote control active on udp://%s:%s", host, port)
        elif RemoteConfig.ENABLED:
            print("\n" + "=" * 60 + "\n"
                  " REMOTE CONTROL FAILED TO BIND - continuing without it.\n"
                  " Another process may hold the port. See the log for details.\n"
                  + "=" * 60)
            self.logger.error("Remote control enabled but failed to bind; GUI continues without it.")
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_mainwindow_remote.py -v`
Expected: PASS (4 passed).

Then run the whole suite to catch regressions:
Run: `cd Python_GUI && python -m pytest tests/ -v`
Expected: PASS (all).

- [ ] **Step 6: Review checkpoint (user commits)**

```bash
git add Python_GUI/MainWindow.py Python_GUI/tests/test_mainwindow_remote.py
```
Summarize and hand off. Do not run `git commit`.

---

## Task 6: Standalone client + loopback integration (drift guard) + example

**Files:**
- Create: `Python_GUI/remote/client.py`
- Create: `Python_GUI/examples/sweep_example.py`
- Test: `Python_GUI/tests/test_remote_loopback.py`

**Interfaces:**
- Consumes: the running `RemoteControlService` over real UDP (host/port from `address()`).
- Produces: `remote.client.ExoRemote` with `__enter__`/`__exit__`, `set_param(joint, controller, param, value, bilateral=False)`, `get_matrix()`, `wait_for_matrix(timeout)`, `controllers(joint)`, `ping()`, `subscribe(streams)`, `unsubscribe()`, `stream(which, timeout)`, `last_ack()`, and `RemoteError(message, code=None)`.

- [ ] **Step 1: Write the failing test**

Create `Python_GUI/tests/test_remote_loopback.py`:

```python
import threading
import time

import pytest

from remote.service import RemoteControlService


def _pump_until_done(qapp, thread, timeout=8.0):
    """Run the Qt event loop cooperatively until the worker thread finishes.

    The client runs in a worker thread doing blocking UDP; the main thread must
    pump events so the service's readyRead fires and it can reply. Pump until the
    thread is done (not just until the first reply) so the client's __exit__
    unsubscribe is serviced too, and the thread exits cleanly.
    """
    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        qapp.processEvents()
        time.sleep(0.005)
    thread.join(timeout=2.0)
    qapp.processEvents()


def test_loopback_set_param_and_matrix(qapp):
    svc = RemoteControlService(host="127.0.0.1", port=0)
    assert svc.is_bound()
    host, port = svc.address()
    svc.set_controller_matrix([
        ["Ankle(L) (68)", "68", "spline", "1", "node1_y", "use_pid"],
    ])
    svc.set_param_names(["percent_gait", "torque_l"])

    captured = []
    svc.setParamRequested.connect(lambda p: captured.append(p))

    result = {}

    def worker():
        from remote.client import ExoRemote
        try:
            with ExoRemote(host, port, timeout=3.0) as exo:
                result["matrix"] = exo.wait_for_matrix(timeout=3.0)
                result["reply"] = exo.set_param("Ankle(L)", "spline", "node1_y", 3.0)
        except Exception as e:  # surface worker failures to the assertions below
            result["error"] = repr(e)

    t = threading.Thread(target=worker)
    t.start()
    _pump_until_done(qapp, t)

    assert "error" not in result, result.get("error")
    assert result["reply"]["ok"] is True
    assert captured == [[False, 68, 1, 0, 3.0]]
    assert result["matrix"][0][2] == "spline"


def test_loopback_rejection_raises(qapp):
    from remote.client import ExoRemote, RemoteError

    svc = RemoteControlService(host="127.0.0.1", port=0)
    host, port = svc.address()
    svc.set_controller_matrix([["Ankle(L) (68)", "68", "spline", "1", "node1_y"]])

    result = {}

    def worker():
        try:
            with ExoRemote(host, port, timeout=3.0) as exo:
                exo.wait_for_matrix(timeout=3.0)
                exo.set_param("Ankle(L)", "spline", "does_not_exist", 1.0)
            result["outcome"] = "no-raise"
        except RemoteError as e:
            result["outcome"] = "raised"
            result["code"] = e.code

    t = threading.Thread(target=worker)
    t.start()
    _pump_until_done(qapp, t)

    assert result["outcome"] == "raised"
    assert result["code"] == "bad_name"


def test_end_to_end_fake_rt_reaches_client(qapp, monkeypatch):
    """Full receive path with NO BLE: emit RtBridge signals as if data were
    received, and confirm a UDP client subscribed through the real MainWindow
    wiring gets the labeled RT frame."""
    from utils import RemoteConfig
    monkeypatch.setattr(RemoteConfig, "PORT", 0)
    from MainWindow import MainWindow

    w = MainWindow()
    try:
        assert w.remote is not None and w.remote.is_bound()
        host, port = w.remote.address()
        # Seed param names so RT frames are labeled, as they would be post-handshake.
        w.rt_bridge.parameterNamesReceived.emit(["percent_gait", "torque_l"])

        got = {}

        def worker():
            from remote.client import ExoRemote
            with ExoRemote(host, port, timeout=3.0) as exo:
                for frame in exo.stream("rt", timeout=3.0):
                    got["frame"] = frame
                    break

        t = threading.Thread(target=worker)
        t.start()

        # Wait until the client's subscribe has registered (UDP has no pre-subscribe
        # buffering, so we must not emit RT before the subscriber exists).
        deadline = time.time() + 3.0
        while not w.remote._subscribers and time.time() < deadline:
            qapp.processEvents()
            time.sleep(0.005)
        assert w.remote._subscribers, "client did not subscribe in time"

        # Emit fake RT (full 16-wide frame, as RtBridge normalizes) until captured.
        deadline = time.time() + 3.0
        while t.is_alive() and time.time() < deadline:
            w.rt_bridge.rtDataUpdated.emit([12.5, 1.1] + [0.0] * 14)
            qapp.processEvents()
            time.sleep(0.01)
        t.join(timeout=2.0)
        qapp.processEvents()

        assert got.get("frame") == {"percent_gait": 12.5, "torque_l": 1.1}
    finally:
        try:
            w.remote._socket.close()
        except Exception:
            pass
        w.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd Python_GUI && python -m pytest tests/test_remote_loopback.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'remote.client'`.

- [ ] **Step 3: Implement the standalone client**

Create `Python_GUI/remote/client.py`:

```python
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
        self._command({"cmd": "get_matrix"})
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd Python_GUI && python -m pytest tests/test_remote_loopback.py -v`
Expected: PASS (3 passed — including the no-BLE end-to-end fan-out through the real MainWindow wiring).

- [ ] **Step 5: Create the worked example**

Create `Python_GUI/examples/sweep_example.py`:

```python
"""Example: sweep a controller parameter over the UDP remote control.

Run the GUI first (python GUI.py) and connect to the exo. Then, from the
Python_GUI folder:  python examples/sweep_example.py

This only SETS parameters and reads telemetry; it never starts a trial or a
motor on its own. Nothing here moves a motor that a GUI user hasn't already
enabled.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "remote"))
from client import ExoRemote, RemoteError  # noqa: E402


def main():
    with ExoRemote() as exo:
        matrix = exo.wait_for_matrix(timeout=5)
        if not matrix:
            print("No controller matrix yet - is the GUI connected to an exo?")
            return
        print("Controllers on Ankle(L):", exo.controllers("Ankle(L)"))

        for gain in (1.0, 2.0, 3.0):
            try:
                exo.set_param("Ankle(L)", "zeroTorque", "p_gain", gain, bilateral=True)
                print(f"set p_gain = {gain} (GUI accepted)")
            except RemoteError as e:
                print(f"rejected: {e} (code={e.code})")
            # Watch the firmware's own accept/reject for this write.
            time.sleep(0.2)
            ack = exo.last_ack()
            if ack:
                print("  firmware ack:", ack["accepted"], ack["reason"])
            time.sleep(2.0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Run the full suite**

Run: `cd Python_GUI && python -m pytest tests/ -v`
Expected: PASS (all tests across all files).

- [ ] **Step 7: Review checkpoint (user commits)**

```bash
git add Python_GUI/remote/client.py Python_GUI/examples/sweep_example.py \
        Python_GUI/tests/test_remote_loopback.py
```
Summarize and hand off. Do not run `git commit`.

---

## Task 7: Feature write-up

**Files:**
- Create: `Modification log with claude/Remote-Control-UDP.md`

**Interfaces:** none (documentation only).

- [ ] **Step 1: Write the modification log**

Create `Modification log with claude/Remote-Control-UDP.md` documenting:
- **What it is:** a localhost UDP listener (default `127.0.0.1:9750`) that lets an external script set controller parameters and receive all BLE-derived telemetry, firing the exact same paths as the GUI's Apply button.
- **How to use it:** run `GUI.py` (banner confirms the port), then a script using `remote/client.py` — reference `examples/sweep_example.py`. Include the JSON message table (`set_param`, `subscribe`, `unsubscribe`, `get_matrix`, `ping`) and the four streams (`rt`, `ack`, `matrix`, `status`).
- **Key behaviors:** command reply means "GUI transmitted it"; firmware accept/reject arrives on the `ack` stream; names resolve against the handshake matrix; raw integer ids work before handshake; a command reply never navigates the GUI.
- **How to disable:** `utils/config.py` -> `RemoteConfig.ENABLED = False`.
- **Design/rationale:** link to `docs/superpowers/specs/2026-07-17-udp-remote-control-design.md`.
- **Safety notes:** localhost only; firmware bounds-checks every write (`ParamUpdateValidation.h`); send scope is parameters only (no motor/trial control).

Reference the existing files in `Modification log with claude/` for tone and structure.

- [ ] **Step 2: Review checkpoint (user commits)**

```bash
git add "Modification log with claude/Remote-Control-UDP.md"
```
Summarize and hand off. Do not run `git commit`.

---

## Self-Review Notes

- **Spec coverage:** send=params-only (Task 2/3), receive rt/ack/matrix/status (Task 4/5), localhost bind (Task 3), default-on + banner (Task 5), names-or-ids resolution (Task 2), service-owns-protocol / standalone client (Task 1/6), `_apply_param_update` split (Task 5), subscriber lifecycle + eviction (Task 3/4), headless tests + loopback drift guard (Task 6), write-up (Task 7). All spec sections map to a task.
- **No BLE/firmware files modified** — verified against Global Constraints.
- **Type consistency:** payload `[bool,int,int,int,float]` is produced by `resolve_set_param` (Task 2), emitted by `setParamRequested` (Task 3), and consumed by `_apply_param_update` (Task 5) — consistent. Matrix row shape is identical across service resolution, `set_controller_matrix`, and client `controllers()`.
