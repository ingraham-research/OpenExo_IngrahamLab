"""Diagnostic probe for the Nano -> GUI controller-list handshake.

TEMPORARY INSTRUMENTATION — this module exists only for the `fix_nano_GUI_handshaking`
investigation. See `Modification log with claude/Nano-GUI-Handshake-Audit.md` §7.

It does two jobs:

**Tier 0 — chunk logging.** Records every BLE notification that makes up the handshake payload
(arrival time, running byte offset, length, raw text) plus event markers, one file per connection.
That replaces inferring damage backwards from mangled controller names with a direct measurement of
where the hole is and how big it is.

**Tier 1 — connect-sequence arm.** Selects which order the GUI subscribes to the UART and Error
characteristics in, and records it alongside the chunk log:

    A (control) : UART -> ERROR         inbound CCCD write lands MID-payload
    B           : ERROR -> UART         no inbound request mid-payload, identical RF traffic
    C           : UART -> wait -> ERROR  no inbound request mid-payload, identical RF traffic

Arms rotate automatically per connection attempt so the operator does not have to remember to
interleave them (blocking the arms would confound the treatment with battery/RF drift over a
session). Set ``EXO_HANDSHAKE_ARM=A|B|C`` to pin one arm, or ``EXO_HANDSHAKE_PROBE=0`` to disable
the probe entirely and restore stock behaviour (arm A, no logging).

Nothing here changes what is sent to the exo, and no motor command is involved.
"""

import os
import threading
import time
from datetime import datetime

ARMS = ("A", "B", "C")

_LOCK = threading.RLock()
_ENABLED = os.environ.get("EXO_HANDSHAKE_PROBE", "1") != "0"
_FORCED_ARM = (os.environ.get("EXO_HANDSHAKE_ARM", "") or "").strip().upper()

_arm_index = 0
_current_arm = "A"
_fh = None
_t0 = None
_offset = 0
_chunk_index = 0

# Set by RtBridge when the handshake payload has been fully reassembled. Arm C waits on this
# before subscribing to the Error characteristic.
payload_complete = threading.Event()


def enabled() -> bool:
    return _ENABLED


def current_arm() -> str:
    return _current_arm


def _log_dir() -> str:
    # Python_GUI/services/HandshakeProbe.py -> Python_GUI/Saved_Data/logs/handshake_probe
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    d = os.path.join(base, "Saved_Data", "logs", "handshake_probe")
    os.makedirs(d, exist_ok=True)
    return d


def next_arm() -> str:
    """Pick the arm for the next connection attempt and make it current."""
    global _arm_index, _current_arm
    with _LOCK:
        if not _ENABLED:
            _current_arm = "A"
        elif _FORCED_ARM in ARMS:
            _current_arm = _FORCED_ARM
        else:
            _current_arm = ARMS[_arm_index % len(ARMS)]
            _arm_index += 1
        return _current_arm


def begin_connection(address: str = "") -> None:
    """Open a fresh chunk log for this connection attempt."""
    global _fh, _t0, _offset, _chunk_index
    with _LOCK:
        end_connection("superseded by a new connection attempt")
        payload_complete.clear()
        _offset = 0
        _chunk_index = 0
        _t0 = time.time()
        if not _ENABLED:
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(_log_dir(), f"handshake_{stamp}_arm{_current_arm}.log")
        try:
            _fh = open(path, "w", encoding="utf-8")
            _fh.write(f"# arm={_current_arm}\n")
            _fh.write(f"# address={address}\n")
            _fh.write(f"# started={datetime.now().isoformat()}\n")
            _fh.flush()
        except Exception:
            _fh = None


def mark(label: str, detail: str = "") -> None:
    """Record a named event (READY, ERROR_SUBSCRIBE_DONE, PAYLOAD_COMPLETE, ...)."""
    with _LOCK:
        if _fh is None:
            return
        try:
            dt = (time.time() - _t0) * 1000.0 if _t0 else 0.0
            _fh.write(f"MARK\t{dt:10.3f}\t{label}\t{detail}\n")
            _fh.flush()
        except Exception:
            pass


def log_chunk(data: str) -> None:
    """Record one received BLE notification belonging to the handshake stream."""
    global _offset, _chunk_index
    with _LOCK:
        if _fh is None:
            return
        try:
            dt = (time.time() - _t0) * 1000.0 if _t0 else 0.0
            _fh.write(
                f"CHUNK\t{dt:10.3f}\t{_chunk_index:4d}\t{_offset:6d}\t{len(data):3d}\t{data!r}\n"
            )
            _fh.flush()
        except Exception:
            pass
        _offset += len(data)
        _chunk_index += 1


def log_payload(payload: str, tail: str = "") -> None:
    """Record the fully reassembled payload, so a later diff needs no re-parsing."""
    with _LOCK:
        if _fh is None:
            return
        try:
            _fh.write(f"PAYLOAD\t{len(payload)}\t{payload!r}\n")
            if tail:
                # Anything after the first newline is DISCARDED by the parser. If this is ever
                # non-empty it is a finding in its own right.
                _fh.write(f"DISCARDED_TAIL\t{len(tail)}\t{tail!r}\n")
            _fh.flush()
        except Exception:
            pass


def end_connection(note: str = "") -> None:
    global _fh
    with _LOCK:
        if _fh is None:
            return
        try:
            if note:
                _fh.write(f"# end: {note}\n")
            _fh.write(f"# closed={datetime.now().isoformat()}\n")
            _fh.close()
        except Exception:
            pass
        _fh = None
