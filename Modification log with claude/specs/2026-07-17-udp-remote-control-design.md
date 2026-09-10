# UDP Remote Control for the OpenExo GUI — Design

**Date:** 2026-07-17
**Branch:** `add_remote_control`
**Status:** awaiting review

## Goal

Let an external script on the same PC change controller parameters at run time, and receive everything
the GUI receives over BLE, without the operator touching the GUI.

The GUI stays the single owner of the BLE link. The remote layer is a second *input* into paths that
already exist — it fires the identical BLE traffic a button click fires.

## Non-goals

- **No firmware changes.** Nothing in `ExoCode/` is touched.
- **No new BLE traffic toward the exo.** The RT fan-out is PC-side only; the Nano/Teensy cannot tell the
  difference (see "Why the exo is unaffected").
- **No trial control.** Motor on/off, mark, calibrate, End Trial stay GUI-only. Scope is parameter
  updates out, telemetry in.
- **Not a network service.** Localhost only. No auth, no LAN exposure.
- **`torque_scale` is not part of this.** Sequenced next, as its own spec; see
  `../2026-07-14-control-loop-and-transparency-backlog.md`.

## Why the exo is unaffected

The exo already streams RT data whenever it has new data (`ComsMCU.cpp:172-202`), and the GUI already
receives every frame, parses it (`RtBridge.feed_bytes`), plots it, and writes it to CSV. Forwarding a
copy taps `RtBridge.rtDataUpdated` — a signal that fires today. There is no new request and nothing
travelling toward the device. This is *no* added exo load, not merely a small one.

Bandwidth: RtBridge normalizes to 16 floats at ~70-100 Hz (`_expected_hz = 100`). ~25 KB/s as JSON
(~200 kbps). Loopback UDP moves multiple Gbps — ~3 orders of magnitude of headroom.

The send lands on the Qt main thread, which already does a `csv.writerow` + pyqtgraph update per frame —
both more expensive than a ~5-20 µs loopback `sendto`. It will not become the bottleneck. The socket is
**non-blocking** regardless, so a stalled or absent reader can never back-pressure the GUI thread; a full
buffer drops the frame, which is correct for telemetry.

## Architecture

```
script ──UDP──> RemoteControlService ──emits──> MainWindow._apply_param_update
                                                        │
                                                        └──> qt_dev.updateTorqueValues ──BLE──> exo
                                                                                                 │
script <──UDP── RemoteControlService <──slots── RtBridge signals <────────────────BLE────────────┘
```

`RemoteControlService` knows nothing about BLE or MainWindow internals. It:
- parses inbound JSON, resolves names to IDs, emits `setParamRequested(list)` — deliberately the same
  5-element shape `ActiveTrialSettingsPage.applyRequested` already emits;
- exposes slots (`publish_rt`, `publish_ack`, ...) that serialize and send to subscribers;
- owns subscriber state.

`MainWindow` wires it to signals that already exist:

```python
self.rt_bridge.rtDataUpdated.connect(self.remote.publish_rt)
self.rt_bridge.paramUpdateAckReceived.connect(self.remote.publish_ack)
self.rt_bridge.controllerMatrixReceived.connect(self.remote.set_controller_matrix)
self.rt_bridge.controllerValuesReceived.connect(self.remote.publish_values)
self.rt_bridge.parameterNamesReceived.connect(self.remote.set_param_names)
self.qt_dev.connected.connect(...); self.qt_dev.disconnected.connect(...)   # -> status stream
self.remote.setParamRequested.connect(self._apply_param_update)
```

## Module layout

New package `Python_GUI/remote/` (not `services/` — `services/` is Qt-coupled infrastructure; the client
must never import Qt):

| File | Imports | Role |
|---|---|---|
| `remote/__init__.py` | — | Empty. Must **not** auto-import `service` (that would pull in Qt). |
| `remote/service.py` | PySide6 + stdlib | GUI-side `QUdpSocket` service. **Owns the wire format** — the authority. |
| `remote/client.py` | libraries OK; no shared-protocol file | `ExoRemote` reference client. Carries its own copy of the wire format. |

**There is no shared protocol module.** Deliberate — this is the one structural rule for `client.py`: it
must not depend on a separate protocol-definition file that both sides import. That coupling was the
"single source of truth, no drift" design, which was rejected. Beyond that one rule, `client.py` is free
to import whatever is convenient — standard library, third-party, or other internal files.

- **`service.py` is the authority.** A documented `PROTOCOL` block at the top of the file — message
  names, field keys, error codes, and the `encode`/`decode` helpers — defines the wire format. This is
  the "one python file in the GUI folder that decides the protocol."
- **`client.py` carries its own copy** of the message builders/parsers plus the `ExoRemote` class,
  rather than importing them from the service. In practice this means inlining the wire helpers: the
  only other place the protocol lives is `service.py`, which is Qt-coupled, so importing from it would
  drag Qt into a plain control script. Inlining keeps the client usable from a bare Python process. This
  is a practical consequence, not a portability mandate — the file may still grow whatever imports it
  finds useful.

**Accepted tradeoff:** the wire format lives in two places and *can* drift. The service is authoritative;
keeping `client.py` in sync when the format changes is a manual step, the explicit cost of dropping the
shared module. Mitigation: the `PROTOCOL` block in `service.py` is the single human-readable spec a
client author reads to conform, the protocol is small, and the loopback test (below) catches drift.

## Protocol

One datagram = one JSON object. UTF-8. No framing (UDP preserves message boundaries).

### Commands (script -> GUI)

```json
{"cmd":"set_param","joint":"Ankle(L)","controller":"spline","param":"p_gain",
 "value":3.0,"bilateral":false,"id":42}
```
- `joint` / `controller` / `param`: name (string) or raw ID (int). Names resolved via the handshake
  matrix; ints used as-is.
- `bilateral`: optional, default false. Mirrors across sides exactly as the GUI checkbox does.
- `id`: optional client token, echoed in the reply for correlation.

```json
{"cmd":"subscribe","streams":["rt","ack","status"],"id":1}
{"cmd":"unsubscribe","id":2}
{"cmd":"get_matrix","id":3}     // cached; replayable mid-trial
{"cmd":"ping","id":4}
```

### Replies (GUI -> script, to sender)

```json
{"ok":true,"id":42}
{"ok":false,"id":42,"error":"unknown controller 'spilne' for joint Ankle(L)","code":"bad_name"}
```

**A reply only means the GUI accepted and transmitted the command.** Whether the *exo* accepted it
arrives later on the `ack` stream. This distinction is why forwarding acks matters as much as RT data.

### Streams (GUI -> subscribers)

| Stream | Source signal | Rate | Payload |
|---|---|---|---|
| `rt` | `rtDataUpdated` | ~100 Hz | 16 floats + labels from `parameterNamesReceived` |
| `ack` | `paramUpdateAckReceived` | per write | `joint_id`, `controller_id`, `param_index`, `accepted`, `reason` (text + code) |
| `matrix` | `controllerMatrixReceived` / `controllerValuesReceived` | at handshake | available knobs + current values |
| `status` | `connected`/`disconnected`/`deviceErrorReceived`/`shutdownProgressReceived` | on change | link + device state |

```json
{"stream":"rt","t":1752710400.123,"values":[...],"names":["percent_gait","torque_left",...]}
{"stream":"ack","joint_id":68,"controller_id":0,"param_index":1,"accepted":false,
 "reason":"value out of bounds","reason_code":5}
```

Acks are forwarded **unconditionally** — every ack, not just ones the script caused. Firmware acks carry
no correlation token (`RtBridge._handle_param_update_ack`), so per-client attribution is impossible;
broadcasting sidesteps it and matches "receive everything the GUI receives."

### Subscription lifecycle

Reply-to-sender: the service streams to whatever address the `subscribe` datagram came from. No
configured port, works with ephemeral ports, survives script restarts.

Explicit subscribe (rather than auto-subscribing any sender) means a one-shot `set_param` script doesn't
get a 100 Hz fire-hose it never asked for.

Teardown: explicit `unsubscribe`, or **N consecutive send failures**. On Windows, sending to a dead
localhost port yields ICMP port-unreachable and the next send fails with `WSAECONNRESET` — so a dead
script drops itself. That same error must be **swallowed on the send path**, never propagated: it is
expected, and left unhandled it is a confusing GUI-side crash caused by an unrelated process exiting.

## Name resolution

Resolved against the handshake matrix (`_controller_matrix`), fed in via `set_controller_matrix` exactly
as it's already fed to `settings_page`. Rows look like:

```
["Ankle(L) (68)", "68", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"]
```

- joint: match `"Ankle(L)"` against row[0]'s name part, or row[1] as int
- controller: match row[2] case-insensitively within the joint
- param: index of the name in row[4:]

Names are matched case-insensitively; unknown names produce `bad_name` with the valid options listed in
the error string (a script author should never have to read source to learn the spelling).

**Do not use `JointConfig.ID_TO_NUM`.** All 8 entries have left/right inverted vs the firmware bitfield
(`ParseIni.h:125-137`: `left=0b01000000` so `left_ankle=68`; the table claims 68=Right). It's dead code
today; resolving from the device-reported matrix means this class of bug can't be inherited. Deleting it
is filed in the backlog.

If no matrix has arrived (not connected / pre-handshake), name-based commands are rejected with
`no_matrix`. Raw-ID commands still work.

## MainWindow refactor

`_on_apply_settings` (MainWindow.py:851) currently ends with `self.stack.setCurrentWidget(self.trial_page)`.
Wiring remote commands into it directly would make **every remote parameter set yank the GUI to the trial
page**, including while the operator is mid-edit elsewhere. Split it:

```python
def _apply_param_update(self, payload) -> bool:
    """Validate, queue the pending ack, send over BLE. No navigation."""
    # (existing body of _on_apply_settings, minus the setCurrentWidget call)

@QtCore.Slot(list)
def _on_apply_settings(self, payload):
    """GUI path: apply, then return to the trial page."""
    self._apply_param_update(payload)
    self.stack.setCurrentWidget(self.trial_page)
```

Remote calls `_apply_param_update`. Behavior of the GUI path is unchanged.

Remote updates deliberately **do not** write the page's "last selection" prefs (`_save_settings` lives in
the page's `_on_apply`, not here), so scripted sweeps don't pollute the operator's saved GUI state.

Known limitation, accepted: `_queue_pending_param_updates` keys pending acks by
`(joint, controller, param_index)` and pops FIFO. If GUI and script write the *same* parameter within the
ack window, the two pendings are indistinguishable — firmware acks carry no token. Consequence is limited
to which pending record the value-cache update attributes; both parties still see every ack. Not worth
solving.

## Startup banner

Default **on**, with a visible notice at startup:

```
============================================================
 REMOTE CONTROL ACTIVE - listening on udp://127.0.0.1:9750
 External scripts can change controller parameters.
 Localhost only. Disable: utils/config.py RemoteConfig.ENABLED
============================================================
```

Emitted from `MainWindow.__init__`, immediately after the service binds — so the banner reports the
*actual* bound port and only prints when the socket is really listening. (Putting it in `GUI.py` would
mean announcing a port before knowing the bind succeeded.)

Emitted with `print()`, **not** `logger.info` — `GUI.py:56` sets the console handler to `WARNING`, so an
info log would never reach the terminal. Also logged at INFO for the record. If the bind fails (e.g. port
in use), print that failure just as loudly and continue **without** the service; the GUI must still start.

## Config

`utils/config.py` gains:

```python
class RemoteConfig:
    ENABLED = True
    HOST = "127.0.0.1"     # localhost only; widening this is a deliberate one-line change
    PORT = 9750
    MAX_SEND_FAILURES = 5  # consecutive failures before dropping a subscriber
```

## Client API

`client.py` is a **self-contained** reference the user extends in place or imports. Its one rule is that
it doesn't depend on a shared protocol file — it carries its own copy of the wire format. It may import
libraries and internal files freely otherwise.

```python
from client import ExoRemote          # or however the user wires it into their script

with ExoRemote() as exo:                      # 127.0.0.1:9750
    exo.wait_for_matrix(timeout=5)            # what knobs exist?
    print(exo.controllers("Ankle(L)"))        # ['zeroTorque', 'spline', ...]

    exo.set_param("Ankle(L)", "spline", "p_gain", 3.0, bilateral=True)
    # blocks for the GUI reply; raises RemoteError on rejection

    for frame in exo.stream("rt", timeout=5):
        print(frame["percent_gait"], frame["torque_left"])   # labeled via param names
```

- Needs only stdlib in practice (`socket`, `json`, `time`), but may import anything useful — the sole
  rule is no dependency on a shared protocol-definition file.
- `set_param` blocks on the GUI reply (short timeout) and raises `RemoteError` on rejection. Firmware
  accept/reject surfaces on the ack stream; `exo.last_ack()` / the `ack` stream exposes it.
- `stream()` yields decoded frames; RT frames are labeled dicts using `parameterNamesReceived`, so
  scripts never index magic float positions.
- Sends a `subscribe` on enter, `unsubscribe` on exit.
- The wire helpers here mirror `service.py`'s `PROTOCOL` block. If that block changes, this file is
  updated by hand to match — the accepted cost of no shared module.

## Testing

The service tests **headless — no exo, no BLE, no motors move**:

1. Feed a fake matrix -> send `set_param` by name -> assert emitted payload `[False, 68, 0, 1, 3.0]`.
2. `bilateral: true` -> assert `build_parameter_updates` mirrors to joint 36.
3. Malformed JSON / unknown name / no matrix -> assert error reply with the right code.
4. Raw-ID command with no matrix -> assert it still works.
5. `publish_rt` -> assert a subscriber receives a labeled frame; assert a non-subscriber does not.
6. Send failure x N -> assert subscriber dropped, no exception escapes.
7. Round-trip the service's `encode`/`decode` for every message type.

Then a loopback integration test: real service + real `client.py`, no device. This test is also the
**drift guard** — because the two sides carry independent copies of the wire format, an end-to-end
`set_param` + stream round-trip is what catches a client that has fallen out of sync with the service.

**Bench validation with the exo is a separate step, only with explicit consent.** Suggested first check:
set a known-safe parameter, confirm the ack stream reports `accepted`, confirm the GUI's own value cache
agrees.

## Files

| File | Change |
|---|---|
| `Python_GUI/remote/__init__.py` | new — empty; no Qt-side auto-import |
| `Python_GUI/remote/service.py` | new — QUdpSocket service; owns the `PROTOCOL` block (authority) |
| `Python_GUI/remote/client.py` | new — self-contained `ExoRemote`; carries its own copy of the wire format |
| `Python_GUI/MainWindow.py` | wire-up + `_apply_param_update` split + startup banner |
| `Python_GUI/utils/config.py` | `RemoteConfig` |
| `Python_GUI/examples/sweep_example.py` | new — worked example using `client.py` |
| `Python_GUI/tests/test_remote_*.py` | new — headless tests |
| `Modification log with claude/Remote-Control-UDP.md` | write-up |

No `protocol.py`: the wire format lives in `service.py` (authority) and is mirrored in `client.py`.

No changes to `ExoCode/`, `RtBridge.py`, or `QtExoDeviceManager.py`.

## Open questions

None blocking. Port 9750 is arbitrary — say if there's a lab convention.
