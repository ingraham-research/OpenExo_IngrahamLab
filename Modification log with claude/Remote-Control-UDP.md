# UDP Remote Control for the Python GUI

**Date:** 2026-07-18
**Scope:** Python GUI only (`Python_GUI/`). No firmware changes.
**Status:** Implemented, all headless tests passing (35/35). Not yet exercised against a real exo — bench validation with a live device is a separate step, pending user consent.
**Detailed design/plan (deeper reference):**
`docs/superpowers/specs/2026-07-17-udp-remote-control-design.md`,
`docs/superpowers/plans/2026-07-18-udp-remote-control.md`

This document is the high-level "what / why / how to modify" summary.

---

## Why

Previously the only way to change a controller parameter at run time was to click through the GUI
by hand. This adds a **localhost UDP listener** so an external script can set controller parameters
programmatically and receive everything the GUI receives over BLE — firing the exact same code paths
a button click fires. Intended use: scripted torque sweeps and closed-loop experiments (e.g. the
planned `torque_scale` parameter).

The exoskeleton is unaffected: the receive path taps signals the GUI already produces from BLE data,
so there is **no new traffic toward the device** and no extra load on the Nano/Teensy.

## What it does

- **Send (script → GUI):** one command, `set_param`, which sets a controller parameter to a value.
  Nothing else — no motor/trial control. It fires the same path as the GUI's Apply button.
- **Receive (GUI → script):** four telemetry streams a client can subscribe to — `rt` (the ~100 Hz
  real-time frames), `ack` (every firmware accept/reject with a reason), `matrix` (the controller
  metadata + current values), and `status` (connect/disconnect, device errors, shutdown progress).
- **Default on**, localhost only (`127.0.0.1:9750`). A banner prints at GUI startup confirming the
  address; disable via `utils/config.py` → `RemoteConfig.ENABLED = False`.

## Wire protocol

One UDP datagram = one JSON object. Commands get an immediate reply to the sender.

| Command | Meaning |
|---|---|
| `{"cmd":"set_param","joint":..,"controller":..,"param":..,"value":..,"bilateral":false,"id":N}` | Set a parameter. `joint`/`controller`/`param` may be **names** (resolved via the handshake matrix) or raw **integer ids**. |
| `{"cmd":"subscribe","streams":["rt","ack",...]}` | Start receiving streams (sent back to the sender's address). |
| `{"cmd":"unsubscribe"}` | Stop. |
| `{"cmd":"get_matrix"}` | Get the cached controller matrix now (replayable mid-session). |
| `{"cmd":"ping"}` | Liveness check. |

Replies: `{"ok":true,"id":N}` or `{"ok":false,"id":N,"error":"...","code":"bad_name"}`.
Stream frames carry a `"stream"` key, e.g. `{"stream":"rt","values":[...],"names":[...]}`.

**Key behavior:** a command reply only means *"the GUI accepted and transmitted it."* Whether the
**exo** accepted the parameter arrives later on the `ack` stream (with the firmware's reason). Names
resolve against the handshake matrix; raw integer ids work even before a handshake. A remote command
never navigates the GUI (unlike the Apply button).

## Files

| File | Role |
|---|---|
| `Python_GUI/remote/__init__.py` | Empty package marker (must not import `service` — would pull in Qt). |
| `Python_GUI/remote/service.py` | **Authority** for the wire format (PROTOCOL block: codec + name resolution) plus `RemoteControlService` (QUdpSocket, dispatch, subscribers, fan-out). |
| `Python_GUI/remote/client.py` | Standalone `ExoRemote` reference client. Stdlib only; carries its own copy of the wire format (no shared protocol module — see below). |
| `Python_GUI/MainWindow.py` | Wires the service to existing `RtBridge`/device-manager signals; splits `_on_apply_settings` into `_apply_param_update` (no navigation, used by remote) + the GUI wrapper; prints the startup banner. |
| `Python_GUI/utils/config.py` | `RemoteConfig` (ENABLED, HOST, PORT, MAX_SEND_FAILURES). |
| `Python_GUI/examples/sweep_example.py` | Worked example using `client.py`. |
| `Python_GUI/tests/test_remote_*.py` | Headless tests (protocol, service, MainWindow wiring, loopback + no-BLE end-to-end). |

No changes to `ExoCode/`, `services/RtBridge.py`, or `services/QtExoDeviceManager.py`.

## How to use it

1. Run `python GUI.py`; the banner confirms `udp://127.0.0.1:9750`. Connect to the exo as usual.
2. From a script, use `remote/client.py` (see `examples/sweep_example.py`):
   ```python
   from client import ExoRemote
   with ExoRemote() as exo:
       exo.wait_for_matrix()
       exo.set_param("Ankle(L)", "spline", "node1_y", 3.0, bilateral=True)
       for frame in exo.stream("rt", timeout=5):
           print(frame)          # labeled dict: {"percent_gait": .., "torque_l": .., ...}
   ```
   `client.py` imports nothing but the standard library, so it can be copied next to any script or a
   different tool (e.g. MATLAB) can speak the same JSON protocol directly.

## How to modify

- **Change the wire format:** edit the `PROTOCOL` block at the top of `remote/service.py` (the
  authority), then update `remote/client.py` by hand to match. There is deliberately **no shared
  protocol module** — this keeps the client dependency-free and portable, at the cost of manual
  sync. The loopback integration test (`tests/test_remote_loopback.py`) is the drift guard: it runs
  the real client against the real service end-to-end and fails if they diverge.
- **Add a stream:** add it to `VALID_STREAMS` in `service.py`, add a `publish_*` slot, and wire the
  source signal in `MainWindow.__init__`.
- **Expose it on the LAN** (not recommended): change `RemoteConfig.HOST`. Note this removes the
  localhost-only safety and would need a firewall exception and an auth check.

## Safety notes

- Localhost only; nothing on the lab network can reach the port as configured.
- The firmware bounds-checks every parameter write (`ExoCode/src/ParamUpdateValidation.h`) and
  returns a rejection reason, surfaced on the `ack` stream — a bad value is rejected by the exo
  exactly as it would be from the GUI.
- Send scope is parameters only. No motor on/off, trial start/stop, or End Trial over UDP.

## Sequenced next

The `torque_scale` controller parameter (a global torque magnitude knob) is designed to be driven by
this remote for programmatic sweeps. Findings and injection-point analysis are in
`docs/superpowers/2026-07-14-control-loop-and-transparency-backlog.md`.
