# external_control — running an optimization backend against the OpenExo GUI

An external program that owns an optimization backend, computes machine actions, and applies them to
the exo by writing controller parameters through the GUI's UDP remote service.

This is the OpenExo counterpart of `main_control_code.py` on the hip exo. The difference is where the
control loop lives: there, that file owns the motors and runs splines at 200 Hz; here the Teensy owns
all of that, and this code is supervisory — it changes one number, infrequently, and confirms the
change landed.

## Files

| File | What it is |
|---|---|
| `main_external_control.py` | The "main code". Mode selection, session setup, the supervisory loop. |
| `Utilities/ExoLink_utilities.py` | The UDP link to the GUI. Resolves names to ids, writes a parameter, waits for the exo's acknowledgement, retries on silence. |
| `Utilities/ActionMap_utilities.py` | Turns a machine action (a torque percentage) into the `splineAlt` parameter writes that implement it. |
| `PSEUDOCODE.md` | The whole design as pseudocode, for hand-typing. Also lists the five traps the code exists to handle. |

## Running it

1. Start the GUI (`python GUI.py`), connect it to the exo, and **start the trial from the GUI**.
2. Start whatever is feeding this code — the self-paced treadmill on UDP 5004 for game theory mode, or
   your timing sender on UDP 5003 for `udp` mode.
3. From this folder: `python main_external_control.py`

It will not start anything on its own. There is no trial start/stop, motor enable or calibration over
UDP, by design — a human operator drives the GUI throughout.

## The mode menu

Kept as close to the hip exo's `main_control_code.py` as the hardware allows.

```
2  game theory backend      human action = self-paced treadmill speed (UDP 5004)
                            machine action = torque percentage
1  everything else
     OP        run the profile shape already on the SD card
     plantar   plantarflexion only, you pick the peak timing (dorsi lobe disabled)
     dorsi     dorsiflexion only, you pick the peak timing (plantar lobe disabled)
     udp       hold at zero torque, then follow peak timings arriving on UDP 5003
     <enter>   zero torque, ignore UDP
```

Two deliberate differences from the hip exo:

- **Game theory has only one option now.** The hip asked `cadence` or `SPT`. Cadence needs stride
  timing, which only exists on the Teensy — and this code derives nothing from raw telemetry. So
  treadmill speed is the only human action available here.
- **`flexion` / `extension` became `plantar` / `dorsi`**, which are the ankle's two lobes and the names
  `splineAlt.csv` uses. They behave the same way: pick a lobe, place its peak, disable the other.

### UDP mode

Same contract as the hip exo: a positive value is a peak timing in percent gait, and a non-positive
value means "stop assisting for now". Two differences worth knowing, both because a UDP update here
costs a BLE round trip rather than being a local variable:

- **Updates are rate limited** to one write every 0.5 s (`udp_min_write_interval`), so a chatty sender
  cannot queue writes faster than the link retires them. A value arriving inside that window is
  **held, not dropped** — a fresher value simply replaces it, so you always get the latest commanded
  state.
- **A disable command bypasses the rate limit** and cannot be coalesced away by a later timing. It is a
  safety command, so it goes straight through.

## Before the first run

- **Set `exo_control_code_path`** at the top of `main()` to wherever the hip exo's code base lives. The
  game theory backend and the UDP receiver are imported from there rather than duplicated, so the
  algorithm has exactly one copy.
- **Check `m_action_max`.** It is the hard ceiling on the machine action, in percent, applied before
  every write. It ships at 60. At 100 the CSV asks for the full 15 Nm, and commanded torque is
  believed to come out about 1.165× the requested value because of how `t_ff` is packed — verify that
  on the bench before raising it.
- **Confirm the controller names.** The code offers to print the matrix the exo advertised at startup.
  Names are truncated by the BLE handshake, so check them rather than assuming.

## Dependencies

`main_external_control.py` and both utilities are **standard library only**, same as
`Python_GUI/remote/client.py`. `numpy` is pulled in only with the game theory backend itself, so
manual mode runs on a bare Python install.

## Design notes

The full design, the rationale, and the alternatives that were rejected are in
`Modification log with claude/specs/2026-09-08-experiment-orchestrator-design.md`.

Two things are worth repeating here because they are easy to undo by accident:

- **Silence is the only failure signal.** A dropped BLE write produces no negative ack, and the GUI
  returns `ok` to a `set_param` even with no exo connected — `ok` only means it queued a write. Every
  write in this code is confirmed against the exo's own acknowledgement.
- **The first write to `splineAlt` must be `TorqScale = 0`.** Writing any parameter to a controller
  that is not the active one switches the joint to it and loads its SD-card defaults, and
  `splineAlt.csv` ships `TorqScale = 95`. `engage_controller_safely()` exists for exactly this.
