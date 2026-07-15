# Control-Loop & Transparency — Findings & Backlog

**Date:** 2026-07-14
**Branch:** `fix_zerotorque_pid`
Running list of diagnosed issues and to-dos from the zero-torque transparency work. See
`docs/superpowers/specs/2026-07-10-*` for the shipped designs.

## Done / working

- **Non-blocking SD logger** — ring buffers + `isBusy()`-gated single-sector SdFat drain. Confirmed:
  during a trial `maxSD` ~2-4 ms (was blocking ~20 ms), zero dropped data. Logging no longer starves
  the loop.
- **Ankle zero-torque = transparency** via `ZeroTorque(use_pid=1)` at PJMC-0's low-gain regime
  (`zeroTorque.csv = 1,3,0,0.001`); `ZeroTorque::calc_motor_cmd` now writes `filtered_torque_reading`;
  `config.ini ankleDefaultController = zeroTorque`.
- **Spline "jitter crazily" solved** — the card had an 8-node (22-param) `spline.csv` while the
  firmware Spline is 5-node (16-param), so index 12 (`use_pid`) was misread as Node7_x=85 (PID on) and
  index 14 (`i_gain`) as Node8_x=100 (huge integral windup). Fix: put the matching 5-node CSV on the
  card. NOT caused by our changes.

## Confirmed root causes still open

### 1. Control-loop stalls (~12 ms), i.e. loop ~315 Hz not 500 Hz — CONTROL PATH, not SD
Measured with `maxLoop`/`maxSD` instrumentation: `maxLoop` ~12 ms, `maxSD` ~2-4 ms => the stall is in
`exo.run()`, not the logger. Leads:
- **`MY_SERIAL.flush()` in `UARTHandler::UART_msg` (UARTHandler.cpp:64)** blocks until UART TX
  completes; hit every 9 ms by the real-time data packet (`_real_time_msg_delay = 9000`). At 256000
  baud a ~50-byte packet is ~2 ms, so the flush is *part* of the 12 ms but maybe not all.
- TODO: instrument `exo.run()` sections (run_side/CAN vs UART poll vs real-time send) to pin the exact
  12 ms before changing comms.
- TODO (fix): make the high-rate, loss-tolerant real-time send **non-blocking** (drop its `flush()`),
  keep `flush()` for config/param/ack messages. Verify GUI connect/plot/param-apply still work.
- Note: even fully fixed, the trial loop baseline is ~control-cost-limited (~315-550 Hz); no-trial is
  ~550 Hz, so control/CAN work per cycle is the ceiling, not logging.

### 2. AK60v3 CAN velocity/current feedback decode is ~40% garbage
`motor.v` / `motor.i` decode to recurring impossible values (~1488 rad/s, ~319 A). Blocks any
velocity-based feedforward and may be tangled with the control-path stalls. **Next major task.**

### 3. Zero-torque residual jitter = delay-limited P-feedback limit cycle
Confirmed: higher kp -> more oscillation (kp 2 -> 3 raised amplitude), `corr(tau,cmd) ~ -0.8`. It's a
loop-delay limit cycle, near its ceiling; no kp is both transparent and quiet.
- Robust fix: **friction feedforward** (cancel friction open-loop from velocity; light torque trim) —
  needs #2 fixed first for a clean velocity signal.
- Interim/help: filtered derivative in `_pid` (D term divides by measured dt, so it amplifies noise
  ~inversely with loop period -> got worse when the faster loop halved dt). Mainly helps D-heavy
  controllers; zero-torque's d=0.001 is tiny so limited benefit there.

## Nice-to-haves

- Firmware guard: warn/reject when a controller CSV's parameter count != the controller's expected
  count (the spline 8-vs-5-node footgun would have been caught instantly).
- One consistent logging path (the debug log is being moved to its own non-blocking stream).
