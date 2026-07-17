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

### 2. AK60v3 CAN velocity/current feedback decode is wrong — FIXED (applied 2026-07-15, unflashed; pending bench validation)
`Motor.cpp` `read_data()` AK60v3 branch decodes pos/vel/cur with `_uint_to_float()`
(unsigned-with-offset, 12-bit for v/i). That's wrong on every field: the AK60-6 V3.0 CAN status
upload is **signed int16 with fixed scales**, not MIT unsigned-offset. Symptom: `motor.v`/`motor.i`
hit impossible values (~1488 rad/s, ~319 A) ~40% of samples.

**Byte layout the firmware already reads is correct** (`p=[0-1], v=[2-3], i=[4-5]`, 16-bit); only
the decode math is wrong. Triple-confirmed:
- CubeMars *AK Series Module Product Manual V3.0.0* §4.3.1 "CAN Upload Message Protocol": position
  int16 ×0.1 (deg), speed int16 ×10 (ERPM, electrical), current int16 ×0.01 (A), temp int8 [6],
  error uint8 [7]. Includes reference decode code that matches below exactly.
- The lab's working V3 Python driver (`TMotorV3._read_cubemars_message`) — identical: signed int16,
  ×0.1 / ×10 / ×0.01.
- The logs (signed decode corr +0.78 vs unsigned −0.35; `0xFFFF`≈0 at rest).

**AK60-6 V3.0 KV80 constants** (official product page, verified): **pole pairs = 14**, internal
reduction **6:1**, peak current 10.3 A (== firmware `_I_MAX`). So ERPM→output-shaft rad/s divides by
`14 * 6`.

**The fix** (replace the three `_uint_to_float` lines in the AK60v3 read branch, `Motor.cpp:172-177`):
```cpp
int16_t pos_int = (int16_t)((msg.buf[0] << 8) | msg.buf[1]);
int16_t vel_int = (int16_t)((msg.buf[2] << 8) | msg.buf[3]);
int16_t cur_int = (int16_t)((msg.buf[4] << 8) | msg.buf[5]);
_motor_data->p = direction_modifier * (pos_int * 0.1f) * (float)DEG_TO_RAD;                 // rad (output)
_motor_data->v = direction_modifier * (vel_int * 10.0f) / (14.0f * 6.0f) * (2.0f*PI/60.0f);  // rad/s (output)
_motor_data->i = direction_modifier * (cur_int * 0.01f);                                     // A
```
Notes:
- Position/current need no pole-pair count and are fixable immediately; velocity uses the confirmed
  14*6. Recommend one known-speed spin to confirm the velocity magnitude before trusting it downstream.
- **Safety-relevant:** `error_types.h:66` computes `motor_torque = motor.i * motor.kt`; the garbage
  current feeds a ~350 Nm phantom torque into that guard. Fixing current matters regardless of velocity.
- SEND path is fine (byte-packing matches the Python driver; the analog-torque feedback loop absorbs the
  minor torque-FF-via-current and `_V_MAX=48` vs manual ±60 scaling differences). Not the bug.
- Blocks velocity-based friction feedforward (transparency residual, item #3).

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
