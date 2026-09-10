# Teensy 4.1 Onboard SD Logging — Design

**Date:** 2026-07-06
**Status:** Approved (pending written-spec review)
**Author:** Zijie Jin + Claude

## Goal

Add an onboard SD-card logging system to the OpenExo Teensy firmware that records
per-motor and per-stride data during a trial, in a plain-text format matching the
user's existing hip-exo logs (`Motor_L_log.txt`, `Motor_R_log.txt`, `Ground_strike_log.txt`).
Logs are written to the Teensy 4.1 built-in SD card and pulled off later for offline
analysis. The immediate motivating use case is characterizing the right-ankle motor's
mid-trial output collapse (needs motor **current**, enabled state, and timeout count),
which the existing BLE→GUI telemetry stream does not capture.

## Non-goals

- No change to the existing BLE/GUI real-time telemetry path (it stays as-is).
- No motor **temperature** logging in this iteration (deferred — the `MotorData`
  struct does not currently parse temperature from the CAN feedback). Current +
  enabled + timeout_count are sufficient to distinguish thermal foldback vs. CAN
  timeout for now.
- No binary logging in this iteration (kept in reserve; see Format rationale).

## Context / constraints

- **Two-MCU architecture.** The Teensy 4.1 runs the real-time control loop and owns
  the SD card (`SD.h` / `BUILTIN_SDCARD`, SdFat-backed) and all `ExoData`. The Nano
  handles BLE. Logging lives entirely on the **Teensy**.
- **Loop cadence.** `Exo::run()` (`ExoCode/src/Exo.cpp:51`) gates on
  `delta_t >= lower_bound` (`LOOP_FREQ_HZ = 500`, Config.h) and returns `true` exactly
  once per control cycle. The `.ino` loop calls `bool ran = exo.run();`. Logging
  piggybacks on `ran == true`.
- **SD already used for reads.** Config/params are read from SD at boot
  (`ParseIni`, `ParamsFromSD`, `ListCtrlParams`). `IniFile.h` declares
  `extern SdFat SD`; `ParamsFromSD`/`ListCtrlParams` use `SD.begin(SD_SELECT)` with
  `#include <SD.h>` and `SD_SELECT = BUILTIN_SDCARD`. The logger must use the same
  underlying volume — reconciling `SD.h` (`SDClass`) vs. the `SdFat` instance is an
  implementation step.
- **Existing field collector.** `PiLogger.h` already enumerates the useful field set
  (per side: motor pos/vel/current/command, joint torque/pos/vel, FSR, stance, error)
  and streams it as tab pairs to Serial. `SdLogger` reuses this field set, writing CSV
  rows to SD instead.

## Design decisions (locked)

| Decision | Choice |
|---|---|
| Session trigger | **Auto**, tied to the controller being active — start when status enters `trial_on` **or** `fsr_calibration` **or** `fsr_refinement` (the firmware's `active_trial` set — motors run in all of these), stop when it leaves them (and on estop). NOTE: keying on `trial_on` alone was the original empty-logs bug — the exo runs the controller during `fsr_refinement` and may never formally reach `trial_on`. |
| File layout | **Mirror hip exo** — separate `Motor_L_log.txt`, `Motor_R_log.txt`, `Ground_strike_log.txt` per session |
| Sample rate | **Decimated ~100 Hz** (`LOG_DECIMATION = 5` over the 500 Hz loop; 2–3 → ~200 Hz) |
| Field set | **Full PiLogger set + current** |
| Serialization | **Human-readable text** (`.txt`, comma-separated). Extension/delimiter are free; text-vs-binary is the only real perf lever, and text is within budget at 100 Hz |
| Temperature | Deferred |

## Architecture

New class `SdLogger` in `ExoCode/src/SdLogger.{h,cpp}`, constructed with `ExoData*`,
instantiated in `ExoCode.ino` next to `exo`. Single public entry point `update()`,
called every `.ino` loop iteration (it internally no-ops unless a cycle ran and the
decimation counter wraps). Modeled structurally on `PiLogger`.

```
ExoCode.ino loop:
    bool ran = exo.run();
    sd_logger.update(ran);      // <-- new; never blocks control
```

### Responsibilities

- **Session lifecycle:** detect trial edges from `_data->get_status()`; open/close files.
- **Row emission:** on each logged cycle, format one row per used side into
  `Motor_L_log.txt` / `Motor_R_log.txt`.
- **Stride events:** on a ground-strike (`side_data->ground_strike`), append a row to
  `Ground_strike_log.txt`.
- **Flush management:** periodic, staggered `flush()` to bound data loss without
  stalling any single loop.

### Boundaries / interface

`SdLogger` depends only on `ExoData*` (read-only) and the SD file API. It exposes
`update(bool ran)` and nothing else. It can be compiled out entirely via a
`ENABLE_SD_LOGGING` guard. Control code has no dependency on the logger.

## Session lifecycle (auto / trial-tied)

- At boot (after config parse), scan `/EXOLOG/` for the next free 4-digit index
  `<NNNN>`; remember it.
- **inactive → active** (status enters `trial_on`/`fsr_calibration`/`fsr_refinement`):
  create `/EXOLOG/<NNNN>/`, open the three files, write each file's start-marker line +
  header line. If the Teensy RTC is set (coin cell present), prefix the folder with
  `YYYYMMDD_HHMMSS`; otherwise use the bare index. Increment `<NNNN>` for next session.
- **active → inactive / estop:** flush and close all files.
- Robustness: the folder is chosen at trial start (not renamed at end), so a power loss
  mid-trial still leaves a findable, mostly-complete folder.

## File format

Each file: line 1 = start marker, line 2 = header, then data rows. Comma-separated,
`.txt` extension, matching hip-exo parser expectations.

**`Motor_L_log.txt` / `Motor_R_log.txt`** — one row per logged cycle (~100 Hz):

```
# exo=<name> rate=<Hz> fw=<const> t0_us=<micros>
Motor,Teensy_time_s,Status,Gait_phase,Position_rad,Velocity_rad_s,Torque_Nm,Commanded_Torque_Nm,Current_A,Filtered_Torque_Nm,Desired_Torque_Nm,Toe_FSR,Stance,Enabled,Timeout_ct,Error
```

Columns 1–7 are the exact hip-exo layout; 8–15 are the "full PiLogger + current"
additions. Field sources (per side, joint = whichever `is_used`, ankle for current build):

| Column | Source |
|---|---|
| Motor | side id (L/R) |
| Teensy_time_s | `micros()/1e6` (or `millis()/1e3`) |
| Status | `_data->get_status()` (e.g. 2=trial_on, 5=fsr_calibration, 6=fsr_refinement) |
| Gait_phase | `side_data->percent_gait` |
| Position_rad | `joint.motor.p` |
| Velocity_rad_s | `joint.motor.v` |
| Torque_Nm | `joint.torque_reading` (measured joint torque) |
| Commanded_Torque_Nm | `joint.motor.last_command` |
| Current_A | `joint.motor.i` |
| Filtered_Torque_Nm | `controller.filtered_torque_reading` |
| Desired_Torque_Nm | `controller.desired_torque` |
| Toe_FSR | `side_data->toe_fsr` |
| Stance | `side_data->toe_stance` |
| Enabled | `joint.motor.enabled` |
| Timeout_ct | `joint.motor.timeout_count` |
| Error | `_data->error_code` |

**`Ground_strike_log.txt`** — one row per ground strike. Note: OpenExo's `ground_strike`
is the rising edge of either FSR out of swing; with the heel FSR unused it is effectively
the **toe-FSR strike onset** (foot loading at start of stance), not a literal heelstrike.

```
Leg,Teensy_time_s,Prev_step_ms,Expected_step_ms
```

Sources: leg id; `micros()/1e6`; previous step duration
(`side_data->last_step_duration`, set in `Side::_update_expected_duration`);
`side_data->expected_step_duration` (the gait-phase estimator — useful for spotting the
timing corruption seen in the earlier spline bug).

## Timing & real-time safety

- **Decimation:** counter increments when `ran == true`; emit rows only when it reaches
  `LOG_DECIMATION` (default 5 → 100 Hz), then reset.
- **Formatting:** `snprintf` into a fixed stack `char` buffer (no `String`/heap), then
  `file.write(buf, len)`. Teensy SD buffers a 512 B sector, so most writes are a memcpy;
  a physical sector flush occurs only when the buffer fills.
- **Flush:** call `flush()` on a fixed cadence (`FLUSH_INTERVAL_MS`, default ~1000 ms),
  and stagger the three files' flushes onto different cycles so no single loop pays
  multiple SD syncs.
- **Verification during bring-up:** wrap the write in `micros()` timing and confirm the
  worst-case stays within `LOOP_TIME_TOLERANCE`. If cluster-allocation stalls appear,
  escalate to the reserve strategy (SdFat `FsFile` + preallocated contiguous file +
  large user buffer). Not implemented unless needed.

**Chosen write strategy: A** — direct `snprintf` + `write` with periodic staggered
flush. Simple, no extra RAM, ample headroom at 100–200 Hz. (B: RAM ring buffer; C:
SdFat preallocated file — both held in reserve.)

## Error handling / edge cases

- `SD.begin` fails or no card → logger self-disables, emits one warning, control loop
  unaffected. Logging must never block or fault control.
- File open fails → skip the session, warn once.
- Estop / abnormal trial end → periodic flush bounds loss to < ~1 s; folder already
  exists so partial logs are recoverable.
- SD shared with boot-time config reads → logger opens files only after config parse
  completes; single reconciled SD volume.
- Multi-joint builds → log whichever joint `is_used` per side; column names are generic.

## Tunables

**Runtime** — read from a `[Logging]` section of `/config.ini` at boot (via `IniFile`,
the same reader used for the rest of config), so they can be changed by editing the SD
card with **no reflash**. Read directly by `SdLogger`, *not* routed through the
`config_to_send` array (that array is a fixed UART-shared struct; only the Teensy logs, so
adding keys there would be invasive and needless). Missing file/section/keys fall back to
the compile-time defaults, so existing SD cards keep working.

| config.ini key (`[Logging]`) | Default (`#define`) | Meaning |
|---|---|---|
| `sdLogEnabled` | `SD_LOG_DEFAULT_ENABLED = 1` | 1 = log during trials, 0 = off |
| `sdLogDecimation` | `SD_LOG_DEFAULT_DECIMATION = 5` | every Nth 500 Hz cycle (5 → ~100 Hz) |
| `sdLogFlushMs` | `SD_LOG_DEFAULT_FLUSH_MS = 300` | per-tick flush cadence (ms) |

**Compile-time** (in `SdLogger.h`):

- `ENABLE_SD_LOGGING` — compile the logger in/out entirely
- `SD_LOG_BASE_PATH` — default `/EXOLOG`
- `SD_LOG_DEBUG` — Serial debug prints

## Open implementation risks (resolve during build)

1. **SD object reconciliation:** `SD.h` `SDClass` vs. `IniFile.h` `extern SdFat SD`.
   Confirm both use one underlying volume; pick one API for the logger's writes.
2. **RTC availability:** use datetime in folder name only if the Teensy RTC is valid;
   otherwise fall back to the scanned index. No dependence on a set clock.
3. **`ran` propagation:** confirm the `.ino` loop passes the `exo.run()` return into
   `sd_logger.update()` (or hook inside `Exo::run()` after the control cascade).

## Future work

- Motor temperature column (needs CAN-feedback parse addition in the motor driver).
- Optional binary logging mode for full 500 Hz.
- Optional IMU / keyboard-event logs to fully mirror the hip-exo file set.
