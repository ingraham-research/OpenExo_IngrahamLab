# Teensy SD Logging Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox (`- [ ]`) syntax.
> This is on-target Teensy firmware: there is no host test runner. "Verify" = compile
> in the Arduino IDE for **Teensy 4.1** (no errors) + the stated on-hardware check after
> flashing. Per the user's instruction, do **not** commit; each task ends at a
> **review checkpoint** where the user inspects the diff.

**Goal:** Add an onboard SD logger to the OpenExo Teensy firmware that writes per-motor
and per-stride `.txt` logs during a trial, matching the user's hip-exo format.

**Architecture:** A single `SdLogger` class (Teensy side) constructed with `ExoData*`,
driven once per `.ino` loop via `update(bool ran)`. It auto-opens a fresh session folder
on the trial-on edge, writes decimated (~100 Hz) motor rows + per-ground-strike step rows,
and flushes on a staggered cadence so no single control cycle stalls.

**Tech Stack:** Arduino/Teensyduino C++, Teensy `SD.h` (`BUILTIN_SDCARD`, SdFat-backed),
existing `ExoData` object graph.

## Global Constraints

- Target board: **Teensy 4.1** (`ARDUINO_TEENSY41`); all new code guarded with
  `#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)` like the rest of `src/`.
- Control loop is 500 Hz (`LOOP_FREQ_HZ = 500`); logging must **never block** control.
  On any SD failure the logger self-disables and control continues.
- Use the existing Teensy `SD` global + `File` (same volume `IniFile`/`ParamsFromSD` use).
- Human-readable text; no `String`/heap in the per-cycle path (`snprintf` into stack buffers).
- Filenames match the hip exo: `Motor_L_log.txt`, `Motor_R_log.txt`, `Ground_strike_log.txt`.
- Do **not** commit. End each task at a review checkpoint.

## Confirmed field paths (from ExoData graph)

| Datum | Access |
|---|---|
| status / estop / error | `data->get_status()` (uint16_t) · `data->estop` (bool) · `data->error_code` (int) |
| trial-on constant | `status_defs::messages::trial_on` |
| side data | `data->left_side`, `data->right_side` (`SideData`) |
| per-side gait/FSR | `side.percent_gait`, `side.toe_fsr`, `side.toe_stance`, `side.ground_strike`, `side.expected_step_duration` |
| used joint | `side.ankle` / `side.hip` / … (`JointData`, `.is_used`) |
| motor | `joint.motor.p`, `.v`, `.i`, `.last_command`, `.enabled`, `.timeout_count` |
| torque | `joint.torque_reading`, `joint.controller.filtered_torque_reading`, `joint.controller.desired_torque` |

## File structure

- **Create** `ExoCode/src/SdLogger.h` — class declaration + tunables.
- **Create** `ExoCode/src/SdLogger.cpp` — implementation.
- **Modify** `ExoCode/ExoCode.ino` — include, instantiate, drive `update(ran)`.
- **Modify** `ExoCode/src/SideData.h` + `SideData.cpp` — add `last_step_duration` field (Task 5).
- **Modify** `ExoCode/src/Side.cpp` — set `last_step_duration` on each new step (Task 5).

---

### Task 1: SdLogger skeleton + wiring (no-op)

Goal: the logger compiles and is called every loop, but does nothing yet — proving the
wiring is inert w.r.t. control.

**Files:**
- Create: `ExoCode/src/SdLogger.h`
- Create: `ExoCode/src/SdLogger.cpp`
- Modify: `ExoCode/ExoCode.ino`

**Interfaces:**
- Produces: `class SdLogger { SdLogger(ExoData*); void update(bool ran); };`

- [ ] **Step 1: Create `ExoCode/src/SdLogger.h`**

```cpp
#ifndef SDLOGGER_H
#define SDLOGGER_H

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <Arduino.h>
#include <SD.h>
#include "ExoData.h"
#include "SideData.h"
#include "JointData.h"
#include "StatusDefs.h"

// ---- Tunables ----
#define ENABLE_SD_LOGGING     1      // 0 compiles the logger out entirely
#define SD_LOG_DECIMATION     5      // log every Nth 500Hz cycle (5 -> ~100Hz)
#define SD_LOG_FLUSH_TICK_MS  300    // one file flushed per tick; 3 files -> ~0.9s each
#define SD_LOG_BASE_PATH      "/EXOLOG"
#define SD_LOG_DEBUG          0      // 1 -> print session/timing info over Serial

class SdLogger
{
    public:
        SdLogger(ExoData* data);
        void update(bool ran);   // call every .ino loop; `ran` = Exo::run() return

    private:
        ExoData* _data;
        bool     _available;      // SD mounted & usable
        bool     _logging;        // a session file set is open
        bool     _prev_active;    // for trial-edge detection
        uint16_t _session_index;
        uint8_t  _decim;
        uint32_t _last_flush_ms;
        uint8_t  _flush_turn;

        File _f_motor_l;
        File _f_motor_r;
        File _f_gs;

        void       _begin_if_needed();
        uint16_t   _scan_next_index();
        void       _handle_session();
        void       _open_session();
        void       _close_session();
        JointData* _used_joint(SideData& side);
        void       _write_motor_row(File& f, SideData& side, const char* label);
        void       _check_ground_strike_events();
        void       _maybe_flush();
};

#endif
#endif
```

- [ ] **Step 2: Create `ExoCode/src/SdLogger.cpp` (no-op body)**

```cpp
#include "SdLogger.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

SdLogger::SdLogger(ExoData* data)
: _data(data), _available(false), _logging(false), _prev_active(false),
  _session_index(1), _decim(0), _last_flush_ms(0), _flush_turn(0)
{}

void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    (void)ran;   // filled in by later tasks
#else
    (void)ran;
#endif
}

// Remaining private methods are added in later tasks.

#endif
```

- [ ] **Step 3: Wire into `ExoCode/ExoCode.ino`** — add the include near the other logging includes (after line 42 `#include "src/PiLogger.h"`):

```cpp
#include "src/SdLogger.h"
```

Inside `loop()`, right after `static Exo exo(&exo_data);` (line ~117) add:

```cpp
    //Onboard SD logger (Teensy side)
    static SdLogger sd_logger(&exo_data);
```

And replace the run call near line 660 (`bool ran = exo.run();`) block so the logger is driven each loop:

```cpp
    //Run the exo calculations (go to exo.h/exo.cpp to follow the cascade of functions this runs)
    bool ran = exo.run();

    //Feed the onboard SD logger (no-op unless a trial is active)
    sd_logger.update(ran);
```

- [ ] **Step 4: Verify (compile + hardware)**
  - Arduino IDE → Board: Teensy 4.1 → Compile. Expected: builds with no errors.
  - Flash. Expected: exo behaves exactly as before a trial (logger is a no-op).

- [ ] **Step 5: Review checkpoint** — pause; user inspects the three-file diff. (No commit.)

---

### Task 2: SD availability + session index scan

Goal: on first update, confirm the SD mounts and compute the next free session index.

**Files:** Modify `ExoCode/src/SdLogger.cpp`

**Interfaces:**
- Consumes: `SD` global, `SD_LOG_BASE_PATH`.
- Produces: `_available`, `_session_index` set once; `_scan_next_index()`.

- [ ] **Step 1: Add `_begin_if_needed()` and `_scan_next_index()` to `SdLogger.cpp`** (above `update`):

```cpp
void SdLogger::_begin_if_needed()
{
    static bool tried = false;
    if (tried) return;
    tried = true;

    _available = SD.begin(BUILTIN_SDCARD);   // idempotent; card already used for config
    if (_available) _session_index = _scan_next_index();

#if SD_LOG_DEBUG
    Serial.print("SdLogger: SD ");
    Serial.print(_available ? "ok, next index " : "FAIL");
    if (_available) Serial.println(_session_index); else Serial.println();
#endif
}

uint16_t SdLogger::_scan_next_index()
{
    char path[32];
    for (uint16_t i = 1; i < 9999; ++i)
    {
        snprintf(path, sizeof(path), "%s/%04u", SD_LOG_BASE_PATH, i);
        if (!SD.exists(path)) return i;
    }
    return 9999;
}
```

- [ ] **Step 2: Call it from `update()`** — replace the `update` body with:

```cpp
void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    _begin_if_needed();
    if (!_available) return;
    (void)ran;   // session handling added in Task 3
#else
    (void)ran;
#endif
}
```

- [ ] **Step 3: Verify** — set `SD_LOG_DEBUG 1` temporarily, compile for Teensy 4.1, flash,
  open Serial Monitor @115200. Expected: `SdLogger: SD ok, next index 1` (or next free
  number if `/EXOLOG/000N` folders already exist). Set `SD_LOG_DEBUG` back to `0`.

- [ ] **Step 4: Review checkpoint** — user inspects diff. (No commit.)

---

### Task 3: Trial-tied session open/close + headers

Goal: a fresh folder + three header-stamped files appear on trial start and close on stop.

**Files:** Modify `ExoCode/src/SdLogger.cpp`

**Interfaces:**
- Consumes: `data->get_status()`, `data->estop`, `status_defs::messages::trial_on`.
- Produces: `_logging`, open `_f_motor_l/_f_motor_r/_f_gs`; `_handle_session()`,
  `_open_session()`, `_close_session()`.

- [ ] **Step 1: Add session methods to `SdLogger.cpp`:**

```cpp
void SdLogger::_handle_session()
{
    const bool active =
        (_data->get_status() == status_defs::messages::trial_on) && !_data->estop;

    if (active && !_prev_active)      _open_session();
    else if (!active && _prev_active) _close_session();
    _prev_active = active;
}

void SdLogger::_open_session()
{
    char dir[32];
    SD.mkdir(SD_LOG_BASE_PATH);
    snprintf(dir, sizeof(dir), "%s/%04u", SD_LOG_BASE_PATH, _session_index);
    SD.mkdir(dir);

    char path[48];
    snprintf(path, sizeof(path), "%s/Motor_L_log.txt", dir);
    _f_motor_l = SD.open(path, FILE_WRITE);
    snprintf(path, sizeof(path), "%s/Motor_R_log.txt", dir);
    _f_motor_r = SD.open(path, FILE_WRITE);
    snprintf(path, sizeof(path), "%s/Ground_strike_log.txt", dir);
    _f_gs = SD.open(path, FILE_WRITE);

    if (!_f_motor_l || !_f_motor_r || !_f_gs) { _close_session(); return; }

    const char* motor_hdr =
        "Motor,Teensy_time_s,Gait_phase,Position_rad,Velocity_rad_s,Torque_Nm,"
        "Commanded_Torque_Nm,Current_A,Filtered_Torque_Nm,Desired_Torque_Nm,"
        "Toe_FSR,Stance,Enabled,Timeout_ct,Error";

    for (File* f : { &_f_motor_l, &_f_motor_r })
    {
        f->print("# OpenExo SD log rate~"); f->print(500 / SD_LOG_DECIMATION);
        f->print("Hz t0_us="); f->println(micros());
        f->println(motor_hdr);
    }
    _f_gs.println("# OpenExo ground-strike log (toe-FSR strike onset; heel unused)");
    _f_gs.println("Leg,Teensy_time_s,Prev_step_ms,Expected_step_ms");

    _decim = 0;
    _flush_turn = 0;
    _last_flush_ms = millis();
    _logging = true;
    _session_index++;   // next trial -> fresh folder

#if SD_LOG_DEBUG
    Serial.print("SdLogger: opened "); Serial.println(dir);
#endif
}

void SdLogger::_close_session()
{
    if (_f_motor_l) { _f_motor_l.flush(); _f_motor_l.close(); }
    if (_f_motor_r) { _f_motor_r.flush(); _f_motor_r.close(); }
    if (_f_gs)      { _f_gs.flush();      _f_gs.close(); }
    _logging = false;
#if SD_LOG_DEBUG
    Serial.println("SdLogger: closed session");
#endif
}
```

- [ ] **Step 2: Call `_handle_session()` from `update()`** — replace body:

```cpp
void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    _begin_if_needed();
    if (!_available) return;
    _handle_session();
    (void)ran;   // row writing added in Task 4
#else
    (void)ran;
#endif
}
```

- [ ] **Step 3: Verify** — compile for Teensy 4.1, flash. In the GUI, start a trial then
  stop it. Power down, pull SD, inspect on a PC: `/EXOLOG/0001/` exists with
  `Motor_L_log.txt`, `Motor_R_log.txt`, `Ground_strike_log.txt`, each containing the `#` marker line
  and the header line. Start a second trial → `/EXOLOG/0002/` appears.

- [ ] **Step 4: Review checkpoint** — user inspects diff. (No commit.)

---

### Task 4: Decimated motor-row logging

Goal: `Motor_L_log.txt` / `Motor_R_log.txt` fill at ~100 Hz with the full column set.

**Files:** Modify `ExoCode/src/SdLogger.cpp`

**Interfaces:**
- Consumes: field paths from the table above.
- Produces: `_used_joint()`, `_write_motor_row()`.

- [ ] **Step 1: Add row methods to `SdLogger.cpp`:**

```cpp
JointData* SdLogger::_used_joint(SideData& side)
{
    if (side.ankle.is_used) return &side.ankle;
    if (side.hip.is_used)   return &side.hip;
    if (side.knee.is_used)  return &side.knee;
    if (side.elbow.is_used) return &side.elbow;
    return nullptr;
}

void SdLogger::_write_motor_row(File& f, SideData& side, const char* label)
{
    JointData* j = _used_joint(side);
    if (j == nullptr || !f) return;

    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "%s,%.4f,%.2f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
        label,
        micros() / 1.0e6,
        side.percent_gait,
        j->motor.p, j->motor.v,
        j->torque_reading, j->motor.last_command, j->motor.i,
        j->controller.filtered_torque_reading, j->controller.desired_torque,
        side.toe_fsr, (int)side.toe_stance,
        (int)j->motor.enabled, j->motor.timeout_count,
        _data->error_code);

    if (n > 0)
    {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        f.write((const uint8_t*)buf, (size_t)n);
    }
}
```

- [ ] **Step 2: Emit rows from `update()`** — replace body:

```cpp
void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    _begin_if_needed();
    if (!_available) return;
    _handle_session();
    if (!_logging || !ran) return;

    if (++_decim >= SD_LOG_DECIMATION)
    {
        _decim = 0;
        _write_motor_row(_f_motor_l, _data->left_side,  "L");
        _write_motor_row(_f_motor_r, _data->right_side, "R");
    }
#else
    (void)ran;
#endif
}
```

- [ ] **Step 3: Verify** — compile, flash, run a short walking trial. Pull SD: each
  `Motor_*_log.txt` should have data rows after the header, row count ≈ trial_seconds × 100,
  columns matching the header, and `Current_A` populated (the right-motor diagnostic).

- [ ] **Step 4: Review checkpoint** — user inspects diff. (No commit.)

---

### Task 5: Per-ground-strike step log

Goal: `Ground_strike_log.txt` gets one row per ground strike, with previous + expected stride ms.

**Files:** Modify `ExoCode/src/SideData.h`, `SideData.cpp`, `Side.cpp`, `SdLogger.cpp`

**Interfaces:**
- Produces: `SideData::last_step_duration` (float, ms); `SdLogger::_check_ground_strike_events()`.

- [ ] **Step 1: Add the field to `ExoCode/src/SideData.h`** — after line 50
  (`float expected_step_duration; ...`) add:

```cpp
        float last_step_duration;       /**< Duration (ms) of the most recent completed step */
```

- [ ] **Step 2: Initialize it in `ExoCode/src/SideData.cpp`** — next to the existing
  `this->expected_step_duration = -1;` line add:

```cpp
    this->last_step_duration = 0;
```

- [ ] **Step 3: Set it in `ExoCode/src/Side.cpp`** in `_update_expected_duration()` — right
  after the existing line `unsigned int step_time = _ground_strike_timestamp - _prev_ground_strike_timestamp;`
  add:

```cpp
    _side_data->last_step_duration = (float)step_time;
```

- [ ] **Step 4: Add `_check_ground_strike_events()` to `SdLogger.cpp`:**

```cpp
void SdLogger::_check_ground_strike_events()
{
    if (!_f_gs) return;
    const float t = micros() / 1.0e6;

    if (_data->left_side.ground_strike)
    {
        _f_gs.print("L,");  _f_gs.print(t, 4); _f_gs.print(",");
        _f_gs.print(_data->left_side.last_step_duration, 1); _f_gs.print(",");
        _f_gs.println(_data->left_side.expected_step_duration, 1);
    }
    if (_data->right_side.ground_strike)
    {
        _f_gs.print("R,");  _f_gs.print(t, 4); _f_gs.print(",");
        _f_gs.print(_data->right_side.last_step_duration, 1); _f_gs.print(",");
        _f_gs.println(_data->right_side.expected_step_duration, 1);
    }
}
```

- [ ] **Step 5: Call it every control cycle in `update()`** — insert right after the
  `if (!_logging || !ran) return;` line, **before** the decimation block:

```cpp
    _check_ground_strike_events();   // every control cycle so strikes aren't missed
```

- [ ] **Step 6: Verify** — compile, flash, walk a trial. `Ground_strike_log.txt` should have ~one row
  per step per leg, `Prev_step_ms` in a plausible ~800–1400 ms range for the working leg.

- [ ] **Step 7: Review checkpoint** — user inspects diff. (No commit.)

---

### Task 6: Staggered flush + write-time profiling

Goal: bound data loss to ~1 s and confirm no single loop stalls from SD syncs.

**Files:** Modify `ExoCode/src/SdLogger.cpp`

**Interfaces:**
- Produces: `_maybe_flush()`.

- [ ] **Step 1: Add `_maybe_flush()` to `SdLogger.cpp`:**

```cpp
void SdLogger::_maybe_flush()
{
    const uint32_t now = millis();
    if (now - _last_flush_ms < SD_LOG_FLUSH_TICK_MS) return;
    _last_flush_ms = now;

    switch (_flush_turn)   // one sync per tick, rotating across files
    {
        case 0: if (_f_motor_l) _f_motor_l.flush(); break;
        case 1: if (_f_motor_r) _f_motor_r.flush(); break;
        case 2: if (_f_gs)      _f_gs.flush();      break;
    }
    _flush_turn = (uint8_t)((_flush_turn + 1) % 3);
}
```

- [ ] **Step 2: Call it at the end of the active branch of `update()`** — after the
  decimation block, still inside `#if ENABLE_SD_LOGGING`:

```cpp
    _maybe_flush();
```

- [ ] **Step 3: (Optional) profile the write cost** — temporarily bracket the decimation
  block with timing and print the worst case behind `SD_LOG_DEBUG`:

```cpp
#if SD_LOG_DEBUG
    static uint32_t worst_us = 0;
    uint32_t t0 = micros();
#endif
    if (++_decim >= SD_LOG_DECIMATION) { /* ...existing write... */ }
    _maybe_flush();
#if SD_LOG_DEBUG
    uint32_t dt = micros() - t0;
    if (dt > worst_us) { worst_us = dt; Serial.print("SdLogger worst write us="); Serial.println(worst_us); }
#endif
```

- [ ] **Step 4: Verify** — compile, flash, walk a longer (>60 s) trial.
  - Data survives an abrupt power-off mid-trial up to the last ~1 s (folder + rows present).
  - With `SD_LOG_DEBUG 1`, worst write stays well under the loop period
    (2000 µs at 500 Hz; `LOOP_TIME_TOLERANCE` gives headroom). If a flush ever spikes near
    the budget, note it — that's the trigger to escalate to the reserve SdFat preallocated
    strategy (design doc §Timing). Set `SD_LOG_DEBUG` back to `0`.

- [ ] **Step 5: Review checkpoint** — user inspects the full feature diff. (No commit.)

---

## Self-review

- **Spec coverage:** session lifecycle (T3), file format/headers (T3), motor columns incl.
  current (T4), stride log (T5), decimation ~100 Hz (T4), real-time safety/flush (T6),
  auto trial trigger (T3), SD-fail self-disable (T2/T3), tunables (T1). Temperature
  explicitly deferred per spec. ✔
- **Placeholders:** none — every step has concrete code/commands.
- **Type consistency:** `update(bool)`, `_used_joint()→JointData*`, `_write_motor_row(File&,
  SideData&, const char*)`, field paths, and `error_code` (int) are consistent across tasks.
- **Note:** `Motor_L_log.txt` etc. are long filenames; Teensy SD (SdFat backend) supports
  LFN, so this is fine. If an 8.3-only volume is ever used, shorten to `MOT_L.txt`.
