# Non-Blocking SD Logger Implementation Plan

> **For agentic workers:** Steps use checkbox (`- [ ]`) syntax for tracking. This is Teensy 4.1
> firmware with **no unit-test harness** for the SD/control path — "verification" means: it compiles,
> the fake-trial self-test (`SD_LOG_SELFTEST_TRIAL`) produces valid logs, `USE_SPEED_CHECK` shows the
> loop holding ~500 Hz with logging ON, and the user's bench run. **The assistant does not commit** —
> the user reviews and commits. SD-card files (`config.ini`) need no reflash; `.cpp`/`.h` changes do.

**Goal:** Replace `SdLogger`'s blocking writes with a RAM ring buffer + `isBusy()`-gated single-sector
draining so SD logging never stalls the control loop, keeping the loop near 500 Hz with logging on.

**Architecture:** `update()` formats rows into per-file DMAMEM ring buffers (microseconds, no card I/O).
A drain step called every loop iteration writes at most one 512-byte sector per call, and only when
`SD.card()->isBusy()` is false, into adaptively pre-allocated contiguous files. Public interface and
the 3-file text format are unchanged.

**Tech Stack:** Teensy 4.1 C++ (Arduino), SdFat (global `SdFat SD` per `IniFile.h`), SD-card CSV/INI.

## Global Constraints

- Keep `SdLogger`'s public interface exactly: constructor, `update(bool ran)`, `close_active()`,
  `self_test()`. `ExoCode.ino:690-693` must not change.
- Keep the 3-file layout and exact column format: `Motor_L_log.txt`, `Motor_R_log.txt`,
  `Ground_strike_log.txt`, same header + `snprintf` row format as current `_write_motor_row`.
- Use the already-mounted global `SdFat SD`; do not create a second filesystem instance.
- Ring buffers live in `DMAMEM`; ~8 KB per motor file, ~2 KB ground-strike (~18 KB total).
- Adaptive pre-allocation: `bytes_per_s = (500/decimation)*72`, reserve `bytes_per_s*5400`, clamp
  `[1 MB, 128 MB]` per motor file; ground-strike fixed 1 MB. Graceful fallback to on-demand growth
  if a contiguous run isn't available or the trial overruns it.
- Overflow: drop oldest bytes, count them, emit a `# GAP <n> bytes` marker; never stall control.
- Config keys unchanged (`sdLogEnabled`, `sdLogDecimation`, `sdLogFlushMs`); `sdLogFlushMs` is now the
  `!isBusy()`-gated durability-sync cadence.
- Assistant does not run motor tests and does not commit.

## File Structure

- Create `ExoCode/src/SdRingBuffer.h` — a self-contained byte ring buffer over caller-supplied
  storage, drop-oldest on overflow. One responsibility, no SD/Arduino-SD dependency.
- Modify `ExoCode/src/SdLogger.h` — swap `File` members for the SdFat file type, add three
  `SdRingBuffer`s, drain/turn state, per-file byte-written counters, adaptive-size helper.
- Modify `ExoCode/src/SdLogger.cpp` — rewrite `_open_session`/`_close_session`/`_write_motor_row`/
  `_check_ground_strike_events`/`_maybe_flush` and add `_service_writes`; producer→buffer, gated
  sector drain, adaptive preAllocate, truncate/sync/close.

---

### Task 1: SdRingBuffer (self-contained, no SD dependency)

**Files:**
- Create: `ExoCode/src/SdRingBuffer.h`

**Interfaces:**
- Produces: `class SdRingBuffer` with `void init(uint8_t* storage, size_t cap)`, `size_t size() const`,
  `size_t space() const`, `void push(const uint8_t* data, size_t n)`, `size_t peek(const uint8_t** out) const`,
  `void consume(size_t n)`, `uint32_t dropped() const`, `void clear_dropped()`.

- [ ] **Step 1: Write the complete header**

```cpp
#ifndef SD_RING_BUFFER_H
#define SD_RING_BUFFER_H
#include <Arduino.h>
#include <string.h>

// Byte ring buffer over caller-provided storage (place storage in DMAMEM).
// Single-producer (control loop) / single-consumer (drain) in the cooperative superloop; NOT ISR-safe.
// On overflow, drops oldest bytes to make room and counts them so the logger can emit a gap marker.
class SdRingBuffer
{
public:
    void init(uint8_t* storage, size_t capacity)
    { _buf = storage; _cap = capacity; _head = 0; _tail = 0; _count = 0; _dropped = 0; }

    size_t   capacity() const { return _cap; }
    size_t   size() const     { return _count; }
    size_t   space() const    { return _cap - _count; }
    uint32_t dropped() const  { return _dropped; }
    void     clear_dropped()  { _dropped = 0; }

    // Append n bytes; if short on room, drop oldest bytes first (counted).
    void push(const uint8_t* data, size_t n)
    {
        if (_cap == 0 || n == 0) return;
        if (n >= _cap) {                       // keep only the last _cap bytes
            _dropped += (uint32_t)(_count + (n - _cap));
            _head = _tail = _count = 0;
            data += (n - _cap); n = _cap;
        } else if (n > space()) {
            _drop(n - space());
        }
        size_t first = _cap - _head; if (first > n) first = n;
        memcpy(_buf + _head, data, first);
        if (n > first) memcpy(_buf, data + first, n - first);
        _head = (_head + n) % _cap;
        _count += n;
    }

    // Point to up to the largest contiguous readable run at the tail (no wrap). Returns its length.
    size_t peek(const uint8_t** out) const
    {
        if (_count == 0) { *out = _buf; return 0; }
        *out = _buf + _tail;
        size_t contig = _cap - _tail;
        return (contig < _count) ? contig : _count;
    }

    void consume(size_t n)
    {
        if (n > _count) n = _count;
        _tail = (_tail + n) % _cap;
        _count -= n;
    }

private:
    void _drop(size_t n)
    {
        if (n > _count) n = _count;
        _tail = (_tail + n) % _cap;
        _count -= n;
        _dropped += (uint32_t)n;
    }
    uint8_t* _buf = nullptr;
    size_t   _cap = 0, _head = 0, _tail = 0, _count = 0;
    uint32_t _dropped = 0;
};
#endif
```

- [ ] **Step 2: Sanity-check the logic (host or Teensy)**

Reason through: push 600 bytes into a 512-cap buffer → 88 dropped, size 512. `peek` after a wrapped
write returns only the contiguous tail run; a second `peek` after `consume` returns the rest. Optional:
drop a `main()` with asserts into a scratch `.cpp` and compile on the host to confirm, since this class
has no Arduino dependency beyond `memcpy`.

- [ ] **Step 3: Commit (USER)** — assistant does not commit.

---

### Task 2: Confirm the SdFat API surface (spike, no behavior change)

**Files:**
- Modify (temporary probe): `ExoCode/src/SdLogger.cpp` `self_test()` or a scratch sketch.

**Why:** `IniFile.h` binds `SD` to `SdFat` under `PREFER_SDFAT_LIBRARY`. The exact file type returned by
`SD.open(...)` and the availability of `preAllocate`/`truncate`/`isBusy` depend on the installed
Teensyduino SdFat. Confirm before the rewrite so Task 3-5 use the right spellings.

- [ ] **Step 1: Add a probe to `self_test()`**

```cpp
// TEMP probe (remove after confirming). Compile with SD_LOG_SELFTEST=1.
auto f = SD.open("/EXOLOG/probe.bin", O_WRONLY | O_CREAT | O_TRUNC);
Serial.print("open ok="); Serial.println((bool)f);
bool pa = f.preAllocate(1UL << 20);   Serial.print("preAllocate ok="); Serial.println(pa);
size_t w = f.write((const uint8_t*)"hello\n", 6); Serial.print("wrote="); Serial.println(w);
bool tr = f.truncate(6);              Serial.print("truncate ok="); Serial.println(tr);
f.sync(); f.close();
Serial.print("card isBusy()="); Serial.println(SD.card()->isBusy());
```

- [ ] **Step 2: Build + run, read Serial**

Expected: all four ops report ok, `isBusy()` returns a bool. Record the concrete type of `f`
(`File32`/`FsFile`/`SdFile`) from a compile error if you force `auto`→explicit; note it for Task 3.

- [ ] **Step 3: Decide the file typedef**

In `SdLogger.h`, add `using LogFile = decltype(SD.open("", 0));` (or the explicit type found above) so
the rest of the code is type-correct regardless of the SdFat variant. If `preAllocate`/`truncate` are
NOT available, fall back: skip preAllocate (accept on-demand growth; ring buffer absorbs the stalls)
and replace `truncate` with close-as-is — note this in the header comment.

- [ ] **Step 4: Remove the probe. Commit (USER).**

---

### Task 3: Session lifecycle on SdFat files (open / adaptive preAllocate / close / truncate)

**Files:**
- Modify: `ExoCode/src/SdLogger.h`
- Modify: `ExoCode/src/SdLogger.cpp` (`_open_session`, `_close_session`, add `_prealloc_bytes`)

**Interfaces:**
- Consumes: `SdRingBuffer` (Task 1), `LogFile` typedef (Task 2).
- Produces: three open `LogFile`s with reserved space, three initialized `SdRingBuffer`s with headers
  queued, `_bytes_written[3]` counters, `_prealloc_bytes(bool motor)`.

- [ ] **Step 1: Header — members**

In `SdLogger.h`, replace the three `File` members and add ring state:

```cpp
#include "SdRingBuffer.h"
// ... inside class private:
using LogFile = decltype(SD.open("", 0));   // SdFat file type (see Task 2)
LogFile      _file[3];                       // 0=L, 1=R, 2=GS
SdRingBuffer _rb[3];
uint64_t     _bytes_written[3] = {0,0,0};
uint8_t      _drain_turn = 0;
uint64_t     _prealloc_bytes(bool motor) const;
void         _service_writes();
```

Add file-scope DMAMEM storage in `SdLogger.cpp`:

```cpp
DMAMEM static uint8_t s_buf_l[8192];
DMAMEM static uint8_t s_buf_r[8192];
DMAMEM static uint8_t s_buf_gs[2048];
```

- [ ] **Step 2: Adaptive pre-alloc size**

```cpp
uint64_t SdLogger::_prealloc_bytes(bool motor) const
{
    if (!motor) return (uint64_t)1 << 20;             // GS: fixed 1 MB
    const uint32_t ROW_BYTES_EST = 72;
    uint32_t rows_per_s = 500u / (_decimation ? _decimation : 1);
    uint64_t want = (uint64_t)rows_per_s * ROW_BYTES_EST * 5400ull;  // ~90 min
    const uint64_t MINB = (uint64_t)1 << 20, MAXB = (uint64_t)128 << 20;
    if (want < MINB) want = MINB;
    if (want > MAXB) want = MAXB;
    return want;
}
```

- [ ] **Step 3: `_open_session` — create, preAllocate, queue headers**

```cpp
void SdLogger::_open_session()
{
    char dir[32];
    SD.mkdir(SD_LOG_BASE_PATH);
    snprintf(dir, sizeof(dir), "%s/%04u", SD_LOG_BASE_PATH, _session_index);
    SD.mkdir(dir);

    const char* names[3] = { "Motor_L_log.txt", "Motor_R_log.txt", "Ground_strike_log.txt" };
    uint8_t* stores[3]   = { s_buf_l, s_buf_r, s_buf_gs };
    size_t   caps[3]     = { sizeof(s_buf_l), sizeof(s_buf_r), sizeof(s_buf_gs) };

    char path[64];
    for (int i = 0; i < 3; ++i)
    {
        snprintf(path, sizeof(path), "%s/%s", dir, names[i]);
        _file[i] = SD.open(path, O_WRONLY | O_CREAT | O_TRUNC);
        if (!_file[i]) { _close_session(); return; }
        _file[i].preAllocate(_prealloc_bytes(i < 2));   // ok if it returns false (fallback)
        _rb[i].init(stores[i], caps[i]);
        _bytes_written[i] = 0;
    }

    const char* motor_hdr =
        "Motor,Teensy_time_s,Status,Gait_phase,Position_rad,Velocity_rad_s,Torque_Nm,"
        "Commanded_Torque_Nm,Current_A,Filtered_Torque_Nm,Desired_Torque_Nm,"
        "Toe_FSR,Stance,Enabled,Timeout_ct,Error\n";
    char hbuf[192];
    for (int i = 0; i < 2; ++i) {
        int n = snprintf(hbuf, sizeof(hbuf), "# OpenExo SD log rate~%dHz t0_us=%lu\n",
                         500 / _decimation, (unsigned long)micros());
        _rb[i].push((const uint8_t*)hbuf, (size_t)n);
        _rb[i].push((const uint8_t*)motor_hdr, strlen(motor_hdr));
    }
    const char* gs_hdr =
        "# OpenExo ground-strike log (toe-FSR strike onset; heel unused)\n"
        "Leg,Teensy_time_s,Prev_step_ms,Expected_step_ms\n";
    _rb[2].push((const uint8_t*)gs_hdr, strlen(gs_hdr));

    _decim = 0; _drain_turn = 0; _last_flush_ms = millis();
    _logging = true; _session_index++;
}
```

- [ ] **Step 4: `_close_session` — flush remainder, truncate, sync, close**

```cpp
void SdLogger::_close_session()
{
    for (int i = 0; i < 3; ++i)
    {
        if (!_file[i]) continue;
        const uint8_t* p; size_t n;
        while ((n = _rb[i].peek(&p)) > 0) {           // blocking OK: trial is ending
            size_t w = _file[i].write(p, n);
            _bytes_written[i] += w; _rb[i].consume(w);
            if (w < n) break;                          // write error; stop
        }
        _file[i].truncate(_bytes_written[i]);
        _file[i].sync();
        _file[i].close();
    }
    _logging = false;
}
```

- [ ] **Step 5: Build; run fake-trial (`SD_LOG_SELFTEST_TRIAL=1`); confirm files open, headers land,
      files truncate to a small size. Commit (USER).**

---

### Task 4: Producer + gated sector drain + gap marker

**Files:**
- Modify: `ExoCode/src/SdLogger.cpp` (`update`, `_write_motor_row`, `_check_ground_strike_events`,
  add `_service_writes`)

**Interfaces:**
- Consumes: `_rb[]`, `_file[]`, `_bytes_written[]`, `_drain_turn` (Task 3).

- [ ] **Step 1: Producer — `_write_motor_row` pushes to a ring buffer instead of the file**

Change the signature to take the buffer index; keep the exact format string:

```cpp
void SdLogger::_write_motor_row(int idx, SideData& side, const char* label)
{
    JointData* j = _used_joint(side);
    if (j == nullptr) return;
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "%s,%.4f,%u,%.2f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
        label, micros() / 1.0e6, (unsigned)_data->get_status(), side.percent_gait,
        j->motor.p, j->motor.v, j->torque_reading, j->motor.last_command, j->motor.i,
        j->controller.filtered_torque_reading, j->controller.desired_torque,
        side.toe_fsr, (int)side.toe_stance, (int)j->motor.enabled, j->motor.timeout_count,
        _data->error_code);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    _rb[idx].push((const uint8_t*)buf, (size_t)n);
}
```

Update the call sites in `update()` to `_write_motor_row(0, _data->left_side, "L");` and
`_write_motor_row(1, _data->right_side, "R");`. Update `_check_ground_strike_events()` to build each
strike line into a `char` buffer and `_rb[2].push(...)` instead of `_f_gs.print(...)`.

- [ ] **Step 2: Drain — one gated sector per call**

```cpp
void SdLogger::_service_writes()
{
    if (!_logging || !_available) return;
    if (SD.card()->isBusy()) return;                 // never wait on the card
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        _drain_turn = (uint8_t)((_drain_turn + 1) % 3);
        int i = _drain_turn;
        if (!_file[i]) continue;

        if (_rb[i].dropped() > 0) {                   // record the gap, then clear
            char g[40]; int gn = snprintf(g, sizeof(g), "# GAP %lu bytes\n",
                                          (unsigned long)_rb[i].dropped());
            _bytes_written[i] += _file[i].write((const uint8_t*)g, gn);
            _rb[i].clear_dropped();
        }
        if (_rb[i].size() >= 512) {
            const uint8_t* p; size_t avail = _rb[i].peek(&p);
            size_t w = (avail >= 512) ? 512 : avail;  // contiguous run; rest drains next turn
            size_t wrote = _file[i].write(p, w);
            _rb[i].consume(wrote);
            _bytes_written[i] += wrote;
            return;                                    // one write per call
        }
    }
}
```

- [ ] **Step 3: Wire `update()`**

Keep `_handle_session()`; on `_logging && ran`: `_check_ground_strike_events()`, decimation +
`_write_motor_row(0,...)/(1,...)`. Then call `_service_writes()` **every** `update()` call (not gated on
`ran`) so draining runs on every fast loop iteration. Remove `_maybe_flush()`'s old body (replaced in
Task 5). Keep the `SD_LOG_DEBUG` block.

- [ ] **Step 4: Build; fake-trial run; read back all 3 files; verify row counts ≈ expected, headers +
      column format intact, no truncated lines except possibly at a `# GAP`. Commit (USER).**

---

### Task 5: Durability sync, gated and off the hot path

**Files:**
- Modify: `ExoCode/src/SdLogger.cpp` (`_maybe_flush` → gated periodic sync)

- [ ] **Step 1: Replace `_maybe_flush` body**

```cpp
void SdLogger::_maybe_flush()
{
    const uint32_t now = millis();
    if (now - _last_flush_ms < _flush_tick_ms) return;
    if (SD.card()->isBusy()) return;                 // defer; don't stall
    _last_flush_ms = now;
    if (_file[_flush_turn]) _file[_flush_turn].sync();
    _flush_turn = (uint8_t)((_flush_turn + 1) % 3);
}
```

Call `_maybe_flush()` from `update()` after `_service_writes()`.

- [ ] **Step 2: Build; fake-trial run; power-cut mid-run (USER) and confirm the file is readable up to
      ~the last sync. Commit (USER).**

---

### Task 6: Integration verification (USER-run)

- [ ] **Step 1: Loop-rate proof.** Enable `USE_SPEED_CHECK` in `Exo.cpp`, run a trial with
      `sdLogEnabled=1`, and confirm the printed loop period stays near 2 ms / ~500 Hz (vs the
      ~5.7 ms / ~170 Hz measured before). This is the core success criterion.
- [ ] **Step 2: Log integrity.** Confirm `Motor_L/R` and `Ground_strike` parse in the existing Python
      analysis scripts unchanged; check for any `# GAP` markers (should be none at normal rates).
- [ ] **Step 3: Transparency re-test.** Re-run the ankle zero-torque test with logging ON and confirm
      the chatter now matches the logging-OFF case — i.e. logging no longer degrades control.
- [ ] **Step 4: Lock in.** Restore `USE_SPEED_CHECK` and any debug flags to normal; commit (USER).

---

## Self-Review

**Spec coverage:**
- Ring buffer in DMAMEM → Task 1 + Task 3 Step 1. ✓
- `isBusy()`-gated single-sector drain → Task 4 Step 2. ✓
- Adaptive contiguous preAllocate from decimation → Task 3 Steps 2-3. ✓
- truncate/sync/close → Task 3 Step 4. ✓
- Drop-oldest overflow + `# GAP` marker → Task 1 (drop) + Task 4 Step 2 (marker). ✓
- Public interface unchanged / `ExoCode.ino` untouched → Global Constraints; no task modifies the .ino. ✓
- 3-file format preserved → Task 3 Step 3 header, Task 4 Step 1 row format. ✓
- Uses global `SdFat SD`, no second mount → Task 2/3. ✓
- Config keys, `sdLogFlushMs` repurposed → Task 5. ✓
- Periodic gated durability sync → Task 5. ✓
- Verification (fake-trial, USE_SPEED_CHECK, bench) → Task 6. ✓
- SdFat API-version risk + fallback → Task 2 Step 3. ✓

**Placeholder scan:** Ring buffer and all method bodies are complete code. The only deferred specifics
(exact SdFat file type / preAllocate availability) are resolved by the Task 2 spike before they're
used — not left vague in later tasks. ✓

**Type consistency:** `LogFile`, `_rb[]`, `_file[]`, `_bytes_written[]`, `_drain_turn`,
`_write_motor_row(int idx, ...)`, `_service_writes()`, `_prealloc_bytes(bool)` are used consistently
across Tasks 3-5. ✓
