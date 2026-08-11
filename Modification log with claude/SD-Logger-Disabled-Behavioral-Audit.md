# Does the SD logger change behaviour when `sdLogEnabled = 0`? — audit

**Date:** 2026-08-10
**Question:** the exo was damaged on 2026-07-23 *with SD logging already disabled*. Could the logger
still have been involved — e.g. injecting the transient sensor noise seen at End Trial?
**Answer: No.** One real finding (a one-shot boot-path SD burst), but it cannot reach a trial.
**Method:** static audit of `SdLogger.{h,cpp}`, `SdRingBuffer.h`, the `ExoCode.ino` hooks, and the
bus topology. No code changed by this audit.

---

## The one real finding: `_begin_if_needed()` runs before the enabled check

`SdLogger::update()` (`SdLogger.cpp:43-47`):

```cpp
void SdLogger::update(bool ran)
{
#if ENABLE_SD_LOGGING
    _begin_if_needed();                        // <-- BEFORE the flag is consulted
    if (!_available || !_enabled) return;
```

So with `sdLogEnabled = 0`, the first `loop()` iteration still performs, **blocking**:

| operation | cost |
|---|---|
| `SD.begin(BUILTIN_SDCARD)` — full re-mount (card already mounted by `ParseIni` at boot) | ~10–50 ms |
| `_load_config()` — open + parse `/config.ini` | ~ms |
| `_scan_next_index()` — one `SD.exists()` per existing session folder (bounded at 9999) | ~1–5 ms each |
| `SD.remove("/EXOLOG/debug_log.txt")` — because `SD_LOG_DEBUG` is still `1` | ~ms |
| Serial status prints | negligible |

**It is one-shot.** `static bool tried = false; if (tried) return; tried = true;` — it never retries,
not even if the mount fails. So this is a single stall of order 50–150 ms, and then nothing.

**When it happens matters:** `sd_logger.update(ran)` is at `ExoCode.ino:693`, while the first-run
init block (which sets `motor.is_on = true` and `motor.enabled = true`, `ExoCode.ino:~200-500`) runs
earlier in the *same* first iteration. So the motors are powered and enabled during that stall, and
under the pre-2026-08-10 `send_data()` no CAN frames went out during it — meaning the AK60v3 held
its last command. At that point the last command is whatever the first `exo.run()` produced, i.e.
~0 Nm, so it is benign. It is also **once, at boot, long before any trial**.

**Worth fixing anyway** (not done here): move `_begin_if_needed()` below the `_enabled` test, or
short-circuit on `_enabled` first. A disabled logger should touch the card zero times.

## What else exists when disabled

| thing | verdict |
|---|---|
| `DMAMEM s_sdlog_buf_{l,r,gs,dbg}` — 8192+8192+2048+2048 = **20 KB OCRAM**, statically allocated | Always present. 4 % of the Teensy 4.1's 512 KB OCRAM. Never `init()`ed when disabled (that happens in `_open_session()`). No behavioural effect. |
| `sd_logger.update(ran)` per loop iteration | function-local-static guard check + `_begin_if_needed()` early return + one boolean test. Nanoseconds. |
| `static SdLogger sd_logger(&exo_data)` constructor | trivial field init, once. |
| `SdLogger::close_active()` in `get_system_reset` | `_instance->_logging` is false ⇒ no-op. |

## What is definitively NOT running when disabled

Everything below sits **after** `if (!_available || !_enabled) return;` and is therefore
unreachable — verified by position in `update()`:

- the entire `SD_LOG_DEBUG` per-second block (the `ran/s` / `maxLoop` / `maxSD` line, its
  `Serial.print`, and its `_rb[3].push()`)
- `_handle_session()` — so no session open/close, no `_scan_next_index()` beyond the boot call
- `_check_ground_strike_events()`, `_write_motor_row()`, all ring-buffer pushes
- `_service_writes()` and `_maybe_flush()` — **all SD I/O**

## Why it cannot cause the sensor transient

1. **No shared bus.** The torque sensor is read with `analogRead()`
   (`TorqueSensor.cpp:76,136,171`) — an ADC pin. The SD card is the **built-in SDIO slot**:
   `SD_SELECT` is `#define`d to `BUILTIN_SDCARD` in `ParamsFromSD.h`, `ListCtrlParams.h` and
   `ParseIni.h`, and `SdLogger` passes `BUILTIN_SDCARD` directly. Every caller uses the same
   backend, and none of it shares pins with the analog front end.
2. **The only plausible coupling is electrical** — SD write current causing a supply droop that
   perturbs the ADC. That requires writes. When disabled there are none after the first loop
   iteration.
3. **Timing.** The one SD burst is at boot, before FSR calibration, let alone before End Trial.

**The SD logger is cleared as a cause of the 2026-07-23 event.**

## Related always-on costs our branch added (audited, all clean)

- **`heel_fsr_present()`** is now called from `Side::read_data()`, `Side::check_calibration()`,
  `_check_ground_strike()` and `_check_thresholds()` — several times per control cycle. It is
  `static int cached` behind a `< 0` test, so after the first call it is a compare-and-return.
  Negligible, but it is a new per-cycle call on the hot path.
- **`SD_LOG_DEBUG` is still `1`.** Harmless while disabled (its block is unreachable), but it should
  be `0` for production — it is the only reason the boot path does an `SD.remove()`.
- **Two independent "is the SD mounted" caches**: `ParamsFromSD::_sd_ready()`'s `static bool mounted`
  and `SdLogger::_begin_if_needed()`'s `static bool tried`. Not a bug today, but they can disagree,
  and each will happily call `SD.begin()` after the other has.

### Checked and cleared: the per-cycle `t_sat` serial print

`Motor.cpp::send_data()` contains an **unconditional** `logger::print("...t_sat:: ")` — 2 calls per
control cycle, ~1000/s. It looked like a prime loop-timing suspect. It is not:

- it is **byte-identical on `backup_branch_with_UW_edits`** (line 271 there), so it cannot explain a
  branch difference;
- `logging::level` is `LogLevel::Release` (`Config.h:64`) and `logger::print` gates on
  `level <= logging::level`, so the default-`Debug` messages are suppressed before `Serial.print`;
- the `const char*` overload builds no `String`, so there is no heap churn. (Note `logger::println`
  *does* construct `String(msg) + "\n"` as an argument **before** the level test — that one does
  allocate even when suppressed. Not used on this path.)

## The high GUI torque reading at End Trial is very probably an artifact, not a transient

The reported behaviour — plot refreshes on the End Trial click, then 1–2 huge readings appear — is
the exact signature already documented in
`End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` §"artifact", and **that part of that
document was not retracted**:

- `_on_rt_update` **always plots** (`MainWindow.py:244`) but **only writes CSV if the file is open**
  (`MainWindow.py:252`), and `_on_end_trial` closes the CSV while notifications are still arriving —
  a ~38 ms window where samples are plotted but never logged;
- `clear_plots()` has just run, so those samples land on a **freshly-cleared, autoscaling** axis and
  render full-height;
- `RtBridge.feed_bytes` keeps `_buffer`, `_payload` and `_command` as persistent state that is
  **never reset at trial end**, so a truncated frame leaves stale digits that concatenate with the
  next fragment (`'15'` + `'100'` → `15100` → 151.0).

**Implication:** the high *reading* is likely a GUI parsing artifact and should not be treated as
evidence of a real sensor transient. The physical torque lock-up is real; the number on screen
probably is not. Resetting `RtBridge`'s parser state at trial end is a small, unambiguous fix and is
still not done.
