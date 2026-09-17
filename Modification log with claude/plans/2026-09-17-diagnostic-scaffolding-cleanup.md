# Firmware diagnostic-scaffolding cleanup - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use `superpowers:executing-plans` (or
> `superpowers:subagent-driven-development`) to work through this task by task. Steps use checkbox
> (`- [ ]`) syntax.

**Goal:** Delete the firmware diagnostics that never produced information, and put the ones that did their
job behind a single build switch, so a production build is quiet and a debugging build still has everything.

**Architecture:** One master switch `EXO_DIAG` in `ExoCode/src/Config.h`, default `0`, mirroring the GUI's
`--verbose-log`. Gated code stays compiled-out, not deleted. Deletions are limited to scaffolding that was
proven not to work (stage breadcrumbs) or whose experiment is over (two bisect switches).

**Tech stack:** Arduino C++ for `arduino:mbed_nano:nano33ble` (Nano) and `teensy:avr:teensy41` (Teensy),
built with the IDE-bundled `arduino-cli`. No firmware unit-test harness exists; verification is compile +
grep + one bench trial per task.

**Spec / rationale:** `Modification log with claude/Diagnostic-Scaffolding-Cleanup.md` (verdict table and
risks) and `Nano-Hang-Watchdog-And-Breadcrumbs.md` §0.6b (original audit), §7.0 (upload quirk), §11.6
(verification commands).

## Global constraints

- **This operator never has the assistant commit.** Every task ends by handing the diff over: no
  `git commit`, no `git push`. The operator reviews, flashes and commits.
- **No motor-moving tests.** Every bench trial in this plan is zero torque: GUI connected, trial started,
  `stress_test_ble.py` writing `TorqScale = 0`, motors never enabled.
- **Verify before handover:** compile BOTH boards for every firmware task, even when only the Nano changes.

```
ACLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
CFG="$HOME/.arduinoIDE/arduino-cli.yaml"
"$ACLI" --config-file "$CFG" compile --fqbn arduino:mbed_nano:nano33ble ExoCode/ExoCode.ino
"$ACLI" --config-file "$CFG" compile --fqbn teensy:avr:teensy41        ExoCode/ExoCode.ino
```

- **Flashing habit:** power-cycle the Nano before uploading. The watchdog survives a warm reset and can kill
  an upload mid-flash (§7.0).
- **Keep always, do not touch:** `setConnectionInterval(20, 24)` and `EXO_BLE_INTERVAL_SEL`; the watchdog and
  boot-loop guard (`exo_wdt_feed`, `EXO_WDT_MAGIC`, `exo_wdt_boot_count`); the BLE stall detector
  (`EXO_BLE_STALL_MS`, `exo_ble_stall_reset`, `s_last_rx_ms`, `s_ping_seen`) with the GUI's 2 s ping and
  TX-busy counter; all three ArduinoBLE patches; `EXO_FW_TAG`; the reset-reason banner itself;
  `ConnParamsMonitor`; the CSV exo-time unwrap.
- **Order matters.** Task 3 removes a function that Task 2 depends on; do the tasks in order.
- **Stop condition:** if a task's bench trial looks wrong, revert that task's files
  (`git checkout -- <files>`) rather than debugging forward.

## Baseline before Task 1

- [ ] Flash the Nano from the current tree and run one bench trial, so later comparisons have a baseline.
- [ ] Record the banner line from the GUI log (`grep "Nano last reset reason" <app_crash log>`). It looks
      like: `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t108d_w0d_x0,CPi24_l0_t960_u0_n1,UPi…`
- [ ] Note the `no_ack` share from the stress-test summary (16.8% and 15.0% in the 2026-09-16 runs).

---

### Task 1: Add `EXO_DIAG` and gate the boot-time link-stats fetch

**Why first:** it is the only gated item with a real runtime cost - a blocking UART request during `setup()`.

**Not gated here, on purpose:** the Teensy's `rt_i2c_stats` counters themselves stay compiled in. They are a
few increments per RT frame, they cost nothing, and `EXO_DIAG` only removes the boot-time fetch and the
banner field that report them. The crash-trap self-test already has its own switch
(`EXO_CRASH_TRAP_SELFTEST`, currently `0`) and needs no change.

**Files:**
- Modify: `ExoCode/src/Config.h` (next to `RT_BLE_FORWARD`, ~line 56)
- Modify: `ExoCode/ExoCode.ino:849-850`
- Modify: `ExoCode/src/ExoBLE.cpp:305` (banner assembly)

- [ ] **Step 1: Add the switch** in `Config.h`, after the `RT_BLE_FORWARD` block:

```cpp
    //Master switch for DIAGNOSTIC instrumentation that has already served its purpose but is worth
    //keeping for the next investigation: the BLE send-path timing, the Teensy RT-I2C counters and their
    //boot-time UART fetch, and the param-update ack counters. 0 = production (quiet, no runtime cost),
    //1 = debugging build. Fixes and safety (watchdog, stall detector, connection interval) are NOT gated.
    //Mirrors the GUI's --verbose-log. See "Modification log with claude/Diagnostic-Scaffolding-Cleanup.md".
    #define EXO_DIAG 0
```

- [ ] **Step 2: Gate the boot-time fetch.** `ExoCode.ino`, around line 849:

```cpp
    #if EXO_DIAG
    exo_wdt_stage(EXO_STAGE_LINK_STATS);
    UART_command_utils::get_link_stats(handler, 1500.0f);
    #endif
```

(The `exo_wdt_stage` line disappears in Task 3; leaving it inside the `#if` now keeps this task small.)

- [ ] **Step 3: Gate the banner field.** `ExoBLE.cpp`, in `setup()`, where the boot banner is built:

```cpp
        #if EXO_DIAG
            s_boot_banner = exo_reset_reason_string() + exo_link_stats_string();
        #else
            s_boot_banner = exo_reset_reason_string();
        #endif
```

- [ ] **Step 4: Compile both boards** (commands in Global constraints). Expected: both succeed.
- [ ] **Step 5: Check the switch actually reaches the code:**

```bash
grep -rn "EXO_DIAG" ExoCode/ | grep -v "PARAM_ACK_DIAG"
```

Expected: the definition in `Config.h`, plus the two `#if` sites.

- [ ] **Step 6: Bench trial.** Flash the Nano, connect, start a trial, run `stress_test_ble.py` for ~1 min.
  - The banner keeps `RST:…`, `B23…`, `CPi…`, `UPi…` and **loses** the `I2Cf…` field.
  - Boot to advertising is up to ~1.5 s quicker.
  - The `no_ack` share is unchanged from the baseline (this task changes no BLE behaviour).
- [ ] **Step 7: Hand over** the diff for review; the operator commits.

---

### Task 2: Gate the BLE send-path diagnostics

**Files:**
- Modify: `ExoCode/src/ExoBLE.cpp:31-75` (statics, buckets, `exo_ble_link_diag`), `:593-599` (timing
  around `writeValue`), `:681-683` (reset in `on_rx_recieved`)

**Interfaces:**
- `exo_ble_link_diag()` keeps its signature and stays callable from `ExoBLE.cpp:462`
  (`exo_ble_stall_reset(exo_ble_link_diag())`). With `EXO_DIAG 0` it returns `EXO_DIAG_MARKER` only, so the
  banner still prints a `BLESTALL_n…` line, with `w0_s0` and no `_FAIL`. `exo_ble_stall_reset()` in
  `SystemReset.h` is NOT changed - it keeps its `diag` parameter.

- [ ] **Step 1: Gate the statics and the buckets.** Wrap `s_max_write_us`, `s_sends_since_rx`,
      `s_write_failed`, `exo_us_bucket()` and `exo_count_bucket()` in `#if EXO_DIAG` … `#endif`.
- [ ] **Step 2: Make the accessor unconditional** so call sites need no `#if`:

```cpp
uint8_t exo_ble_link_diag()
{
#if EXO_DIAG
    return (uint8_t)((exo_us_bucket(s_max_write_us) & 0x0Fu)
                     | (s_write_failed ? 0x10u : 0x00u)
                     | ((exo_count_bucket(s_sends_since_rx) & 0x03u) << 5)
                     | EXO_DIAG_MARKER);
#else
    return EXO_DIAG_MARKER;   //bit 7 only: "recorded, but this build measured nothing"
#endif
}
```

- [ ] **Step 3: Gate the measurement** in `ExoBLE::send_message()`, keeping the write itself unconditional:

```cpp
#if EXO_DIAG
    const uint32_t t0 = micros();
#endif
    const bool write_ok = _gatt_db.TXChar.writeValue(buffer, bytes_to_send);
#if EXO_DIAG
    const uint32_t dt = micros() - t0;
    if (dt > s_max_write_us) { s_max_write_us = dt; }
    if (!write_ok)           { s_write_failed = true; }
    s_sends_since_rx++;
#else
    (void)write_ok;
#endif
```

- [ ] **Step 4: Gate the reset block** in `ble_rx::on_rx_recieved()` (the three `s_*` assignments). Leave
      `s_last_rx_ms = millis();` and the `s_ping_seen` line OUTSIDE the `#if`: the stall detector needs them.
- [ ] **Step 5: Compile both boards.** Expected: both succeed, no `-Wunused` warnings naming `ExoBLE.cpp`.
- [ ] **Step 6: Prove the detector still works** (it is safety, and this task is the one that could break it):

```bash
grep -n "s_last_rx_ms\|s_ping_seen\|exo_ble_stall_reset" ExoCode/src/ExoBLE.cpp
```

Expected: the stamp in `on_rx_recieved`, the `s_ping_seen` set, and the stall check in `handle_updates`,
none of them inside an `#if EXO_DIAG`.

- [ ] **Step 7: Bench trial** as in Task 1, plus: leave the GUI connected and idle for ~30 s with the trial
      running, confirming the link stays up and the plot keeps moving.
- [ ] **Step 8: Hand over** the diff.

---

### Task 3: Delete the stage breadcrumbs

**Why they go:** no store ever survived a watchdog reset (B14), so every `STAGEn`/`g` reading was
meaningless. This is dead weight, not a disabled instrument.

**Files:**
- Modify: `ExoCode/ExoCode.ino` - remove `(void)exo_wdt_stage_record();` (line ~801) and all 15
  `exo_wdt_stage(EXO_STAGE_*)` calls (lines ~802-932, including the one added inside the `#if EXO_DIAG`
  in Task 1)
- Modify: `ExoCode/src/ExoBLE.cpp:226` - the `EXO_STAGE_BLE_BEGIN` stamp
- Modify: `ExoCode/src/SystemReset.h` - `EXO_STAGE_*` defines (~lines 280-305), `exo_wdt_stage()`,
  `exo_wdt_stage_record()`, the `exo_noinit_*` declarations (~312-318), and the `,STAGEn%u_g%u_dog%u`
  branch in `exo_reset_reason_string()` (~line 909)
- Modify: `ExoCode/src/SystemReset.cpp` - the `.noinit` block: `exo_noinit_stage`, `exo_noinit_boots`,
  `exo_noinit_boot_count()`, `exo_noinit_stage_set()`, `exo_noinit_stage_get()` (~lines 31-96)

**The trap:** `exo_wdt_stage_record()` does two jobs. The breadcrumb half is dead, but it also latches the
byte `exo_ble_stall_reset()` parks in `GPREGRET2`, which the `BLESTALL` banner line decodes. Keep that half,
under an honest name.

- [ ] **Step 1: Replace `exo_wdt_stage_record()`** in `SystemReset.h` (~line 346) with the latch alone:

```cpp
/**
 * @brief Latch the byte the PREVIOUS boot parked in GPREGRET2, once, before anything clears it.
 *
 * The only writer is exo_ble_stall_reset(), which parks exo_ble_link_diag() there. MUST run before
 * exo_crash_record(), which is the only thing that clears GPREGRET2; exo_crash_record() calls this itself
 * as its first action, so ordering holds no matter which caller gets there first.
 */
inline uint8_t exo_diag_byte_record()
{
#if defined(EXO_HAVE_WDT)
    static bool captured = false;
    static uint8_t latched = 0;
    if (!captured)
    {
        captured = true;
        latched = (uint8_t)NRF_POWER->GPREGRET2;
    }
    return latched;
#else
    return 0;
#endif
}
```

- [ ] **Step 2: Rename every call site.** `grep -rn "exo_wdt_stage_record" ExoCode/` lists them: the latch
      call at the top of `setup()` (`ExoCode.ino:801`, which **must stay** - it is what makes the byte
      readable at all), the `BLESTALL` branch, the `STAGEn` branch (deleted in Step 4), and the call inside
      `exo_crash_record()`. All become `exo_diag_byte_record()`.
- [ ] **Step 3: Replace the `STAGEn` branch** in `exo_reset_reason_string()` (~line 905). A watchdog reset
      should still report how many consecutive watchdog boots there have been, which is the boot-loop guard:

```cpp
    if (reasons & 0x00000002ul)
    {
        char dog[24];
        snprintf(dog, sizeof(dog), ",dog%u", (unsigned)exo_wdt_boot_count());
        return String(head) + names + String(dog) + exo_fw_tag_string();
    }
```

- [ ] **Step 4:** Delete the remaining sites listed above, file by file.
- [ ] **Step 5: Confirm nothing is left:**

```bash
grep -rn "exo_wdt_stage\|EXO_STAGE_\|exo_noinit\|STAGEn" ExoCode/
```

Expected: no hits. `exo_wdt_feed`, `EXO_WDT_MAGIC`, `exo_wdt_boot_count` and `exo_diag_byte_record` MUST
still be present.

- [ ] **Step 6: Compile both boards.**
- [ ] **Step 7: Bench trial**, and this one matters more than the others:
  - The banner still reports `RST:…` with a reason name, `B23…`, `CPi…`, `UPi…`, and no `STAGEn` field.
  - Power-cycle the Nano once and confirm it reconnects and streams.
  - Watchdog still armed: the GUI shows a normal reset reason, not a boot loop.
- [ ] **Step 8: Hand over** the diff.

---

### Task 4: Delete the two finished bisect switches

**Files:**
- Modify: `ExoCode/src/Config.h` - the `RT_BLE_FORWARD` define (~line 56) and its comment block (~line 31)
- Modify: `ExoCode/src/ComsMCU.cpp:244-249` - the `#if RT_BLE_FORWARD` guard around
  `_exo_ble->send_message(rt_data_msg);`, which becomes unconditional
- Modify: `ExoCode/src/SystemReset.h:808-814` - drop the `fw<n>` field from `exo_fw_tag_string()`; keep
  `rt<n>` (it reports `REAL_TIME_I2C`, which stays)
- Modify: `ExoCode/src/SdLogger.h:85` and `ExoCode/ExoCode.ino:155-…` - the `SD_LOG_SELFTEST_TRIAL` define
  and the `#if SD_LOG_SELFTEST_TRIAL` block in `setup()`

- [ ] **Step 1:** Confirm the current values are the safe ones before deleting: `REAL_TIME_I2C 1`,
      `RT_BLE_FORWARD 1`, `SD_LOG_SELFTEST_TRIAL 0`.

```bash
grep -rn "define REAL_TIME_I2C\|define RT_BLE_FORWARD\|define SD_LOG_SELFTEST_TRIAL" ExoCode/
```

- [ ] **Step 2:** Make the RT forward unconditional in `ComsMCU.cpp`, keeping the surrounding comment about
      payload length.
- [ ] **Step 3:** Delete both defines, the `#if` blocks, and the `fw` banner field.
- [ ] **Step 4:** Confirm nothing is left:

```bash
grep -rn "RT_BLE_FORWARD\|SD_LOG_SELFTEST_TRIAL" ExoCode/
```

Expected: no hits. `REAL_TIME_I2C` still present and `1`.

- [ ] **Step 5: Compile both boards.**
- [ ] **Step 6: Bench trial.** The real-time plot must still move (this task touched the RT send path), and
      the banner keeps `rt1` and loses `fw1`.
- [ ] **Step 7: Hand over** the diff.

---

### Task 5: Fold the ACK counters into `EXO_DIAG`

**Precondition:** the worn session that the counters were added for has been analysed. Until then, leave
`PARAM_ACK_DIAG 1` and skip this task.

**Files:**
- Modify: `ExoCode/src/ComsMCU.cpp:18-27` (the `PARAM_ACK_DIAG` define and its comment)

- [ ] **Step 1:** Replace the standalone define with the master switch:

```cpp
//Param-update ack counters, see "Modification log with claude/ACK-Loss-Investigation.md" §3.4. Gated by
//EXO_DIAG: a debugging build sends 8 fields (the three counters), production sends the usual 5.
#define PARAM_ACK_DIAG EXO_DIAG
```

- [ ] **Step 2: Compile both boards.**
- [ ] **Step 3: Bench trial** with `EXO_DIAG 0`: every ack in the GUI log reads `event_info='Sa5'` again,
      and the stress test still confirms every write.
- [ ] **Step 4:** Flip `EXO_DIAG` to `1`, compile (do not flash), and confirm `Sa8` would return by reading
      the diff; flip back to `0`.
- [ ] **Step 5: Hand over** the diff.

---

## After all tasks

- [ ] `grep -rn "exo_wdt_stage\|EXO_STAGE_\|exo_noinit\|RT_BLE_FORWARD\|SD_LOG_SELFTEST_TRIAL" ExoCode/`
      returns nothing.
- [ ] `EXO_DIAG 0` is the committed default.
- [ ] The GUI suite still passes: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests`
      (56 tests as of 2026-09-17).
- [ ] One longer bench trial (~5 min, trial on, zero torque) with no disconnect, and a `no_ack` share in
      line with the 2026-09-16 baseline.
- [ ] Update `Diagnostic-Scaffolding-Cleanup.md` §Part 2 to say what was done, and
      `Nano-Hang-Watchdog-And-Breadcrumbs.md` §0.6b to mark the discard list as executed.
