# Nano disconnect — EVIDENCE ONLY

**Purpose:** every observation, with no interpretation, no theory, and no conclusions. This file
exists because the narrative document accumulated several confident explanations that later turned
out to be wrong, and it became hard to tell which statements were measured and which were inferred.

**Rule for this file:** if it was not directly observed, measured, or read out of a file, it does not
belong here. Theories live in `Nano-Hang-Watchdog-And-Breadcrumbs.md`.

**Last updated:** 2026-09-11.

---

## 1. The symptom, as reported by the GUI

Every failure, without exception, logs:

```
WARNING | _mark_disconnected | Device disconnected. Reason: link lost, Intentional: False
```

Measured once precisely from the trial CSV: **last received data sample → disconnect log = 9.62 s**
(last sample 14:41:56.490, disconnect 14:42:06.114 on 2026-09-10).

---

## 2. Run durations

Trial start (`Begin trial sequence completed`) to disconnect, from the device-manager logs.

| # | date | start | disconnect | duration | build |
|---|---|---|---|---|---|
| 1 | 09-10 | 14:38:07 | 14:42:06 | **239 s** | pre-watchdog |
| 2 | 09-11 | 13:10:13 | 13:11:03 | **50 s** | watchdog |
| 3 | 09-11 | 13:51:51 | 13:54:23 | **152 s** | watchdog |
| 4 | 09-11 | 13:54:56 (connect, **no trial started**) | 13:55:08 | **12 s** | watchdog |
| 5 | 09-11 | 14:15:50 | 14:20:08 | **258 s** | watchdog |
| 6 | 09-11 | 14:46:37 | 14:48:29 | **112 s** | B9 |
| 7 | 09-11 | 15:23:55 | 15:37:04 | **789 s** | B10 |
| 8 | 09-11 | 15:57:27 | 16:00:38 | **191 s** | B13 |
| 9 | 09-11 | 16:39:51 | 16:50:17 | **626 s** | B15 |
| 10 | 09-11 | — | **no failure** | **~40 min, stopped by user** | B16 `rt0` |
| 11 | 09-11 | — | **no failure** | **~30+ min, stopped by user** | B17 `rt1_fw0` |
| 12 | 09-11 | 19:11:57 | 19:12:38 | **41 s** | B18 (bounded spin) |
| 13 | 09-11 | 19:18:12 | 19:18:36 | **24 s** | B18 |
| 14 | 09-11 | 19:36:53 | **no failure** | **1,821 s (30 min 21 s), ended by user** | B19 (interval 15-30 ms) |

Historical, from mining 182 device-manager logs and 86 trial CSVs (2026-09-02, before this session):
stress tests ended at **606.0 s** and **609.4 s**; **no trial in 86 ever exceeded 609.4 s**.

---

## 3. Bisect results

The two bisect builds differ only in the named compile flags.

| build | `REAL_TIME_I2C` | `RT_BLE_FORWARD` | RT over I2C | RT over BLE | result |
|---|---|---|---|---|---|
| baseline | 1 | (n/a, always forwarded) | yes | yes | failed every run, 50–789 s |
| B16 `rt0` | **0** | — | **no** | **no** | ~40 min, no failure |
| B17 `rt1_fw0` | 1 | **0** | **yes** | **no** | ~30+ min, no failure |

During B17 the user confirmed the **green LED_PWR was blinking throughout**, which is driven by
`ComsMCU::_life_pulse()` inside the `if (new_rt_data)` branch.

---

## 4. Reset-reason banners, verbatim

| time | banner |
|---|---|
| 09-10 14:37:24 | `RST:0x00000001:RESETPIN` |
| 09-11 13:10:00 | `RST:0x00000001:RESETPIN` |
| 09-11 13:11:31 | `RST:0x00000004:SREQ,BLESTALL_n1` |
| 09-11 13:51:38 | `RST:0x00000001:RESETPIN` |
| 09-11 13:54:56 | `RST:0x00000002:DOG,STAGE_0_dog1` |
| 09-11 14:15:33 | `RST:0x00000001:RESETPIN,B9` |
| 09-11 14:20:32 | `RST:0x00000002:DOG,STAGE_0_dog1,B9` |
| 09-11 14:46:11 | `RST:0x00000001:RESETPIN,B10` |
| 09-11 14:49:06 | `RST:0x00000002:DOG,STAGEn15_g0_dog1,B10` |
| 09-11 15:23:46 | `RST:0x00000001:RESETPIN,B13,I2Cf0_e0_c0_t218_w0` |
| 09-11 15:37:52 | `RST:0x00000002:DOG,STAGEn0_g0_dog1,B13,I2Cf0_e165_c2_t0_w112` |
| 09-11 15:57:18 | `RST:0x00000001:RESETPIN,B14_b1,I2Cf0k_e0x10_c0_t107d_w0d` |
| 09-11 16:01:03 | `RST:0x00000002:DOG,STAGEn0_g0_dog1,B14_b1,I2Cf21k_e285x10_c2_t266d_w300d` |
| 09-11 16:39:41 | `RST:0x00000001:RESETPIN,B15_b1,I2Cf0k_e0x10_c0_t107d_w0d_x0` |
| 09-11 16:50:55 | `RST:0x00000002:DOG,STAGEn0_g0_dog1,B15_b1,I2Cf64k_e278x10_c2_t266d_w0d_x300` |

Notes on fields, stated as facts about what the firmware writes, not as interpretation:

- `STAGE`/`STAGEn`/`g` were **always 0** except B10's `STAGEn15`. `b` was **always 1**.
- `BLESTALL_n1` appeared **exactly once**, on the only `SREQ` reset observed.
- `DOG` appeared on **every** watchdog reset, always with `dog1`, never `dog2` or higher.

### 4.1 I2C counter readings (B13 onward)

B13 fields were later found to be corrupted by the UART's int16×100 packing (range ±327.67); B14
onward are pre-scaled and clamped at 300.

| build | reading | scaled meaning |
|---|---|---|
| B14 (post-failure) | `I2Cf21k_e285x10_c2_t266d_w300d` | 21,000 frames; 2,850 errors; last code 2; 26.6 s since success; worst gap **at the 300 cap** |
| B15 (post-failure) | `I2Cf64k_e278x10_c2_t266d_w0d_x300` | 64,000 frames; 2,780 errors; last code 2; 26.6 s since success; worst gap **<100 ms**; longest consecutive-failure run **at the 300 cap** |
| B15 (post-flash, idle) | `I2Cf0k_e0x10_c0_t107d_w0d_x0` | zero frames, zero errors — taken before any trial |

B15 trial length was 626 s; 64,000 frames over 626 s ≈ 102 Hz, consistent with the configured rate.

Error rates as measured: **2,850 / 21,000 = 13.6%** (B14), **2,780 / 64,000 = 4.3%** (B15).

---

## 5. LED observations (user, direct)

The Nano's onboard RGB and the green `LED_PWR` were watched during several failures.

| occasion | RGB | green LED_PWR | could reconnect? |
|---|---|---|---|
| early freeze | **solid white** | not noted | no, required power cycle |
| next freeze | **solid red** | not noted | no |
| one failure | **still blinking, colour unchanged at the drop** | **still flashing** | **no** |
| 09-11, caught live | **solid red** | **solid, not flashing** | recovered itself |
| 09-11, caught live | — | — | observed: solid green for a few seconds, then cyan, then GUI reported disconnect |
| B17 `rt1_fw0`, 30+ min | — | **always blinking** | n/a, no failure |

---

## 6. Trial CSV analysis (`trial_20260910_143806.csv`)

21,057 rows, 229 s of data.

**Timing:**
- Largest device-side gap in exo time across the whole trial: **0.08 s**
- Largest host-side receive gap: **0.211 s** (at 14:40:19)
- Final 15 s window: largest host gap **0.082 s**, the smallest of the whole run
- Host wall-vs-exo lag drift returned to **−25 ms** at the end, no runaway

**Torque, in the 103 s after the controller was engaged (9,460 samples):**
- Peak **desired** torque anywhere: **10.98 Nm**
- Peak **commanded** torque: **30.00 Nm** (the `MAX_JOINT_TORQUE_NM` clamp)
- PID term (`commanded − desired`) exceeded the setpoint in **86%** of samples
- Right joint at the clamp in **5.7%** of samples (p95 = 30.00); left in 0.5%
- Sign reversals: **~10 per second** on both joints
- Before the controller was engaged: max commanded torque **0.00 Nm** for 150 s

---

## 7. Hardware / environment observations

- The failure has been reproduced on **3 different Nano boards**.
- The Teensy continues running through every failure: the user confirms the exo keeps applying
  torque normally, and "Exoskeleton time" (the Teensy's `millis()`) advances smoothly to the last
  delivered sample.
- SAM-BA upload failures began **2026-09-10** (the first watchdog flash) and occur consistently
  around **page 10–15 of 91**. They did not happen before that date. Pressing reset repeatedly
  (~6 times) recovers the board.
- The user is confident the upstream authors run the same or very similar hardware.
- The user reports the upstream authors do get real-time signals on their GUI.

---

## 8. Verified facts read out of source files

These are not observations of behaviour, but they are checkable statements about the code, as
distinct from inferences drawn from them.

**This repo:**
- `Config.h`: `_real_time_msg_delay = 9000` µs. `REAL_TIME_I2C 1` since the first commit (2023-04-13).
- `RealTimeI2C.h`: `MAX_LEN = 16`, `MAX_WIRE_PAYLOAD = 15`, `BILATERAL_HIP_ANKLE_RT_LEN = 11`.
  In-code comments state the actual payload is 13 floats.
- `RealTimeI2C.cpp`: Teensy is I2C **master** (`MY_WIRE.begin()`); Nano is **slave**
  (`begin(RT_I2C_ADDR)` + `onReceive`). `poll()` uses `noInterrupts()`/`interrupts()` around a
  ~34-byte copy. Payload packed as int16 ×100.
- `BleParser::package_raw_data` encodes each value as **ASCII decimal of `int(value*100)` plus a
  one-byte delimiter** — variable width, not fixed.
- `ExoBLE.h`: `MAX_PARSER_CHARACTERS 12`, used only to size the send buffer.
- `ExoBLE.cpp`: `send_message()` calls `writeValue()` **once** with the whole packed frame.
  `BLE.setConnectionInterval(6, 6)` = 7.5 ms. A separate `send_chunked()` used for the handshake
  uses 19-byte chunks and carries a comment stating the default ATT MTU is 23.
- `ComsMCU.cpp:117`: `ComsLed::life_pulse()` called every loop pass.
  `ComsMCU.cpp:197`: `_life_pulse()` (pin 25) called **only inside** `if (new_rt_data)`.
- `uart_commands.h:695`: the UART fallback for RT is non-functional — `rt_data::float_values` is
  `static` in a header, so writer and reader are different translation units.
- `UARTHandler.h:26-27`: UART payloads are packed `short int` × `FIXED_POINT_FACTOR 100`,
  i.e. a range of ±327.67.
- Git: `a546918` (2026-08-11) replaced heap allocations with static arrays; `578bc20` (2026-08-12)
  replaced the exact-length receive guard with a bounds check.

**Upstream (`naubiomech/OpenExo`, `upstream/main`, last commit 2026-06-11):**
- `RealTimeI2C.cpp` still contains `new uint8_t(byte_buffer_len)` and `new float(rt_data::len)` —
  `new T(n)` allocates one element, not an array.
- Its receive guard is still `if (byte_len != byte_buffer_len) return;`.
- `_real_time_msg_delay = 9000` µs — identical to this repo.
- `rt_data::len = BILATERAL_HIP_ANKLE_RT_LEN`.

**Toolchain (`mbed_nano` 4.6.0, ArduinoBLE 2.1.0 from the sketchbook; the repo vendors 1.2.1, which
is NOT what builds):**
- `Wire.cpp`: the I2C slave is `arduino::MbedI2C::receiveThd()` — a `while(1) { slave->receive(); }`
  **polling RTOS thread**, not an interrupt handler. `onReceiveCb` is called from that thread, after
  the critical section is exited. `read()` and `available()` each take a critical section.
- `HCI.cpp:636`: `sendAclPkt()` begins `while (_pendingPkt >= _maxPkt) { poll(); }` — **no timeout,
  no bound, no escape**.
- `HCI.cpp:666`: `_pendingPkt++` on every ACL packet sent.
- `HCI.cpp:861`: `_pendingPkt` is decremented **only** by `handleNumCompPkts()`.
- `HCI.cpp:304`: `ATT.setMaxMtu(pktLen - 9)` and `_maxPkt = leBufferSize->maxPkt`, both taken from
  the controller's LE Read Buffer Size response. Neither value has been observed at runtime.
- `ArduinoCore-mbed` 4.2.4 release notes record a fix: *"Wire: exit critical section before calling
  onReceiveCb()"* (PR #1032), described as preventing callback deadlocks on nRF52840.

---

## 8.5 B18 and B19 measurements

**B18 (bounded `sendAclPkt` wait, 50 ms), two failures, identical signature:**

```
RST:0x00000004:SREQ,BLESTALL_n1_w10_s3,B18_b255_rt1_fw1,I2Cf6k_e285x10_c2_t266d_w0d_x300
RST:0x00000004:SREQ,BLESTALL_n1_w10_s3,B18_b255_rt1_fw1,I2Cf4k_e287x10_c2_t266d_w0d_x300
```

- `w10` = longest `writeValue()` in the silent window fell in bucket 10 = **32.8-65.5 ms**.
  The configured timeout is 50 ms, which lands in bucket 10.
- `s3` = 100+ notifications attempted during the window; no `_FAIL`, so `writeValue()` never
  returned false.
- Banner changed from `DOG` (every prior failure) to `SREQ,BLESTALL`.
- LED observed live during one failure: **RGB fixed red while the green LED_PWR kept flashing.**
  The two indicators toggle at different rates - RGB every 100 loop passes, green every 10 RT
  arrivals - so at a ~20 Hz loop the RGB toggles once per 5 s (appears fixed) and the green at
  ~1 Hz (visibly flashing).
- `b255` appeared on every B18/B19 boot, where every earlier build reported `b1`.

**B19 (connection interval `(6,6)` -> `(12,24)`), one run, no failure:**

- Ran **30 min 21 s** and was ended deliberately by the user, not by a failure.
- **164,163 samples at 90.2 Hz**, versus the 2026-09-10 reference trial's **92.0 Hz**.
- Sample-gap distribution: median **10.0 ms**, p90 **20 ms**, p99 **30 ms**, max **70 ms**
  (reference: 10.0 / 20 / 30 / 80 ms).
- Exo time in this file wraps repeatedly - it is packed int16 x100 and wraps at +/-327.67 s - so
  spans computed from that column are meaningless over long runs. Rates above use wall-clock epoch.

**Baseline for comparison:** the ten failures observed before B19 had a mean time-to-failure of
**225.5 s**. Under a constant-hazard assumption, surviving 1,821 s has probability
**e^(-1821/225.5) = 0.00027**, i.e. **~1 in 3,700**. (For scale, the 13.2-minute run that the 2 s
ping produced was ~1 in 33.)

---

## 9. Things that were measured and found NOT to differ

- RT message rate: **identical** between this repo and upstream (9000 µs).
- The 2 s GUI ping: one 13-minute run occurred with it enabled, and the **next** run with it still
  enabled failed at 191 s.

---

## 10. Explicitly unmeasured

Listed so they are not mistaken for evidence:

- `_pendingPkt` and `_maxPkt` values at runtime — never read.
- The negotiated ATT MTU — never read.
- Actual bytes per RT notification on the wire — never measured.
- Whether pull-up resistors are present on the Nano's `Wire` pins — never checked.
- Whether the upstream authors experience the same failure — never asked.
- Whether B19 holds up across repeated, independent runs — **only one run so far.**
- The actual negotiated connection interval after the change — never read back.
- The Nano's program counter during a freeze — no SWD access.
