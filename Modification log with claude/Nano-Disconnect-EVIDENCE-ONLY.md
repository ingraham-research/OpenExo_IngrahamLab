# Nano disconnect — EVIDENCE ONLY

**Purpose:** every observation, with no interpretation, no theory, and no conclusions. This file
exists because the narrative document accumulated several confident explanations that later turned
out to be wrong, and it became hard to tell which statements were measured and which were inferred.

**Rule for this file:** if it was not directly observed, measured, or read out of a file, it does not
belong here. Theories live in `Nano-Hang-Watchdog-And-Breadcrumbs.md`.

**Last updated:** 2026-09-14 (§12 added; the §11 correction banner added).

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
- **The connection interval actually in effect — never read, in ANY build.** `setConnectionInterval()`
  sets only what is REQUESTED, once, at connection setup. ArduinoBLE discards the central's
  accept/reject (`L2CAPSignalingClass::connectionParameterUpdateResponse()` is an empty function) and
  its `LE_META_EVENT` enum has no `CONN_UPDATE_COMPLETE` (0x03), so the firmware never learns what
  was applied. Windows is documented to sometimes accept such a request without applying it, and to
  do so randomly. **Every reference to "7.5 ms" or "15-30 ms" in these documents describes what was
  requested, not what was in force.**
- Actual bytes per RT notification on the wire — never measured.
- Whether pull-up resistors are present on the Nano's `Wire` pins — never checked.
- Whether the upstream authors experience the same failure — never asked.
- Whether B19 holds up across repeated, independent runs — **only one run so far.**
- The actual negotiated connection interval after the change — never read back.
- The Nano's program counter during a freeze — no SWD access.

---

# APPENDIX - EXTERNAL REPORTS (NOT OUR OBSERVATIONS)

**This appendix is deliberately fenced off from everything above.** Nothing here was measured on our
hardware. It is included because it is checkable fact rather than inference - direct quotation from a
public bug tracker, with dates and authors - and because it bears directly on several things above.
**Do not let it substitute for our own measurements.**

Source: <https://github.com/arduino-libraries/ArduinoBLE/issues/45>, *"Weak signal doesn't trigger
disconnect() and hangs in multiple places"*. Opened 2019-12-19 by fgaetani, 19 comments, last
activity 2025-03-20, **state: OPEN**, labels `type: imperfection` / `status: waiting for information`.

## A1. Facts about the library itself (verified directly, 2026-09-12)

- Our sketchbook ArduinoBLE is `2.1.0`.
- `2.1.0` is the **latest release** (published 2026-06-22 per the GitHub releases API).
- Upstream `master` **still contains the unbounded `while (_pendingPkt >= _maxPkt) { poll(); }`** at
  `src/utility/HCI.cpp:638`. Checked by fetching the raw file from `master` on 2026-09-12.
- `BLE.setSupervisionTimeout()` exists in 2.1.0 (`BLELocalDevice.cpp:408`) and is **not called
  anywhere in our firmware**.
- `L2CAPSignalingClass::addConnection()` **receives** the central's chosen `interval`, `latency` and
  `supervisionTimeout` as arguments, from the HCI LE Connection Complete event (`HCI.cpp:1152`). It
  uses `interval` and `supervisionTimeout` and discards `latency`. None are stored or exposed.

## A2. Quotations - the hang (our presentation B)

fgaetani, opening post, 2019-12-19:

> Code execution remained locked in the `writeValue()` function, specifically in the
> `HCIClass::sendAclPkt()` function. The code remained locked in the `while` loop, because the device
> is disconnected.

Proposed patch, same post:

> ```cpp
> int k = 0;
> while (_pendingPkt >= _maxPkt) { k++; if (k > _maxPkt) break; poll(); }
> ```

## A3. Quotations - the unreachable device (our presentation A)

morettigiorgio, 2020-01-24:

> if i forced a BLE.disconnect() (during the connection lost), central.connected() and BLE.central()
> returned the correct false, but my Arduino peripheral (Arduino Nano 33 BLE and Arduino Nano 33 BLE
> Sense) is no more discoverable, not even with another BLE.advertise() cmd. It's very strange... I
> have to restart

JoeyTolentino, 2020-02-07:

> when I walk away, and I lose connectivity, the little display still indicates that "something is
> connected" and the IMU data is still updating as it should; however, I'm unable to see the device
> to reconnect to it when I'm near by. [...] The only way to reconnect would be to press the reset
> button, or cut power.

## A4. Quotations - the patch does NOT fix presentation A

fgaetani, 2020-02-26:

> The reported issue #45 is different, in my case the microcontroller lock in loop in that cycle and
> by modifying in that way I solved it. While the other issue [...] The board remains apparently
> connected and is no longer visible from other devices. The only solution is to reset the
> microcontroller manually or through a watchdog

morettigiorgio, 2020-01-24:

> I tested the above modify too [...] but without having solved.

JoeyTolentino, 2020-02-07:

> I've tried adding the recommended code by @fgaetani and I did not realize a behaviour change by the
> hardware.

## A5. Quotations - trigger and reproduction

Hoffa25, 2020-05-01, giving a repro procedure:

> Move the phone to a spot were the connection is really weak and it continuously jumps between
> connected and disconnected (9-10 meters and/or some object in front). Most of the time the error
> will occur within 5 minutes. To make it come quicker you can force disconnects by putting objects
> around the arduino (your hands, another smartphone, tablet, whatever blocks the signal well). When
> disconnected keep covering the arduino for 10-30s and then remove. Repeat until error occurs.

Hoffa25, same post, on the aftermath:

> As you see the loop() stops looping. Also a really weird thing starts: every 30 s from then on the
> eventhandlers get triggered with disconnected and connected. Even if I have removed my app!?

polldo (Arduino), 2020-07-02, **failing to reproduce**:

> Using your sketch I was not able to observe the reported error. [...] Even after several
> disconnection events caused by the weakness of the bluetooth signal, the nano33ble continues to
> loop and both the connection and disconnection events are correctly triggered. However, the strange
> behavior is that when the connection is weak the returned RSSI value appears to be 0.

## A6. Related issue history

- Issue #33, *"BLE nano 33 does not report or disconnect from central"* (2019-10-02) - the
  peripheral-does-not-notice-disconnection issue. **Closed 2019-12-02** by PR #44.
- PR #44, *"Cordio: BLE thread loop fixes"* - among other things restored `WSF_MS_PER_TICK` to match
  what mbed was compiled with, the author noting it *"needs to match the value mbed was compiled with
  for the supervision timeout to match."*
- Multiple users in #45 state that #44 did **not** resolve the behaviour in practice
  (alexisicte 2019-12-19, morettigiorgio 2020-01-24).

## A7. What this appendix does NOT establish

- **Not** that our failure and theirs have the same cause. The symptoms match; the trigger they all
  describe is weak signal, and ours occurs on a bench at ~1 m. Untested either way.
- **Not** that our RSSI behaves like theirs. We have never logged RSSI.
- **Not** anything about connection intervals. Nobody in that thread measured one either.

---

# §11. MEASURED CONNECTION INTERVALS AND OUTCOMES - 2026-09-12

> **2026-09-13 note - observations recorded in §12:** the `UPi` banner field records only the FIRST connection-parameter
> update of a connection (read from `ExoBLE.cpp:477-487`). The **15 ms** row in §11.1 was identified from `UPi12` alone. On
> 2026-09-13 the same banner, `CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1`, was reproduced three times on the same host and
> firmware with a Windows-side log running, and that log reported 7.5 ms for each connection after a ~1.8 s period at
> 15 ms (§12.3). The rows below are left exactly as originally written.

**All of this is measurement, which is why it belongs in this document.** Intervals are read either from
the HCI event fields surfaced in the connect banner (`CPi` = LE Connection Complete, `UPi` = LE Connection
Update Complete) or from trial-CSV packet timing. Where both exist they agree.

## 11.1 The ladder

| interval in force | how measured | runs | outcome |
|---|---|---|---|
| **7.5 ms** | `UPi6` + SUCCESS; CSV 52.2 bursts/s | 13 | **12 failures.** Mean ~216 s, min 0.96 s, max 789 s. One run CUT CLEAN at 637.7 s |
| **15 ms** | `UPi12` + SUCCESS; CSV 41.1 and 39.0 bursts/s | 2 | **2 failures**, 118 s and 429 s |
| **28.75 ms** | `UPi12`->settled; CSV 33.7 bursts/s | 1 | clean, **cut by operator at 1,050 s** |
| **~30 ms** | `CPi24`, `u0`; CSV 34.9 bursts/s | 3 | clean, cut at **1,309 / 1,821 / 1,896 s** |
| **30.0 ms** (B23) | `CPi6`, `u1`, `UPi24` + SUCCESS | 1 | ongoing at time of writing, past 20 min |

Every clean run above was **ended by the operator**, never by a failure.

## 11.2 Banner readings, verbatim

| time | build | banner fields | note |
|---|---|---|---|
| 16:36:49 | B21 | `CPi24_l0_t960_u0_n1` | |
| 16:45:59 | B21 | `CPi24_l0_t960_u0_n1` | |
| 16:46:31 | B21 | `CPi24_l0_t960_u0_n1` | |
| 16:47:11 | B21 | `CPi24_l0_t960_u0_n1` | four separate boots, byte-identical |
| 17:04:16 | B22 | `CPi24_l0_t960_u1_n1,UPi12_t960_s1_n1` | requested 25-28.75 ms; **CSV shows it ran at 28.75 ms**, so `UPi12` was transient |
| 17:26:34 | B20 | `CPi23_l0_t960_u1_n1,UPi6_t960_s1_n1` | opened at arm C's 28.75 ms; **granted 7.5 ms** |
| 17:39:17 | B20 | `CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1` | opened at 7.5 ms, **nothing requested**, Windows moved it to 15 ms itself |
| 17:42:08 | B20 | same | identical condition, replicated |
| 17:58:22 | B23 | `CPi6_l0_t960_u1_n1,UPi24_t960_s1_n1` | opened at 7.5 ms, request fired, **granted 30.0 ms** |

## 11.3 Facts established by those readings

- **The supervision timeout is 9.6 s** (`t960`, every reading). It equals, to the digit, the constant lag
  between the last CSV sample and the GUI's `Device disconnected` line, measured independently at 9.6 s on
  both A' runs (and 0.1 s on a manual end).
- **Windows persists the negotiated interval per device**, across reflashes and GUI restarts. n=3:
  `i23` after a 28.75 ms session, `i6` twice after 7.5 ms sessions.
- **Windows grants the MAXIMUM of the requested range.** n=2: `(20,23)` -> 28.75 ms; `(20,24)` -> 30.0 ms.
- **Windows also changes the interval with no request from us.** `u0` with `UPi12`: it moved a 7.5 ms link
  to 15 ms on its own.
- **Windows 11 10.0.26200 granted both 7.5 ms and 28.75 ms**, with status SUCCESS.

## 11.4 Failure fingerprint - unchanged across every failure measured

`SREQ,BLESTALL_n1_w10_s3` plus `c2` and `x300`, on B18, both A' runs, and both 15 ms runs.
`c2` = the Teensy's `endTransmission()` saw a NACK, i.e. **the Nano stopped ACKing first**.
`x300` = at or above the cap, i.e. **>= 3 s of unbroken I2C silence**.

## 11.5 Stream health, measured from trial CSVs

| run | rows | mean rate | gaps > 100 ms | max gap |
|---|---|---|---|---|
| ~30 ms, 1,309 s | 115,442 | 88.2 Hz | **2**, totalling 0.3 s | 145 ms |
| 28.75 ms, 1,050 s | 99,620 | 94.9 Hz | - | 265 ms |
| 7.5 ms, died 66 s | 6,338 | 96.0 Hz | - | 52 ms |

**The failure is abrupt.** The 7.5 ms run held 96.0 Hz with a p99 gap of 41 ms right up to its last
sample. No ramp, no rising gap distribution, no warning.

## 11.6 Still NOT measured

- Where between 15 ms and 28.75 ms the fatal threshold sits.
- Whether 25 ms is safe. Only 28.75 ms and ~30 ms have run long.
- Whether the interval stays put **mid-session**. The banner only covers the first ~2 s of a connection,
  and `addConnection()` runs once, at connect.
- `_pendingPkt` / `_maxPkt` at runtime; the negotiated ATT MTU; the Nano's program counter at a freeze.

---

# §12. MEASUREMENTS 2026-09-13 AND 2026-09-14

All times local. "Windows-side log" = the `CONN_PARAMS` lines written by `Python_GUI/services/ConnParamsMonitor.py` into the
device-manager log (see `Windows-Connection-Parameter-Logger.md`), which read WinRT
`BluetoothLEDevice.GetConnectionParameters()` on each connection-parameter / status event and every 10 s.

## 12.1 Trials

| CSV | firmware | condition | duration | how it ended | rows | rate | gaps > 100 ms | max gap |
|---|---|---|---|---|---|---|---|---|
| `trial_20260913_160933.csv` | B23 | bench, zero torque, treadmill motors running | 1,773.4 s | End Trial | 154,650 | 87.2 Hz | 158 | 213 ms |
| `trial_20260913_165734.csv` | B23 | real participant walking, `splineAlt` at torque | 1,804.3 s | End Trial | 165,512 | 91.7 Hz | ~310 | 1,081 ms |
| `trial_20260913_184405.csv` | B20 `(6,6)` | bench | 146 s | End Trial | 14,021 | 96.1 Hz | 1 | 118 ms |
| `trial_20260913_184703.csv` | B20 | bench | 15 s | End Trial | 1,434 | 95.8 Hz | 0 | 39 ms |
| `trial_20260913_184745.csv` | B20 | bench | 386.3 s | End Trial `'Z'` at 18:54:13.390 (operator: battery dying) | 37,104 | 96.0 Hz | 7 in the first 330 s | 136 ms in the first 330 s |
| `trial_20260913_185510.csv` | B20 | bench | 291.3 s | **link lost** | 27,965 | 96.0 Hz | not computed | not computed |
| `trial_20260913_191826.csv` | B23 (reflashed) | bench | 64.2 s recorded | log ends 19:19:33 with no End Trial and no disconnect entry | 6,130 | 95.4 Hz | not computed | not computed |
| `trial_20260914_140122.csv` (second laptop) | B23 | bench, zero torque | 1,189 s | End Trial `'Z'` at 14:21:13.396 | 94,547 | 79.5 rows/s | not computable - `epoch` has 1 s resolution in this copy (§12.8) | - |

No failure occurred in any B23 trial.

**`trial_20260913_165734.csv`, torque by torqueScale segment** (parameter-write times from the device-manager log):

| segment | trial time | desired p95 / max (L, R) | commanded p95 / max | at 30 Nm clamp (L / R) | measured p95 (L / R) | stance onsets per minute (L / R) |
|---|---|---|---|---|---|---|
| controller not yet engaged | 0-51.7 s | 0 / 0 | 0 / 0 | 0 / 0 | 5.4 / 4.3 Nm | 12.8 / 10.5 |
| torqueScale 0 | 51.7-61.2 s | 0 / 0 | 13.6-13.9 / 27.8-30.0 Nm | 0 / 0.23% | 4.6 / 4.7 Nm | 56.9 / 50.6 |
| torqueScale 50 | 61.2-1,518.1 s (24.3 min) | 5.8 / 6.5 Nm | 12.2 (L), 11.1 (R) / 30.0 Nm | 0.02% / 0.02% | 5.4 / 5.4 Nm | 50.0 / 50.0 |
| torqueScale 70 | 1,518.1-1,804.3 s (4.8 min) | 8.1 / 9.1 Nm | 15.5 (L), 13.5 (R) / 30.0 Nm | 0.02% / 0.17% | 6.8 / 7.4 Nm | 49.1 / 49.1 |

`trial_20260913_160933.csv`: desired and commanded torque 0.00 Nm on both sides for the whole trial.

**`trial_20260913_165734.csv`, stream around the operator's range test** (operator: walked far from the laptop with the
participant between, "towards 1700-1750" on the plot), rows/s in 10 s blocks of exo time:
1,740: 94.9 - 1,750: 90.5 - **1,760: 67.7 - 1,770: 60.1 - 1,780: 71.6 - 1,790: 71.9 - 1,800: 71.1 - 1,810: 56.1 -
1,820: 40.7 - 1,830: 41.8 - 1,840: 71.2** - 1,850: 89.5 - 1,860: 92.7 - 1,870: 89.3 - 1,880: 92.0. Largest gap in that span
421 ms. Six largest gaps of the whole trial (trial s / ms): 1,326.3 / 1,081; 45.1 / 783; 718.5 / 756; 13.8 / 543;
8.4 / 500; 701.4 / 454.

## 12.2 Connect banners, verbatim

| time | banner |
|---|---|
| 16:09:24 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t109d_w0d_x0,CPi48_l0_t960_u1_n1,UPi24_t960_s1_n1` |
| 16:57:00 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi24_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:26:02 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t108d_w0d_x0,CPi24_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:30:52 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi24_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:34:08 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi24_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:43:49 | `RST:0x00000001:RESETPIN,B20_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi24_l0_t960_u1_n1,UPi6_t960_s1_n1` |
| 18:46:49 | `RST:0x00000004:SREQ,B20_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:47:37 | `RST:0x00000004:SREQ,B20_b255_rt1_fw1,I2Cf0k_e0x10_c0_t106d_w0d_x0,CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 18:54:57 | `RST:0x00000001:RESETPIN,B20_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1` |
| 19:18:00 | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t107d_w0d_x0,CPi6_l0_t960_u1_n1,UPi24_t960_s1_n1` |
| 09-14 13:58:42 (second laptop) | `RST:0x00000001:RESETPIN,B23_b255_rt1_fw1,I2Cf0k_e0x10_c0_t109d_w0d_x0,CPi36_l0_t960_u1_n1,UPi24_t960_s1_n1` |

## 12.3 Windows-side log, per connection (2026-09-13)

Timestamps are seconds within the minute shown. "Link up" = first non-zero reading. Every reading had latency 0 and
timeout 9,600 ms.

| connection | device log | link up | then | banner read |
|---|---|---|---|---|
| 18:26 B23 | `device_manager_20260913_182554.log` | 01.300 at 30 ms | 02.321 -> 15 ms; 03.941 -> 30 ms | 02.774 |
| 18:30 B23 | `device_manager_20260913_183042.log` | 51.256 at 30 ms | 52.281 -> 15 ms; 53.869 -> 30 ms | 52.673 |
| 18:34 B23 | `device_manager_20260913_183335.log` | 06.991 at 30 ms | 08.007 -> 15 ms; 09.625 -> 30 ms | 08.455 |
| 18:43 B20 | `device_manager_20260913_184340.log` | 48.591 at 30 ms | 48.944 -> 7.5 ms; no further change | 49.524 |
| 18:46 B20 | same log | 48.319 at 7.5 ms | 48.545 -> 15 ms; 50.389 -> 7.5 ms | 49.129 |
| 18:47 B20 | same log | 36.512 at 7.5 ms | 36.757 -> 15 ms; 38.585 -> 7.5 ms | 37.311 |
| 18:54 B20 | `device_manager_20260913_185448.log` | 56.608 at 7.5 ms | 56.839 -> 15 ms; 58.699 -> 7.5 ms | 57.440 |
| 19:17 B23 | `device_manager_20260913_191753.log` | 59.629 at 7.5 ms | 59.807 -> 30 ms; 00.227 (19:18) -> 15 ms; 01.967 -> 30 ms | 00.783 |

- In every connection, every heartbeat after the last change repeated the settled value, for the full logged duration
  (longest: the 18:47 connection, through a 386 s trial).
- No reading changed after the connect sequence in any connection.
- Before the link existed, every `[attach]` read returned interval 0, latency 0, timeout 0.
- WinRT read durations: 0.01-0.21 ms at 30 ms (0.77 ms once at attach); 0.01-2.34 ms at 7.5 ms while the RT stream was
  running.

**2026-09-14, second Windows 11 laptop** (terminal output pasted by the operator; `device_manager_20260914_135830.log` is on
that laptop, not this PC): attach 13:58:40.444, zeros; link up 13:58:41.175 at 45 ms; 41.753 -> 30 ms; 42.158 -> 15 ms;
banner read 42.789; 44.004 -> 30 ms. Every heartbeat from 13:58:50.446 to 14:21:11.708 read 30.00 ms, and no reading changed
after 13:58:44.004. WinRT read durations 0.02-14.95 ms, 11 heartbeats above 3 ms. End Trial `'Z'` 14:21:13.396; GUI
`disconnect()` 14:21:19.503; a single disconnect callback, `Intentional: True`, at 14:21:23.245.

## 12.4 The `(6,6)` failure of 2026-09-13, with the Windows-side log running

```
18:55:10.574  beginTrial() called                                    (device_manager_20260913_185448.log)
18:55:11.622  first CSV sample                                       (trial_20260913_185510.csv)
18:59:56.325  CONN_PARAMS [heartbeat] interval=7.50ms latency=0 timeout=9600ms
19:00:02.901  last CSV sample
19:00:06.326  CONN_PARAMS [heartbeat] interval=7.50ms latency=0 timeout=9600ms read=0.03ms
19:00:12.481  CONN_PARAMS [status] no active link (Windows reports zeros) read=0.36ms
19:00:12.482  CONN_PARAMS [event] no active link (Windows reports zeros) read=0.74ms
19:00:12.487  Device disconnected. Reason: link lost, Intentional: False
```

- Last CSV sample to `link lost`: **9.586 s**.
- Every heartbeat from the connect sequence to 19:00:06.326 read 7.5 ms. No `CHANGED` line after 18:54:58.699.
- The board was reflashed without reconnecting first, so no post-failure banner was read.

## 12.5 CSV phase-lock readings

`R(T) = |mean(exp(2*pi*i*t/T))|` over 2 s windows of the `epoch` column, median over windows. Values near 0.07 are the noise
floor.

| CSV | interval reported by Windows (where logged) | reading |
|---|---|---|
| `trial_20260912_170437.csv` (B22) | - | R(28.75) 0.78, R(30) 0.05 |
| `trial_20260912_163701.csv` (B21) | - | R(30) 0.78, R(15) 0.35; 0.125 ms grid: peak exactly 30.000 |
| `trial_20260912_175832.csv` (B23) | - | R(30) 0.46 |
| `trial_20260912_172644.csv` (B20, `UPi6`) | - | R(7.5) 0.82-0.85 in 10 s blocks from 20 s on |
| `trial_20260913_160933.csv` (RUN1) | - | R(30) 0.75-0.79 in most 2-min blocks, dips to 0.16-0.38 in five; 60-360 s: peak 30.000 R 0.78 |
| `trial_20260913_165734.csv` (RUN2) | - | 2-min blocks: R(30) 0.13 at 0-120 s; 0.39-0.51 from 120 s to 1,320 s; 0.28, 0.40, 0.41, 0.18, 0.17 after |
| `trial_20260913_184405.csv` | 7.5 ms throughout | R(7.5) <= 0.14 in every 20 s block; no peak above 0.11 on the 10-50 ms grid |
| `trial_20260913_184745.csv` | 7.5 ms throughout | R(7.5) <= 0.13 in every 20 s block to 280 s; 0.54, 0.59, 0.69 at 280, 300, 320 s |
| `trial_20260912_173932.csv` (died 118 s) | - | no lock at any legal interval; 0.125 ms grid, 40-118 s: 29.500 R 0.26, 29.375 R 0.23, 29.625 R 0.22 |
| `trial_20260912_174229.csv` (died 430 s) | - | no lock at any legal interval (all <= 0.25 in 60 s blocks) |

R at 15.625, 31.25 and 46.875 ms (Windows timer-tick multiples) was <= 0.17 in both 2026-09-12 failure CSVs.

**Connection-event start gaps** (a start = first sample after > 4 ms of silence), % of start-to-start gaps in each band:

| segment | 12-18 ms | 20-25 ms | 26-34 ms | 40-50 ms |
|---|---|---|---|---|
| `trial_20260913_184405.csv` (Windows: 7.5 ms) | 33.9 | 17.5 | 20.1 | 0.9 |
| `trial_20260913_184745.csv`, 0-280 s (Windows: 7.5 ms, no lock) | 45.9 | 9.6 | 12.2 | 3.2 |
| `trial_20260913_184745.csv`, 280 s on (Windows: 7.5 ms, locked) | 35.2 | 2.5 | 1.1 | 0.3 |
| `trial_20260912_173932.csv`, 40-118 s | 0.1 | 8.5 | 81.0 | 0.2 |
| `trial_20260912_174229.csv` | 28.6 | 23.6 | 13.3 | 1.9 |
| `trial_20260913_160933.csv`, 60-360 s (30 ms) | 0.9 | 11.8 | 49.3 | 0.2 |

## 12.6 GUI log counts

**"Controller list looks incomplete" warnings per connection**, pairing each `app_crash_*.log` with its device-manager log,
grouped by the build tag in the banner (counted up to the 18:30 session of 2026-09-13):

| builds | sessions | connections | warnings | rate |
|---|---|---|---|---|
| no build tag (before B19) | 90 | 163 | 23 | 14.1% |
| B9-B18 and B20 | 13 | 25 | 0 | 0% |
| B19, B21, B22 | 6 | 11 | 0 | 0% |
| B23 | 5 | 5 | 1 (18:30:56, `Ankle(R) (36) missing constantT`) | 20% |

**Two disconnect-callback lines within 50 ms** (`Intentional: True` then `Intentional: False`) - 11 occurrences in 10
device-manager logs: 2026-07-06 20:56:16, 07-07 14:44:44, 07-21 14:39:58, 08-12 17:40:46, 08-20 15:53:13 and 15:55:52,
08-25 15:58:49, 08-26 15:00:11, 09-09 13:27:22, 09-12 16:47:36, 09-13 17:27:49 (gaps 0-7 ms).

End of `trial_20260913_165734.csv`: CSV closed 17:27:39.787; `'Z'` delivered 17:27:39.800; GUI `disconnect()` 17:27:45.895;
callbacks 17:27:49.837 (`True`) and 17:27:49.839 (`False`). Torque-scale writes to joint 68 then failed with `Not connected`
at 17:27:49.881, 17:27:55.016 and 17:28:00.167.

## 12.7 Now measured, and still not

**Now measured (Windows 11 hosts, from 2026-09-13):** the interval in force for the whole connection, including mid-session;
what Windows does during the connect sequence.

**Still not measured:** where between 7.5 ms and 28.75 ms the fatal threshold sits; why the 18:43 `(6,6)` connection had no
15 ms step; what the 2026-09-12 118 s run's link was doing; `_pendingPkt` / `_maxPkt`; the Nano's program counter at a
freeze; a post-failure banner for the 2026-09-13 `(6,6)` failure.

## 12.8 Second-laptop CSV and data delivery on the Teensy clock

**Format of `trial_20260914_140122.csv` as found on this PC:**

```
epoch,mark,Desired Torque (L),Measured Torque (L),Desired Torque (R),Measured Torque (R),Toe FSR (L),In Stance (L),Toe FSR (R),In Stance (R),Commanded Torque (L),Commanded Torque (R),Status,Exoskeleton time (seconds)
1789419684,0,0,0.05,0,0.28,0,0,0,0,0,0,5,201.1
```

Every other trial CSV from this GUI writes six decimals, e.g. `1789340974.645777,0,0.000000,0.100000,...`
(`trial_20260913_160933.csv`). In this copy the `epoch` column holds whole seconds only. Desired and commanded torque are
0.00 Nm throughout.

**Samples delivered, from the `Exoskeleton time (seconds)` column** (nominal RT period 9 ms = 111.1 Hz; a hole = a step in
exo time above 100 ms):

| CSV | host | rows | exo span | rows/s | % of 111.1 Hz | holes > 100 ms | total hole time | longest hole | worst 5-min block |
|---|---|---|---|---|---|---|---|---|---|
| `trial_20260914_140122.csv` | second laptop, bench | 94,547 | 1,189.4 s | 79.5 | 71.6% | 2 | 0.2 s | 0.10 s | 78.6 rows/s |
| `trial_20260913_160933.csv` | operator's laptop, bench | 154,650 | 1,773.3 s | 87.2 | 78.5% | 57 | 6.8 s | 0.20 s | 82.3 rows/s |
| `trial_20260913_165734.csv` | operator's laptop, person | 165,512 | 1,804.2 s | 91.7 | 82.6% | 68 | 10.3 s | 0.45 s | 82.0 rows/s |
| `trial_20260912_175832.csv` | operator's laptop, bench | 184,407 | 1,926.5 s | 95.7 | 86.2% | 0 | 0.0 s | 0.00 s | 95.5 rows/s |

## 12.9 Second laptop, worn session, 2026-09-14 - operator report and pasted parameter log

**Operator report:** the exo was worn by the operator for the whole session; the session lasted 26+ minutes and was ended
with End Trial; the logger's heartbeat read 30 ms until the end; `main_external_control.py` was run multiple times with
multiple controllers, mostly in UDP mode. The terminal output was not kept. The device-manager log and the trial CSV are on
that laptop, not this PC.

**`Python_GUI/external_control/Logs/Active Test/Param_write_log.txt` from that laptop, verbatim** (the file is overwritten
each orchestrator session, so this is the last session only):

```
Python time,Target,Value,Result,Attempts
1789428536.7199788, Ankle(L) TorqScale, 0.0, accepted, 1
1789428542.3954585, Ankle(R) TorqScale, 0.0, accepted, 2
1789428542.687822, Ankle(L) PlantarNm, 14.0, accepted, 1
1789428543.0301085, Ankle(R) PlantarNm, 14.0, accepted, 1
1789428543.467759, Ankle(L) DorsiNm, 7.466666666666667, accepted, 1
1789428543.8213584, Ankle(R) DorsiNm, 7.466666666666667, accepted, 1
1789428548.599465, Ankle(L) TorqScale, 50.0, accepted, 1
1789428549.0521646, Ankle(R) TorqScale, 50.0, accepted, 1
1789428676.6550918, Ankle(L) TorqScale, 0.0, accepted, 1
1789428677.0294697, Ankle(R) TorqScale, 0.0, accepted, 1
```

- First entry 16:28:56.720 local, last 16:31:17.029.
- Ten writes; nine accepted on the first attempt, one (Ankle(R) TorqScale 0.0) on the second, 5.68 s after the preceding
  entry. `DEFAULT_ACK_TIMEOUT = 5.0` s in `OpenExoLink_utilities.py`.
- TorqScale 50.0 held from 16:29:08.6 to 16:31:16.7 (~128 s) in this session.

## 12.10 ArduinoBLE version history on the development PC, and the early-July disconnects

Gathered 2026-09-15 for the open question in `Nano-Hang-Watchdog-And-Breadcrumbs.md` §15.11.

- Library zips kept by the Arduino toolchain in `F:\Random storage\Arduino15\staging\libraries`:
  **`ArduinoBLE-1.5.0.zip` dated 2026-01-26 14:25** and **`ArduinoBLE-2.1.0.zip` dated 2026-07-06 14:00**. No 1.2.1 zip.
- Every file in the sketchbook `ArduinoBLE` carries mtime **2026-07-14 11:32**, except the two we touched: `HCI.cpp`
  2026-09-12 and `HCI.cpp.orig-openexo-backup` 2026-09-11. Its `library.properties` says `version=2.1.0`; the copy this repo
  carried until 2026-09-15 said `version=1.2.1`.
- GUI device-manager logs run from `device_manager_20260702_155947.log` onward. The **first** line reading
  `Reason: link lost, Intentional: False` anywhere in that record is **2026-07-06 20:54:16**.
- Counts of that line: **10** across the 31 logs dated before 2026-07-14, **92** across the 125 logs from 2026-07-14 on.
  Several of the 10 occur within ~10 s of an `End-trial reset 'Z'` (2026-07-08 17:28:59 -> 17:29:09;
  2026-07-10 18:26:35 -> 18:26:45). Others have a trial running and no `'Z'` (2026-07-06 20:54:23 trial start ->
  20:56:16 link lost; 2026-07-07 14:42:37 -> 14:44:44).
- The early-July disconnects were on Nano MACs `C0:63:FA:86:05:F4` and `65:43:3E:4B:42:FA`, not the
  `D1:A7:1E:F3:E6:43` used from September onward.
- Read from the 1.2.1 source as this repo carried it until 2026-09-15: `BLELocalDevice.cpp:321` `setConnectionInterval()`
  forwards to `L2CAPSignaling`; `L2CAPSignaling.cpp:38` `addConnection()` holds the same `updateParameters` test and sends
  `CONNECTION_PARAMETER_UPDATE_REQUEST` (0x12); `HCI.cpp:418` has `while (_pendingPkt >= _maxPkt) {`.
- `ExoCode/src/ExoBLE.cpp` has contained `BLE.setConnectionInterval(6, 6)` since commit `248fa9f` (2023-04-13).
