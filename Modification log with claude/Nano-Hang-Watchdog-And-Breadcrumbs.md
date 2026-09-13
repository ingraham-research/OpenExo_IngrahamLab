# Nano hang: watchdog + loop breadcrumbs

**Date:** 2026-09-10
**Branch:** `disconnection_troubleshooting`
**Status:** **FLASHED AND BENCH-TESTED. Mode 2 confirmed by the device (§3.7); the failure is now
characterised as an INTERRUPT-LEVEL STOP (§3.8), not a loop hang.** Root cause still unknown.
**BISECT COMPLETE (§3.9): the BLE notification stream is a NECESSARY CONDITION; I2C is not.**
Raw observations are now kept separately in `Nano-Disconnect-EVIDENCE-ONLY.md` - read that first.
Superseded note: build B16 was a `REAL_TIME_I2C 0` bisect - see §3.8/§3.9 before reading
anything into blank plots. The watchdog fired on the real hardware and the
Nano recovered itself - the first time in this whole investigation that a disconnect did not require a
power cycle. That first run also exposed two bugs in this change; both are fixed (§3.6) and recompile
clean for `arduino:mbed_nano:nano33ble` AND `teensy:avr:teensy41`, but the FIXES are not yet flashed.
Watchdog + breadcrumbs are committed (`d76707d`); the link-stall detector is committed (`8d214fd`);
the §3.6 fixes are uncommitted.
**Covers TWO changes:** the hardware watchdog + breadcrumbs (§3.1-3.4), and the **BLE link-stall
detector** (§3.5), which adds a **GUI<->firmware protocol dependency** (a 2 s ping) and therefore
touches `Python_GUI/` as well as `ExoCode/`. Both are temporary.
**Intended lifetime:** TEMPORARY. This is diagnostic scaffolding for the mid-trial freeze. §5 is a
complete, mechanical removal procedure — read it before deleting anything by hand.

---

**!! READ SECTION 12 BEFORE TRUSTING SECTION 9 MECHANISM !!** The connection interval was **never
verified**: ArduinoBLE discards the central accept/reject and does not handle CONN_UPDATE_COMPLETE,
and Windows is documented to accept such requests without applying them, randomly. The RESULT (two
clean 30-min runs versus 10/10 failures) is observation and stands. The EXPLANATION is inference by
elimination, not measurement.

## 0. TL;DR

The mid-trial Nano freeze was **proven to be a hang** — not a crash, not a reset, not a brownout.
Nothing in the firmware could ever recover it, and the only recovery available (a power cycle)
destroyed the evidence every time. This change adds the nRF52840 hardware watchdog so a hang
**self-recovers warm in ~5 s**, plus a one-byte breadcrumb so the next boot reports **which phase of
`loop()` it died in**.

This does not fix the bug. It converts an unrecoverable dead end into a data source. Once the root
cause is found and fixed, **all of it should come out** — see §5.

---

> ## ✅ RESOLVED 2026-09-12: THE CONNECTION INTERVAL IS THE CAUSE - MEASURED, NOT INFERRED
>
> **MEASURED LADDER: 7.5 ms -> 12/12 fail. 15 ms -> 2/2 fail (118 s, 429 s). 28.75 ms -> clean 1,050 s.
> ~30 ms -> clean 5,026 s.** The fatal threshold is between 15 and 28.75 ms.
>
> **⚠ `(12,24)` IS SUPERSEDED - it contains the fatal 15 ms and cannot correct a host that has
> persisted it. PRODUCTION DEFAULT IS NOW `(20,24)` = 25-30 ms, build B23 (§14.33).**
>
> `(6,6)` is granted by Windows as **7.5 ms** - confirmed twice over, by the `UPi6` banner AND by
> burst-spacing analysis of the traffic (§14.25) - and fails **12/12**. **Caution: its failure time
> has a fat tail reaching at least 789 s, so no single `(6,6)` run under ~13 min can show safety**
> (§14.26). Pooled: 12 failures in 3,240 s at the fast interval, 0 in 6,076 s at a relaxed one,
> p ~ 1.7e-10. A relaxed
> interval (~29-30 ms) has run **6,076 s clean across four runs**. Arm C proves it is the interval
> and not the act of asking: it **sends a request** and survives. See §14.20-§14.22.
> **Also new and operationally important: Windows PERSISTS the negotiated interval across
> connections (§14.23), so test order matters and a banner is no longer self-describing.**
>
> ## ⚠ THE INTERVAL IS CURRENTLY SET BACK TO THE FAILING VALUE, ON PURPOSE
>
> **2026-09-12: arm A' is DONE and the failure came back 2/2 (10.6 s and 75.7 s), so `(6, 6)` is
> now 12/12 failures.** `EXO_BLE_INTERVAL_PINNED` is back to `0` = `(12, 24)` = banner `B21`.
> Set it to `1` to reproduce the failure on demand - it is a control, not a revert, and not an
> abandonment of §9. Do not fly arm A' on a person. Details and the hot-laptop caveat: §14.3.
>
> **2026-09-12:** §13 records that this is a **known upstream bug, open since 2019 and unfixed in
> the current release** (ArduinoBLE issue #45) - including independent reports of presentation A,
> and independent evidence that §8's patch does **not** cure it. §14 is the ranked list of what to
> test next, starting with a possible on-demand repro.

---

## §0. CURRENT STATE OF TRUTH - read this and nothing else if you are in a hurry

**Added 2026-09-12 as the authoritative status block.** This document is ~2,800 lines written across
three days of live debugging, and it contains roughly a dozen claims that were later retracted or
superseded **in place**. That history is deliberately preserved - it is how the reasoning can be
audited - but it means **no individual section below can be trusted on its own.** This section and the
ledger in §0.3 are the index of what is actually true.

### §0.1 The answer

**The Nano's mid-trial BLE death is caused by a short connection interval.** Measured, not inferred:

| interval in force | result |
|---|---|
| **7.5 ms** | **12 / 12 failures** (mean ~216 s, max 789 s) |
| **15 ms** | **2 / 2 failures** (118 s, 429 s) |
| 28.75 ms | clean 1,050 s |
| ~30 ms | clean 5,026 s across three runs, plus B23 ongoing |

14 failures at <= 15 ms; **zero failures in 6,076+ s at >= 28.75 ms**. The fatal threshold lies between
15 ms and 28.75 ms and has not been located. Pooled against the fast-interval failure rate (one per
270 s), p ~ 2 x 10^-12.

The causal claim is not merely correlational: **arm C sends an L2CAP parameter-update request and
survives, while arm A' sends one and dies** (§14.18). That rules out "the act of asking Windows is what
hurts", which was the one alternative an A/B/A could not exclude.

### §0.2 What is in the firmware now, and why

| item | status | why |
|---|---|---|
| `EXO_BLE_INTERVAL_SEL 0` -> `setConnectionInterval(20, 24)` = 25-30 ms, build **B23** | **THE FIX** | §14.33. `max` is what Windows grants you; `min` is only the trigger that decides what gets corrected (§14.36) |
| Bounded wait in sketchbook `ArduinoBLE/HCI.cpp` `sendAclPkt` | **KEEP**, report upstream | An unbounded wait on a remote party is a defect regardless of trigger. **Known upstream bug, open since 2019** (§13). Lost on any library update |
| Connection-parameter readout (`CPi`/`UPi` banner fields) | **KEEP while diagnosing** | Two more sketchbook patches to `HCI.cpp`: the LE Connection Complete capture, and a handler for **LE meta subevent 0x03** which upstream has never implemented (§14.2, §14.28) |
| Hardware watchdog (nRF52840 WDT, 5 s) | **KEEP** - §11.5's "decide" is superseded | Catches presentation **B** (main loop stopped). The community's only remedy for the 2019 upstream bug (§13.4). Costs: breaks uploads, so power-cycle before flashing (§7.0) |
| BLE link-stall detector (8 s) + GUI 2 s ping + TX busy counter | **KEEP** | Catches presentation **A** (loop alive, link dead, would never re-advertise). **Now also the only protection against a mid-session interval drift, which the fix structurally cannot cover** (§14.37) |
| CSV exo-time unwrap in `MainWindow.py` | **KEEP** | Unrelated real bug, four months old (§10.2) |
| Breadcrumbs, send-path diag, Teensy I2C counters, link-stats command, bisect flags | **REMOVE** when convenient | §11.2-§11.3. Purpose served; the breadcrumbs never worked at all |

### §0.3 RETRACTION LEDGER - every claim in this document that is no longer true

Ordered as they appear. **If you find any of these asserted in the body, the body is the stale copy.**

| # | claim, as originally written | status | where |
|---|---|---|---|
| 1 | "The Nano never froze; this is host-side" | **retracted** - the device confirmed it (`BLESTALL`) | §3.7 |
| 2 | "The failure is an interrupt-level stop" | **retracted** - `MbedI2C::receiveThd()` is a polling RTOS thread, not an ISR | § line 466 |
| 3 | "Missing I2C pull-ups" | **retracted** - bisect round 2 exonerated I2C entirely | §3.9 |
| 4 | "The link is bandwidth-saturated" | **withdrawn** | § line 609 |
| 5 | "The 2 s ping is a likely fix" (one 13.2-min run) | **retracted** - ~1 in 33, not evidence | § line 1006 |
| 6 | "No headroom to drain a backlog at 7.5 ms" | **withdrawn** - ~48% headroom, and body-blocking recovered unpatched | §12.5 |
| 7 | "B18 gave self-recovery" | **retracted** - the watchdog did that, and the GUI never auto-reconnects | §8 |
| 8 | "Windows is constantly asked to change parameters" | **retracted** - it is ONE request, at connect | §12.3 |
| 9 | "Teensy I2C `f`/`e` counters are cumulative since Teensy boot" | **retracted** - frames rose while errors FELL. Reset semantics unknown; **do not build on `f`/`e`**. `c` and `x` are fine | §14.x |
| 10 | "Arm B degrades under load and recovers" | **retracted** - the CSV shows 88.2 Hz with two gaps >100 ms in 22 min. The **GUI rendering** stuttered, not the link. The "recoverability not throughput" reframing built on it is withdrawn | §14.x |
| 11 | "The 9.6 s CSV-to-disconnect lag is `EXO_BLE_STALL_MS` + ~1.6 s" | **corrected** - it is the **supervision timeout**, measured at `t960` = exactly 9.6 s | §14.x |
| 12 | "My arm C design is void as a confound-breaker" | **retracted** - it worked exactly as designed; I had over-read a transient | §14.17 |
| 13 | "Windows applies 15 ms to ANY request, requested value irrelevant" (the preset hypothesis, §14.13) | **dead** - Windows honours the requested range and grants its **maximum** | §14.16, §14.36 |
| 14 | "The link never ran at 7.5 ms, on any build" | **retracted** - `UPi6` + SUCCESS. It did. §9's original account is restored | §14.21 |
| 15 | "§9's 133-connection-events-per-second arithmetic is void" | **itself void** - 7.5 ms is confirmed, so the arithmetic is restored. **But see the live problem in §0.4** | §14.21 |
| 16 | "Modal burst-to-burst spacing measures the connection interval" | **corrected** - it measures **events-per-burst x interval**. Use the **bursts/s <= events/s** constraint instead | §14.21, §14.32 |
| 17 | "`(12,24)` is the fix" | **SUPERSEDED** - it contains the measured-fatal 15 ms and cannot correct a host that has persisted it. Use `(20,24)` | §14.30, §14.32, §14.33 |
| 18 | "Windows' interval choice is deterministic (4/4 identical)" | **reinterpreted** - that was **inertia, not independence**: Windows persists the negotiated interval per device (n=3), and all four runs had inherited 30 ms | §14.23, §14.35 |
| 19 | §11.5's watchdog "decide - judgement call" | **superseded: KEEP it** | §13.4 |
| 20 | Published Win10 thresholds ("won't accept >20 ms, won't go below 15 ms") | **do not apply to this host** - Windows 11 10.0.26200 granted both 7.5 ms and 28.75 ms with SUCCESS | §14.24 |

### §0.4 THE ONE BIG THING STILL UNKNOWN: *why* a short interval is fatal

**§9's mechanism is in worse shape than when the investigation started, and this should not be glossed
over.** Its story was `_pendingPkt` saturation driven by 133 connection events/s at 7.5 ms. But **15 ms
is only 66.7 events/s and kills just as reliably** (2/2). So whatever the mechanism is, it bites at half
the rate that explanation requires, and no account currently covers both rungs.

Also open: where between 15 and 28.75 ms the threshold sits; whether 25 ms (which `(20,24)` permits) is
safe or merely untested - only 28.75 and ~30 ms have run long, and `(23,24)` is the tighter alternative
if that ever matters.

### §0.5 METHOD NOTES worth keeping - they cost us real time

1. **A survival only counts if it exceeds the longest failure ever seen in that condition** - and
   comfortably, not marginally. `(6,6)`'s failures reach **789 s**, so the 637.7 s run that looked like a
   counter-example was a 5.2% draw, expected about once in 13 attempts (§14.25).
2. **Never trust a banner alone - cross-check the CSV.** `UPi` is one sample from the first ~2 s of a
   connection. Arm C's banner said 15 ms while it actually ran at 28.75 ms (§14.16).
3. **To read the interval from a CSV, use the hard constraint `bursts/s <= connection events/s`**, not the
   modal gap: 2 x 15 ms and 1 x 28.75 ms are degenerate and that degeneracy misled us once (§14.32).
4. **GUI disconnect timestamps overstate time-to-failure by a constant 9.6 s** - the supervision timeout.
   Negligible for long runs, enormous at the short end (§14.x).
5. **Windows persists the negotiated interval per device**, so **test order matters** and consecutive runs
   of the same build are not necessarily the same experiment (§14.23, §14.29).

### §0.5b B23 VALIDATION RUN - 32.1 min clean, and NO mid-session drift

`(20,24)`, ended by the operator at **1,926.6 s**, 184,407 rows.

| metric | value |
|---|---|
| mean rate | **95.7 Hz** - the highest of any relaxed run (`(12,24)` managed 88.2) |
| gaps > 100 ms | **3**, totalling 0.6 s = **0.03%** of the run |
| per-second rate | median 96 Hz, min 54, **1,925 / 1,927 seconds at >= 80 Hz** |

Single-run p ~ **1 in 1,256**. Pooled relaxed exposure **8,002 s, zero failures** against 29.6 expected,
p ~ **1.4 x 10^-13**.

**§14.37's residual risk did NOT materialise.** Interval stability by quarter:

| quarter | bursts/s | ~30 ms gap band |
|---|---|---|
| Q1 0-8 min | 37.73 | 49.9% |
| Q2 8-16 min | 37.89 | 49.0% |
| Q3 16-24 min | 35.98 | 49.9% |
| Q4 24-32 min | 35.54 | 51.6% |

Flat across 32 minutes, so **Windows did not drift the interval down mid-session.** This is a *within-run*
comparison using one method throughout, so it is unaffected by the calibration problem below.

### §0.5c CORRECTION: the "bursts/s <= events/s" test is NOT a hard constraint

**§0.5 item 3 and §14.32 both call this a hard constraint. That was overstated, and this corrects it.**

B23's 37.7 bursts/s **exceeds the 33.3 connection events/s available at the 30 ms its banner reported** -
which is arithmetically impossible, so the method, not the banner, is wrong. The cause is the burst
detector itself: it splits bursts at an 8 ms gap, and at ~2.9 samples per connection event the host's
timestamp spread *within* one event can exceed 8 ms and be counted as two bursts.

**Direction of the bias:** over-counting bursts inflates `bursts/s`, which makes the derived
`interval <= 1000/bursts_per_sec` **too tight**. So every exclusion stated from it is less firm than
written. The "~15 ms gap population" discriminator has the same weakness - B23 shows 13.5% in that band in
Q1, and a genuine 15 ms gap cannot exist at a 30 ms interval, so part of that band is splitting artefact.

**What this does NOT change:**

- **The ladder in §0.1 is unaffected.** It rests on the banner's **HCI event fields** (`UPi6` = 7.5 ms,
  `UPi12` = 15 ms, `UPi24` = 30 ms), read directly off LE Connection Update Complete. Direct measurements,
  not estimates.
- **Arm C's transient is still correctly diagnosed** (§14.16). That rested on a large *distributional*
  difference - 2.6% vs 12-22% in the 15 ms band, 62% vs 45-50% at 30 ms - not on the precise ceiling.
- **Within-run drift checks remain valid**, since they compare the same method against itself.

**Use it as a corroborating instrument, never as a primary measurement.** If an absolute interval is ever
needed to better than a factor of ~1.3, the banner's `UPi` is the measurement and a sniffer (§14.6) is the
arbiter. A better CSV estimator would need the burst threshold calibrated against a known interval first.

### §0.6b DEBUG-FEATURE AUDIT - what to keep and what to discard

**Added 2026-09-12 at the operator's request**, now that the root cause is known. This supersedes the
inventory in §11.1, which was written before the cause was found and before upstream issue #45 was known.
Ordered by verdict, not by file.

#### KEEP PERMANENTLY - these are fixes or safety, not scaffolding

| item | where | why it stays |
|---|---|---|
| `setConnectionInterval(20, 24)` + the `EXO_BLE_INTERVAL_SEL` selector | `SystemReset.h`, `ExoBLE.cpp` | **The fix.** Keep the selector too: it documents the measured ladder and makes a re-test one character |
| Bounded wait in `sendAclPkt` | sketchbook `ArduinoBLE/HCI.cpp` | Upstream bug open since 2019 (§13). **Report upstream** - a sketchbook patch dies on every library update |
| Hardware watchdog + boot-loop guard | `SystemReset.*`, `ExoCode.ino` | Only thing that recovers a stopped main loop. §11.5's "decide" is superseded by §13.4 |
| Link-stall detector + 2 s ping + TX busy counter | `ExoBLE.cpp`, `QtExoDeviceManager.py` | Only thing that recovers presentation A - **and now the only cover for a mid-session interval drift, which the fix structurally cannot reach** (§14.37) |
| CSV exo-time unwrap | `MainWindow.py` | Unrelated four-month-old bug (§10.2) |

#### KEEP FOR NOW - diagnostics that are still earning their keep

| item | why not yet |
|---|---|
| `CPi` / `UPi` connection-parameter readout (3 patches in sketchbook `HCI.cpp`) | The *why* is still unknown (§0.4), the fatal threshold is unlocated, and §14.37's mid-session drift is unproven. **This is the instrument that would detect all three.** Revisit once a mechanism is established |
| `EXO_FW_TAG` build tag | Costs nothing and settled "which binary is on the board" repeatedly. §11.3 already suggested keeping it |
| The 9.6 s supervision-timeout knowledge | Not code - but **`EXO_BLE_STALL_MS` (8 s) is deliberately shorter than it**, which is why failures show `BLESTALL` rather than a stack disconnect. Do not change one without the other |

#### DISCARD - purpose served, or never worked

| item | where | verdict |
|---|---|---|
| **Stage breadcrumbs** | `SystemReset.*`, `ExoCode.ino`, `ExoBLE.cpp` | **Never worked at all** - no store survived a watchdog reset (B14 proved `.noinit` is zeroed). Pure dead weight. Remove first |
| Send-path diagnostics (`s_max_write_us`, `_w`/`_s`/`_FAIL`) | `ExoBLE.cpp` | Did its job: `w10` proved the bounded spin fires. **Careful:** `exo_wdt_stage_record()` also latches this, so remove them together (§11.3 item 4) |
| Teensy RT-I2C counters (`rt_i2c_stats`) | `RealTimeI2C.*` | Cleared I2C as a cause, and **`f`/`e` turned out to be untrustworthy** (ledger #9). `c` and `x` were useful but have nothing left to settle |
| Link-stats UART command | `uart_commands.h`, `ExoCode.ino`, `SystemReset.h` | Carrier for the above; goes with it |
| `RT_BLE_FORWARD` bisect flag | `Config.h`, `ComsMCU.cpp` | Experiment over. Confirm `REAL_TIME_I2C` stays **1** |
| `SD_LOG_SELFTEST_TRIAL` | `SdLogger.h`, `ExoCode.ino` | Self-test scaffolding that halts in `setup()`. Verify it is **0** and consider removing the block |

#### DECIDE LATER - depends on the resync discussion

| item | note |
|---|---|
| Whether the stall detector should ALSO resync trial state | If `Nano-Reboot-Trial-Resync-TODO.md` is built, the detector's reboot becomes a recoverable event rather than a lost trial, which changes how aggressive 8 s should be |
| Whether to keep the ping at 2 s | Removing the ping removes the §10.1 collision hazard at its source, but disarms the detector (`s_ping_seen`). One unit, decide together |

**Sequence:** breadcrumbs -> send-path diag + `exo_wdt_stage_record()` -> I2C counters + link-stats ->
bisect flags. **One real trial per step**, per §11.6 - each touches the live BLE path, and a mistake there
looks exactly like the bug this investigation was chasing.

### §0.6 NOT YET DONE

- **Motor stress test at non-zero torque, on a different laptop.** The last gate before §11's removal plan
  can start. Everything above is zero-torque bench work on one (RF- and RAM-compromised) host.
- **Report the `sendAclPkt` bound upstream** on issue #45. A patch in the sketchbook dies on every library
  update; upstream is the only durable home for it.

## 1. The finding: it is a HANG (this is the part worth keeping)

Three sessions of instrumentation had failed to identify the freeze. 2026-09-10 settled it, on two
independent channels.

### 1.1 The reset register, read warm for the first time

Previously every recovery was a power cycle, and a true power-on clears both `RESETREAS` and
`GPREGRET` — so every reading ever taken described the recovery, not the freeze (see
`Nano-Reset-Pin-Spurious-Reset.md` for that confound).

This time the user pressed the Nano's **RESET BUTTON** instead. A pin reset is *warm*: both registers
survive. The reading came back **`0x00000001` — `RESETPIN` alone**.

The firmware zeroes `RESETREAS` on every boot (`SystemReset.h`, `exo_reset_reason_code()`), so any
reset occurring during the freeze would have left its bit set. None had:

| bit | meaning | present? | therefore |
|---|---|---|---|
| `LOCKUP` | CPU double-faulted into hardware lockup | no | no lockup |
| `SREQ` | software reset | no | `mbed_error_hook`'s `NVIC_SystemReset` never ran |
| `DOG` | watchdog | no | (expected — none was configured) |
| `0x0`/`PORBOR` | power-on or brownout | no (`0x1` is not `0x0`) | no brownout, no power loss |
| `RESETPIN` | pin reset | **yes** | the button, and nothing else |

**Nothing reset the MCU. It stopped and stayed stopped until a thumb.** The absent `CRASH_0x` record
is consistent: a hang never calls `mbed_error()`, so nothing is ever written.

### 1.2 The onboard LED, independently

`ComsMCU::local_sample()` calls `ComsLed::life_pulse()` on **every** `loop()` pass
(`ComsMCU.cpp:117`), which toggles the green and blue pins every 100 calls. Colours are binary
(`digitalWrite`, not PWM), pins 22/23/24, active-low (`Board.h:107-110`).

So a live Nano's LED **never shows a steady colour** — it alternates fast enough to blend:

| state | alternates between | looks like |
|---|---|---|
| config OK, advertising | green / blue | teal |
| config OK, connected | cyan / off | dim cyan |
| config timeout, advertising | yellow / magenta | orange-pink |
| config timeout, connected | **white / red** | **pinkish white** |

At the freeze the LED **froze on one endpoint of the toggle** — white on one freeze, **red** on the
next. Random phase, which is exactly what a stopped 50/50 toggle looks like; a meaningful *state*
colour would be the same every time. That means `life_pulse()` stopped, which means `local_sample()`
stopped, which means **`loop()` stopped.**

### 1.3 Scope of the stop

`Mid-Trial-Freeze-Nano-Radio-Silence.md` established that ArduinoBLE runs Cordio in its own RTOS
thread, so a stalled `loop()` alone leaves the BLE link **alive** — frozen plots, no disconnect. This
freeze produced the 9.6 s supervision timeout, so the Cordio thread stopped too.
**Both threads. MCU-wide stop, with no fault and no reset.**

### 1.4 Ruled out along the way — do not re-chase these

- **Not the Teensy, not power loss.** The user confirmed the exo kept assisting normally through the
  freeze. The Teensy drives the motors independently of the Nano, so "the exo still works" clears the
  Teensy and rules out a dead battery, but says nothing about the Nano.
- **Not the host or GUI.** The trial CSV shows the host *healthiest* in its final 15 s window
  (largest receive gap 0.082 s, versus 0.211 s earlier in the run) with the wall-vs-exo lag returning
  to −25 ms rather than running away. No backlog, no progressive stall. "The GUI froze then
  disconnected" is the plots stopping when data stopped, then the 9.6 s timeout firing.
- **Not the actuation load, on this evidence.** The 103 s before the freeze was electrically violent
  — the PID term exceeded the setpoint in 86 % of samples and pinned the right joint to the 30 Nm
  clamp for over 5 % of the window, off an 11 Nm setpoint — but that is a separate (real) problem,
  and the LED/RESETREAS evidence points at a software hang, not a brownout. Time-to-death was ~316 s
  here versus 606/609 s in earlier stress runs; **n=1**, worth an A/B someday, not a conclusion.
- **Config timeout is NOT the cause.** `get_config` does appear to time out (the red channel is lit,
  and red is set in exactly one place — `ExoCode.ino:821`, the timeout branch), leaving the Nano's
  `ExoData` built from a zeroed config. But `ComsMCU` has **zero** references to `get_joint_with`,
  `is_used` or joint IDs, and every BLE handler sets only a local shadow flag before forwarding the
  command to the Teensy unconditionally — so a zeroed config is both invisible in normal operation
  *and* has no surface on which to wedge the loop. Worth fixing on its own; not this bug.

---

## 2. Why a watchdog, and why it need not know where the bug is

The WDT counts off the 32.768 kHz LFCLK, independently of the CPU. Code must periodically feed it; if
it is not fed within the timeout, the chip resets. **You do not install it at the fault site** — you
install one feed on a path known to be healthy, and it fires when anything, anywhere, stops that path
being reached. §1.2 proved which path that is.

This is precisely why it succeeds where every software timeout failed: a software timeout has to be
executed by the very CPU that has stopped executing.

**A watchdog reset is warm**, so `DOG` lands in `RESETREAS` and `GPREGRET`/`GPREGRET2` survive —
breaking the loop where recovery required the power cycle that erased the evidence.

---

## 3. What was added

### 3.1 `ExoCode/src/SystemReset.h`

| # | Change | Notes |
|---|---|---|
| 1 | Moved the four `EXO_CRASH_*` defines **above** the new watchdog block | Cosmetic but load-bearing: the watchdog's boot-loop guard uses `EXO_CRASH_MAGIC_MASK`/`_COUNT_MASK`, and macros must be defined before use |
| 2 | Forward declaration `inline void exo_crash_record(uint8_t*, uint8_t*);` | So `exo_wdt_boot_count()` can latch the crash record before overwriting `GPREGRET` |
| 3 | The `HARDWARE WATCHDOG` block | `EXO_HAVE_WDT`, `EXO_WDT_MAGIC 0xD0`, `EXO_WDT_MAX_AUTO_RESETS 3`, `EXO_WDT_TIMEOUT_S 5`, `EXO_WDT_RELOAD_MAGIC`, `EXO_WDT_LFCLK_HZ`, the six `EXO_STAGE_*` codes, and `exo_wdt_stage()` / `exo_wdt_stage_record()` / `exo_wdt_feed()` / `exo_wdt_boot_count()` / `exo_wdt_start()` |
| 4 | Three edits inside `exo_crash_record()` | `(void)exo_wdt_stage_record();` at the top; a `latched_marker_raw` copy; the else-branch now only zeroes `GPREGRET` when it is not holding the dog count |
| 5 | A `DOG` block in `exo_reset_reason_string()` | Appends `,STAGE_n_dogN` |

`exo_wdt_start()` configuration: `CRV = 5 * 32768 - 1`, `RREN` = reload register 0 only,
`CONFIG = SLEEP:Run | HALT:Pause`. **`SLEEP=Run` is essential** — mbed idles the core between events,
so without it a hang that parks in sleep would never trip the dog, which is most of them.
`HALT=Pause` stops a debugger single-step from resetting the board.

### 3.2 `ExoCode/ExoCode.ino`

| # | Change | Where |
|---|---|---|
| 6 | `#include "src/SystemReset.h"` | Nano block, after `#include "src/ComsLed.h"` |
| 7 | `exo_wdt_start();` | **End** of the Nano's `setup()` |
| 8 | `exo_wdt_feed()` plus six `exo_wdt_stage()` calls | Nano `loop()`, wrapping the five `mcu->...()` calls |

**Why it arms last (#7):** boot spends up to 18 s in `readSingleMessageBlocking()`
(10 s `kReadyTimeoutMs` + 8 s `kReceiveTimeoutMs`) and up to 8 s in `get_config()`, with nothing
feeding. Arming earlier would reset the board mid-boot, **forever**. The nRF52840 WDT has no stop
task — once started it cannot be turned off, which is why the arming point matters.

**Why the feed is in `loop()` and not a callback (#8):** Cordio's RTOS thread survives a stalled
`loop()` (§1.3). Feeding from anything that outlives the hang would defeat the entire mechanism.

### 3.3 Three bugs found and fixed while integrating with the existing crash trap

1. **The watchdog had no boot-loop guard, and it defeated the crash trap's own.**
   `mbed_error_hook` deliberately gives up after `EXO_CRASH_MAX_AUTO_RESETS` so a reproducible fault
   leaves the device wedged-but-visible rather than thrashing — but a dog simply reboots the halted
   mbed, and a Nano that hangs on every boot would reboot forever with nothing counting.
   Fixed: `EXO_WDT_MAGIC 0xD0` count in `GPREGRET`; after 3 consecutive `DOG` reboots
   `exo_wdt_start()` **stops arming**, so the device stays up and connectable and the breadcrumb can
   actually be read instead of being rebooted away every 5 s.
2. **The dog counter would have eaten the crash record.** `exo_wdt_boot_count()` runs at the end of
   `setup()`; `exo_crash_record()` runs later, from `ExoBLE::setup()`. On a fault -> mbed halt -> dog
   reboot, the counter would have overwritten `GPREGRET` before anything read the `CRASH_0x` code.
   Fixed by latching the record first (hence the forward declaration).
3. **`exo_crash_record()` would have wiped the dog counter.** Its else-branch zeroes `GPREGRET` on
   every boot with no crash magic — which is *every watchdog boot* — silently disarming the guard on
   its first use. Fixed with the `latched_marker_raw` test.

### 3.4 Verified safe against false trips

The End-Trial reset is a **non-blocking state machine** (`ComsMCU::_maybe_system_reset()`): the 3 s
`_reset_ack_timeout_ms` and 300 ms `_reset_flush_ms` are elapsed-time checks, not blocking waits, so
`loop()` keeps iterating and feeding through the whole sequence. `update_UART` polls at
`CONT_MCU_TIMEOUT` = 1000 us. 5 s is large headroom.

**A watchdog that fires on healthy code is worse than none.** First flash: connect, run a short
normal trial, confirm no surprise reboots. If a spurious `DOG` appears, the `STAGE_n` in the banner
localizes it immediately.

---

### 3.5 The BLE link-stall detector - SECOND failure mode, and it spans the GUI

**2026-09-10, later the same session: a second, distinct failure was caught in the act.** The GUI
auto-disconnected, but the Nano's **RGB was still blinking and the green LED_PWR still flashing** -
`loop()` alive AND RT data still arriving over I2C (`_life_pulse()` at `ComsMCU.cpp:197` is called
*only* inside the `if (new_rt_data)` branch). The user could **not** reconnect, and the RGB **did not
change colour** at the drop - so `advertising_onoff(0)` never ran, so `BLE.connected()` never
changed, so **the Nano never noticed the disconnect.** A button reset recovered it.

Two presentations:

| | `loop()` | RT data | LED | re-advertises? |
|---|---|---|---|---|
| **Mode 1 - hang** | stopped | stopped | frozen SOLID on a toggle endpoint | no |
| **Mode 2 - link stall** | alive | alive | still blinking, unchanged | no |

**Likely one root cause.** ArduinoBLE 2.1.0 - the sketchbook copy that actually builds, *not* the
vendored 1.2.1, see `arduino-toolchain-on-this-pc` - at `HCI.cpp:636`:

```cpp
int HCIClass::sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data)
{
  while (_pendingPkt >= _maxPkt) {   // no timeout, no bound, no escape
    poll();
  }
  ...
  _pendingPkt++;
```

`_pendingPkt` is decremented **only** by `handleNumCompPkts()` (`HCI.cpp:861`), driven by the
controller's "Number of Completed Packets" event. If those stop arriving it climbs monotonically.
Below `_maxPkt` you get **mode 2** - `loop()` runs, nothing reaches the host, the disconnect is never
processed. At `_maxPkt` you get **mode 1** - the next send spins forever inside `loop()`. `_maxPkt` is
single-digit on the nRF52, so which one you see is a race, **which is why the trigger has never been
reproducible.** A spin is also not a fault, which is exactly why `RESETREAS` read `RESETPIN` alone
with no crash record. **Strong hypothesis that fits every observation - `_pendingPkt` was never
directly measured.**

**The detector does not depend on that hypothesis being right.** It asks only "can the GUI still
reach us", which is the actual user-visible failure whatever causes it. From inside the firmware
nothing looks wrong in mode 2 - `sendAclPkt()` returns 0, `writeValue()` succeeds, our queue drains -
so the only trustworthy signal is end-to-end, and that requires the GUI to say something
periodically. Hence the ping, and hence this change touching `Python_GUI/`.

| # | Change | Where |
|---|---|---|
| 9 | `EXO_STALL_MAGIC 0xE0`, `EXO_BLE_STALL_MS 8000`, `exo_ble_stall_reset()`, `exo_stall_record()`, `,BLESTALL_n` reporting, and the stall magic added to the `exo_crash_record()` preservation test | `SystemReset.h` |
| 10 | `ble_names::ping = 'p'` plus its `{ble_names::ping, 0}` row in the command table | `ble_commands.h` |
| 11 | `case ble_names::ping: break;` - a no-op, present only so the parser does not log an unknown command every 2 s | `ComsMCU.cpp` |
| 12 | `s_last_rx_ms` / `s_ping_seen` file statics; RX stamped in `on_rx_recieved()` **before** parsing; the detector in `handle_updates()`, placed **before** the unchanged-status early return; clock re-armed on connect | `ExoBLE.cpp` |
| 13 | `pingDevice()` - fire-and-forget `b"p"`, silent on both success and failure | `Python_GUI/services/QtExoDeviceManager.py` |
| 14 | `self._ble_ping_timer`, 2000 ms, wired to `qt_dev.pingDevice` | `Python_GUI/MainWindow.py` |

**Fail-safe by design:** the timeout is enforced **only after a ping has actually been seen**
(`s_ping_seen`). Connecting with a GUI, or any other client, that does not ping can never reboot the
board. 2 s ping against an 8 s timeout is four missed pings, deliberately under the ~9.6 s host-side
supervision timeout so recovery starts before the host even declares the link gone.

**It leaves a trace, and that is the point.** `exo_ble_stall_reset()` writes `EXO_STALL_MAGIC` and
warm-resets, so `GPREGRET` survives and the next banner reads **`,BLESTALL_n1`**. That is positive
confirmation from the device that the link died while it was still running - it turns the diagnosis
above from "fits the evidence" into "the detector fired". Checked ahead of the DOG and plain SREQ
branches so it is never confused with an End-Trial reboot, the other thing that produces `SREQ`.

**Version-skew note, matters when dismantling:** new firmware + old GUI is safe - no pings, so
`s_ping_seen` stays false and the detector never arms. Old firmware + new GUI is harmless but noisy:
`BleParser::_handle_command` logs `Command is not in list: p` every 2 s. **So remove the GUI half
first, or both at once.**

---

### 3.6 FIRST BENCH RESULT, and the two bugs it exposed

**2026-09-10, first flash.** Mid-trial the GUI reported `link lost`. The LEDs showed mode 2 - RGB
still blinking, green LED_PWR still flashing - and then, without any intervention, **the device came
back and the GUI reconnected.** That had never happened before; every previous disconnect in this
investigation required a power cycle, which is precisely what kept destroying the evidence.

Banner on reconnect:

```
NANO LAST RESET REASON
 RESETREAS = 0x00000002  (DOG,STAGE_0_dog1)
```

So the watchdog fired and recovered the board. **But `STAGE_0` is the "no breadcrumb recorded"
value** - the loop stamps 1-6 and never 0 - so the dog did not bite during a normal loop phase. Two
bugs, both mine, both in this change:

**Bug A - the dog was armed before anything fed it.** `exo_wdt_start()` was called at the end of
`setup()`, but the first `exo_wdt_feed()` is on the first `loop()` pass *after* the one-time
constructions:

```
exo_wdt_start();        // END of setup() - ARMED here
loop() pass 1:
  new ExoData(...)      // unfed
  new ComsMCU(...)      // unfed - ExoBLE::setup() runs in here: BLE.begin(),
                        // GATT registration, advertising - AND exo_crash_record(),
                        // which ZEROES GPREGRET2
  exo_wdt_feed();       // first feed, only now
```

BLE initialisation runs in that window with nothing feeding the dog, and `exo_crash_record()` blanks
the breadcrumb inside the same window - hence exactly `DOG,STAGE_0_dog1`. **That was a false trip on
the boot path, not the hang we are hunting.**

*Fix:* `setup()` now only latches the previous boot's breadcrumb
(`(void)exo_wdt_stage_record();`), and `exo_wdt_start()` moved to the **end of `loop()`**, so the dog
is armed only after one complete pass has proven the loop can finish. `exo_wdt_start()` returns early
on `RUNSTATUS`, so calling it every pass costs one register read. A new `EXO_STAGE_ARMED (7)` is
stamped at arm time, so a boot-path trip is now identifiable instead of showing as a bare `STAGE_0`.

**Bug B - a watchdog reset erased any pending BLESTALL record.** `exo_wdt_boot_count()` overwrites
`GPREGRET` with the dog magic, and it deliberately latched the *crash* record first (§3.3 item 2) -
but the **stall** record lives in the same register and was never latched. So the plausible sequence
here (link stalls -> detector fires and warm-resets -> boot-path false trip -> dog reset) would
silently destroy the `BLESTALL_n` marker, which is the one piece of evidence the detector exists to
produce. **We may well have lost a genuine mode-2 confirmation to this.**

*Fix:* `exo_wdt_boot_count()` now calls `exo_stall_record()` alongside `exo_crash_record()` before
touching `GPREGRET` (forward-declared for ordering, same pattern as the crash record).

**Also fixed:** `MainWindow.py`'s `DOG` explanation still read *"but this firmware configures no
watchdog. Investigate."* - true when written, wrong now. It now explains the reset and points at the
`STAGE_n` field.

**Bug C - THE WATCHDOG SURVIVES A WARM RESET, and the boot path cannot outlive it.**
Found after reflashing with the Bug A/B fixes and getting `DOG,STAGE_0_dog1` *again*, which the fixed
code should have made impossible. It was not impossible, and the banner said why:
`exo_wdt_start()` early-returns on `RUNSTATUS` **before** it stamps `EXO_STAGE_ARMED`, and that early
return is the only remaining path that leaves the breadcrumb at 0. So the dog was **already running
at boot**.

The nRF52840 WDT is **not cleared by `NVIC_SystemReset`, nor by a pin reset, nor by a re-flash** -
only by a power-on reset, or by the dog itself firing. It was assumed to start clean. It does not.
Consequence:

```
link stalls -> stall detector NVIC_SystemReset  ->  WDT STILL RUNNING, 5 s window
            -> boot path: readSingleMessageBlocking up to 18 s
                          + get_config up to 8 s      <- NOTHING FEEDING
            -> dog bites at 5 s -> DOG, breadcrumb blank -> STAGE_0
                                -> and exo_wdt_boot_count() overwrites the BLESTALL marker
            -> THAT reset finally clears the WDT, so the next boot comes up clean
```

So **every warm reset cost an extra reboot and destroyed the evidence it was supposed to carry** -
which is exactly the observed `DOG,STAGE_0_dog1`, repeatedly. Flashing does the same thing, because
the upload reset also leaves the old dog running.

*Fix:* feed the dog everywhere the boot path can block longer than the window. `exo_wdt_feed()` is one
register write and is harmless when no watchdog is running, so these are unconditional:

| site | why |
|---|---|
| `GetBulkChar.cpp`, the `while (!Serial1.available())` 'R' loop | blocks up to `kReadyTimeoutMs` = 10 s |
| `GetBulkChar.cpp`, the `while (!messageComplete)` loop | blocks up to `kReceiveTimeoutMs` = 8 s |
| `uart_commands.h`, the `get_config()` `while (1)` retry loop | blocks up to `CONFIG_TIMEOUT` = 8 s |
| `ExoBLE.cpp`, around `BLE.begin()` and after GATT registration | longest single block on the rest of the boot path |
| `ExoCode.ino`, around the `new ExoData` / `new ComsMCU` constructions in `loop()` | these run on pass 1 before the loop reaches its normal feed |

**Lesson worth keeping even if all of this is dismantled:** on nRF52840, once the watchdog has ever
been started, *every* code path that can block longer than the window must feed it - including the
whole boot path - because a warm reset does not give you a clean slate.

**What this run does and does not establish.** Establishes: the watchdog works, fires on real
hardware, and recovers the board warm with the evidence intact - the core dead-end of this whole
investigation is broken. Does **not** establish anything about the hang itself: `STAGE_0` was a
boot-path false trip, so the next run with the §3.6 fixes is the first one whose `STAGE_n` will
actually mean something.

---

### 3.7 MODE 2 CONFIRMED BY THE DEVICE - 2026-09-11

```
2026-09-11 13:11:31 | Reset reason characteristic read: 'RST:0x00000004:SREQ,BLESTALL_n1'
```

**This is the result the whole exercise was for.** `BLESTALL_n1` is written only by
`exo_ble_stall_reset()`, so the Nano is reporting, in its own words, that it stopped hearing the
GUI's 2 s pings for 8 s **while its own stack still reported `BLE.connected()` as true**, and
rebooted itself. Mode 2 is real, and it is not an inference any more.

The `SREQ` is expected - our detector calls `NVIC_SystemReset()` deliberately. An actual Mbed fault
would have printed `CRASH_0x..` instead (that branch takes priority in `exo_reset_reason_string()`).

Timeline, and every number lands where the model says it should:

| time | event |
|---|---|
| 13:10:13.7 | trial starts |
| ~13:10:53.9 | Nano stops transmitting (back-calculated from the 9.6 s supervision timeout) |
| ~13:11:01.9 | 8 s of ping silence -> `EXO_BLE_STALL_MS` expires, warm reset |
| 13:11:03.5 | GUI logs `link lost` - **after** the Nano had already rebooted |
| 13:11:29.8 | reconnect, banner intact |

The ~28 s from reset to reconnect is the ~18 s blocked bulk read (§6) plus the scan - the predicted
boot cost.

**Three fixes validated at once by this single line:**
- the **stall detector works** and fires on a real mode-2 event;
- **Bug B's fix held** - the `BLESTALL` marker survived to be read;
- **Bug C's fix held** - there is **no `DOG`** on the line, so the watchdog that survives a warm
  reset no longer bites its way through the boot path and overwrites the evidence.

**Also fixed here:** `MainWindow.py`'s `SREQ` text still read *"unexpected otherwise - could be an
Mbed fault handler reboot"*, which led the user to suspect a crash on seeing this. It now enumerates
the three legitimate `SREQ` sources (`BLESTALL_n` = recovery working as designed, `CRASH_0x` = real
Mbed fault, bare `SREQ` = End Trial) and says which case is actually unexplained.

**What is still NOT known:** *why* the link dies. Mode 2 is confirmed as a phenomenon; the
`_pendingPkt` spin (§3.5) remains the leading explanation and is still unmeasured. What has changed
is that the failure is now survivable and self-reporting instead of a dead end.

---

### 3.8 2026-09-11, second half: the failure characterised (and one retraction)

The single most important result of the whole investigation, and it reframes everything above.

**The evidence, from one run (B15, 626 s trial, banner
`DOG,STAGEn0_g0_dog1,B15_b1,I2Cf64k_e278x10_c2_t266d_w0d_x300`):**

| field / observation | reading |
|---|---|
| `f64k` | 64,000 RT frames over a 626 s trial = ~100 Hz. The counters are sound |
| `w0d` | worst gap between successful I2C sends **under 100 ms** for the whole trial |
| `x300` (capped) | then **>=300 consecutive failures** - at 100 Hz, **>=3 s of unbroken silence** |
| `c2` | address NACK: **the Nano stopped acknowledging on the bus** |
| user observation | onboard RGB **solid red** AND green LED_PWR **solid, not flashing** - both frozen |

`w0d` next to `x300` is the important pairing: the link was perfectly healthy to the last moment and
then fell off a cliff. Not a degradation of the chronic loss - a distinct, total, unrecovered stop.
`w` stayed 0 because `worst_gap` only updates on a SUCCESS, and there was never another one.

**!! RETRACTED, same day - see the correction immediately below. !!** This section originally argued
that the Nano must have "stopped servicing interrupts entirely", on the reasoning that the I2C slave
ACK is generated by the peripheral and its ISR, so a stalled `loop()` could not stop it.

**That reasoning was wrong, and it was written before reading the core's `Wire.cpp`.** On
`mbed_nano` 4.6.0 the I2C slave is **`arduino::MbedI2C::receiveThd()` - a `while(1)
{ slave->receive(); ... }` polling RTOS THREAD**, not an interrupt handler. So ACKs stopping requires
only that **thread** to be starved of CPU. No interrupt-level stop is needed, and there is no evidence
for one. Every conclusion in this document that rests on "interrupts were disabled" should be read as
**"the main thread stopped yielding"** instead.

**This also explains why every instrument came back empty**, and that line is now closed:

| store | result | conclusion |
|---|---|---|
| `GPREGRET2` | `g0` every time, with a known non-zero stage in flight | wiped by a **watchdog** reset (it survives SREQ fine - BLESTALL proves that) |
| `.noinit` RAM | `b1` on every boot from the boot counter added in B14 | the mbed linker does not honour the section; the variable lives in `.bss` and is zeroed at startup |
| the LEDs | frozen | die with the interrupts |

**THE CORRECTED MODEL (2026-09-11 evening).** One root cause, two presentations, no interrupt-level
stop required:

> The **BLE controller stops completing packets**. That simultaneously (a) kills the link, so the host
> hits its ~9.6 s supervision timeout and disconnects, and (b) stops `_pendingPkt` from ever
> decrementing, so it saturates and the main thread enters ArduinoBLE's unbounded
> `while (_pendingPkt >= _maxPkt) { poll(); }` (`HCI.cpp:636`) and never leaves. A main thread that
> never yields freezes the LEDs (`life_pulse()` lives there) and starves `receiveThd`, which stops
> servicing the TWIS - so the Teensy sees address NACKs.

The spin is therefore the **amplifier**, not the root: it turns "the BLE link died" into "the whole
Nano looks dead". And the two observed modes are the same failure caught at different stages:

| mode | when it was caught |
|---|---|
| **2** (`BLESTALL`, loop alive, RT still flowing) | BEFORE `_pendingPkt` saturated |
| **1** (`DOG`, everything frozen solid) | AFTER it saturated |

That is what a filling queue looks like, and it explains why the presentation seemed random.

This also resolves the objection that killed the naive version: `Mid-Trial-Freeze-Nano-Radio-Silence.md`
established that a stalled `loop()` ALONE leaves the link alive (frozen plots, no disconnect), yet a
disconnect always happens. Putting the root failure at the controller - one level below both threads -
produces the disconnect and the freeze from a single cause.

**So ignore every `STAGEn`/`g` value in this document's history** - the breadcrumb never had a working
home. Three builds went into that store. Do not build a fourth without first proving the store
survives, which is what the `b<n>` boot counter is for.

**A separate real finding: the RT I2C link is chronically lossy.** Error rates of 4.3% (B15, 2,780 of
64,000) and 13.6% (B14, 2,850 of 21,000) in *healthy* operation, all address NACKs, at the Wire
default of 100 kHz on both sides (nothing calls `setClock`). That independently matches the
**6-13% RT sample loss** measured months ago in `Mid-Trial-Freeze-Nano-Radio-Silence.md` from a
completely different angle, and is very likely its mechanism. Arithmetic that makes it plausible:
~28 bytes at 100 kHz is ~2.5 ms per transfer arriving every 10 ms, a 25% bus duty cycle, so any gap
in the slave re-arming between transfers collides. **This is worth fixing on its own merits and is
probably not the disconnect bug** - the chronic loss is scattered singles (`w0d`), the failure is one
unbroken run of hundreds.

**Ruled out this session (do not re-chase):**

- **The 2 s GUI ping is not the fix.** One 13-minute run (past the 609 s ceiling of 86 prior trials)
  looked like a breakthrough; the very next run with the ping still on died at 191 s. Variance, and
  an over-read of a single sample.
- **Not board-specific hardware** - reproduced across **3 different Nano boards**.
- **Not the host** - already established from CSV timing (§1.4), and unchanged since.
- **Not the RT payload rate alone** - failures span 40 s to 13 min with identical traffic.

**Numbers worth keeping.** Time-to-failure across this session: 40 s, 1.9 min, 2.5 min, 3.2 min,
4.3 min, 10.4 min, 13.2 min. Enormous spread, no visible trend against torque, trial length, or
whether motors were engaged. Any theory has to explain that variance.

**The bisect build (B16).** `REAL_TIME_I2C` forced to **0** in `Config.h`, which stops the Teensy
pushing RT frames over I2C and stops the Nano forwarding them over BLE. Chosen over lowering the RT
rate because runs already reach 10+ minutes, so a proportional stretch would be indistinguishable
from "fixed" in any practical window - a clean yes/no is worth more than a scaling curve.
**Expected while running it:** no plots, no CSV data, and the green LED_PWR permanently solid (it is
toggled only inside the `if (new_rt_data)` branch). That is the experiment working, not a new fault.
The banner carries **`rt0`** so the build is never in doubt. The UART fallback for RT is dead
(`uart_commands.h:695` - `rt_data::float_values` is `static` in a header, so writer and reader are
different translation units), which is why this removes the data rather than rerouting it.
**If it runs indefinitely the RT path is implicated; if it still freezes, it is not.**

**Leading root-cause hypotheses, none confirmed.** In order of fit to "interrupts stop being
serviced, both threads die, correlated with BLE activity, wildly variable time-to-failure":

1. **A spin or deadlock in the BLE stack that starves everything.** ArduinoBLE 2.1.0 has a known
   unbounded spin - `while (_pendingPkt >= _maxPkt) { poll(); }` at `HCI.cpp:636` - in exactly the
   subsystem involved. If that is ever entered while holding an RTOS lock the Cordio thread needs,
   or from a context with interrupts masked, both threads stop and only the hardware WDT recovers.
   Still unmeasured: `_pendingPkt` is private and reading it means patching the sketchbook library.
2. **A high-priority ISR that never returns.** The radio ISR outranks everything on this chip; if it
   wedges, I2C and the main loop both starve while the CPU is still technically executing - which is
   exactly the observed signature.
3. **Stack overflow corrupting RTOS state.** mbed threads have fixed stacks. Would not necessarily
   produce a clean fault, and a corrupted scheduler could stop both threads. No `LOCKUP` was ever
   seen in `RESETREAS`, which argues against a double fault but not against silent corruption.

**The definitive tool is SWD**: attach a probe, let it freeze, halt the CPU and read the PC. Every
indirect method tried so far dies along with the MCU. Not currently available - the Nano's SWD pads
are not easily reachable on this build - which is why the bisect is the chosen path.

---

### 3.9 2026-09-11 evening: the bisect result, and a re-diagnosis from evidence only

**All raw observations now live in `Nano-Disconnect-EVIDENCE-ONLY.md`.** That file was created
because this one had accumulated several confident explanations that turned out to be wrong, and it
had become hard to tell measured facts from inferences. Read it first; this section is interpretation.

#### The bisect

| build | RT over I2C | RT over BLE | result |
|---|---|---|---|
| baseline | yes | yes | **failed every run**, 50-789 s across 9 runs |
| B16 `rt0` | no | no | **~40 min, no failure** |
| B17 `rt1_fw0` | **yes** | no | **~30+ min, no failure** |

Round 2 is the single-variable test: I2C was fully restored (user confirmed the green LED_PWR
blinking throughout, which only happens inside `if (new_rt_data)`), and **nothing changed**.

**Conclusion, and it is the strongest thing in this investigation: the BLE notification stream is a
NECESSARY CONDITION for the failure. I2C is not.** That retires the pull-up theory, the bus-lockup
theory, and the "did upstream secretly add resistors" puzzle. The user refused the pull-up theory on
the grounds that upstream runs the same hardware and would have mentioned a deal-breaker component;
that objection was correct.

#### What one writeValue() actually does

Read out of the library rather than assumed (two earlier estimates in this document were wrong
because they were not):

```cpp
// ArduinoBLE 2.1.0, ATT.cpp:601 handleNotify
uint8_t notification[_peers[i].mtu];
length = min((uint16_t)(_peers[i].mtu - notificationLength), (uint16_t)length);
HCI.sendAclPkt(...);          // exactly ONE packet, never chunked
```

So **one `writeValue()` produces exactly one ACL packet**, truncated to the negotiated MTU. And since
RT frames arrive intact in the GUI, **the MTU must already be negotiated well above 23** - the comment
in `ExoBLE.cpp` about a 23-byte default describes the pre-negotiation value, not what is in use.
Frames are ~70 bytes (`BleParser::package_raw_data` writes ASCII decimal of `int(value*100)` plus a
delimiter, 3-7 bytes per value), so the link carries roughly 100 packets/s and ~56 kbps.
**The link is NOT bandwidth-saturated**; an earlier claim in this document that it was is withdrawn.

#### The two presentations, kept separate on purpose

| | main thread | RT data | BLE | recovery |
|---|---|---|---|---|
| **A** | **alive** (RGB blinking, green flashing) | flowing | dead, never re-advertises | `BLESTALL_n1` fired once |
| **B** | **stopped** (RGB solid, green solid) | stopped | dead | watchdog only |

**These are not obviously the same failure and this document previously asserted they were.** In A
the main thread demonstrably keeps running, which rules out anything that requires it to be wedged.
Whether A and B share a root cause is **not established**.

#### The mechanism hypothesis for presentation B

`_pendingPkt` increments on every ACL packet and is decremented **only** by `handleNumCompPkts()`,
driven by the controller's completed-packet events. `_maxPkt` comes from the controller's LE Read
Buffer Size and is single-digit on typical Cordio builds. At ~100 notifications/s:

> If the controller stops returning completed-packet events for only **~40-100 ms**, `_pendingPkt`
> reaches `_maxPkt` and the main thread enters `while (_pendingPkt >= _maxPkt) { poll(); }`
> (`HCI.cpp:636`) - which has **no timeout, no bound and no escape** - and never leaves. A main thread
> that never returns freezes both LEDs (`life_pulse()` lives there) and starves `receiveThd`, the
> polling thread that services the I2C slave, so the Teensy sees address NACKs.

That reproduces presentation B exactly, and explains the bisect: at 100 notifications/s a ~100 ms
stall bricks it, whereas with `fw0` the occasional status/battery traffic would need **many seconds**
of stalled completions to accumulate the same backlog, so it is never reached.

**Status: PLAUSIBLE, NOT PROVEN.** `_pendingPkt` and `_maxPkt` have never been read at runtime.

**The trigger is probably mundane.** A ~100 ms stall in packet completion is the kind of thing RF
interference or a busy central produces routinely. If this hypothesis is right, **the defect is not
the stall - it is that a 100 ms transient becomes a permanently dead exoskeleton** because a library
wait loop has no timeout. Everything else built in this investigation (watchdog, stall detector,
breadcrumbs) is downstream of that one missing bound.

#### The experiment that is also the fix

Bounding the spin tests the hypothesis and repairs it in the same change:

- presentation **B disappears**, failures become brief recoverable glitches -> hypothesis confirmed
- presentation **B persists unchanged** -> hypothesis wrong, cheaply, and the spin is exonerated

See §8 for the patch itself. It lives in the sketchbook copy of ArduinoBLE, **outside this repo**,
which is why it is documented here in detail.

#### Errors made this session, recorded so the pattern is visible

1. The 2 s GUI ping called a likely fix on one 13-minute run; the next run with it still enabled
   failed at 191 s.
2. "Interrupt-level stop" asserted before reading `Wire.cpp`, which shows the I2C slave is a polling
   **thread**, so thread starvation suffices and no interrupt claim was ever needed.
3. Missing I2C pull-ups proposed as likely cause despite the user having already pointed out that
   upstream runs the same hardware.
4. "~10 ACL packets per RT frame" - assumed MTU 23 without reading `handleNotify`.
5. "13 bytes per float" - assumed fixed-width fields without reading `package_raw_data`.

The common thread is reasoning from assumed constants instead of opening the file first. The
evidence-only document exists to make that failure mode harder to repeat.

---

## 4. How to read the output

The reset reason is parked in `ErrorChar` during `ExoBLE::setup()` and printed by the GUI at connect
(`MainWindow._on_reset_reason`). No GUI change was needed — the stage is appended into the names
field the same way `CRASH_` already was.

| banner | meaning |
|---|---|
| `RST:0x00000004:SREQ,BLESTALL_n1` | **Mode 2 confirmed** - the link died while the Nano kept running, and our own detector rebooted it. This is the line that proves the diagnosis |
| `RST:0x00000002:DOG,STAGE_7_dog1` | `STAGE_7` = ARMED: died on the boot path before the loop's first breadcrumb. Before the §3.6 fix this presented as the uninformative `STAGE_0` |
| `RST:0x00000002:DOG,STAGE_4_dog1` | **Hung**, watchdog recovered it. Died in `update_UART`. First consecutive dog reboot |
| `RST:0x...:...,CRASH_0x1234_n1` | A real trapped mbed fault (unchanged behaviour) |
| `RST:0x00000001:RESETPIN` | Button, re-flash, or this board's power-on (confounded — see `Nano-Reset-Pin-Spurious-Reset.md`) |
| `dog3` and no further recovery | Boot-loop guard tripped: 3 consecutive hangs, watchdog no longer arming, device deliberately left up so it can be read |

Stage codes (`SystemReset.h`): 1 `LOOP_TOP` (completed a clean pass), 2 `HANDLE_BLE`,
3 `LOCAL_SAMPLE`, 4 `UPDATE_UART`, 5 `UPDATE_GUI`, 6 `HANDLE_ERRORS`, 7 `ARMED` (armed, but the
loop had not yet reached its first breadcrumb - a boot-path trip). A stage of 0 now means only
that the previous boot recorded nothing, e.g. it ended in a true power-on.

---

## 5. HOW TO DISMANTLE

Remove in this order; later items are referenced by earlier ones. Every edit carries a comment naming
the watchdog, so `grep -rn "exo_wdt\|EXO_WDT\|EXO_STAGE" ExoCode/` finds all eight sites.

1. **`ExoCode/ExoCode.ino`, Nano `loop()`** — delete the `exo_wdt_feed()` call and all six
   `exo_wdt_stage(...)` lines, restoring the plain five-call block:
   ```cpp
   mcu->handle_ble();
   mcu->local_sample();
   mcu->update_UART();
   mcu->update_gui();
   mcu->handle_errors();
   ```
2. **`ExoCode/ExoCode.ino`** - delete `exo_wdt_start();` and its comment from the **end of the Nano
   `loop()`** (it was moved there from `setup()` by the §3.6 fix), and delete
   `(void)exo_wdt_stage_record();` and its comment from the end of the Nano `setup()`.
3. **`ExoCode/ExoCode.ino`, Nano include block** — delete `#include "src/SystemReset.h"` **only if**
   nothing else there needs it. It also arrives transitively via `uart_commands.h`, so leaving it is
   harmless.
4. **`SystemReset.h`, `exo_reset_reason_string()`** — delete the `if (reasons & 0x00000002ul)` DOG
   block. Leave the `CRASH_` block above it alone.
5. **`SystemReset.h`, `exo_crash_record()`** — revert all three edits: drop
   `(void)exo_wdt_stage_record();`, drop `const uint8_t latched_marker_raw = latched_marker;`, and
   restore the else-branch to the unconditional pair:
   ```cpp
   NRF_POWER->GPREGRET  = 0;
   NRF_POWER->GPREGRET2 = 0;
   ```
6. **`SystemReset.h`** — delete the whole `HARDWARE WATCHDOG` block, from the banner comment down to
   the end of `exo_wdt_start()`.
7. **`SystemReset.h`** — delete the `exo_crash_record` forward declaration above that banner.
8. **`SystemReset.h`** — optional: move the four `EXO_CRASH_*` defines back below where the watchdog
   block was, and drop the two-line comment added above them. Purely cosmetic; leaving them is fine.

**The link-stall detector (items 9-14) - remove the GUI half FIRST**, per the version-skew note in
§3.5, or old firmware logs an unknown command every 2 s:

9. **`Python_GUI/MainWindow.py`** - delete the `self._ble_ping_timer` block (four lines plus its
   comment) from `__init__`.
10. **`Python_GUI/services/QtExoDeviceManager.py`** - delete the whole `pingDevice()` method.
11. **`ExoCode/src/ExoBLE.cpp`** - delete the detector block in `handle_updates()`, the
    `s_last_rx_ms = millis();` line added to the connect branch, the stamping block in
    `on_rx_recieved()`, and the two file statics `s_last_rx_ms` / `s_ping_seen`.
12. **`ExoCode/src/ComsMCU.cpp`** - delete `case ble_names::ping: break;`.
13. **`ExoCode/src/ble_commands.h`** - delete `ble_names::ping` and its `{ble_names::ping, 0}` row.
14. **`ExoCode/src/SystemReset.h`** - delete `EXO_STALL_MAGIC`, `EXO_BLE_STALL_MS`,
    `exo_ble_stall_reset()`, `exo_stall_record()`, the `BLESTALL_n` block in
    `exo_reset_reason_string()`, and restore the `exo_crash_record()` preservation test to check
    `EXO_WDT_MAGIC` alone (or to unconditional zeroing if the watchdog is going too).

15. **`Python_GUI/MainWindow.py`** - restore the `"DOG":` entry in `_RESET_REASON_MEANING` to its
    original text, or leave it: it is one descriptive string and is harmless either way.

16. **The Bug C boot-path feeds** - delete the `exo_wdt_feed();` calls (each carries a comment
    naming the watchdog) from: the two wait loops in `ExoCode/src/GetBulkChar.cpp` plus its
    `#include "SystemReset.h"`; the `get_config()` retry loop in `ExoCode/src/uart_commands.h`;
    the two in `ExoBLE::setup()` in `ExoCode/src/ExoBLE.cpp`; and the three around the one-time
    constructions in the Nano `loop()` in `ExoCode/ExoCode.ino`.

**Then verify** (see the `arduino-toolchain-on-this-pc` note — `--config-file` is mandatory):

```
ACLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
CFG="$HOME/.arduinoIDE/arduino-cli.yaml"
"$ACLI" --config-file "$CFG" compile --fqbn arduino:mbed_nano:nano33ble ExoCode/ExoCode.ino
"$ACLI" --config-file "$CFG" compile --fqbn teensy:avr:teensy41    ExoCode/ExoCode.ino
```

**If you keep one thing, keep the watchdog itself** (items 2, 3, 6 — plus 5 and 7, which item 6
depends on) and drop only the breadcrumbs (items 1 and 4). The breadcrumbs are pure diagnostics; the
watchdog is a robustness property on a device that straps to a leg and drives 30 Nm motors. That is a
judgement call for the user, not a requirement — the procedure above removes everything cleanly.

**Unrelated, do not confuse:** `EXO_CRASH_TRAP_SELFTEST` in `SystemReset.h` is pre-existing, still
`0`, and untouched by this change. It is still worth running once (set to 1, boot, set back) —
until it fires, an empty `GPREGRET` cannot distinguish "no fault occurred" from "the trap is broken".

---

## 6. Deliberately NOT done: the Teensy answering `'R'` at runtime

A watchdog reset recovers BLE but lands in a degraded session, and this is why:

`readSingleMessageBlocking()` (`ExoCode.ino`) spams `'R'` waiting for the bulk controller-parameter
payload, but `send_bulk_char()` is only called from the **Teensy's `setup()`** — never re-sent at
runtime. So a Nano-only reset gets no answer: ~18 s of blocked boot, then `rxBuffer_bulkStr` is left
empty — and `ExoBLE.cpp:374` gates the handshake on that buffer being non-empty, so **the GUI
connects but never receives the controller list or parameter names.** This is very likely the same
mechanism behind the old "PC connects but the Nano will not serve its GATT table" log entries.

It was scoped and rejected as larger than it looks: `send_bulk_char()` uses **Serial8**, the same
physical port as `MY_SERIAL`/`UARTHandler` (`UARTHandler.h:40`), and the Nano's side likewise shares
Serial1. A raw `'R'` sniff inside the Teensy's 500 Hz control loop would compete with the framed UART
protocol for bytes. A clean fix needs a proper chunked UART command, because the payload is far
larger than one `UART_msg_t`.

**User decision 2026-09-10: not needed.** A warm reboot that reports the bug is sufficient; a clean
session afterwards is not the point of this scaffolding.

---

## 7. Known limits

### 7.0 SIDE EFFECT: the watchdog breaks Nano uploads (workaround: power-cycle first)

**Symptom:** `bossac` reports a SAM-BA error partway through the upload, consistently around
**page 10-15 of 91**. Seen on two different Nanos. Never on the Teensy.

**Why it is the watchdog.** The upload *starts*, so the 1200-baud touch worked and the board reached
the bootloader - this is not a touch-reset failure. Something resets the chip mid-flash. 91 pages x
4 KB is ~373 KB, matching the ~371 KB sketch, so failing at page 10-15 is ~40-60 KB written, roughly
11-16% of the way, which at typical bossac CDC throughput is about 4-6 s. `EXO_WDT_TIMEOUT_S` is
**5**. The nRF52840 WDT survives a warm reset (§3.6 Bug C), so double-tapping into the bootloader
leaves it counting, and the bootloader does not feed it. It resets the chip out of the bootloader
mid-upload. It also explains why repeatedly tapping reset eventually works: once the dog actually
fires it clears itself, and the next tap gets a clean window.

**Timeline confirms it:** the user reports these failures began 2026-09-10, i.e. with the first
watchdog flash. They did not happen before.

**Workaround, use this every time:** **power-cycle the board before uploading**, do not merely tap
reset. A true power-on reset is the only thing that clears the WDT.

**Considered and rejected as a full fix:** arming only once the GUI subscribes would keep an idle
just-booted board uploadable, but it is only partial - a dog armed in a previous session is still
running after a stall-reset, so the power-cycle rule remains the real answer. Not worth the
complexity while this is temporary scaffolding.

**A second candidate was ruled out but is worth remembering** if the symptom ever outlives the
watchdog: brownout during flash writes (erase/program draws more current than idle, so a marginal
cable or port can sag) produces the same mid-upload, repeatable-page signature, and would also spare
the Teensy. The timeline is what separates them.

### 7.1 Other limits

- **This does not fix the freeze.** It recovers from it and reports where it stopped.
- **The stage is a phase, not a line number.** It says which of five `ComsMCU` calls was executing.
  Narrowing further means adding sub-stages inside whichever phase turns up.
- **A fault overwrites the stage.** `mbed_error_hook` writes its error byte to `GPREGRET2`, the same
  register the breadcrumb uses. They are told apart by the `GPREGRET` magic, and the `CRASH_` branch
  takes priority. Both registers are 8 bits on the nRF52840, so there is no room for both — and
  fault-vs-hang are mutually exclusive anyway.
- **Never flashed.** Compile-clean on both boards is the only validation so far.
- **After 3 consecutive hangs the watchdog stops arming** (by design, §3.3). A device in that state
  looks like the old behaviour — hung, needing a power cycle. Check the banner for `dog3`.

---

## 8. The ArduinoBLE bounded-wait patch (OUTSIDE this repository)

**Applied 2026-09-11. This is the candidate fix AND the experiment that tests §3.9's hypothesis.**

### 8.1 What was changed

One file, one function:

```
C:\Users\<user>\Documents\Arduino\libraries\ArduinoBLE\src\utility\HCI.cpp
  int HCIClass::sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data)
```

Before — no timeout, no bound, no escape:

```cpp
while (_pendingPkt >= _maxPkt) {
    poll();
}
```

After — gives up and drops the packet:

```cpp
const unsigned long _aclWaitStart = millis();
while (_pendingPkt >= _maxPkt) {
    poll();
    if ((millis() - _aclWaitStart) >= ARDUINOBLE_ACL_WAIT_TIMEOUT_MS) {
        return -1;
    }
}
```

`ARDUINOBLE_ACL_WAIT_TIMEOUT_MS` is 50 by default and overridable. A long explanatory comment sits
immediately above the function in that file, covering the reasoning, the evidence, the return-value
change and how to revert — deliberately placed there because **anyone debugging BLE on this setup
will be reading that function anyway**, which is exactly where the warning is useful.

### 8.2 Why 50 ms

At the configured 7.5 ms connection interval that is ~7 connection events — far longer than any
healthy congestion needs to clear, so it should never fire in normal operation. If the link really is
dead, every send then costs 50 ms and the sketch loop drops to roughly 20 Hz: degraded but **alive**,
which also means the watchdog (5 s) will not fire and the BLE link-stall detector (8 s of GUI silence)
can do its job and warm-reboot cleanly.

### 8.3 Return value

`0` still means "handed to the transport"; the timeout path returns `-1`. **Verified safe for
ArduinoBLE 2.1.0:** all eleven callers are in `ATT.cpp` and every one discards the result. If a future
library version starts checking it, make sure it does not respond by retrying — that would defeat the
entire point.

### 8.4 This is outside version control

The build uses the **sketchbook** copy of ArduinoBLE (2.1.0), not the 1.2.1 vendored in this repo's
`Libraries/` folder — see `arduino-toolchain-on-this-pc`. So this change:

- is **invisible to this repository's git history**
- **will be silently lost** if ArduinoBLE is updated or reinstalled
- applies to **every** Arduino project on this machine that uses ArduinoBLE

A pristine copy was saved alongside it as `HCI.cpp.orig-openexo-backup` before patching.

**To verify the patch is still applied:**

```
grep -c ARDUINOBLE_ACL_WAIT_TIMEOUT_MS \
  "$HOME/Documents/Arduino/libraries/ArduinoBLE/src/utility/HCI.cpp"
```

Zero means it is gone — most likely the library was updated — and the freeze will return.

**To revert:** restore `HCI.cpp.orig-openexo-backup` over `HCI.cpp`, or delete the timeout lines and
the comment block.

### 8.5 What this build (B18) tests

B18 is normal operation (`rt1_fw1`) plus this patch. It is a genuine experiment, not just a fix:

| outcome | conclusion |
|---|---|
| **presentation B stops happening** (no more everything-frozen-solid, no more `DOG`) | the unbounded spin was the amplifier — §3.9's hypothesis confirmed, and this is a real fix |
| failures become brief glitches the exo rides through | same conclusion, even stronger |
| **presentation B continues unchanged** | the spin is exonerated, the hypothesis is wrong, and the cost was one afternoon |
| presentation **A** (`BLESTALL`, loop alive) still occurs | expected either way — A never depended on the spin, and the stall detector already handles it |

Note that A and B were never shown to share a root cause (§3.9), so this patch is only expected to
address B.

### 8.6 If it works, what should happen to it

This is a genuine defect in ArduinoBLE — an unbounded wait on a remote party in a library used on
battery-powered wearables — and it is worth reporting upstream to `arduino-libraries/ArduinoBLE`
regardless of what happens here. Doing so is also the only durable fix, since any local patch in the
sketchbook is one library update away from vanishing.

---

## 9. The connection interval - the first change aimed at WHY the link dies

Everything before this section manages the **aftermath** of the failure. This is the first change
that targets the failure itself.

### 9.1 What was changed

`ExoBLE.cpp`, one line:

```cpp
BLE.setConnectionInterval(6, 6);     // was: 7.5 ms, pinned as BOTH min and max
BLE.setConnectionInterval(12, 24);   // now: 15-30 ms, a range
```

Units are 1.25 ms. `(6, 6)` is **7.5 ms - the minimum the BLE spec allows - with min equal to max**,
so the central had no range to negotiate within.

### 9.2 Why that matters (the mechanism)

BLE is not a stream. Once connected, both radios sleep and wake together every *N* ms for a
**connection event**; data moves only during those events, and the **central owns the schedule**.

`(6, 6)` demanded **133 connection events per second, indefinitely, with no flexibility** from a
Windows host that is also running a mouse, keyboard, headphones, scanning, and - critically -
sharing the 2.4 GHz band with Wi-Fi through a coexistence scheme that makes the two take turns.
When that host is busy it does the only thing it can: **it skips connection events.**

With no slack in the interval, a burst of skipped events cascades:

1. Nothing can be transmitted - packets are queued with no appointment to send them in.
2. `_pendingPkt` saturates in **~40-100 ms** at ~100 notifications/s.
3. The main thread enters the `HCI.cpp:636` wait (forever pre-B18; 50 ms per send after).
4. ~9.6 s later the host's supervision timeout expires and it declares `link lost`.

**And the trapped main thread is why the device stayed unreachable.** At runtime, re-advertising
happens in exactly one place - `advertising_onoff(current_status == 0)` at `ExoBLE.cpp:470`, inside
`handle_updates()`, which is called from `ComsMCU.cpp:76` in the **main loop**. A trapped main thread
means `BLE.poll()` never processes the disconnect, `BLE.connected()` never changes, and the Nano
never re-advertises. **The link loss is transient; the trapped thread is what made the consequence
permanent.** (Credit where due: the user pointed this out - this document had it listed as an open
question when the answer was already established.)

**Throughput is not lost**, which is the counter-intuitive part: multiple packets can be exchanged
per connection event, so fewer, larger events carry the same data. §9.3 confirms this empirically.

### 9.3 Result - one run

| | reference 2026-09-10 (7.5 ms) | B19 (15-30 ms) |
|---|---|---|
| sample rate | 92.0 Hz | **90.2 Hz** |
| median gap | 10.0 ms | 10.0 ms |
| p90 / p99 / max gap | 20 / 30 / 80 ms | 20 / 30 / 70 ms |
| duration | 229 s, **failed** | **1,821 s (30 min), no failure, ended by the user** |

**The rate check matters as much as the duration.** Had the longer interval throttled the
notification rate, the run would have been confounded - reducing BLE traffic alone was already known
to suppress the failure for 30+ minutes (§3.9 bisect). It did not: 90.2 Hz against 92.0 Hz, with a
near-identical gap distribution. So the survival is attributable to the interval change itself.

Against a baseline mean time-to-failure of 225.5 s, a clean 1,821 s run has probability ~**1 in
3,700** under a constant-hazard null.

### 9.4 Status: CANDIDATE FIX, not confirmed

**One run.** The 2 s GUI ping produced a 13.2-minute run (~1 in 33) and was wrongly called a likely
fix; the next run with it enabled failed at 191 s. This is roughly 100x stronger evidence, but the
discipline is the same.

What is still needed:

- **Two or three independent runs**, spread over time. Exposure time is fungible under a
  constant-hazard model, but repeats cover what it cannot: possible elevated hazard right after
  connecting (three failures clustered at 12, 24 and 41 s), and RF conditions that drift.
- **Read back the negotiated interval** to confirm the host actually accepted 15-30 ms.

### 9.5 If it holds

Then the root cause was **a rigid, minimum-value connection interval**, and the fix is one line. The
watchdog, link-stall detector, bounded spin and breadcrumbs would all become a **safety net** rather
than the thing standing between the user and a dead trial - worth keeping the first three, and the
breadcrumbs can go (§5).

Worth noting the bounded-spin patch (§8) retains independent value regardless: an unbounded wait on
a remote party is a defect, and it is what turned a transient radio hiccup into a permanently
unreachable device.

---

## 10. Two other defects found along the way

Neither is part of the disconnect root cause. Both were found because of it, and both are real.

### 10.1 The liveness ping could corrupt a command and wedge the Nano's parser

**Introduced by this investigation**, on 2026-09-11, and found before it ever bit.

The 2 s ping is a GUI **write**, on the same characteristic the GUI sends commands on, driven by a
Qt timer. But a single logical command is often several separate BLE writes - a parameter update is
five:

```python
await write(b"f")                                   # command byte
for val in (joint, controller, index, value):
    await write(struct.pack("<d", float(val)))      # 4 x 8 bytes
```

Every `await` yields to the event loop, so a ping scheduled at that moment runs **between** them and
injects its byte into the middle of the payload. The Nano cannot recover: `BleParser` buffers until
it has collected exactly `expecting * 8` bytes, an **exact equality**, so one stray byte makes that
33 instead of 32 - never equal. `_waiting_for_data` stays true forever, every later command byte is
swallowed as payload, and it keeps appending into `byte _buffer[64]` (`BleParser.h:59`) until it runs
off the end of the array.

Rough exposure: a ~50 ms write sequence against a 2 s ping is ~2.5% per parameter update. That is
low per update and very high across a session of the external control loop, which sends them
continuously (`Python_GUI/external_control/`).

**Fix:** an advisory busy counter in `QtExoDeviceManager`. `_submit_tx()` increments it around every
writer coroutine (in a `finally`, so a hung or raising write still unwinds), and `pingDevice()`
returns early while it is non-zero. Skipping costs nothing - **any** inbound byte refreshes the
firmware's link-stall timer, so ordinary commands keep it fed on their own.

**A lock was tried first and deliberately abandoned.** Holding an `asyncio.Lock` across each whole
sequence would also have stopped two *commands* interleaving - but writers here can block for a long
time (`beginTrial` holds an `await asyncio.sleep(1)`, `send_end_trial_sequence` waits up to 2 s), and
bleak's `write_gatt_char` is known on WinRT to occasionally never return at all - there is a comment
in `send_end_trial_sequence` saying exactly that. One hung write would then have blocked **every**
later command for the rest of the session. A counter cannot do that.

**The trade is explicit:** this fixes ping-versus-command. Two overlapping *command* sequences can
still corrupt each other exactly as they always could. That hazard predates the ping and has never
been observed. If it ever needs fixing, do it by making callers not overlap - not by making writes
wait on each other.

### 10.2 The CSV time column still wrapped every 655 s

**Pre-existing, unrelated to the disconnect, and nearly four months old.**

Every real-time channel is packed int16 x100, so `Exoskeleton time (seconds)` wraps at +/-327.67 s.
Commit `6322e17` (2026-07-21, *"Modified real-time plotting on GUI so it ddoesnt cause wrapping
visual glitch at 327s"*) fixed this - but only for the **plot**. It touched exactly two files,
`ActiveTrialPage.py` and its test. `MainWindow.py`, where the CSV is written, was never touched, and
no commit in the repo's history ever added wrap handling there.

Nothing surfaced it because **almost no trial ever ran long enough**: the failure history tops out at
789 s and most runs died in 2-4 minutes, so barely any trial reached even the first wrap. The
30-minute B19 run made it obvious - that file's time column spans **-145 s**.

**Fix:** the same correction applied in `MainWindow.py`'s row writer, with its own per-file state
(the CSV's lifetime is per file, not per plot). The exo-time column is located **by name**, for the
same reason the channel selection is - the index moves when the firmware payload changes.

**Verified** against the B19 trial: unwrapped span **1,820.7 s** against a wall-clock span of
**1,820.8 s** - 0.1 s of agreement over 30 minutes, 3 wraps correctly detected, fully monotonic.

**One deliberate difference from the plot.** `_x_for_sample` re-anchors on a genuine reboot, because
a live axis should restart. The CSV **does not**: re-anchoring would splice two boots into one smooth
monotonic column and hide that the device restarted. Letting the offset accumulate keeps the
discontinuity visible - which matters now that the watchdog and stall detector reboot the Nano
routinely.

Old CSVs are fully recoverable: nothing was lost, only wrapped, so the same unwrap can be applied
offline to any existing file.

---

## 11. REMOVAL PLAN - what comes out once the fix is confirmed

**Precondition: do not start this until the connection-interval fix (§9) is confirmed** by several
independent runs plus a motor stress test at non-zero torque. As of writing it rests on **one**
30-minute run. If it is not confirmed, this plan is void and the diagnostics are still earning their
keep.

### 11.1 Inventory, by verdict

**KEEP - these are fixes, not scaffolding:**

| item | where | why keep |
|---|---|---|
| Connection interval `(12, 24)` | `ExoBLE.cpp` | the fix itself |
| Bounded `sendAclPkt` wait | ArduinoBLE `HCI.cpp` (**outside repo**) | an unbounded wait on a remote party is a defect regardless of trigger; it is what turned a transient hiccup into a permanently unreachable device. **Report upstream** - that is the only durable fix, since any sketchbook patch dies on a library update |
| CSV exo-time unwrap | `MainWindow.py` | unrelated real bug (§10.2) |
| TX busy counter | `QtExoDeviceManager.py` | only needed while the ping exists - see 11.4 |

**REMOVE - pure diagnostics, purpose served:**

| item | where | note |
|---|---|---|
| Stage breadcrumbs | `SystemReset.h`, `ExoCode.ino`, `ExoBLE.cpp` | `EXO_STAGE_*`, `exo_wdt_stage()`, `exo_noinit_*`. **Never worked** - no store survived a watchdog reset |
| Send-path diagnostics | `ExoBLE.cpp` | `s_max_write_us`, `s_sends_since_rx`, `exo_ble_link_diag()`, the `_w`/`_s`/`_FAIL` banner fields. Did its job: `w10` proved the timeout fired |
| Teensy I2C counters | `RealTimeI2C.*` | `rt_i2c_stats`. Cleared I2C as a cause; nothing left to measure |
| Link-stats UART command | `uart_commands.h`, `ExoCode.ino`, `SystemReset.h` | `get_link_stats` / `update_link_stats`, `uart_scale_clamp`, `exo_link_stats*` |
| Build tag and flag readout | `SystemReset.h` | `EXO_FW_TAG`, `_b`/`_rt`/`_fw` fields. Invaluable while iterating, noise afterwards |
| Bisect flags | `Config.h`, `ComsMCU.cpp` | `RT_BLE_FORWARD` entirely; `REAL_TIME_I2C` stays but **must be 1** |

**DECIDE - judgement calls, see below**

### 11.2 Phase 1 - experiment scaffolding (safe, immediate)

Lowest risk, do this first:

1. `Config.h`: delete `RT_BLE_FORWARD` and its comment block; confirm `REAL_TIME_I2C` is **1**.
2. `ComsMCU.cpp`: remove the `#if RT_BLE_FORWARD` guard around `_exo_ble->send_message(rt_data_msg);`.
3. `SystemReset.h`: drop the `_rt`/`_fw` fields from `exo_fw_tag_string()`.

### 11.3 Phase 2 - diagnostics

4. **Breadcrumbs.** `ExoCode.ino`: all `exo_wdt_stage(...)` calls and the `exo_wdt_stage_record()` in
   `setup()`. `ExoBLE.cpp`: the `EXO_STAGE_BLE_BEGIN` stamp. `SystemReset.h`: `EXO_STAGE_*`,
   `exo_wdt_stage()`, `exo_wdt_stage_record()`, the `STAGEn`/`g` banner block. `SystemReset.cpp`: the
   whole `.noinit` block and `exo_noinit_boot_count()`.
   **Careful:** `exo_wdt_stage_record()` is also what latches the link diagnostic - remove them
   together, in this phase, or the `BLESTALL` line loses its `_w`/`_s` fields with no warning.
5. **Send-path diagnostics.** `ExoBLE.cpp`: the statics, `exo_us_bucket`, `exo_count_bucket`,
   `exo_ble_link_diag()`, the timing around `writeValue`, and the reset block in `on_rx_recieved`.
   `SystemReset.h`: `exo_ble_stall_reset()` drops its `diag` parameter; the banner keeps `BLESTALL_n`
   and loses `_w`/`_s`/`_FAIL`. **Keep `s_last_rx_ms` and `s_ping_seen`** - the detector needs them.
6. **Teensy I2C counters + link-stats.** `RealTimeI2C.*`: `rt_i2c_stats` and the `endTransmission()`
   capture (revert to the bare call). `uart_commands.h`: both command names, the handler, both switch
   cases, `UART_command_utils::get_link_stats`, `uart_scale_clamp`. `ExoCode.ino`: the boot-time
   request. `SystemReset.h`/`.cpp`: `exo_link_stats*` and `exo_link_stats_string()`. `ExoBLE.cpp`:
   drop `+ exo_link_stats_string()` from the banner.
7. **Build tag.** `SystemReset.h`: `EXO_FW_TAG` and `exo_fw_tag_string()`, and the four calls
   appending it. *Consider keeping it* - it cost nothing and settled "which binary is on the board"
   several times.

### 11.4 Phase 3 - the ping package (decide together)

`ping` command, GUI ping timer, `pingDevice()`, TX busy counter, and the **BLE link-stall detector**
are one unit: the detector cannot arm without the ping (`s_ping_seen`), and the counter only exists
because of the ping.

- **Remove all of it** if the interval fix holds. The detector then has nothing to detect, and
  removing the ping removes the §10.1 collision hazard **at its source** rather than guarding it.
- **Keep all of it** as a safety net for the one case the watchdog cannot cover: presentation A, where
  the loop stays alive but the link is dead. That was observed and is not explained by anything the
  interval fix addresses.

**Removal order matters** - GUI half first. New firmware with an old GUI is safe (no pings, so
`s_ping_seen` stays false and the detector never arms). Old firmware with a new GUI log-spams
`Command is not in list: p` every 2 s.

### 11.5 DECIDE - the watchdog

> **SUPERSEDED 2026-09-12 by §13.4: KEEP IT.** The defect behind presentation A has been open
> upstream since 2019, is still present in the latest ArduinoBLE release, and the reporters'
> own conclusion was that a hardware watchdog is the only remedy. The "for removing" argument
> below assumed the fault might simply be gone; that assumption no longer holds. The original
> weighing is kept for the record.

Genuinely a judgement call, and not the author's to make:

- **For keeping:** it is the only thing that can recover a device that has stopped executing. It costs
  a handful of instructions per loop. On something that straps to a leg and drives 30 Nm motors, a
  5 s hardware backstop is cheap insurance.
- **For removing:** it breaks uploads (§7.0 - the WDT survives a warm reset and kills the bootloader
  mid-flash, requiring a power-cycle-before-upload habit), and if the interval fix holds it may never
  fire again.

If kept, keep the boot-loop guard with it (`EXO_WDT_MAGIC`, `exo_wdt_boot_count()`) - without it a
device that hangs on every boot reboots forever.

### 11.6 Verification after each phase

```
ACLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
CFG="$HOME/.arduinoIDE/arduino-cli.yaml"
"$ACLI" --config-file "$CFG" compile --fqbn arduino:mbed_nano:nano33ble ExoCode/ExoCode.ino
"$ACLI" --config-file "$CFG" compile --fqbn teensy:avr:teensy41    ExoCode/ExoCode.ino
python -c "import ast,io; ast.parse(io.open('Python_GUI/MainWindow.py',encoding='utf-8').read())"
python -c "import ast,io; ast.parse(io.open('Python_GUI/services/QtExoDeviceManager.py',encoding='utf-8').read())"
```

Then **one real trial per phase** - not just a compile. Each phase touches the live BLE path, and a
mistake there looks exactly like the bug this whole investigation was chasing.

`grep -rn "exo_wdt\|EXO_WDT\|EXO_STAGE\|exo_link_stats\|rt_i2c_stats\|RT_BLE_FORWARD" ExoCode/`
finds every remaining site at any point.

---

## 12. THE INTERVAL WAS NEVER VERIFIED - read this before trusting §9's mechanism

Added 2026-09-12 after checking ArduinoBLE's source and the external literature. **§9's empirical
result is unaffected. §9's explanation of WHY is not established.**

### 12.1 What ArduinoBLE actually does

Read out of `ArduinoBLE 2.1.0` (the sketchbook copy that builds), not assumed:

- `L2CAPSignalingClass::addConnection()` sends a **Connection Parameter Update Request exactly once,
  at connection setup**, and only when the central's chosen interval falls outside our
  `[_minInterval, _maxInterval]`:

  ```cpp
  if (interval < _minInterval || interval > _maxInterval) { ...send request... }
  ```

- **`connectionParameterUpdateResponse()` is completely empty.** The library receives the central's
  accept/reject and throws it away. No retry, no fallback, no record.
- **`LE_META_EVENT` does not include `CONN_UPDATE_COMPLETE` (0x03).** The enum has CONN_COMPLETE,
  ADVERTISING_REPORT, LONG_TERM_KEY_REQUEST, REMOTE_CONN_PARAM_REQ, READ_LOCAL_P256_COMPLETE,
  GENERATE_DH_KEY_COMPLETE, ENHANCED_CONN_COMPLETE - and nothing for the update-complete event.

**Consequence: the firmware never learns what connection interval is in effect.** It knows only the
value the central picked at connection time, passed into `addConnection()`. Everything this document
said about "7.5 ms" and "15-30 ms" describes **what was requested**, never what was applied.

### 12.2 What the external sources say

- The **central has final say** and may accept, reject, or do nothing
  ([TI BLE5-Stack GAP guide](https://software-dl.ti.com/lprf/simplelink_cc2640r2_latest/docs/ble5stack/ble_user_guide/html/ble-stack-5.x/gap.html),
  [Punch Through](https://punchthrough.com/ble-connection-parameters-guide/)).
- **Windows 10 is documented to ACCEPT a request and then not apply it** - responding
  "Connection Parameter Update Response (accepted)" while the interval stays at its initial value,
  and doing so **randomly**: sometimes applied, sometimes not, on the same stack
  ([Microsoft Q&A](https://learn.microsoft.com/en-us/answers/questions/848142/windows-10-ble-connection-parameter-update-issue),
  [Nordic DevZone](https://devzone.nordicsemi.com/f/nordic-q-a/27235/win-10-ble-stacks-acting-as-a-central-problems-accepting-ble-connection-parameter-changes)).
- Windows generally honours the **PPCP characteristic** (Preferred Peripheral Connection Parameters)
  rather than L2CAP requests as the way to express preferences.

### 12.3 The competing hypothesis, and why it was rejected

If Windows **rejected** `(6,6)`, the link stayed at Windows' own default - and `(12,24)` would land at
roughly that same value. Both builds would then have run at the **same interval**, and the only
difference between them would be whether a single L2CAP packet was sent at connection setup. That
raised the alternative "the fix is that we stopped asking Windows to change parameters."

**Rejected**, and the user's objection is the reason: the request is sent **once, at connect**. There
is no mechanism by which one packet at setup causes a link to die 40-800 seconds later, at random.

**And rejecting it strengthens §9 by elimination.** If the intervals had been identical, the observed
difference (ten failures at a 225 s mean, versus two clean 30-minute runs) would have no cause at
all. Since the behaviour difference is real and large, **the intervals almost certainly did differ** -
meaning Windows did apply the 7.5 ms request, and §9's account survives.

That is an argument from elimination, not from measurement. It is weaker than a reading would be.

### 12.4 A speculation that would explain the variance

**Marked clearly as speculation - no evidence either way.**

If Windows applying these requests really is random per connection, then **each connection was a coin
flip** between "pinned at 7.5 ms" and "left at Windows' comfortable default." That would explain the
time-to-failure spread that has never been accounted for - 12 s, 24 s, 41 s at one end against
626 s, 789 s at the other - as **two populations** rather than one process with a fat tail. It also
fits the distribution looking exponential-ish but not cleanly so (CV 1.13, median/mean 0.59 against
exponential's 1.00 and 0.69).

Checkable with the same one-line log: if short runs correlate with a pinned 7.5 ms interval and long
ones do not, that is the answer.

### 12.5 What this does and does not change

| | status |
|---|---|
| `(6,6)` failed 10/10, mean 225 s | **unchanged, observed** |
| `(12,24)` gave two clean 30-min runs at full rate, ~1 in 14 million | **unchanged, observed** |
| The change is worth keeping | **unchanged** - it works, whatever the reason |
| "The link ran at 7.5 ms" | **NOT VERIFIED.** Likely, by elimination (§12.3), not measured |
| "133 connection events/s loaded the host" | **unsupported** - rests on the unverified interval |
| "No headroom to drain a backlog at 7.5 ms" | **WITHDRAWN** - 133 events/s against ~90 notifications/s is ~48% headroom, and body-blocking episodes recovered on unpatched firmware, proving a reduced rate is survivable |

**The removal plan in §11 does not depend on any of this.** It is gated on the fix working, which is
an observation, not on why it works.

---

## 13. UPSTREAM CORROBORATION - ArduinoBLE issue #45, open since 2019

Found 2026-09-12, by the user, after §8 was already written and shipped. **We did not find this
first; we rediscovered a six-year-old open bug from scratch.** That is worth recording honestly,
and it changes several things.

- <https://github.com/arduino-libraries/ArduinoBLE/issues/45> - *"Weak signal doesn't trigger
  disconnect() and hangs in multiple places"*, fgaetani, opened **2019-12-19**, last touched
  **2025-03-20**, **still OPEN**, labelled `type: imperfection` / `status: waiting for information`.
- **Upstream `master` still contains the unbounded loop today** (`HCI.cpp:638`), and our sketchbook
  copy is `2.1.0`, which is the **latest release** (2026-06-22). We are not behind; there is no
  version to upgrade to that fixes this.

### 13.1 What it confirms, verbatim

The opening post names our exact function, our exact loop, and our exact symptom:

> Code execution remained locked in the `writeValue()` function, specifically in the
> `HCIClass::sendAclPkt()` function. The code remained locked in the `while` loop, because the
> device is disconnected.

and proposes essentially §8's fix, bounded by an iteration count rather than a clock:

```cpp
int k = 0;
while (_pendingPkt >= _maxPkt) { k++; if (k > _maxPkt) break; poll(); }
```

**Our clock-bounded version is the better of the two**, and for a reason worth stating: `_maxPkt`
iterations of `poll()` is not a unit of time. `poll()` returns immediately when the transport has
nothing to read, so a handful of iterations can elapse in microseconds and abandon a send that was
about to succeed. A wall-clock bound expresses what is actually meant - *the controller has gone
quiet for 50 ms*.

Independent confirmation matters here because §8's mechanism was, until today, entirely our own
reconstruction from reading the source plus one `w10` measurement. It now has a second,
unconnected witness who arrived at the same place from a different application.

### 13.2 What it confirms about presentation A - and this is the bigger one

Presentation A (link dead, **sketch still running normally**, device invisible, only a reset
recovers it) has been our least-explained observation. The thread describes it independently, twice,
in terms that match ours almost word for word.

**morettigiorgio, 2020-01-24:**

> if i forced a BLE.disconnect() (during the connection lost), central.connected() and BLE.central()
> returned the correct false, but my Arduino peripheral (Arduino Nano 33 BLE and Arduino Nano 33 BLE
> Sense) **is no more discoverable, not even with another BLE.advertise() cmd**. It's very
> strange... I have to restart

**JoeyTolentino, 2020-02-07:**

> when I walk away, and I lose connectivity, the little display still indicates that "something is
> connected" and **the IMU data is still updating as it should**; however, I'm unable to see the
> device to reconnect to it when I'm near by. [...] **The only way to reconnect would be to press
> the reset button, or cut power.**

Compare our own log: *"Auto disconnected, but green light still flashing, white/pinkish led still
blinking, exactly as if nothing happened... I can't reconnect... And I just clicked reset button,
now I can reconnect."*

That is the same fault, on the same board family, reported by strangers six years ago. **It is not
something about this exoskeleton, this wiring, this GUI or this laptop.** Three of our own Nanos
reproducing it was already suggestive; this settles it.

Note also that morettigiorgio's report kills a hypothesis we never fully closed out: **calling
`BLE.advertise()` again does not rescue it.** Our §9 note that the only runtime re-advertise path
lives in the blocked main loop was a plausible explanation for why the device never came back. It is
now clear that re-advertising would not have been enough anyway - the controller/stack is wedged at
a level `advertise()` cannot reach.

### 13.3 What it says AGAINST us - read this before over-claiming §8

**The thread's consensus is that bounding the loop does not fix presentation A.** Three separate
people say so:

- **fgaetani**, the patch's author, 2020-02-26: *"The reported issue #45 is different, in my case the
  microcontroller lock in loop in that cycle and by modifying in that way I solved it. While the
  other issue [...] the board remains apparently connected and is no longer visible from other
  devices. The only solution is to reset the microcontroller manually or through a watchdog."*
- **morettigiorgio**: tested the patch, *"without having solved"* the invisible-device problem.
- **JoeyTolentino**: *"I've tried adding the recommended code by @fgaetani and I did not realize a
  behaviour change by the hardware."*

**This matches our own results exactly and we should say so plainly.** We patched `HCI.cpp` and
*kept failing* - B18 fired the new timeout (`BLESTALL_n1_w10_s3`) and the device still had to be
manually reconnected. At the time that read as a disappointment. It is better understood as our data
agreeing with three independent reports: **the bounded spin converts a hard hang into a recoverable
degraded state, and does nothing about the underlying link death.**

So the honest statement of what §8 buys is narrower than §8 implies:

| claim | status after #45 |
|---|---|
| The unbounded loop is a real defect that can hang the sketch | **corroborated** - independent report, our `w10` measurement |
| Bounding it prevents the *hang* presentation (B) | **corroborated** |
| Bounding it prevents the *disconnection* (A) | **contradicted** - by three reporters and by our own B18 run |

### 13.4 What it says about the watchdog

The thread's own workaround for presentation A is **the nRF52840 hardware watchdog**, using the same
registers we arrived at independently (`NRF_WDT->CONFIG / CRV / RREN / TASKS_START`), fed from the
loop and gated on connection state. fgaetani: *"The only solution is to reset the microcontroller
manually or through a watchdog."*

**This changes the §11.5 recommendation.** That section framed the watchdog as a judgement call whose
"for removing" argument was *"if the interval fix holds it may never fire again."* Against a defect
that has been open upstream for six years, with no fix in the latest release, with independent users
concluding the watchdog is the only remedy - **the watchdog should be kept.** §11.5 is updated
accordingly. Its real cost is the upload hazard (§7.0), which is a documented habit, not a risk to
the wearer.

### 13.5 The unresolved tension: is our trigger really "weak signal"?

Every report in the thread attributes the failure to **weak signal / repeated link loss** - walking
away, hands over the antenna, 9-10 m through obstacles. Ours happens with the laptop on the same
bench.

Two readings, and we cannot currently choose between them:

1. **Same fault, different route in.** What actually kills it is not distance but *missed connection
   events*. Range is just the easiest way to produce those. A 7.5 ms interval on a congested 2.4 GHz
   band could produce the same event-loss density at 1 m that 10 m produces at a relaxed interval.
   This is consistent with §9 and would unify everything.
2. **Different fault.** Ours is load-driven, theirs is range-driven, and the shared symptom is just
   where two different upsets both land.

Reading 1 is the more economical, and it makes a **prediction we can test** (§14.1): deliberately
attenuating the signal should reproduce the failure on demand.

One more datum from the thread, flagged because it contradicts something we assumed: **polldo, an
Arduino maintainer, could not reproduce it at all** (2020-07-02) on the reporter's own sketch. A
fault that some people hit constantly and a maintainer cannot trigger once is why the issue is still
labelled *"waiting for information"* six years on - and is a fair description of our own experience
of it being maddeningly intermittent. It also explains why this was never fixed: **nobody upstream
has ever had a reliable repro.** If §14.1 works, we would have one, and it would be worth posting.

---

## 14. WHAT TO TEST NEXT - ranked by information per unit of effort

Written 2026-09-12 in answer to *"any additional test we can run to understand this better?"*, and
substantially reshaped by §13. Ordered so that the cheapest, most decisive things come first.

The thing standing in the way of all of this is **the absence of a repro**. Every experiment
currently costs one run of unknown length - anywhere from 12 s to never - and needs several runs to
mean anything. §14.1 attacks that directly and should be done before anything else, because it makes
every other test on this list roughly twenty times cheaper.

### 14.1 FIRST: try to make the failure happen on demand, by attenuating the signal

**Cost: zero code, five minutes. Potential payoff: the entire investigation becomes tractable.**

Every reporter in issue #45 triggers this with **weak signal**, and one of them gives a recipe
(Hoffa25, 2020-05-01):

> Move the phone to a spot were the connection is really weak [...] To make it come quicker you can
> force disconnects by putting objects around the arduino (your hands, another smartphone, tablet,
> whatever blocks the signal well). When disconnected keep covering the arduino for 10-30s and then
> remove. Repeat until error occurs. **Most of the time the error will occur within 5 minutes.**

Concretely, on the current build, with the trial streaming as usual:

1. Cup both hands tightly around the Nano for ~20 s, release for ~10 s. Repeat.
2. If that does nothing, escalate: wrap it loosely in aluminium foil, or put the laptop in another
   room, or both.
3. Watch for either presentation - GUI stops updating (A), or the device goes unreachable (B) - and
   note which, plus whether the LEDs keep blinking.

**Interpreting the result, and note that every outcome is informative:**

| outcome | what it means |
|---|---|
| Fails within a few minutes, repeatably | **We have a repro.** Use it for everything below, and consider posting it to issue #45 - upstream has been stuck for six years precisely because a maintainer could not reproduce it (polldo, 2020-07-02) |
| Fails, but only on the old `(6,6)` build | Ties §9 and §13 together: interval sets how much attenuation it takes. Strong support for §13.5 reading 1 |
| Never fails, however hard we attenuate, on either build | Our fault is **not** the range-driven one in #45 despite the identical symptom. That pushes toward §13.5 reading 2, and is worth knowing |

**Do this with zero torque and the exo on the bench**, not on a person - the point is to provoke a
freeze, and a freeze on a worn device is exactly what we are trying to prevent.

### 14.2 Log what the central actually chose - **BUILT 2026-09-12, compiles, not yet flashed**

§12's whole problem is that we never read the connection interval. It turns out **we do not need a
sniffer for the initial value**: the HCI *LE Connection Complete* event carries it, and ArduinoBLE
already passes it into `L2CAPSignalingClass::addConnection()` - along with the latency and the
supervision timeout - and then discards all three (`HCI.cpp:1152`, `L2CAPSignaling.cpp:43`).

```cpp
void L2CAPSignalingClass::addConnection(uint16_t handle, uint8_t role, ...,
                                        uint16_t interval,
                                        uint16_t /*latency*/, uint16_t supervisionTimeout, ...)
```

So: stash `interval`, `latency`, `supervisionTimeout` and the local `updateParameters` flag into
globals at the top of `addConnection()`, and surface them in the existing connect banner. Four
values, one new banner field, e.g. `CP_i<interval>_l<latency>_t<timeout>_u<0|1>`.

**What it answers immediately:**

- What interval Windows opened the link at - in 1.25 ms units, so `6` is 7.5 ms and `24` is 30 ms.
- Whether we even *asked* for a change (`updateParameters`), which distinguishes "Windows already
  opened inside our range" from "we asked and it may or may not have listened."
- The supervision timeout, which §14.5 needs.

**What it does NOT answer:** whether a *later* L2CAP update was applied. That still needs §14.6. But
combined with §14.1, it gives the correlation §12.4 asks for: if short-lived connections turn out to
be the ones that opened at `6` and long-lived ones opened higher, the two-populations speculation is
confirmed and §9's mechanism is established rather than merely argued by elimination.

**Caveat, stated up front:** this is a second edit to the sketchbook ArduinoBLE, which §8 already
warns is invisible to git and dies on any library update. Keep it in `HCI.cpp` if possible - the file
already carries the patch banner - rather than spreading across a second file.

#### As built

**It is in `HCI.cpp` only** - the single file that already carries the §8 patch banner, so the whole
sketchbook footprint remains one file. The capture is taken at the source (the HCI *LE Connection
Complete* handler) rather than in `L2CAPSignaling.cpp`, at **both** connection-complete sites - the
plain one and the Enhanced one, which is easy to miss because only one of them is obvious.

```cpp
extern "C" {
volatile uint16_t exo_ble_cp_interval = 0;
volatile uint16_t exo_ble_cp_latency  = 0;
volatile uint16_t exo_ble_cp_timeout  = 0;
volatile uint16_t exo_ble_cp_count    = 0;   //0 = never connected
}
```

`extern "C"` so the firmware can declare them without matching C++ mangling. Values are stored
**raw**, in wire units, so nothing is lost to rounding.

**Repo side**, `exo_ble_cp_string()` in `SystemReset.h` renders `,CPi<iv>_l<lat>_t<to>_u<0|1>_n<n>`:

| field | meaning |
|---|---|
| `i` | interval, **1.25 ms units**. `i6` = 7.5 ms, `i24` = 30 ms |
| `l` | slave latency, in connection events |
| `t` | supervision timeout, **10 ms units**. `t500` = 5 s. The link's dead-man's switch - see §14.5 |
| `u` | **would we have sent an L2CAP update request?** Derived from `i` against our compile-time range using the same test as `addConnection()` |
| `n` | connections since boot. `n0` -> `,CPnone`, and `i`/`l`/`t` are meaningless |

**`u` is the field that earns this whole change.** It separates *"a relaxed interval helps"* from
*"not ASKING helps"* - the one alternative explanation an A/B/A cannot rule out by itself, because
`(6,6)` always triggers a request while `(12,24)` often will not.

#### Where it is delivered, and why not in the boot banner

The connection parameters do not exist at boot, so they cannot go in the reset banner the way
`exo_link_stats_string()` does. Instead the boot banner is **cached** (`s_boot_banner`) and ErrorChar
is **rewritten at connect**, in `handle_updates()`'s connection branch, with the CP field appended.

Two things make that safe, both worth recording because ErrorChar is `BLENotify`:

1. **Nobody is subscribed yet.** `QtExoDeviceManager` reads ErrorChar once (`:365`) and only then
   subscribes (`:392`). So `writeValue()` stores locally and emits **no packet** - nothing is added to
   the connect burst, which is the burst that already loses controller rows.
2. **It lands before that read.** This runs on the first main-loop pass after the link comes up; the
   GUI's read is seconds later. Losing the race would cost only the CP field - degraded, never wrong.

**And one real bug avoided:** the connect path must NOT call `exo_reset_reason_string()` again.
That function latches state out of GPREGRET as a side effect, and calling it twice is exactly how the
crash/stall record got erased once before. Hence the cached string rather than a recompute.

`s_boot_banner` is declared with the other file-scope statics at the top of `ExoBLE.cpp`, ahead of
`begin()`, which uses it.

**Cost:** +352 bytes flash, +16 bytes RAM on the Nano. Teensy unchanged. Both targets compile.

**A deliberate fragility:** if ArduinoBLE is ever updated or reinstalled, those `extern` symbols
vanish and **the firmware fails to LINK**. That is on purpose. A silent revert to "we have no idea
what the interval is" is precisely the situation §12 was written about; a link error is a much
better outcome than a banner that quietly stops telling the truth.

#### What to look for on the first flashed connection

- **`u1` on `B21`** would mean Windows opens outside 15-30 ms and we *are* still sending a request on
  the working build - which kills the "not asking is the fix" alternative outright.
- **`u0` on `B21` and `u1` on `B20`** would mean the arms differ in *both* interval and
  request-sent, leaving the two confounded and making §14.6 the only way to separate them.
- **`i` identical across arms** would be the loud result: Windows ignoring us entirely, and §9's
  mechanism would need rewriting from scratch even though the A/B/A effect is real.

### 14.3 The A/B/A control we never ran - **ARM A' DONE 2026-09-12: FAILURE RETURNED 2/2**

Still the single largest hole in §9's evidence. §9 rests on a **between-groups** comparison: ten
failures on the old build, two clean runs on the new one. Nothing has ever gone *back*.

This is worth doing even though it costs a run of deliberately-broken firmware, because the
alternative explanation for §9 has never been excluded: **something else changed between those two
sets of runs.** The ten failures and the two clean runs are separated by several days, a library
patch, a ping, a lock, and at least one host reboot. An A/B/A puts the interval back as the only
moving part.

**Promoted to first** at the user's direction - ahead of §14.1 - on the reasoning that §14.1's
attenuation repro is more informative once we know whether the current build is genuinely protected.

#### How to switch arms

One symbol, in `SystemReset.h`:

```c
#define EXO_BLE_INTERVAL_PINNED   1u
//        1 = (6, 6)   7.5 ms PINNED  - the original, known-failing configuration   -> build B20
//        0 = (12, 24) 15-30 ms range - the candidate fix from section 9            -> build B21
```

`ExoBLE.cpp` switches on it, and **`EXO_FW_TAG` is derived from it** rather than being a separate
number to remember. That is deliberate: flipping the interval and forgetting to bump the tag would
make the banner claim the wrong build, and the entire value of an A/B/A is that every log is
unambiguously attributable to one arm. Read the arm straight off the banner - `B20` or `B21`.

Both targets compile on this toggle (Nano 372,120 bytes / 37%; Teensy 324,460 bytes).

#### Protocol

1. **Arm A' - now.** `EXO_BLE_INTERVAL_PINNED 1`, flash, run a normal trial. Zero torque, on the
   bench. Record time-to-failure and which presentation (A: loop alive, link dead / B: everything
   frozen).
2. **Arm B - after.** Set it to `0`, flash, confirm the failure goes away again.
3. Optionally repeat A' once more. Two returns and two disappearances would be about as strong as
   this design can get without a sniffer.

#### RESULTS - arm A' (6,6), 2026-09-12, both runs in one session

| run | trial start | disconnect | **TTF** | banner |
|---|---|---|---|---|
| A'-1 | 15:57:03.131 | 15:57:13.707 | **10.6 s** | `SREQ,BLESTALL_n1_w10_s3,B20_b255_rt1_fw1,I2Cf3k_e288x10_c2_t266d_w0d_x300` |
| A'-2 | 15:58:51.852 | 16:00:07.507 | **75.7 s** | `SREQ,BLESTALL_n1_w10_s3,B20_b255_rt1_fw1,I2Cf9k_e285x10_c2_t266d_w0d_x300` |

**The failure came back, 2 for 2.** With the ten prior failures that is **12 for 12 on `(6, 6)`**.

`B20` on both confirms the toggle and makes these logs unambiguously arm A' - the thing the build-tag
derivation was for.

**The signature is byte-identical across both runs AND identical to B18's:**
`BLESTALL_n1_w10_s3`. One stall detection, worst `writeValue` in the 32.8-65.5 ms bucket (so the 50 ms
bounded spin fired), three sends since the last inbound byte. Three independent failures, one
fingerprint. This is one repeatable fault, not a family of coincidences.

**`c2` on both runs is the most load-bearing field here.** Per `SystemReset.h`'s own note, an
`endTransmission()` code of 2 means the Nano stopped ACKing the I2C bus - *the Nano died first*, and
the host noticed afterwards. Together with `x300` (capped: at least 3 s of unbroken I2C silence) both
runs say the same thing about ordering.

**Do not trust `f` and `e`.** The frame count rose 3k -> 9k while the error count *fell* 2880 -> 2850,
which no cumulative pair of counters can do. Whatever their reset semantics actually are, they are not
what the header comment implies, and nothing here should be built on them. (An earlier note in this
session called them "cumulative since Teensy boot" - that is wrong, and `e` falling is the proof.)
`c` and `x` are unaffected - they are a last-code and a max-run, not totals.

##### The hot-laptop confound, and why it is now mostly defused

A'-1 ran while the host laptop's fan was at full tilt, and 10.6 s is faster than *any* baseline failure
(minimum 12 s). That looked like it might be host thermal throttling rather than the interval.

**A'-2 answers it.** Same session, same thermal state, minutes later - **75.7 s**, squarely inside the
historical baseline (12-789 s, mean 225 s). A pathologically loaded host would have produced another
fast death, not a 7x longer one. So the hot laptop is not driving the result; A'-1 was a fast draw from
the same distribution we have always seen.

That 7x spread between two runs minutes apart, on one host in one thermal state, is itself worth
recording: **the process has a large stochastic component that is independent of host load.** It is
consistent with the exponential-ish time-to-failure already noted (CV 1.13), and with the
two-populations speculation in section 12.4.

#### Arm B (12,24) - **PASSED: 21 min 49 s, no failure, ended manually**

Flashed immediately after A'-2, **deliberately while the host laptop was still under the same load**,
so that the comparison is within-session rather than across days. Ran **1,309 s (21 min 49 s)** and
was **ended manually by the operator, not by a failure**. Banner `SREQ,B21_...` with no `BLESTALL`
suffix - the documented clean End-Trial reboot.

**THE A/B/A IS NOW COMPLETE AND POSITIVE:**

| arm | interval | result |
|---|---|---|
| A (original) | `(6,6)` | 10 failures |
| B | `(12,24)` | 2 clean 30-min runs |
| **A' (return)** | `(6,6)` | **2 failures, 0.96 s and 66 s, same session** |
| **B' (return)** | `(12,24)` | **1 clean 21m49s run, same session, same hot host** |

The failure went away, came back on cue, and went away again. That is the control §9 never had.

**Cumulative exposure on `(12,24)`: 5,026 s across three runs with zero failures.** Against the
null that B behaves like A (exponential, mean 225.5 s) that is ~22 expected failures, so
p ~ **2 x 10^-10**. Even this single run alone is **~1 in 333**. The earlier two runs were on a
different build and a cooler host, which is why the single-run figure is quoted alongside.

Survival probabilities under the null that B behaves like A, exponential against the full 12-failure
record (mean 225.5 s):

| survived | p |
|---|---|
| 7 min | ~1 in 6 |
| **15 min** | **~1 in 54** |
| 30 min | ~1 in 2,900 |

Against only today's hot-host pair (mean 43 s) 15 minutes is astronomically unlikely, but n=2 makes
that mean far too shaky to quote. The conservative column is the one to use.

##### RETRACTED: "B degrades under load and recovers"

**Written during the run from the GUI display, and the CSV does not support it. Retracted.**

Arm B's *data stream did not degrade at all*: 115,442 rows over 1,308.9 s, mean **88.2 Hz**, the
**worst single second was 73 Hz**, and in nearly 22 minutes there were exactly **two** gaps over
100 ms, totalling 0.3 s - 0.02% of the run. Max gap 145 ms. Nothing recovered because nothing broke.

What was visibly dropping and recovering was **the GUI's rendering**, not the link. The data kept
arriving at ~88 Hz the whole time.

So the "recoverability rather than throughput" reframing built on that observation is **withdrawn**.
It was wrong on both halves - B never degraded, and (see below) A' never degraded either.

*(It does fit host memory pressure hitting Qt repaints while leaving the BLE path alone - but that is
an aside, not a claim.)*

##### THE FAILURE IS ABRUPT - no ramp, no warning

This is the real finding in these files, and it rules out a whole class of models.

**A'-2 ran at 96.0 Hz with a p99 inter-sample gap of 41 ms right up to the sample before it stopped.**
Its worst second before the end was a full-rate second. There is no slow decay, no rising gap
distribution, no creeping loss. The link is healthy, and then it is dead.

Any model of the form *"congestion builds until it tips over"* is inconsistent with this. Whatever
happens, happens **fast** - between one sample and the next.

##### TIME-TO-FAILURE HAS BEEN OVERSTATED BY A CONSTANT 9.6 s IN EVERY RUN

Both A' runs show **exactly 9.6 s** between the last CSV sample and the GUI's
`Device disconnected` log line. Arm B's *manual* end shows 0.1 s. So the GUI timestamp is not when
the link died - it is when Windows finally noticed.

**CORRECTED 2026-09-12, same day:** that was first attributed to `EXO_BLE_STALL_MS` (8 s) plus
~1.6 s for Windows to notice the reboot. **Wrong.** The first §14.2 reading gives `t960` - Windows set
the **supervision timeout to 960 x 10 ms = exactly 9.6 s**. The lag IS the supervision timeout, by
definition: the central declares the link lost after 9.6 s without a valid packet. Two independent
measurements - CSV timing and an HCI event field - agreeing to the digit.

**Corrected times-to-failure, measured from the data rather than the log:**

| run | GUI-reported | **actual (last sample)** |
|---|---|---|
| A'-1 | 10.6 s | **0.96 s** |
| A'-2 | 75.7 s | **66.0 s** |

**A'-1 died in under one second** - the link was gone almost the instant the trial started.

**This applies retroactively to the whole failure record.** Every TTF in these documents taken from a
GUI disconnect line is ~9.6 s too long. It barely matters for the long ones; it matters enormously at
the short end, where the "12 s minimum" baseline failure was really about 2.4 s. The baseline mean of
225.5 s becomes ~216 s. Nothing in §9's conclusion moves, because the arms shift together.

##### THE INTERVALS DID DIFFER - measured from the CSVs, with no firmware change

Inter-sample gaps quantise to connection events, so the existing CSVs already carry a measurement of
the link timing. Histogramming them (2.5 ms bins, gaps under 70 ms):

| band | A' `(6,6)` | B `(12,24)` |
|---|---|---|
| < 2.5 ms (same connection event) | 44.0% | **52.3%** |
| 12.5-20 ms | **19.7%** (the mode) | 5.9% |
| 22.5-30 ms | 9.2% | **23.7%** (the mode) |

**The modal inter-burst gap moves from ~15-20 ms on A' to ~25-27.5 ms on B.** The two arms are
measurably different at the wire level: **Windows did not treat them identically.**

That already rules out one of the three outcomes pre-committed for §14.2 - *"`i` identical across
arms, Windows ignoring us entirely"*. It is not what happened.

B's shape is also exactly what `ExoBLE.cpp`'s comment predicted for a relaxed interval: a majority of
samples arriving **within** a connection event (52.3% under 2.5 ms) separated by one-event gaps near
26 ms. That is multiple packets per event, which at 7.5 ms the link never needed to do.

**What this does NOT establish:** the absolute value on arm A'. A 15-20 ms modal gap is consistent
both with a true 7.5 ms interval carrying a packet every second event (133 events/s against ~90
samples/s gives 0.68 packets per event, so 1-2 event gaps are expected) **and** with Windows having
quietly picked ~15-17.5 ms and ignored the request. Host-side Python timestamps smear the
quantisation, so this cannot separate them. §14.2 reads the number directly and settles it.

##### FIRST §14.2 READING, 2026-09-12 16:36 - `CPi24_l0_t960_u0_n1`

Build `B21`, first connection after flashing. Decoded:

| field | raw | **meaning** |
|---|---|---|
| `i24` | 24 | **connection interval = 30.0 ms** (24 x 1.25 ms) |
| `l0` | 0 | slave latency 0 |
| `t960` | 960 | **supervision timeout = 9.6 s** (960 x 10 ms) |
| `u0` | 0 | **no L2CAP update request was sent** |
| `n1` | 1 | first connection since boot |

**Windows opens this link at 30 ms on its own.** Our `(12, 24)` range is 15-30 ms, and 30 ms sits
exactly on its upper edge, so `addConnection()`'s test (`interval < _min || interval > _max`) is false
and **we ask for nothing at all** on the working build.

**1. The supervision timeout is measured, and it explains the 9.6 s constant** - see the correction
above. It also has a consequence nobody had noticed: **`EXO_BLE_STALL_MS` is 8 s, which is SHORTER
than the 9.6 s supervision timeout.** Our detector always wins that race, which is why every failure
shows `BLESTALL` rather than a clean stack-level disconnect. We have never once let the BLE stack's own
link-loss mechanism run to completion.

**2. It is evidence FOR §14.5's broken-supervision-timer theory.** With a 9.6 s timeout, a peripheral
whose supervision timer works should notice a dead link within 9.6 s and drop to advertising by itself.
Presentation A, before the stall detector existed, left the Nano unreachable **indefinitely** - far
longer than 9.6 s. So Windows' supervision timer fired (it logged "link lost" on schedule) and **the
Nano's did not**. That asymmetry is exactly what PR #44's `WSF_MS_PER_TICK` complaint predicts: the
nRF52840/mbed supervision timer running at the wrong rate. §14.5 moves up the list.

**3. `u0` means the confound is REAL, not ruled out.** This is outcome #2 of the three pre-committed
above, and it is the awkward one:

| | arm A' `(6,6)` | arm B `(12,24)` |
|---|---|---|
| request sent? | **yes** (30 outside [6,6]) | **no** (`u0`, measured) |
| interval in force | ~15-20 ms (from CSV gaps) | **30 ms** (measured) |

The arms differ in **both** variables. So *"a relaxed interval helps"* and *"not ASKING helps"* are
still both alive, and the A/B/A cannot separate them - as §12.3 warned.

**But the CSV histogram does rule out the strongest version of the sceptical case.** If Windows simply
ignored the `(6,6)` request, arm A' would also have run at 30 ms and shown B's ~26 ms modal gap. It
showed ~15-20 ms instead. So **Windows did act on the request** - though evidently not down to 7.5 ms.
The likely reading: Windows moved from its 30 ms default toward our request and stopped at its own
floor (~15 ms is a documented Windows lower bound for non-HID peripherals), i.e. a *partial* grant.

##### WINDOWS IS DETERMINISTIC - four independent connections, identical

Four connections on `B21`, each from a **separate boot** (End Trial `'Z'` reboots the Nano, which is
why every one reads `n1` - so these are four fresh negotiations, not four reads of one):

| time | reading |
|---|---|
| 16:36:49 | `CPi24_l0_t960_u0_n1` |
| 16:45:59 | `CPi24_l0_t960_u0_n1` |
| 16:46:31 | `CPi24_l0_t960_u0_n1` |
| 16:47:11 | `CPi24_l0_t960_u0_n1` |

**Byte-identical, four for four.** Windows picks 30 ms / latency 0 / 9.6 s every time, with no
variance at all.

**This narrows §12.4's two-populations speculation but does not kill it.** What is now measured is
that Windows' *opening* choice is perfectly stable. §12.4 was about whether Windows' *response to a
request* is random - and arm B never sends one (`u0`), so these four readings cannot speak to it.
Testing that needs arm A' readings, which is what the next section makes possible.

##### A LIMITATION IN §14.2 AS FIRST BUILT - and the fix (`UP`)

`exo_ble_cp_capture()` fires on **LE Connection Complete**, which is the value the link *opens* at -
before any L2CAP request has even been sent. So on arm A' it would have read `CPi24_..._u1` and told
us only *that* we asked, never what Windows did about it. **The central question of §12 would still
have been unanswered.**

Fixed by handling the event ArduinoBLE has never handled: **LE meta subevent 0x03, LE Connection
Update Complete.** Upstream's `LE_META_EVENT` enum has no enumerator for it and its switch has no
case, which - together with `connectionParameterUpdateResponse()` being empty - is *why* the library
can send a request and never learn its fate. The local patch adds the case (with a cast, so 0x03 stays
out of `HCI.h` and the whole patch remains in one file) and records the result.

New banner field, emitted only when such an event actually arrived:

```
,UPi<interval>_t<timeout>_s<status+1>_n<count>
```

**Its absence is as informative as its presence:**

| banner | meaning |
|---|---|
| `u1` and **no `UP` field** | We asked and **the central never answered at all.** This is the "no negotiation room, so the host gives up" hypothesis, directly observed |
| `u1` + `UPs1` | **Granted.** `UPi` is the interval now actually in force - the number §12 has wanted since it was written |
| `u1` + `UPs` > 1 | **Rejected**, with HCI status (`s` - 1) |
| `u0` and no `UP` | Nothing asked, nothing changed - the expected arm B shape |

Status is stored **+1** so that 0 can mean "no event seen"; `s1` = HCI status 0x00 = SUCCESS. Both `UP`
values are zeroed on every new connection, so they always describe the current one.

**Second timing bug, found and fixed before flashing.** The update event lands tens to hundreds of ms
*after* the connection completes, so the banner write in the connection branch is always too early to
contain it. `handle_updates()` now refreshes the banner whenever `exo_ble_cu_status` changes, placed
**before** the unchanged-status early return (the status does not change when an update arrives, so
anything after that return would never run) and gated on `!_tx_subscribed` - which confines writes to
the pre-subscribe window, where no notification can be generated and the refresh still lands ahead of
the GUI's read at ~1.9 s.

##### §14.9 THE EXPERIMENT THAT BREAKS THE CONFOUND - one line, decisive

`u0` makes this both possible and necessary. **Choose a range that excludes Windows' 30 ms default -
so a request IS sent - but which is otherwise almost identical to 30 ms.**

```c
BLE.setConnectionInterval(20, 23);   // 25.0 - 28.75 ms
```

30 ms falls outside `[20, 23]`, so `updateParameters` becomes true and we get `u1`. If Windows honours
it we land at ~28.75 ms - **4% away from the 30 ms that just survived 22 minutes.** The link timing is
effectively unchanged; the only thing that meaningfully changes is that **we asked**.

| result | conclusion |
|---|---|
| **Survives** (20+ min) | Sending a request is harmless. The *interval value* was the killer. §9 confirmed, the sceptical alternative dead |
| **Fails** | **Sending the request is the cause**, not the interval. §9's fix is real but its mechanism is wrong, and the true fix is "never ask Windows for anything" |

Either way it resolves the single largest remaining ambiguity, for one line and one run.

**BUILT 2026-09-12 as arm C / build `B22`.** Both targets compile. Selected by a second toggle in
`SystemReset.h`, with `EXO_FW_TAG` derived from both so the banner can never misreport the arm:

```c
#define EXO_BLE_INTERVAL_PINNED   0u   // 1 -> (6,6)   = B20
#define EXO_BLE_INTERVAL_ARM_C    1u   // 1 -> (20,23) = B22   (requires PINNED 0)
                                       // both 0       = B21 = (12,24)
```

`exo_ble_cp_string()`'s `our_min`/`our_max` follow the same selector, or `u` would be computed against
a range we never asked for.

##### RECOMMENDED ORDER: `B20` FIRST, AND IT COSTS ABOUT A MINUTE

Arm C needs 20+ minutes to mean anything. **Arm A' on `B20` needs about one minute**, because it dies
in 1-66 s - and with the `UP` field it now answers the question §12 was written about:

- `u1` + no `UP` -> Windows **never replied** to `(6,6)`. The "no negotiation room, host gives up"
  hypothesis, observed rather than argued.
- `u1` + `UPs1_i<n>` -> granted, and `UPi` is **what arm A actually ran at** - currently known only as
  a ~15-20 ms bound inferred from CSV gap histograms.
- Several `B20` connections also finally test §12.4 directly: if `UPi` varies connection to connection,
  that is the two-populations mechanism confirmed.

**It also changes how to read arm C.** If `B20` shows Windows never answers `(6,6)`, then arm C stops
being "asked vs did not ask" and becomes the much sharper "a request that gets answered vs one that
gets ignored" - a far more specific claim, and one worth reporting upstream on issue #45.

Keep the range rather than pinning `(23,23)`: pinning min = max is itself the property under suspicion,
and mixing it in here would reintroduce the confound this experiment exists to remove.

##### ARM C FIRST READING - `CPi24_l0_t960_u1_n1,UPi12_t960_s1_n1`

Build `B22`, `setConnectionInterval(20, 23)` = 25.0-28.75 ms requested. 2026-09-12 17:04.

| field | meaning |
|---|---|
| `CPi24` | opened at **30 ms**, as on all four B21 connections |
| `u1` | **a request WAS sent** - as designed |
| `UPs1` | **HCI status 0x00 = SUCCESS. Windows GRANTED it.** |
| `UPi12` | the interval now in force is **15.0 ms** |
| `UPt960` | supervision timeout unchanged, 9.6 s |

**We asked for 25.0-28.75 ms. Windows said SUCCESS and gave us 15.0 ms** - below our requested
minimum, a value we never asked for and explicitly excluded.

##### MY ARM C DESIGN IS VOID AS A CONFOUND-BREAKER - stated plainly

§14.9 assumed that requesting 25-28.75 ms would land us near 28.75 ms, isolating "we asked" from
"the interval changed". **Windows defeated that by ignoring the requested value entirely.** Arm C is
therefore **not** a confound-breaker - it is a *replication of arm A'*: request sent, 15 ms in force.
The experiment as designed does not do what it was built to do, and §14.9's table should not be used.

##### WHAT IT DOES ESTABLISH - and it is a lot

**1. Windows' actual policy is now visible: ANY parameter-update request -> 15 ms.** Regardless of what
was requested. It replies SUCCESS and applies 15 ms. Two independent requested ranges - `(6,6)` =
7.5 ms and `(20,23)` = 25-28.75 ms - and the same 15 ms result.

**2. It confirms the CSV inference, by a completely independent route.** The gap histogram put arm A'
at a ~15-20 ms modal inter-burst gap, and that was explicitly flagged as unable to distinguish "true
7.5 ms with a packet every second event" from "Windows quietly picked ~15 ms". **Direct HCI readout now
says 15 ms.** Two methods, one answer.

**3. §12 IS SUBSTANTIALLY CLOSED, and its headline claim is confirmed false:**

| arm | request? | **interval actually in force** |
|---|---|---|
| A / A' `(6,6)` | yes | **15 ms** (never 7.5 ms) |
| B `(12,24)` | no (`u0`) | **30 ms** |
| C `(20,23)` | yes | **15 ms** (measured) |

**"The link ran at 7.5 ms" is false. It never did, on any build.** Windows never honoured it.

**4. §9's arithmetic was wrong and needs correcting.** §9 argued from "7.5 ms demands 133 connection
events per second". At 15 ms it is **66.7 events/s** - half that. Every number in §9 derived from 133
events/s is void. (The already-withdrawn "no headroom to drain" argument was therefore doubly wrong.)

**5. The real variable is 15 ms versus 30 ms - a factor of two, not a factor of four.** That is the
whole difference between 12 failures and 5,026 s of clean running.

##### THE CONFOUND IS WELDED SHUT BY WINDOWS, NOT BY OUR CHOICES

On this host, **request sent <=> 15 ms**, and **no request <=> 30 ms**. The two are perfectly
correlated *by Windows' own behaviour*. So no choice of requested range can separate them: you cannot
get 15 ms without asking, and you cannot ask without getting 15 ms.

**Prediction for the running arm C trial: it should FAIL**, on the same timescale as A' (1-66 s), since
it is the same condition. If it does, that is a *replication* of A' reached by a different requested
range - which strengthens §9, because it shows the failure tracks **the interval in force**, not our
particular choice of numbers.

##### §14.10 ARM D - the confound-breaker that should actually work

`addConnection()` has a **second** trigger for `updateParameters`, and it does not touch the interval:

```cpp
uint16_t updatedMinInterval = interval;   // = 24, the central's OWN value
uint16_t updatedMaxInterval = interval;   // = 24
if (_minInterval && _maxInterval) {
    if (interval < _minInterval || interval > _maxInterval) { ...override... }   // NOT taken
}
if (_supervisionTimeout && supervisionTimeout != _supervisionTimeout) {
    updatedSupervisionTimeout = _supervisionTimeout;
    updateParameters = true;                                                     // taken
}
```

So:

```cpp
BLE.setConnectionInterval(12, 24);     // contains 30 ms -> interval branch NOT taken
BLE.setSupervisionTimeout(800);        // 8 s != Windows' 960 -> request IS sent
```

This sends a request whose **requested interval is min = max = 24 - exactly the 30 ms Windows already
chose.** A request goes out, but it asks Windows to keep what it has.

| outcome | conclusion |
|---|---|
| `UPi12` again (15 ms) | Windows goes to 15 ms on **any** request. Confound confirmed structural, unbreakable on this host, and the mechanistic question is academic *here* |
| `UPi24` + `UPt800` | **CONFOUND BROKEN.** A request sent, 30 ms retained. Survival then isolates "does asking, by itself, hurt?" |

It also doubles as §14.5's supervision-timeout test, which §14.2's `t960` reading promoted. One wrinkle
to note honestly: it requests min = max, which is the pinned property under suspicion - but pinned at
**30 ms**, and only if Windows chooses to honour it at all.

##### §14.11 A CHEAP WAY TO MAP WINDOWS' RESPONSE FUNCTION

The `UP` field makes this nearly free, and it needs **no trial at all** - just connect, read the banner,
disconnect. About a minute per probe.

Flash a sequence of requested ranges and record `UPi` for each: `(6,6)`, `(10,10)`, `(16,16)`,
`(20,23)`, `(24,24)`, `(25,30)`, `(40,60)`. If `UPi` is 12 for every one of them, Windows has a single
hard-coded response to being asked and the picture is complete. If it tracks the request anywhere in
that span, there is a usable range and the confound may be escapable after all.

Worth doing before spending any more 20-minute runs, because it determines which long runs are even
meaningful.

##### §14.12 IS THIS WINDOWS BEHAVIOUR KNOWN? - searched 2026-09-12

**Partly, and the part that is documented matches us well. The specific thing we measured is not
documented anywhere found.**

**What IS documented:**

- **Windows accepts an L2CAP parameter-update request and then does not apply it.** Reported in detail
  on [Nordic DevZone](https://devzone.nordicsemi.com/f/nordic-q-a/27235/win-10-ble-stacks-acting-as-a-central-problems-accepting-ble-connection-parameter-changes)
  by `ianm`, with sniffer logs: link opens at 20 ms, peripheral requests 40 ms, the L2CAP
  request/response pair appears **three times**, and the `LL Connection Update Indication` **never
  follows**. Interval stays at 20 ms. The same peripheral code negotiates correctly against Android.
  Also on [Microsoft Q&A](https://learn.microsoft.com/en-us/answers/questions/848142/windows-10-ble-connection-parameter-update-issue)
  and [MSDN](https://social.msdn.microsoft.com/Forums/vstudio/en-US/a6b0526d-f729-46f1-b0c8-35e995dc4bb0/windows-does-not-answer-ble-parameter-update-request?forum=wdk).
- **A >20 ms threshold.** Requests above ~20 ms, and slave latency >1, "usually don't take" on Win10.
  **We asked for 25-28.75 ms - above that threshold.**
- **Nordic's official advice is to NOT use L2CAP requests at all.** David Edwin (Nordic) recommends the
  **PPCP characteristic** (Preferred Peripheral Connection Parameters) as the way to express
  preferences to Windows, and says not to send update requests more often than every 30 s.
- **Windows 11 has exactly three connection-parameter PRESETS**, and no custom values:
  `BluetoothLEPreferredConnectionParameters` - **Balanced**, **ThroughputOptimized**,
  **PowerOptimized** ([Microsoft](https://learn.microsoft.com/en-us/uwp/api/windows.devices.bluetooth.bluetoothlepreferredconnectionparameters)).
  Introduced in Windows 11 build 10.0.22000. [btframework](https://www.btframework.com/connparams.htm)
  states plainly that through this API you **"cannot set any custom values"**, and that on earlier
  Windows there is no supported way at all.
  **Microsoft does not publish the actual interval values for the three presets.**

**What is NOT documented anywhere found:** our exact observation - Windows replying **SUCCESS and then
applying a value we never asked for and had explicitly excluded** (`UPi12` = 15 ms against a requested
25-28.75 ms). The literature describes "accepts then does nothing". Ours *did* something, just not what
was asked.

##### §14.13 HYPOTHESIS: Windows is not negotiating, it is SWITCHING PRESETS

**Clearly labelled a hypothesis.** It is not documented, but it fits every measurement we have, and it
is the simplest thing that does:

> Windows does not treat an L2CAP request as a value to negotiate. It treats it as a **signal that the
> peripheral wants different parameters**, and responds by switching to one of its three fixed presets.
> With no request it sits on **Balanced**. Any request is read as "this device wants more throughput",
> so it switches to **ThroughputOptimized** - whatever number was actually asked for.

| observation | does the model explain it? |
|---|---|
| 30 ms on 4/4 connections with no request (`u0`) | yes - Balanced |
| 15 ms after requesting 7.5 ms (arm A', from CSV) | yes - ThroughputOptimized |
| 15 ms after requesting 25-28.75 ms (arm C, measured) | yes - same preset, request value irrelevant |
| Status `SUCCESS` rather than reject | yes - Windows genuinely *did* change something |
| Requested value ignored entirely | yes - there is nothing to negotiate, only presets |
| "cannot set any custom values" in Microsoft's own API | consistent - presets are all Windows has |

**It makes a sharp prediction:** §14.11's probe sequence will return `UPi12` for **every** requested
range. If any probe returns something other than 12, this model is dead.

**And it implies something uncomfortable: we have been doing the one thing guaranteed to hurt us.**
Asking for a *faster* interval and asking for a *slower* one produce the identical result - Windows'
fast preset - so the only way to get its slow preset is **never to ask at all.** Which is exactly what
arm B does, by accident, and exactly why arm B is the only configuration that has ever survived.

##### §14.14 A HOST-SIDE LEVER WE HAVE NEVER CONSIDERED

If §14.13 is right, the firmware is the wrong place to be fighting this. Windows 11 exposes the
presets directly, and **this machine is Windows 11 Pro 10.0.26200, so the API is available**:

- `BluetoothLEDevice.RequestPreferredConnectionParameters(...)` - the **supported** way to select a
  preset, from the host side.
- `BluetoothLEConnectionParameters.ConnectionInterval` - a **read** API. The GUI could log the interval
  in force without any firmware or library patch at all.

Reachable from the GUI through bleak's WinRT backend (`BleakClient._requester` is the
`BluetoothLEDevice`). Two things worth knowing before trying it:

1. **We would want `Balanced` or `PowerOptimized`, NOT `ThroughputOptimized`.** Throughput-optimised
   means a *shorter* interval, which by everything above is the direction that kills the link. This is
   the opposite of the instinct the name invites.
2. It is host-side, so it does not travel with the exo - a labmate's laptop would not have it unless
   the GUI does it. That makes it a **complement** to the firmware fix, not a replacement.

The read API is worth doing regardless: it is a second, independent measurement of the interval, from
the other end of the link, with no library patch to lose on the next ArduinoBLE update.

##### §14.15 ARM C IS STILL ALIVE AT 5 MINUTES - and that is a problem for §9

§14.9's prediction was that arm C would fail on A''s timescale, because both run at 15 ms. At 5 minutes
(~1 in 4 under the null, so not yet conclusive) it has not. **If arm C survives long, the interval
hypothesis is in trouble**, because A' and C would be the same interval with opposite outcomes.

**The gap in the evidence is specific and cheap to close: we never read arm A''s `UP` field.** A' being
15 ms rests on (a) a CSV gap-histogram mode of 15-20 ms and (b) the *assumption*, from arm C, that
Windows always lands on 15 ms. If A' actually got something else - or got several update events, which
`UPn` would show, and which `ianm`'s three-times-repeated request makes plausible - then A' and C are
not the same condition and §9 survives intact.

**So `B20` with the `UP` field is now the single highest-value flash available**, and it costs about a
minute. It was already the recommendation; arm C surviving makes it urgent rather than merely useful.

##### §14.16 ARM C SURVIVED 20 MINUTES - AND THE BANNER WAS LYING TO US

**Arm C ran 1,050 s and was cut short by the operator, not by a failure.** And the CSV says it was
**not** running at the 15 ms its banner reported.

Measured with a burst-aware estimator - group samples separated by <8 ms into one burst (they share a
connection event), then histogram **burst-start to burst-start** spacing in 1.25 ms BLE units. This is
the right estimator here because the three arms have very different burst fractions (34-52% of samples
arrive inside an event), which badly skews any naive inter-sample statistic:

| arm | mean burst-to-burst | bursts/s | samples/burst | modal spacing | outcome |
|---|---|---|---|---|---|
| **A' `(6,6)`** | **21.7 ms** | **46.1** | 2.08 | broad, units 12-17 | **DIED at 66 s** |
| B `(12,24)` | 28.7 ms | 34.9 | 2.53 | units 21-23 | clean 1,309 s |
| **C `(20,23)`** | **29.6 ms** | **33.7** | 2.81 | **unit 23 = 28.75 ms, sharp (37.5% in two bins)** | **clean 1,050 s** |

**Arm C's sharpest peak is at 28.75 ms, which is EXACTLY the maximum we requested** (`(20,23)` =
25.0-28.75 ms). Windows honoured the request and picked the slowest value we allowed. Its peak is the
sharpest of the three arms, which is what an explicitly granted interval should look like.

**So `UPi12` was a TRANSIENT.** Windows went to 15 ms at connection time - which is what the banner
captured at ~1.9 s - and then moved to 28.75 ms afterwards. The banner refresh is gated on
`!_tx_subscribed`, so anything after the GUI subscribes is invisible to it. **The "mid-connection change
we cannot see" limitation, which §14.15 listed as a known gap and declined to build for, turns out to be
a real and consequential phenomenon rather than a hypothetical.** The CSV caught what the banner could
not.

##### §14.17 RETRACTED: "my arm C design is void as a confound-breaker"

**That was written off the transient banner reading and it was wrong. Arm C worked exactly as designed.**
§14.9's original table applies after all, and §14.16 is the row it predicted.

§14.13's preset hypothesis ("any request -> 15 ms, requested value irrelevant") is **dead**: Windows
honoured the requested range and landed inside it. Windows is negotiating, not switching presets - it
just takes a detour through 15 ms on the way.

##### §14.18 THE CONFOUND IS BROKEN, AND §9 IS CONFIRMED

This is the result the whole §14 sequence was built to get:

| arm | request sent? | interval in force | outcome |
|---|---|---|---|
| A' `(6,6)` | **yes** | **~21.7 ms (fast)** | **DIED, 12/12** |
| B `(12,24)` | no (`u0`) | ~28.7 ms | clean |
| C `(20,23)` | **yes** | ~29.6 ms | **clean, 1,050 s** |

**Arm C sends a request and survives. Arm A' sends a request and dies.** The two differ in the interval
and not in whether a request was sent.

- **"Not asking is what helps" is DEAD.** Arm C asks, and lives.
- **"The interval value is what matters" is CONFIRMED** by a controlled comparison, not by elimination.

§12.3's argument-from-elimination and §12.5's "NOT VERIFIED" row can both be retired. The mechanism
question that has been open since §9 was written now has a measured answer at the level of *which
variable*: **it is the connection interval.**

**Total clean exposure at a relaxed interval is now 6,076 s across four runs** (B: 1,821 + 1,896 + 1,309;
C: 1,050) against **12/12 failures** at the fast interval.

##### §14.19 WHAT IS STILL OPEN

1. **A''s exact interval.** 21.7 ms mean burst spacing is clearly faster than the survivors' ~29 ms, but
   the distribution is broad (units 12-17) and does not quantise cleanly at multiples of either 6
   (7.5 ms) or 12 (15 ms). Host timestamps are too jittery to resolve it, and - now that `UPi12` is
   known to be a transient - the banner alone will not settle it either. **B20 is still worth the
   minute, but read its CSV too, not just its banner.**
2. **Where the threshold is.** We know ~22 ms dies and ~29 ms lives. The boundary is unlocated, and
   `(12,24)` sits close enough to it to be worth knowing about.
3. **WHY a faster interval is fatal.** Still unanswered, and §9's old arithmetic cannot be reused: at
   46 bursts/s rather than the assumed 133 connection events/s, every number in §9 derived from 133
   is void.
4. **Why Windows detours through 15 ms at all**, and whether that transient is itself harmful.

##### A NOTE ON THE INSTRUMENT

The banner and the CSV disagreed, and **the CSV was right.** Worth remembering: the banner is a
single sample taken ~1.9 s into a connection, and the link can move afterwards. The traffic is the
ground truth for what actually happened during a trial. Any future reading of `CPi`/`UPi` should be
cross-checked against the burst-spacing estimator above before it is trusted.

##### §14.20 B20 READING - `CPi23_l0_t960_u1_n1,UPi6_t960_s1_n1`. **WINDOWS GRANTED 7.5 ms.**

2026-09-12 17:26, build `B20`, `setConnectionInterval(6, 6)`.

| field | meaning |
|---|---|
| `CPi23` | link **opened at 28.75 ms** - NOT the 30 ms seen on all four B21 connections |
| `u1` | request sent, as expected |
| `UPs1` | **HCI status 0x00 = SUCCESS** |
| **`UPi6`** | **the interval granted is 6 x 1.25 = 7.5 ms - the BLE spec MINIMUM** |

**§9 WAS RIGHT ALL ALONG. The link really did run at 7.5 ms.** Windows honoured `(6,6)` completely,
down to the floor of the specification.

##### §14.21 RETRACTED: "the link never ran at 7.5 ms, on any build"

Written in §14.x earlier today on the strength of arm C's reading, and **wrong**. The reasoning chain
that produced it:

1. Arm C's banner said `UPi12` (15 ms) against a requested 25-28.75 ms.
2. From that single data point I generalised to "Windows applies 15 ms to any request" (§14.13).
3. Arm A' was then *assumed* to be 15 ms, and "7.5 ms" declared false.

Step 2 was an over-generalisation from n=1, and step 1 turned out to be a transient (§14.16). Both the
preset hypothesis and the "never 7.5 ms" claim are dead. **§9's original account - including its
133-connection-events-per-second arithmetic - is restored and confirmed.** The earlier note voiding that
arithmetic is itself void.

**The CSV agrees, once read correctly.** A'-2's mean burst-to-burst spacing was 21.7 ms, which I read as
"the interval is 15-20 ms". It is not the interval - it is **~3 connection events**: 133 events/s against
46.1 bursts/s is 2.9 events per burst, and 3 x 7.5 ms = 22.5 ms, against 21.7 ms measured. The broad hump
at units 12-17 is 2-3 events of 6 units each, smeared by host timestamp jitter. **Burst spacing measures
events-per-burst x interval, not the interval** - a correction that applies to every use of that
estimator above.

##### §14.22 THE FINAL PICTURE - all intervals MEASURED, not inferred

| arm | requested | **granted (measured)** | conn events/s | outcome |
|---|---|---|---|---|
| **A / A' `(6,6)`** | 7.5 ms | **7.5 ms** (`UPi6`, SUCCESS) | **133** | **12 / 12 FAILED** |
| B `(12,24)` | nothing sent (`u0`) | ~30 ms | 33 | **clean, 5,026 s** |
| C `(20,23)` | 25-28.75 ms | **28.75 ms** | 35 | **clean, 1,050 s** |

- **The interval is the variable.** Arm C sends a request and lives; arm A' sends a request and dies.
- **7.5 ms is fatal. ~29-30 ms is not.** 4x in interval, 4x in connection events per second.
- **§12 is CLOSED.** Every interval in this table is now a measurement.

##### §14.23 NEW AND OPERATIONALLY IMPORTANT: WINDOWS PERSISTS THE INTERVAL ACROSS CONNECTIONS

`CPi23` is the surprise. Every B21 connection opened at `i24` = 30 ms. This one opened at **`i23` =
28.75 ms - precisely the value arm C had negotiated in the previous session.** Windows carried the
negotiated interval forward into a later, separate connection, after a reflash and a GUI restart.

**So "Windows' default" is not a constant - it is the last value it settled on with this device.** That
has consequences that reach backwards and forwards:

- **The "Windows is deterministic, 4/4 identical" finding (§ above) is conditional.** Those four
  connections all followed B21 sessions, which never send a request, so 30 ms simply persisted. It was
  not independence; it was inertia.
- **TEST ORDER NOW MATTERS, and this is a live hazard.** A `B21` run immediately after a `B20` run may
  **open at 7.5 ms**. And because 7.5 ms is below `(12,24)`'s minimum of 12, that connection WILL send a
  request (`u1`) rather than the `u0` every previous B21 connection showed - so it is a materially
  different condition from the B21 runs that produced 5,026 s of clean running.
- **Every future reading must record what ran before it.** A banner alone is no longer self-describing.
- It may also explain otherwise-unaccounted historical variance: sessions inherited whatever the previous
  session left behind.

**Practical consequence: after any `B20` run, check `CPi` on the next connection before trusting it.** If
it opens at `i6`, the carry-over is in effect and the run is not comparable to the earlier B21 runs.

##### §14.24 THE PUBLISHED WINDOWS THRESHOLDS DO NOT APPLY TO THIS BUILD

§14.12 collected reports that Win10 will not accept intervals above ~20 ms and will not go below ~15 ms.
**This Windows 11 build (10.0.26200) did both:** it granted **7.5 ms** (below the reported floor) and
**28.75 ms** (above the reported ceiling), both with status SUCCESS. Windows 11's stack is evidently more
compliant than the Win10 reports describe, and those thresholds should not be carried forward into any
reasoning about this host.

##### §14.25 B20's CSV - 7.5 ms CONFIRMED FROM TRAFFIC, and the 11-minute survival explained

The B20 run was cut by the operator at **637.7 s** with no failure, which looked like a direct
contradiction of `(6,6)` failing 12/12. It is not. Two separate questions, both answered here.

**Q1: was B20 really at 7.5 ms, or was `UPi6` a transient like arm C's `UPi12`?**

| run | bursts/s | samples/burst | mean burst-to-burst | modal peak | implied interval |
|---|---|---|---|---|---|
| A'-2 `(6,6)` DIED 66 s | 46.10 | 2.08 | 21.7 ms | spread u12-u16 | 133.3/46.10 = **2.89 events x 7.5 = 21.7** |
| **B20 `(6,6)` cut 637.7 s** | **52.23** | **1.84** | **19.1 ms** | **u12-u13, 40% in two bins** | 133.3/52.23 = **2.55 events x 7.5 = 19.1** |
| C `(20,23)` cut 1,050 s | 33.72 | 2.81 | 29.6 ms | u23 sharp | **1 event x 28.75 = 28.75** |
| B `(12,24)` cut 1,309 s | 34.85 | 2.53 | 28.7 ms | u21-u23 | ~1 event x ~28 |

**B20 ran FASTER than the run that died** - 19.1 ms mean burst spacing against A'-2's 21.7 ms, 52.2
bursts/s against 46.1. Its modal peak sits sharply at **2 connection events of 7.5 ms** (u12 = 15.00 ms,
u13 = 16.25 ms, 40% of all gaps in those two bins), and the arithmetic closes exactly: 133.3 events/s at
7.5 ms divided by 52.23 bursts/s = 2.55 events per gap, x 7.5 ms = 19.1 ms measured.

**So `UPi6` was NOT a transient. B20 genuinely ran at 7.5 ms, and option (a) is dead.** §14.20's reading
is confirmed by a second, independent method - the same cross-check that caught arm C's transient now
vindicates B20's banner.

**Q2: then why did it survive 11 minutes?**

**Because it was cut inside `(6,6)`'s normal failure range.** The 12 recorded `(6,6)` failures are
exponential-ish with a mean of ~216 s and a **maximum of 789 s**. B20 was stopped at **637.7 s - below
that maximum.** It had not survived the distribution; it simply had not failed yet when the operator
ended it.

- P(survive 637.7 s | mean 216 s) = **5.2%**, about 1 in 19.
- Across 13 `(6,6)` attempts the *expected* number of runs lasting that long is 13 x 0.052 = **0.68**.
  **Observing one is exactly what the distribution predicts.**

**This run is a statistical fluctuation, not a counter-example.** Had it been allowed to continue it would
most likely have failed; at 637.7 s the hazard is unchanged, since an exponential process has no memory.

**And option (b) - "7.5 ms is necessary but not sufficient, the host must also be stressed" - is NOT
established by this run.** It remains possible, but this run is not evidence for it, because the run never
had to explain anything: its length is ordinary for `(6,6)`.

##### §14.26 WHY THE RELAXED-INTERVAL RUNS *ARE* EVIDENCE, AND THIS ONE IS NOT

The distinction is entirely about whether a run exceeds the fast interval's observed failure range:

| | longest run | vs `(6,6)`'s 789 s maximum |
|---|---|---|
| B20 `(6,6)` | 637.7 s | **inside** - proves nothing |
| C `(20,23)` | 1,050 s | **beyond** |
| B `(12,24)` | 1,309 s | **beyond** |
| B `(12,24)` earlier | 1,821 s and 1,896 s | **far beyond** |

Pooling properly: `(6,6)` has **12 failures in ~3,240 s of total exposure**, a rate of one per 270 s.
The relaxed arms have **0 failures in 6,076 s**, where that rate predicts **22.5**.
P(0 failures) = e^-22.5 ~ **1.7 x 10^-10**.

**The interval effect survives this run intact.** §14.22's conclusion stands, and the "RESOLVED" header is
justified - with the proviso recorded here that `(6,6)`'s failure time has a **fat tail reaching at least
789 s**, so **no single short-to-medium `(6,6)` run can ever demonstrate safety.** Anyone re-testing the
fast interval needs to beat 789 s before a survival means anything, and realistically several runs.

##### §14.27 THE OPERATOR'S TWO OBJECTIONS, ANSWERED

Both were raised against §14.23 and both were worth raising.

**"If Windows persists the interval, A' followed a `(12,24)` session and should have survived."**
Persistence sets the **opening** interval only; a request then overrides it. B20's own banner shows both
steps in one line: `CPi23` (inherited 28.75 ms) -> `UPi6` (request granted, 7.5 ms). A' opening at 30 ms
and being pulled to 7.5 ms by its own request is exactly what persistence predicts, and it died. No
contradiction. **However the objection correctly exposed that persistence rests on n=1** - `i23` once,
after one session that negotiated `i23`, against `i24` four times after sessions that asked for nothing.
§14.23 was written with more confidence than one observation supports. **Still to test:** reconnect
several times and watch whether `CPi` holds or drifts.

**"B20 is surviving too."** Answered above: cut at 637.7 s, inside `(6,6)`'s known range, p = 5.2%,
expected to happen about once in 13 attempts. **The objection was right to demand the CSV** - had B20
turned out to be running at ~29 ms, §14.20 would have collapsed.

##### §14.28 SECOND B20 SESSION - `CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1`. THREE NEW FACTS.

2026-09-12 17:39, same `B20` build, second session.

| field | meaning |
|---|---|
| **`CPi6`** | the link **OPENED at 7.5 ms** - the value the *previous* B20 session negotiated |
| **`u0`** | **we sent NO request.** 6 is inside `[6,6]`, so `addConnection()`'s test is false |
| **`UPi12_s1`** | yet an update event arrived anyway: **Windows moved the link to 15 ms, unprompted** |

**1. PERSISTENCE IS CONFIRMED, n=2.** §14.27 downgraded it to a single observation after the operator
rightly objected. This is the second: `i23` carried over from arm C, and now `i6` carried over from the
previous B20 session - across a GUI restart. **Windows remembers the negotiated interval per device.**

**2. WINDOWS MOVES THE INTERVAL ON ITS OWN.** With `u0` we asked for nothing, and Windows still issued a
Connection Update to 15 ms. This is new, and it **re-explains arm C's transient**: the `UPi12` there was
never a response to our request at all - Windows has its own drift toward ~15 ms. So `UPi12` showing up in
two completely different configurations is Windows' own behaviour, not a reply to us.

**3. THIS RUN IS NOT A 7.5 ms TEST.** It is running at ~15 ms. Which makes it, by accident, **the missing
middle rung of the ladder** (§14.19 item 2): we have 7.5 ms fatal and ~29-30 ms clean, and 15 ms untested.
**Pending its CSV - `UPi` has been a transient before and must always be cross-checked.**

##### §14.29 THE TRAP: CONSECUTIVE `(6,6)` RUNS ARE NOT THE SAME EXPERIMENT

Because of persistence, `(6,6)` is **self-defeating after its first session**:

| session | Windows opens at | inside `[6,6]`? | request? | ends up at |
|---|---|---|---|---|
| 1st B20 | 28.75 ms (from arm C) | no | **yes** -> granted | **7.5 ms** |
| 2nd B20 | **7.5 ms** (from session 1) | **yes** | **no** | Windows drifts to **15 ms** |

`(6,6)` only pins 7.5 ms when Windows happens to open somewhere else. Once it has settled at 7.5 ms, our
range contains it, nothing is sent, and Windows is free to relax on its own.

**Consequence that reaches backwards:** some of the original 12 `(6,6)` failures may not have been at
7.5 ms at all. They all failed, so if any ran at 15 ms then **15 ms is also fatal** - which is exactly what
the current run will tell us.

**And it means every interval claim needs its CSV, not just its banner.** A banner now has three possible
relationships to reality: what we asked, what Windows opened at, and where Windows drifted to.

##### §14.30 A RISK TO THE SHIPPED FIX - `(12,24)` CANNOT CORRECT A 15 ms CARRY-OVER

**This is the practically important consequence and it was not visible before today.**

`(12,24)` means 15-30 ms. **15 ms is inside that range.** So:

> If Windows ever settles at 15 ms, a `(12,24)` build **sends no request** (`u0`) and **cannot pull the
> link back up**. The session runs at 15 ms.

Every clean B21 run so far opened at `i24` = 30 ms, because each followed another B21 session - inertia,
not safety (§14.23). **A `(12,24)` run immediately after a `(6,6)` run can open at 7.5 or 15 ms and stay
there.** That is a live hazard for the next person who tests in that order, and it is also exactly the
situation a labmate's laptop could end up in after one bad session.

**RECOMMENDED CHANGE: `setConnectionInterval(20, 24)` = 25-30 ms** instead of `(12, 24)`.

| | `(12,24)` = 15-30 ms | **`(20,24)` = 25-30 ms** |
|---|---|---|
| Windows opens at 30 ms | accepts (`u0`) | accepts (`u0`) |
| Windows opens at 15 ms | **accepts - NO correction** | **outside -> request sent -> pulled up** |
| Windows opens at 7.5 ms | outside -> corrected | outside -> corrected |
| Proven survivable? | 5,026 s clean | 28.75 ms proven by arm C (1,050 s) |

`(20,24)` excludes every interval we have reason to distrust while staying inside the band that has
actually been proven clean. It is strictly safer than `(12,24)` and costs nothing - arm C already
demonstrated that a request being sent is harmless (§14.18).

**Do not make this change until the current 15 ms run reports.** If 15 ms turns out to be perfectly safe,
`(12,24)` is fine as it stands and this is unnecessary churn. If 15 ms fails, `(20,24)` becomes mandatory.

##### §14.31 ON HOST STRESS - the operator's observation

The operator notes RAM is still above 90% and the fan unchanged from when the fast failures occurred.
**That argues against option (b)** ("7.5 ms needs host stress as a co-factor") as an explanation for the
previous B20 run surviving 637.7 s, and in favour of the simpler account already given in §14.25: that run
was **cut inside `(6,6)`'s fat tail**, at a 5.2% survival probability, which across 13 attempts is expected
to happen about once. No host-state explanation is needed, and the unchanged host state removes the only
motivation for inventing one.

##### §14.32 THE 15 ms RUN DIED AT 118 s. `(12,24)` IS UNSAFE AND IS SUPERSEDED.

The second B20 session - the one Windows had quietly moved to 15 ms (§14.28) - **failed at 118 s**
(127.8 s GUI-reported minus the 9.6 s supervision lag), with the unchanged fingerprint
`BLESTALL_n1_w10_s3`, `c2`, `x300`.

**Its interval is confirmed at 15 ms from the traffic, by a hard constraint rather than an inference:
bursts per second can never exceed connection events per second.**

| run | bursts/s | -> interval must be <= | ~15 ms gaps | ~30 ms gaps | interval | outcome |
|---|---|---|---|---|---|---|
| B20a | 52.23 | 19.1 ms | 51.1% | 8.9% | **7.5 ms** | cut 638 s |
| **B20b** | **41.09** | **24.3 ms** | **12.3%** | **44.8%** | **15 ms** | **DIED 118 s** |
| C | 33.72 | 29.7 ms | 2.6% | 62.4% | 28.75 ms | clean 1,050 s |

B20b's 41.09 bursts/s **excludes 28.75 ms and 30 ms outright** (only 34.8 and 33.3 events/s exist at
those intervals), and its gaps are cleanly bimodal at 1x15 and 2x15 ms. Banner and traffic agree.

**THE MEASURED LADDER:**

| interval | outcome |
|---|---|
| **7.5 ms** | 12 / 12 failures, mean ~216 s, max 789 s |
| **15 ms** | **DIED at 118 s** |
| 28.75 ms | clean 1,050 s |
| ~30 ms | clean 5,026 s across three runs |

**So §14.30's risk is real, not hypothetical: `(12,24)` = 15-30 ms CONTAINS a fatal interval.** A host
that has persisted 15 ms is accepted with `u0`, no request is sent, and the firmware cannot pull the link
back up. **And Windows has 15 ms persisted on this machine right now**, so flashing `(12,24)` next would
very likely run an entire trial at the interval that just died in under two minutes.

**`(12,24)` is superseded. It survived 5,026 s only because every one of those runs inherited 30 ms from
a previous 30 ms session - inertia, not protection (§14.23).**

##### §14.32b 15 ms CONFIRMED FATAL, 2 / 2 - replicated immediately

A second run in the identical condition (banner `CPi6_l0_t960_u0_n1,UPi12_t960_s1_n1` again - opened at
7.5 ms persisted, `u0` so nothing requested, Windows moved it to 15 ms on its own) **failed at ~429 s**
(438.5 s GUI-reported minus the 9.6 s supervision lag). Same fingerprint yet again:
`BLESTALL_n1_w10_s3`, `c2`, `x300`.

| run | bursts/s | interval must be <= | ~15 ms gaps | ~30 ms gaps | TTF |
|---|---|---|---|---|---|
| 15 ms #1 | 41.09 | 24.3 ms | 12.3% | 44.8% | **118 s** |
| 15 ms #2 | 38.99 | 25.6 ms | 22.6% | 12.1% | **429 s** |
| 28.75 ms (C) | 33.72 | 29.7 ms | 2.6% | 62.4% | clean 1,050 s |

Both runs' burst rates **exclude 28.75 ms and 30 ms outright**, and both show the 1x / 2x 15 ms gap
structure. Banner and traffic agree on both.

**So §14.32 was not resting on a single run.** 15 ms fails at 118 s and 429 s - a spread consistent with
the same fat-tailed process seen at 7.5 ms, and well inside it.

**UPDATED LADDER, all measured:**

| interval | result |
|---|---|
| **7.5 ms** | **12 / 12 failures** (mean ~216 s, max 789 s) |
| **15 ms** | **2 / 2 failures** (118 s, 429 s) |
| 28.75 ms | clean 1,050 s |
| ~30 ms | clean 5,026 s over three runs |

**14 failures at <= 15 ms. Zero failures in 6,076 s at >= 28.75 ms.** The threshold is between 15 and
28.75 ms, and the decision to abandon `(12,24)` is now backed by two independent failures rather than one.

**Also newly non-zero in this banner:** `w271d` - the Teensy's worst gap between successful RT
transmissions reached **27.1 s**, where every earlier banner read `w0d`. And `e300x10` is very likely at
its clamp. Neither changes the conclusion; both are consistent with a longer run spent largely dead.

##### §14.33 PRODUCTION DEFAULT IS NOW `(20, 24)` = 25-30 ms - build `B23`

```c
#define EXO_BLE_INTERVAL_SEL      0u
//   0 = (20,24) = 25.0-30.0 ms  -> B23  PRODUCTION DEFAULT
//   1 = ( 6, 6) =  7.5 ms       -> B20  FATAL, experiment only
//   2 = (12,24) = 15.0-30.0 ms  -> B21  SUPERSEDED, accepts a fatal 15 ms
//   3 = (20,23) = 25.0-28.75 ms -> B22  arm C
```

| Windows opens at | `(12,24)` | **`(20,24)`** |
|---|---|---|
| 30 ms | accept (`u0`) | accept (`u0`) |
| **15 ms** | **accept - NO correction, FATAL** | **outside -> request -> pulled up** |
| 7.5 ms | outside -> corrected | outside -> corrected |

`(20,24)` excludes every interval measured or suspected fatal while staying inside the band actually
proven clean. **It is also what recovers this machine from its current 15 ms carry-over.** Arm C already
established that sending a request is harmless (§14.18), so the correction costs nothing.

Kept as a range rather than pinning `(24,24)`: min == max was the property under suspicion for this whole
investigation and there is no reason to reintroduce it.

**Refactor while here:** the two-boolean scheme (`EXO_BLE_INTERVAL_PINNED` / `_ARM_C`) had a precedence
wart - PINNED silently overrode ARM_C - and is replaced by the single `EXO_BLE_INTERVAL_SEL`.
`EXO_FW_TAG` and `exo_ble_cp_string()`'s `our_min`/`our_max` both derive from it, so the banner, the
requested range and the compiled interval cannot disagree. Both targets compile.

##### §14.35 FIRST `B23` CONNECTION - `CPi6_l0_t960_u1_n1,UPi24_t960_s1_n1`. THE CORRECTION WORKS.

2026-09-12 17:58, build `B23`, `setConnectionInterval(20, 24)` = 25-30 ms.

| field | meaning |
|---|---|
| `CPi6` | opened at **7.5 ms** - persisted from the B20 sessions, exactly the hazard §14.30 described |
| **`u1`** | **a request WAS sent** - 6 is below our minimum of 20, so the range correctly rejected it |
| **`UPi24_s1`** | **Windows GRANTED 30.0 ms** |

**This is §14.33's design working on its first attempt, measured rather than hoped for.** The machine was
sitting on a persisted 7.5 ms - an interval that has failed 12/12 - and `(20,24)` detected it, asked, and
was granted 30 ms. Under `(12,24)` the same machine would have been *corrected* too (7.5 < 12), but under
`(12,24)` a **15 ms** carry-over would have been silently accepted, which is what made it unsafe.

**PERSISTENCE CONFIRMED n=3:** `i23` after arm C, `i6` after B20a, `i6` again now. It is a settled fact,
not the single observation §14.27 had to downgrade it to.

##### §14.36 A CLEAN RULE: WINDOWS GRANTS THE **MAXIMUM** OF THE REQUESTED RANGE

Two independent data points, and they agree:

| requested | granted |
|---|---|
| `(20, 23)` = 25.0-28.75 ms | **28.75 ms** - our max |
| `(20, 24)` = 25.0-30.0 ms | **30.0 ms** - our max |

Windows takes the **slowest value we allow**, which is consistent with a host that prefers to save power.
That gives the two ends of the range completely separate jobs, and it is worth stating plainly because it
makes the parameter easy to reason about:

- **`max` is what you actually get.** Set it to the interval you want.
- **`min` is only a trigger threshold.** It decides what Windows values get corrected: anything faster than
  `min` falls outside the range, so a request fires. It is **not** a floor you might be given.

So `(20,24)` reads as *"correct anything faster than 25 ms, and when correcting, go to 30 ms."* And
`(12,24)`'s defect is exactly that its `min` sat **below a measured-fatal value**, so 15 ms was inside the
accept band. This rule is why `(23,24)` would be the tighter option if 25 ms ever needs excluding too - it
changes only the trigger, not the outcome.

##### §14.37 THE RESIDUAL RISK - WE CAN ONLY CORRECT AT CONNECTION SETUP

`L2CAPSignalingClass::addConnection()` runs **once, at connect.** Nothing in the firmware can correct the
interval after that.

And we know Windows moves intervals unprompted: B20b opened at 7.5 ms with `u0` and Windows took it to
15 ms on its own (§14.28). **If Windows ever drifts a 30 ms link down mid-session, nothing pulls it back,
and the banner will not show it** - the banner refresh is gated on `!_tx_subscribed`, so it only ever
reports the first ~2 s of a connection.

**Empirically this has not happened:** four runs at ~28.75-30 ms have stayed clean for 1,050-1,896 s, and
Windows' observed drift has been *upward* toward whatever we asked for, never downward. But it is unproven,
and **the CSV burst-rate check is the only way to confirm what a trial actually ran at.** That check should
stay part of the routine for any run that matters, which is the single most useful habit to come out of
today.

##### §14.34 WHAT IS SETTLED, AND WHAT IS NOT

**Settled, by measurement:**

- The connection interval is the cause. Arm C sends a request and survives; A' sends one and dies.
- 7.5 ms and 15 ms are both fatal. 28.75 ms and 30 ms are not.
- The fatal threshold lies **between 15 ms and 28.75 ms**.
- Windows persists the negotiated interval per device, across reflashes and GUI restarts (n=2).
- Windows also moves the interval **on its own**, with no request from us (`u0` + `UPi12`).

**Not settled:**

1. **WHY a faster interval is fatal.** Entirely open. §9's original story was about `_pendingPkt`
   saturation under 133 connection events/s - but 15 ms is only 66.7 events/s and still kills, so
   whatever the mechanism is, it bites at half that rate too.
2. **Where between 15 and 28.75 ms the threshold sits.** `(20,24)` is chosen to stay clear of it rather
   than to locate it.
3. **Whether 25 ms is genuinely safe**, or merely untested. Only 28.75 and ~30 ms have run long.
   `(20,24)` permits 25 ms if Windows picks it. If that ever worries us, `(23,24)` = 28.75-30 ms is the
   tighter version, confined entirely to proven values.

##### THE COST OF THE FIX, measured

| | A'-2 `(6,6)` | B `(12,24)` |
|---|---|---|
| mean rate | 96.0 Hz | **88.2 Hz** |

**About 8% fewer samples per second.** That is the throttling the `ExoBLE.cpp` comment said to watch
for, and it is real but modest - and it buys a link that does not die. Worth stating plainly rather
than leaving as a footnote, because it is a genuine trade and somebody should get to decide whether
8% matters for their protocol.

##### What this does and does not yet establish

- **Established:** on this host, today, `(6, 6)` fails reliably and fast, with one repeatable signature.
  Arm A' is a solid arm. It is no longer true that "nothing ever went back."
- **Not yet established:** that `(12, 24)` is what prevents it. That is arm B, and it is only decisive
  if it survives **in this same session, on this same hot host** - which is the whole point of running
  it now rather than tomorrow.
- **Still unestablished either way (section 12):** *why*. Nothing here reads back the interval actually
  in force. Even a clean arm B would tell us THAT the setting matters, not WHY.
- **One alternative an A/B/A cannot kill by itself:** `(6, 6)` *always* triggers the L2CAP parameter
  request, because Windows never opens a link at exactly 7.5 ms, whereas `(12, 24)` may often need no
  request at all. So "B survives" might mean *not asking* is what helps, rather than a relaxed interval.
  Section 12.3 rejects that (one packet at connect cannot kill a link 40-800 s later) and that rejection
  still stands - but **section 14.2 would settle it directly**, because it logs the `updateParameters`
  flag alongside the interval. If arm B survives, 14.2 becomes the next thing to build.

#### What each outcome means

| arm A' result | reading |
|---|---|
| Fails within ~4 min, as the baseline did (mean 225 s, worst 789 s) | **§9 is causal.** The interval is the variable, and everything from §12.3's argument-by-elimination gets replaced by an actual controlled comparison |
| **Does not fail**, runs 30 min clean | **§9 is in serious trouble.** Something else fixed it between the two groups - the library patch, the ping, or the host reboot - and §9 needs rewriting. This is the outcome worth taking seriously, and the reason to run the control at all |
| Fails, but much later than 225 s | Ambiguous on one run. The baseline itself spanned 12 s to 789 s, so a single long survival is weak evidence - see §12.4 on the two-populations speculation. Needs repeats |

**One caution on interpretation:** arm A' is *not* a clean re-run of the original ten failures. The
library patch, the ping, the TX lock and the CSV fix are all still in place. So a failure on A' is
attributable to the interval, but a *survival* on A' does not by itself exonerate the interval - it
could mean one of those other changes is doing the work. That asymmetry is why the "does not fail"
row above says §9 is in trouble rather than refuted.

### 14.4 Log RSSI alongside the telemetry

`central.rssi()` is available and we never use it. Sample it once a second into the existing stream.

- If §13.5 reading 1 is right, **RSSI should sag in the seconds before a failure** - which would
  convert "random" into "predictable", and is the single most useful thing we could learn.
- If RSSI is flat and strong right up to the instant it dies, that is strong evidence our fault is
  *not* the range-driven one in #45, whatever the symptoms share.
- Watch specifically for **RSSI reading exactly 0**, which polldo and Hoffa25 both report as a marker
  of a link in trouble on this stack.

Cheap, repo-side (no library edit), and it costs one channel.

### 14.5 Set the supervision timeout explicitly - a candidate FIX for presentation A, not just a probe

**This is the most interesting untried idea on the list, and it is a fix rather than a measurement.**

Presentation A is *"the link is dead but the Nano still believes it is connected, so it never
returns to advertising."* The supervision timeout is the **exact mechanism the BLE spec provides for
noticing that** - it is the peripheral's dead-man's switch: no valid packet from the central within
the timeout, and the controller must declare the link lost and report a disconnect.

Two facts make this worth pursuing:

1. **We never set it.** `BLE.setSupervisionTimeout()` exists in 2.1.0 (`BLELocalDevice.cpp:408`) and
   is not called anywhere in `ExoBLE.cpp`. `L2CAPSignaling`'s `_supervisionTimeout` is therefore 0,
   so we accept whatever Windows picks and never request anything else.
2. **There is upstream history of this timer being wrong on this exact board.** PR #44 - the one that
   closed issue #33, the "peripheral does not notice disconnection" issue - was in part a fix for
   `WSF_MS_PER_TICK` not matching the value mbed was compiled with, with the PR author noting *"the
   `WSF_MS_PER_TICK` needs to match the value mbed was compiled with for the supervision timeout to
   match."* A tick-rate mismatch makes the supervision timer run at the wrong rate - and a timer
   that runs too slow is, precisely, a peripheral that never notices the link is gone. Users
   reported #44 did not actually resolve it.

So: call `BLE.setSupervisionTimeout()` with a short, explicit value and see whether presentation A
starts self-recovering - the controller reports a disconnect, our existing disconnect handling runs,
and the main loop's `advertising_onoff()` puts it back on the air without a reboot.

**Three cautions, none of them small:**

- **Units are 10 ms**, per the BLE spec, range 0x000A-0x0C80 (100 ms to 32 s), and the spec requires
  `timeout > (1 + latency) * maxInterval * 2`. With `(12, 24)` = 30 ms max and zero latency the floor
  is 60 ms, so anything from about 100 ms up is legal - but a value that tight will drop the link on
  ordinary interference. Something in the **2-5 s** region is the sane starting point. Verify the
  units against the library rather than trusting this paragraph.
- **It changes the L2CAP request too.** Look again at `addConnection()`: setting `_supervisionTimeout`
  makes `updateParameters` true whenever the central's value differs, so we would start sending a
  parameter-update request on *every* connection, including ones where the interval already suited
  us. That is a second moving variable, and it partially re-entangles this with §12. Run §14.2 first so
  the request is at least visible.
- **If the tick rate really is wrong, setting the value does not fix the rate.** The request would be
  honoured in name and still expire at the wrong wall-clock time. That failure mode would look like
  "we set 3 s and it disconnects after 30 s, or never" - which is itself a diagnosis, so the test is
  still worth running.

### 14.6 BLE sniffer - the only source of ground truth

An nRF52840 dongle running `nRF Sniffer for Bluetooth LE` into Wireshark, ~£10 and an evening.

It is the only way to settle, rather than infer:

- The actual connection interval in force, continuously, including after any L2CAP update.
- Whether Windows sent *Connection Parameter Update Response (accepted)* and then ignored it - the
  documented Windows behaviour in §12.2, and currently pure inference on our part.
- What the last packets before a failure look like, and **which side stops transmitting first.** That
  single observation separates "the Nano's radio stopped" from "Windows stopped scheduling" and would
  cut the remaining hypothesis space roughly in half.
- Whether the supervision timeout is expiring at its nominal wall-clock value (§14.5).

Ranked last only because of setup cost. If §14.1 through §14.5 leave us still guessing, this stops
being optional.

### 14.7 Worth doing regardless of diagnosis: make the GUI reconnect itself

Not a test - a mitigation, recorded here so it is not lost. The GUI currently never attempts to
reconnect; every failure needs a human. An automatic retry would not fix presentation A (the device
is genuinely unreachable until it resets), but combined with the watchdog it would close the loop:
device reboots itself, GUI notices and re-attaches, the trial continues with a gap in the data rather
than ending.

Deliberately **not** done yet, because auto-reconnect would mask exactly the events we are currently
trying to count. It belongs in the §11 cleanup phase, after the diagnosis is settled.

### 14.8 Suggested order

**Revised 2026-09-12 at the user's direction:**

1. **§14.3** A/B/A on the interval - **in progress.** Arm A' is built and flashed; does the failure
   come back? Answering this first means §14.1 is then run against a build we know the status of.
2. **§14.1** attenuation repro - changes the cost of everything below it.
3. **§14.2** log the negotiated parameters - **BUILT, awaiting a flash.**
4. **§14.4** log RSSI - repo-side, pairs with §14.1 to test the weak-signal hypothesis directly.
5. **§14.5** explicit supervision timeout - the candidate fix for presentation A.
6. **§14.6** sniffer - if the above has not settled it.

**None of this blocks tomorrow's labmate stress test.** That run answers a different and more
immediate question - does the current build survive a real session on a healthy host - and should go
ahead unchanged, with nothing added that could perturb it.
