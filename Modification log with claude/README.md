# Modification log — index

Human-readable "what / why / how to modify" write-ups for changes made in this repo, one file per
investigation. Deeper design specs and implementation plans live in the `specs/` and `plans/` subfolders here
(moved out of a top-level `docs/` tree on 2026-09-08; `docs/` no longer exists).

**Last audited: 2026-08-12.** Every status line below was checked against the code and git history
on that date. If you add a document, add a row here.

---

## Live documents

Read these. They describe the current state of the code.

| Document | Date | What it covers | Status |
|---|---|---|---|
| `Nano-Disconnect-EVIDENCE-ONLY.md` | 2026-09-11 | **Observations only, no interpretation** - run durations, verbatim reset banners, LED sightings, the I2C counter readings, the CSV timing/torque analysis, and a list of what was explicitly NOT measured. Created because the narrative document had accumulated confident explanations that later proved wrong and it became hard to separate measured from inferred | **Current.** The companion narrative/theory document is `Nano-Hang-Watchdog-And-Breadcrumbs.md` |
| `Nano-Hang-Watchdog-And-Breadcrumbs.md` | 2026-09-10 | **Proves the mid-trial Nano freeze is a HANG** - not a crash, reset or brownout - on two independent channels: a warm RESET-BUTTON read of `RESETREAS` (`RESETPIN` alone, so nothing ever reset the MCU) and the onboard LED freezing on one endpoint of `ComsLed::life_pulse()`'s green/blue toggle (so `loop()` stopped). Adds the nRF52840 watchdog (5 s, fed from `loop()`, armed at the END of `setup()`) plus a `GPREGRET2` stage breadcrumb reported as `,STAGE_n_dogN`. Fixes three boot-loop/evidence-destroying bugs in the watchdog<->crash-trap interaction. **Also catches a SECOND failure mode** - `loop()` alive, RT data flowing, LED still blinking, but the Nano never notices the disconnect and never re-advertises - traced to the unbounded `while (_pendingPkt >= _maxPkt)` spin in ArduinoBLE 2.1.0 `HCI.cpp:636`, which explains BOTH modes and why the trigger was never reproducible. Adds a 2 s GUI ping plus a firmware stall detector that warm-resets and reports `,BLESTALL_n`, i.e. it self-confirms the diagnosis. **Touches `Python_GUI/` too.** Clears the Teensy, the host/GUI and the config timeout as causes | **THE FAILURE IS AN INTERRUPT-LEVEL STOP (2026-09-11, §3.8)** - the Nano stops ACKing I2C for >=3 s AND both LEDs freeze, so it is not servicing interrupts at all, which is far narrower than a stalled loop. Mode 2 also confirmed by the device (`SREQ,BLESTALL_n1`). Root cause still unknown; B16 is a `REAL_TIME_I2C 0` bisect. Also: GPREGRET2 and `.noinit` both fail to survive a watchdog reset so every `STAGEn`/`g` reading in the doc is meaningless, the RT I2C link runs at a chronic 4-13% NACK rate which likely explains the long-standing RT sample loss, and the 2 s ping is NOT the fix. **2026-09-12: this turned out to be ArduinoBLE upstream issue #45, open since 2019 and still unfixed in the latest release - see §13. It independently confirms the §8 diagnosis, independently reports our "device alive but unreachable" case, and independently reports that the §8 patch does NOT fix that case. §14 is the ranked list of what to test next.** **CANDIDATE FIX FOUND 2026-09-11 (§9): the connection interval was pinned at the BLE minimum (7.5 ms) as both min AND max, leaving the host no room to schedule. Relaxed to 15-30 ms, and a 30-minute run followed at full telemetry rate (90.2 Hz vs 92.0 Hz reference) against a baseline that failed in 50-789 s - ~1 in 3,700 under the null. ONE RUN, not yet confirmed.** §8 documents a bounded-wait patch to the SKETCHBOOK ArduinoBLE (outside this repo, dies on any library update). §10 covers two other defects found along the way: a ping/command write collision that could wedge the Nano's parser, and the CSV time column still wrapping every 655 s because the 2026-07-21 fix only touched the plot. **§11 is a phased removal plan** for the diagnostic scaffolding once the fix is confirmed. Originally 2026-09-10 - the watchdog fired and the Nano recovered itself, the first self-recovery of this whole investigation.** That run exposed two bugs in the change (dog armed before anything fed it, so it false-tripped on the boot path as `STAGE_0`; and a dog reset erased any pending `BLESTALL` record) - both fixed in §3.6, recompiled clean, **fixes not yet flashed**. Watchdog committed `d76707d`, stall detector `8d214fd`, §3.6 fixes uncommitted. **TEMPORARY scaffolding - §5 is a complete mechanical dismantling procedure** |
| `External-Control-Orchestrator.md` | 2026-09-08 | The new `Python_GUI/external_control/` package: an external program that owns an optimization backend (game theory / SPT) and drives the exo by writing `splineAlt` parameters through the GUI's UDP remote, confirming every write against the firmware ack. Covers the five wire-level traps (silence as the only failure signal, acks with no request id, the swallowed fast ack, controller-switch-loads-defaults, bilateral double-ack), the mode menu, and UDP timing mode. Also logs this session's docs move, the corrected `splineAlt` status, the `disconnection_troubleshooting` branch audit, and the deferred 20/25 Nm plantar torque investigation | **Bench-proven on the real exo 2026-09-09 for the `OP`, `plantar`, `dorsi` and `udp` modes; `game_theory_mode` still untested against hardware.** User audit: `main_external_control.py` done, the two `Utilities/` files still in progress |
| `Nano-Reset-Pin-Spurious-Reset.md` | 2026-09-09 | First bench data from the `RESETREAS` instrument. Every spontaneous freeze reads **`RESETPIN`**, while a deliberate End-Trial `'Z'` correctly reads `SREQ` on the same hardware. Rules out `LOCKUP` (no hard fault, kills the ArduinoBLE-spin theory) and `PORBOR` (not a brownout, and not a hand power-cycle). Leaves the nRESET line being pulled low. Flags `Board.h:423` `rst_pin = 4`, declared for this board version and never used by the firmware, as a possible floating conductor on the reset net | **CONFOUNDED as of 2026-09-09 evening - see the correction banner at the top.** Power-on appears to read `RESETPIN` on this board (no `PORBOR` in 13 readings despite power cycling), so the readings carry no information and the `LOCKUP` exclusion is withdrawn. Needs the GPREGRET fix before it can be trusted. No code change made |
| `Mid-Trial-Freeze-Nano-Radio-Silence.md` | 2026-09-02 | The random mid-trial plot freeze + "unexpectedly disconnected". Proves the trigger is a **BLE link supervision timeout** (a 9.6 s constant across 8 freezes, identical to the End-Trial `'Z'` reboot fingerprint), that the **Teensy is alive to the last sample** so the Nano is what dies, and — via the Cordio RTOS thread — that a hung `loop()` **cannot** produce it. Rules out RF range. Also measures chronic 6–47 % real-time stream loss and the ~607 s stress-test ceiling. Ships a `RESETREAS` readout to identify crash vs. hang vs. brownout | **Trigger explained, root cause still UNKNOWN — the document says so plainly.** Instrument committed as `8f79871` on `disconnection_troubleshooting`; compiles clean for `nano33ble` (4.6.0) **and** `teensy41`, but **never flashed, never bench-validated**. Read §7 "Known limits" before acting on a `PORBOR` result |
| `SplineAlt-Shape-Parameterised-Controller.md` | 2026-08-27 | The `splineAlt` ankle controller: a curve built from lobe shape parameters (peak/rise/dwell/fall/magnitude/scale) instead of 12 nodes, with a periodic spline that wraps across the 0/100 % gait seam. Also the reference account of the BLE handshake **column ceiling** (`PREFIX_COLS = 4`, 30 usable columns, silent truncation), the 9-character name limit, the Nano/Teensy `MAX_MESSAGE_SIZE` name collision, and the per-controller handshake payload budget. Also covers **unwiring TREC and SPV2 from the ankle** (chirp/step deliberately kept as characterization tools) and why their enum IDs were left as gaps | **Implemented, uncommitted, host-verified only.** Compiled with `g++` and checked against scipy — **never built for Teensy, never flashed, never run on motors** |
| `Spline-Run-Analysis-And-RT-Stream-Fix.md` | 2026-08-12 | End-to-end analysis of running the Spline controller from the GUI (F1–F10 + safety hazards + logging blind spots); branch comparison vs `backup_branch_with_UW_edits`; the Teensy→Nano→GUI real-time stream fix; the error-manager disable; the F5 uncalibrated-sensor guard | **Current.** Contains three of its own in-document corrections (F1, F6, F8) — read the retraction blocks, not just the headings |
| `End-Trial-Diagnosis-Correction.md` | 2026-08-10 | The authoritative account of the End-Trial lock-up: says plainly that the root cause is **unknown**, and lists the defensive fixes that make the failure class impossible anyway | **Current.** Supersedes both files in `superseded/` |
| `Fresh-Torque-Path-Safety-Audit.md` | 2026-08-11 | Complete map of every CAN transmit site and every path torque can reach the motor by; why the `enable()`/`zero()` frames are dangerous on an AK60v3 | **Current, with a caveat banner.** Its "three accidents" verdict and its 51.4 Nm figure were overtaken by `3c08c77` / the current-decode investigation — see the banner at the top |
| `Motor-Current-Decode-Investigation.md` | 2026-08-11 | Why the SD motor logs show impossible currents; the AK60v3 MIT field scaling (`_I_MAX` 10.3 vs the motor's ±12.0) | **Current.** Itself retracts an earlier "decode is 6× wrong" claim |
| `SD-Card-Logging-and-End-Trial-Reset.md` | 2026-07-06→08 | The onboard SD logger and the end-trial reset / shutdown handshake | **Current** as a description of the feature. NB the logger is currently **disabled** in `config.ini` (`sdLogEnabled = 0`) — see the round-2 spline doc for why |
| `SD-Logger-Disabled-Behavioral-Audit.md` | 2026-08-10 | Whether the SD logger can affect behaviour while disabled. Answer: no | **Current** |
| `Motor-Freeze-Controller-Change-And-End-Trial.md` | 2026-07-22 | The AK60v3 hold-last-command freeze on controller change and End Trial; the `_sd_ready()` and deferred-reset fixes | **Current**, status line corrected 2026-08-12 (committed as `a6413c9`) |
| `Spline-Jitter-Diagnosis.md` | 2026-07-22 | Round 1 of the spline jitter hunt: the missing gain scheduler and the integer-quantized `percent_gait` | **Current**, branch reference corrected 2026-08-12. Extended by the 08-12 analysis |
| `Spline-Jitter-Round-2-SD-Logging-Regression.md` | 2026-07-23 | Round 2: SD logging as a loop-rate regression, and the spline node change | **Current** |
| `BLE-Handshake-Controller-List-Loss.md` | 2026-07-23 | Controllers vanishing from the GUI dropdown; RF row loss during the handshake; the detection that was shipped | **Current**, with a known gap in the exact row check noted at the top |
| `Heel-FSR-Disable.md` | ~2026-07-06 | Runtime heel-FSR gating and FSR-refinement status visibility | **Current** |
| `Remote-Control-UDP.md` | 2026-07-18 | The localhost UDP listener for programmatic control of the GUI | **Current**, status corrected 2026-08-12 (validated on the exo, `60dd3ed`) |

## `superseded/`

Kept for the record — this is the history of a hardware-damage investigation and should not be
deleted — but **do not act on the conclusions in these files.**

| Document | Why it moved | Anything still live in it? |
|---|---|---|
| `superseded/End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` | Root cause **retracted 2026-08-10**. The chain it describes cannot occur: `check_response()` early-returns on `user_paused`, and the `'w'` handler sets `user_paused` atomically with `enabled = 0`, so the `'w'` vs `'G'` race it is built on does not exist | Yes — its §"artifact" section (the GUI plotting samples it never logs at End Trial) was **not** retracted and is cited by `SD-Logger-Disabled-Behavioral-Audit.md`. The frame decode and the damage record are also unaffected |
| `superseded/Branch-Comparison-End-Trial-Regression-And-Must-Keep-Edits.md` | **Part 1 retracted 2026-08-10** — it inherited the disproven arming chain above | **Yes, and it matters: Part 2 still stands** — the must-keep vs optional tiering of our edits versus `backup_branch_with_UW_edits`. The measured CAN-starvation asymmetry in Part 1 also stands (it was measured, not inferred) |

---

## How to read the End-Trial thread

Five documents touch it. In order:

1. `SD-Card-Logging-and-End-Trial-Reset.md` — the original reset/shutdown design.
2. `Motor-Freeze-Controller-Change-And-End-Trial.md` — the AK60v3 hold-last-command mechanism.
3. `superseded/End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` — a root cause that turned
   out to be wrong.
4. `superseded/Branch-Comparison-End-Trial-Regression-And-Must-Keep-Edits.md` — Part 1 built on 3
   (wrong); Part 2 is independent (live).
5. **`End-Trial-Diagnosis-Correction.md` — start here.** It disproves 3 and 4-Part-1 and states
   plainly that the root cause is still unknown.

## Conventions

- Every document opens with **Date / Scope / Status**, and says whether the code was compiled,
  flashed and tested. Most of this work was verified on the host only — take those lines literally.
- When a conclusion is disproven, add a banner at the top of the original rather than editing the
  reasoning away, and record the correction in the document that supersedes it.
- Cross-reference by filename so a reader can follow the thread.
