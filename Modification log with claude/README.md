# Modification log — index

Human-readable "what / why / how to modify" write-ups for changes made in this repo, one file per
investigation. Deeper design specs and implementation plans live in the `specs/` and `plans/` subfolders here
(moved out of a top-level `docs/` tree on 2026-09-08; `docs/` no longer exists).

**Last audited: 2026-09-29.** Every status line below, and the status line at the top of every document,
`specs/` and `plans/` file, was checked against the code and git history on that date (the previous full
audit was 2026-08-12). If you add a document, add a row here.
on that date. If you add a document, add a row here.

---

- **`Nano-Reboot-Trial-Resync-TODO.md`** - **PARKED 2026-09-12**, no code written. A Nano reboot mid-trial leaves Nano and Teensy silently out of sync. **The trial data IS lost from the moment the link dies** - the Teensy keeps running, but the SD logger is off (`sdLogEnabled = 0`), so the GUI CSV is the only record (the doc's §1.2 was corrected to say so on 2026-09-12; this index line had not been). **Interim rule (§2.6): after an unexpected mid-trial disconnect, do not reconnect and act - end the trial.** `get_status` 0x03 already exists on both sides, so the Teensy likely needs no change. **Contains a live data-integrity hazard (§2.1): pressing Start Trial after a reconnect calls `set_default_parameters()` and silently wipes tuned controller parameters mid-session.** Wants its own branch.

## Live documents

Read these. They describe the current state of the code.

| Document | Date | What it covers | Status |
|---|---|---|---|
| `Diagnostic-Scaffolding-Cleanup.md` | 2026-09-17 | What comes out of the debug scaffolding and what hides behind a flag, on the operator's rule: delete what never produced information, gate what did its job. **Part 1 (done):** the BLE layer's separate `device_manager_*.log` is merged into the one application log - nothing trimmed. Verbose by default since 2026-09-29; `--quiet` (or `EXO_QUIET=1`) trims (the old opt-in `--verbose-log` was silently ignored when misspelled). **Part 2 (plan):** an `EXO_DIAG` switch for the firmware diagnostics, deletion of the breadcrumbs and two dead test switches, phased one flash and one bench trial at a time | **Part 1 implemented**, 56/56 tests pass; verbosity flip 2026-09-29, suite 98/98. **Part 2 is a plan awaiting the operator's go-ahead**; verdicts re-checked against the code 2026-09-17. The worn ACK test it was waiting on is done (2026-09-18, `ACK-Loss-Investigation.md` §3.7), so its Task 5 (the ACK counters) is unblocked |
| `ACK-Loss-Investigation.md` | 2026-09-16, extended 2026-09-17/18 | Why the external loop reports "No ack" for writes the exo applied. Two measured mechanisms: **(A)** the Teensy→Nano UART ACK gets damaged and the Nano sends up a made-up "rejected, reason 1" frame (benchtop); **(B)** the ACK notification is silently dropped on a degraded BLE link (09-14 worn session with the laptop deliberately moved around: 53 Hz RT, 37% of ACKs lost, with one unacknowledged write **proven applied** from desired torque). Also: the ACK latency distribution, why a longer-lived spare-ACK guard was rejected (it compounds lost ACKs into give-ups), and a latent no-checksum/no-value-echo safety gap. **Extended 2026-09-17/18:** diagnostic counters in the ACK prove a silent attempt is almost always an applied command whose ACK was lost Nano → GUI (§3.4-§3.5); ACK latency is the Nano's outgoing notification backlog, not the write rate (§3.5); and the write loop is rebuilt as `OpenExoLink` V0.3 (§3.6): one command on the air, a newer value replaces a pending resend, warnings instead of exits. Design and plan: `specs/2026-09-17-ack-aware-write-scheduler-design.md`, `plans/2026-09-18-ack-aware-write-scheduler.md` | **✅ V0.3 validated worn 2026-09-18** (§3.7): 15-min stress run, 2,042 writes, 0 loop exits, never two commands on the air, 310 stale resends skipped. 6+1 commands never arrived under that load, each replaced by a newer value within about 2 s. Committed `805a3c2`, 92/92 tests pass. **Still open:** fold the temporary `PARAM_ACK_DIAG` counters (still flashed) into `EXO_DIAG` (cleanup plan Task 5). V0.2 (resend-and-give-up) is kept in §3 as history |
| `Nano-Disconnect-EVIDENCE-ONLY.md` | 2026-09-11, extended 2026-09-14 | **Observations only, no interpretation** - **§12 (2026-09-13/14): the real-person torque trial on B23, per-connection Windows-side interval logs, the `(6,6)` failure observed with that logger, CSV phase-lock readings, handshake-warning and duplicate-callback counts.** Earlier sections: run durations, verbatim reset banners, LED sightings, the I2C counter readings, the CSV timing/torque analysis, and a list of what was explicitly NOT measured. Created because the narrative document had accumulated confident explanations that later proved wrong and it became hard to separate measured from inferred | **Current.** The companion narrative/theory document is `Nano-Hang-Watchdog-And-Breadcrumbs.md` |
| `Nano-Hang-Watchdog-And-Breadcrumbs.md` | 2026-09-10 → 09-15 | The mid-trial Nano BLE death, end to end: the proof that it is a hang (warm `RESETREAS` read + frozen LED), then an interrupt-level stop (§3.8); the nRF52840 watchdog and breadcrumbs; the BLE link-stall detector and 2 s GUI ping; the bounded `sendAclPkt` patch to ArduinoBLE (§8) and upstream issue #45, open since 2019 (§13); and the interval experiments that found the cause (§14-§15). About 3,300 lines of live debugging with every retraction kept in place - **start at §0**, the authoritative status and retraction ledger (#1-#26). **Touches `Python_GUI/` too** | **✅ Cause found and fixed: a short BLE connection interval.** 15 failures at 7.5 ms, none ever at 28.75-30 ms. Production is build B23, `setConnectionInterval(20, 24)` (`SystemReset.h` `EXO_BLE_INTERVAL_SEL 0u`), merged via PR #13 (`b3cff2c`, 2026-09-15). Cleared for person trials: a 30-min real-person walking trial at torque (2026-09-13), then a 20-min bench run and a 26+ min worn session on a second laptop (2026-09-14). The "15 ms fails 2/2" rung is **retracted** - those runs were at 7.5 ms (§15). Still open: *why* a short interval is fatal. The watchdog, stall detector and diagnostics are temporary scaffolding; their removal is now `Diagnostic-Scaffolding-Cleanup.md` Part 2 (not started). Header status line brought up to date 2026-09-29 |
| `Windows-Connection-Parameter-Logger.md` | 2026-09-13 | The GUI-side logger (`Python_GUI/services/ConnParamsMonitor.py`) that records the BLE connection interval **Windows actually applies**, for the whole connection, from WinRT `GetConnectionParameters()` plus change events and a 10 s heartbeat. Why it exists (the firmware banner's `UPi` shows only the first update), how to read `CONN_PARAMS` lines, what it has shown (Windows' ~1.7 s 15 ms step while connecting; no interval change before a `(6,6)` failure), cost and safety, limits (Windows 11 only; a private Bleak attribute), removal | **In production use, committed `a5542b4`.** Bench-validated on 9 connections on 2026-09-13, including one `(6,6)` failure, and on a second Windows 11 laptop on 2026-09-14 (a 20-minute bench trial, then a 26+ minute worn session with torque). No automated tests (waived by the operator). Longest run so far: 26+ minutes. **Since 2026-09-17 its lines are in the session log `app_crash_*.log`** (`device_manager_*.log` before); always in the file, on the terminal unless the GUI is launched with `--quiet` |
| `GUI-End-Trial-Duplicate-Disconnect-Callback.md` | 2026-09-13 | Why the GUI sometimes says "Disconnected unexpectedly" after a normal End Trial: Bleak fires `disconnected_callback` twice, and `_mark_disconnected()` clears the intentional flag after the first. 11 occurrences since July. **Not a range drop** | **Found, NOT fixed** (re-checked against the code 2026-09-29). Source and log analysis only; fix shape recorded |
| `External-Control-Orchestrator.md` | 2026-09-08 | The new `Python_GUI/external_control/` package: an external program that owns an optimization backend (game theory / SPT) and drives the exo by writing `splineAlt` parameters through the GUI's UDP remote, confirming every write against the firmware ack. Covers the five wire-level traps (silence as the only failure signal, acks with no request id, the swallowed fast ack, controller-switch-loads-defaults, bilateral double-ack), the mode menu, and UDP timing mode. Also logs this session's docs move, the corrected `splineAlt` status, the `disconnection_troubleshooting` branch audit, and the deferred 20/25 Nm plantar torque investigation | **Bench-proven on the real exo 2026-09-09 for the `OP`, `plantar`, `dorsi` and `udp` modes; `game_theory_mode` still untested against hardware.** **User audit (their own line-by-line read) is still outstanding and was largely reset by the V0.3 refactor** — `OpenExoLink_utilities.py` restart from the top, `WriteScheduler_utilities.py` never read, `main_external_control.py` worth a second pass; see the doc's audit-status table. **⚠ The write path described here was replaced on 2026-09-18:** loop writes no longer wait for each ACK (`OpenExoLink` V0.3, new `Utilities/WriteScheduler_utilities.py`) - see `ACK-Loss-Investigation.md` §3.6 |
| `SplineAlt-Shape-Parameterised-Controller.md` | 2026-08-27 | The `splineAlt` ankle controller: a curve built from lobe shape parameters (peak/rise/dwell/fall/magnitude/scale) instead of 12 nodes, with a periodic spline that wraps across the 0/100 % gait seam. Also the reference account of the BLE handshake **column ceiling** (`PREFIX_COLS = 4`, 30 usable columns, silent truncation), the 9-character name limit, the Nano/Teensy `MAX_MESSAGE_SIZE` name collision, and the per-controller handshake payload budget. Also covers **unwiring TREC and SPV2 from the ankle** (chirp/step deliberately kept as characterization tools) and why their enum IDs were left as gaps | **Implemented, flashed and validated on hardware** (user confirmed 2026-09-08); committed `c22eb05`. It is the controller the external loop drives (`External-Control-Orchestrator.md`). Feed-forward clamp raised ±15 → ±25 Nm and `MAX_JOINT_TORQUE_NM` 25 → 30 on 2026-09-09 (`4b4af87`). *(This row said "never flashed" until the 2026-09-29 audit; the document itself had already been updated.)* |
| `Spline-Run-Analysis-And-RT-Stream-Fix.md` | 2026-08-12 | End-to-end analysis of running the Spline controller from the GUI (F1–F10 + safety hazards + logging blind spots); branch comparison vs `backup_branch_with_UW_edits`; the Teensy→Nano→GUI real-time stream fix; the error-manager disable; the F5 uncalibrated-sensor guard | **Current.** Contains three of its own in-document corrections (F1, F6, F8) — read the retraction blocks, not just the headings |
| `End-Trial-Diagnosis-Correction.md` | 2026-08-10 | The authoritative account of the End-Trial lock-up: says plainly that the root cause is **unknown**, and lists the defensive fixes that make the failure class impossible anyway | **Current.** Supersedes both files in `superseded/` |
| `Fresh-Torque-Path-Safety-Audit.md` | 2026-08-11 | Complete map of every CAN transmit site and every path torque can reach the motor by; why the `enable()`/`zero()` frames are dangerous on an AK60v3 | **Current, with a caveat banner.** Its "three accidents" verdict and its 51.4 Nm figure were overtaken by `3c08c77` / the current-decode investigation — see the banner at the top. The 25 Nm clamp it relies on is **30 Nm** since 2026-09-09 |
| `Motor-Current-Decode-Investigation.md` | 2026-08-11 | Why the SD motor logs show impossible currents; the AK60v3 MIT field scaling (`_I_MAX` 10.3 vs the motor's ±12.0) | **Current.** Itself retracts an earlier "decode is 6× wrong" claim |
| `SD-Card-Logging-and-End-Trial-Reset.md` | 2026-07-06→08 | The onboard SD logger and the end-trial reset / shutdown handshake | **Current** as a description of the feature. NB the logger is currently **disabled** in `config.ini` (`sdLogEnabled = 0`) — see the round-2 spline doc for why |
| `SD-Logger-Disabled-Behavioral-Audit.md` | 2026-08-10 | Whether the SD logger can affect behaviour while disabled. Answer: no | **Current** |
| `Motor-Freeze-Controller-Change-And-End-Trial.md` | 2026-07-22 | The AK60v3 hold-last-command freeze on controller change and End Trial; the `_sd_ready()` and deferred-reset fixes | **Current**, status line corrected 2026-08-12 (committed as `a6413c9`) |
| `Spline-Jitter-Diagnosis.md` | 2026-07-22 | Round 1 of the spline jitter hunt: the missing gain scheduler and the integer-quantized `percent_gait` | **Code fixes current** - in `main_working_branch` via PR #7 (`d8d04e7`, 2026-08-19). Not the whole story: the jitter that remained is mechanical (Round 3 §5b). Extended by the 08-12 analysis |
| `Spline-Jitter-Round-2-SD-Logging-Regression.md` | 2026-07-23 | Round 2: SD logging as a loop-rate regression, and the spline node change | **Current for the SD-logging regression** - it is why the logger is off. All committed as `8954667`. Its "resolved" did not last: the jitter was back in August, see Round 3 |
| `Jitter-Round-3-Both-Branches-PJMC-PID.md` | 2026-08-12 | Why both `fix_spline_jitter` and the UW backup branch jittered on 08-12: PJMC is byte-identical on both, the swing command is −P × the torque-sensor reading, the gain scheduler escalates on noise, and RAW/UW have no output clamp. **§5b: the jitter is mechanical** - the torque-sensor HF is the same with the motor off | **Analysis only, no code changed.** Read §5b before the TL;DR and §6, which predate it (banner added 2026-09-29). Concluded 2026-08-19 (`9fb77d4`): largely normal for this transmission, and PID was disabled on ZeroTorque. *(Missing from this index until 2026-09-29.)* |
| `2026-07-14-control-loop-and-transparency-backlog.md` | 2026-07-14 | Findings and to-dos from the July zero-torque transparency work: control-loop stalls, the AK60v3 status-frame decode fix, the zero-torque limit cycle, and where a `torque_scale` knob must go (before the PID) | **Stale backlog, not maintained after July - read the 2026-09-29 status banner at its top.** ZeroTorque's PID was reverted (`9fb77d4`), the decode fix shipped, `torque_scale` exists only as `splineAlt`'s `TorqScale`, the loop stalls are still open. *(Missing from this index until 2026-09-29.)* |
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
| `superseded/Mid-Trial-Freeze-Nano-Radio-Silence.md` | **Moved here 2026-09-29**; banner-superseded since 2026-09-12. Its "root cause unknown" is answered - the BLE connection interval (`Nano-Hang-Watchdog-And-Breadcrumbs.md` §0) - its RF-range suspicion is ruled out, and the `RESETREAS` instrument it shipped turned out to be confounded on this board | Yes - the raw observations: the 9.6 s supervision-timeout constant across 8 freezes, the Teensy alive to the last sample, the RT stream-loss measurements, and the "two presentations" framing (A = loop alive / link dead, B = loop stopped). Its banner lists what is wrong |
| `superseded/Nano-Reset-Pin-Spurious-Reset.md` | **Moved here 2026-09-29**; banner-superseded since 2026-09-12. The premise dissolved: there were no spurious resets, `RESETPIN` is what a normal power-on reads on this board, and the real failure was the connection interval | Yes - one durable lesson: `RESETREAS` has **no** diagnostic value on this hardware, which is why every later diagnosis used GPREGRET magic values (`BLESTALL` / `DOG` / `CRASH_0x`) |

---

## `specs/` and `plans/`

Design specs and the step-by-step plans that implemented them. Each file's status line was brought up to date on
2026-09-29. **The plans' checkboxes were never ticked**, finished work included - do not read them as progress.

| Spec / plan | Status |
|---|---|
| `specs/2026-07-06-teensy-sd-logging-design.md`, `plans/2026-07-06-teensy-sd-logging.md` | Implemented. Disabled at runtime since 2026-07-23 (`sdLogEnabled = 0`) |
| `specs/2026-07-07-end-trial-shutdown-progress-design.md`, `plans/2026-07-07-end-trial-shutdown-progress.md` | Implemented (`ShutdownDialog.py`, the `'P'` command) |
| `specs/2026-07-10-nonblocking-sd-logger-design.md`, `plans/2026-07-11-nonblocking-sd-logger.md` | Implemented, `3d24ecb`. Logger disabled at runtime since 2026-07-23 |
| `specs/2026-07-10-zerotorque-transparency-design.md`, `plans/2026-07-10-zerotorque-transparency.md` | Implemented (`03cc4a0`), then **PID reverted** (`9fb77d4`, 2026-08-19) - the default ankle controller freewheels |
| `specs/2026-07-17-udp-remote-control-design.md`, `plans/2026-07-18-udp-remote-control.md` | Implemented, `5e83374`; validated on the exo |
| `specs/2026-09-08-experiment-orchestrator-design.md` | Implemented (`Python_GUI/external_control/`); its write path was later replaced by V0.3 (`ACK-Loss-Investigation.md` §3.6) |
| `specs/2026-09-17-ack-aware-write-scheduler-design.md`, `plans/2026-09-18-ack-aware-write-scheduler.md` | Implemented, `805a3c2`; validated worn 2026-09-18 |
| `plans/2026-09-17-diagnostic-scaffolding-cleanup.md` | **Not started** - awaiting the operator's go-ahead |

## How to read the BLE-disconnect thread

1. `superseded/Mid-Trial-Freeze-Nano-Radio-Silence.md` - the symptom and the first measurements (2026-09-02).
2. `superseded/Nano-Reset-Pin-Spurious-Reset.md` - a `RESETREAS` reading that turned out to mean nothing.
3. **`Nano-Hang-Watchdog-And-Breadcrumbs.md` §0 - start here.** Cause, fix, retraction ledger.
4. `Nano-Disconnect-EVIDENCE-ONLY.md` - the raw observations, with no interpretation.
5. `Windows-Connection-Parameter-Logger.md` - the instrument that settled the interval question.
6. `Nano-Reboot-Trial-Resync-TODO.md` - what a reboot mid-trial still costs (parked).

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
