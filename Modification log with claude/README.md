# Modification log — index

Human-readable "what / why / how to modify" write-ups for changes made in this repo, one file per
investigation. Deeper design specs and implementation plans live in `docs/superpowers/`.

**Last audited: 2026-08-12.** Every status line below was checked against the code and git history
on that date. If you add a document, add a row here.

---

## Live documents

Read these. They describe the current state of the code.

| Document | Date | What it covers | Status |
|---|---|---|---|
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
