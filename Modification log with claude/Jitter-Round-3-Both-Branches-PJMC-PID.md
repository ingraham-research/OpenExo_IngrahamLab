# Jitter Round 3 — why BOTH branches now jitter (2026-08-12 trials)

**Status:** analysis only. No code, config, or SD-card edits made. Nothing run on motors.

**Question asked:** four new GUI logs — two on `fix_spline_jitter`, two on `backup_branch_with_UW_edits`
("UW backup"). Jitter is present on the fix branch *and* has now appeared on the UW backup branch,
which was reportedly clean the week before. Compare against the "RAW" manufacturer branch
(`upstream/main`).

---

## TL;DR

1. **There is no branch difference to find for PJMC.** `ProportionalJointMoment::calc_motor_cmd` is
   **byte-identical** between the two branches (147 lines, empty diff), and `SDCard/ankleControllers/PJMC.csv`
   is byte-identical across `upstream/main`, UW backup, and `fix_spline_jitter`
   (`0,0,1,1,6,0,0.03,1,1,3,0,0.001`). **All four trials ran PJMC.** Identical behaviour is exactly what
   the code predicts — the UW branch did not "start" jittering, it was always going to.
2. **The jitter is PID chatter on the torque sensor, and it is now directly visible** thanks to the new
   `Commanded Torque` stream. During swing (desired == 0) the motor command is essentially
   `cmd = −3.4 × measured_torque` (r = **−0.96**). The controller is amplifying its own torque-sensor
   reading straight back into the motor.
3. **The gain scheduler makes the worst moments worse.** `100.0%` of the swing commands above 25 Nm occur
   when the scheduler has *escalated* to `kp=6`. Its rule "big error → more authority" is backwards during
   swing, where "error" is just sensor noise.
4. **RAW and UW have no output torque clamp.** `MAX_JOINT_TORQUE_NM` exists only on `fix_spline_jitter`.
   Reconstructed UW swing commands reach **≈38–39 Nm**; the fix branch clips the same excursions at 25 Nm.
   So under identical chatter the UW branch delivers *more* of it to the motor.

---

## 1. What the four logs actually contain

The two branches write **different CSV schemas**, which is how each log is attributable:

| Schema | Columns | Branch |
|---|---|---|
| `In Stance` + `Status` + `Commanded Torque (L/R)` (14 col) | 08-12 15:39, 15:42 | `fix_spline_jitter` (current) |
| `In Stance` + `Status`, no Commanded Torque (12 col) | 07-23, 08-06, 08-08 logs | `fix_spline_jitter` (pre 08-11) |
| `Stance Phase` + `Channel 8` (12 col) | 08-12 16:03, 16:05 | UW backup |

`Status` decodes via `ActiveTrialPage.update_exo_status`: 2 = Trial On, 5 = FSR Calibration, 6 = FSR Refinement.

Controller identification, by `corr(desired, toe_FSR)` — PJMC is FSR-proportional, spline is gait-phase driven:

| Trial | Segment (exo s) | corr(des,FSR) | Verdict |
|---|---|---|---|
| FIX-1 | 124–164 | −0.90 | **PJMC**, stance_max ramped −11.9 → **−19.28** Nm |
| FIX-1 | 169–184 | −0.40 / −0.18 | **Spline** (desired locked at exactly −12.06 = the natural-cubic minimum) |
| FIX-2 | 55–160 | — (desired ≡ 0) | PJMC with stance_max = 0 |
| FIX-2 | 160–170 | +0.06 / −0.31 | **Spline** (−12.06) |
| UW-1 | whole trial | — (desired ≡ 0) | **PJMC, stance_max = 0** |
| UW-2 | 57–72 | — | **zeroTorque** (see §5 — the trace is *frozen*, not smooth) |
| UW-2 | 72–112 | **−1.000** | **PJMC**, stance_max ≈ 12.9 Nm |

> **Note on ordering:** you recalled spline-then-PJMC on the fix branch. The logs show the reverse —
> the spline segment is at the **end** of both fix trials. The identification is solid: `Spline::calc_motor_cmd`
> clamps its feed-forward at ±15 Nm on both branches, so the −19.28 Nm segment **cannot** be spline; and the
> −12.06 Nm segments match the natural-cubic minimum of the fix-branch node set to 0.01 Nm. Worth
> double-checking against your notes, since it changes which segment you attribute a percept to.

---

## 2. Headline: the branches are now indistinguishable

Quiet-period (desired == 0) measured torque, stale/frozen rows removed:

| Trial | Branch | L std | R std | L HFres | R HFres |
|---|---|---|---|---|---|
| 08-12 FIX-1 | fix | 1.397 | 1.409 | 0.40 | 0.35 |
| 08-12 FIX-2 | fix | 1.428 | 1.451 | 0.25 | 0.27 |
| 08-12 UW-1 | UW | 1.391 | 1.327 | 0.34 | 0.37 |
| 08-12 UW-2 | UW | 1.531 | 1.516 | 0.43 | 0.38 |

Statistically the same. For reference, the 08-06 fix-branch trials ranged 0.667–1.322 — so 08-12 sits at the
top of, but not outside, the historical spread. **There is a modest common-mode rise, not a branch effect.**

**Which firmware wrote the older logs (corrected).** The CSV header is written by the *Python GUI*, so it
identifies the GUI checkout, not the flashed firmware. The reliable firmware discriminator is RT channel 8:

- UW firmware: `uart_commands.h` → `rx_msg.data[8] = 8;` — a **hard-coded literal**.
- fix firmware (pre-08-11): `data[8]` = the `status_defs` status word → varies 2/5/6.
- fix firmware (post-08-11, commit a546918): `data[8]/data[9] = t_ff × gearing` = Commanded Torque L/R.

All 07-23, 08-06 and 08-08 logs show a **varying** status word (5/6, occasional 2), which a hard-coded `8`
cannot produce ⇒ **fix firmware**. The 08-12 16:03/16:05 logs show a constant 8.0 ⇒ **UW firmware**.
So there is still no pre-08-12 UW-branch log to compare against, and "UW was clean last week" cannot be
confirmed or refuted directly. Two things could have made UW *look* clean previously — see §5.

### 2b. But the jitter DID get substantially worse between 08-08 and 08-12 — on both branches

This was under-stated in the first draft: an earlier table filtered on `Status == 2`, which silently dropped
every 08-08 trial (they sit in status 5/6 nearly throughout). Re-measured with no status filter, over swing
rows only, using the validated command reconstruction:

| Trial | # in session | cadence (L) | mes std | mes HF | cmd std | **cmd p99** | **>25 Nm** |
|---|---|---|---|---|---|---|---|
| 08-06 171233 | 1 | 40.6 | 1.181 | 0.39 | 4.68 | 24.0 | 0.76% |
| 08-06 171736 | 2 | 13.3 | 0.720 | 0.17 | 2.56 | 8.9 | 0.14% |
| **08-06 171844** | 3 | **29.1** | **0.944** | **0.20** | **3.35** | **10.1** | **0.14%** |
| 08-08 175508 | 1 | 31.5 | 1.749 | 0.90 | 8.70 | 31.4 | 4.39% |
| 08-08 175804 | 4 | 20.4 | 0.928 | 0.24 | 3.17 | 8.8 | 0.21% |
| **08-12 FIX-1** | 1 | **30.7** | **1.465** | **0.39** | **6.15** | **28.0** | **1.43%** |
| 08-12 FIX-2 | 2 | 22.7 | 1.337 | 0.23 | 5.52 | 27.5 | 1.30% |
| 08-12 UW-1 | 3 | 26.5 | 1.391 | 0.34 | 5.43 | 24.5 | 0.90% |
| 08-12 UW-2 | 4 | 36.6 | 1.578 | 0.44 | 6.48 | 26.3 | 1.62% |

**RETRACTED: "a 2–3× regression between 08-08 and 08-12."** That claim compared 08-06 #3 (one of the *quiet*
trials) against 08-12 #1, and it does not survive a like-for-like test. Restricting to **walking-only** swing
rows (≥1 stance strike within ±1 s — this removes the standing/FSR-calibration confound, which matters
because the 08-08 trials sit in status 5/6 throughout), L leg only (right FSR dead all of 08-08):

| day | # | cadence | mes std | mes HF | **cmd p99** | >25 Nm |
|---|---|---|---|---|---|---|
| 08-06 | 1 | 40.6 | 1.260 | 0.43 | **24.7** | 0.90% |
| 08-06 | 2 | 13.3 | 0.688 | 0.14 | **8.8** | 0.00% |
| 08-06 | 3 | 29.1 | 0.988 | 0.21 | **10.3** | 0.12% |
| 08-08 | 1 | 31.5 | 1.811 | 0.84 | **31.4** | 5.25% |
| 08-08 | 2 | 24.2 | 1.121 | 0.61 | **24.1** | 0.94% |
| 08-08 | 3 | 19.7 | 0.814 | 0.23 | **6.5** | 0.00% |
| 08-08 | 4 | 20.4 | 1.077 | 0.33 | **21.9** | 0.51% |
| 08-12 | 1 | 30.7 | 1.560 | 0.44 | **28.5** | 1.67% |
| 08-12 | 2 | 22.7 | 1.671 | 0.30 | **33.5** | 2.28% |
| 08-12 | 3 (UW) | 26.5 | 1.376 | 0.37 | **23.9** | 0.75% |
| 08-12 | 4 (UW) | 36.6 | 1.664 | 0.47 | **27.0** | 1.78% |

**First-trial to first-trial: 08-06 = 24.7, 08-08 = 31.4, 08-12 = 28.5.** The single worst trial in the whole
dataset is 08-08 #1, not anything from 08-12. So the jitter was never *gone* on the earlier days — it was
absent in *some trials* (08-06 #2/#3, 08-08 #3, p99 6.5–10.3) and fully present in others.

**What is actually different about 08-12 is consistency, not severity: no trial settled.** Earlier sessions
were bimodal — start bad, sometimes drop into a quiet regime. On 08-12 all four trials stayed in the 24–34
band, across both branches.

Everything downstream tracks one variable: the **swing-phase torque reading**. Quiet trials sit at
std 0.69–0.99, bad trials at 1.08–1.81, and command chatter is ≈4× that number. Cadence does not explain the
split (08-06 #1 is the fastest trial *and* a bad one; 08-06 #2 is the slowest *and* the quietest), and neither
does time-since-boot (08-06 #1 ran at ≈444 s since boot — its logged exo time is negative from the int16 ×100
wrap at ±327.67 s — and was bad; 08-06 #3 ran to 184 s and stayed quiet).

**The open question is therefore "what makes a trial settle into the quiet regime", not "what regressed".**

---

## 3. The mechanism, quantified for the first time

The `Commanded Torque` channel added on 08-11 (commit a546918) is what makes this newly measurable.
Regressing command against measured torque over all swing rows (desired == 0), excluding clipped samples:

| Trial | Leg | `cmd = a·mes + b` | r | implied P |
|---|---|---|---|---|
| FIX-1 | L | −3.400 | **−0.960** | 3.40 |
| FIX-1 | R | −3.337 | **−0.967** | 3.34 |
| FIX-2 | L | −1.424 | −0.857 | 1.42 |
| FIX-2 | R | −1.527 | −0.826 | 1.53 |

FIX-1's −3.4 matches `kp_zero = 3` from `PJMC.csv` exactly. **The command during swing is nothing but the
torque-sensor reading multiplied by the P gain and sent to the motor.**

Consequences in FIX-1 (`Status == 2` rows only):

- **19.0% (L) / 19.7% (R)** of swing rows command **more than 5 Nm** when the setpoint is zero.
- The **±25 Nm clamp is reached** in 4.46% (L) / 1.78% (R) of trial-on rows.
- Swing command std ≈ **5.3 Nm**; peak ±25 Nm (clipped).

This is the jitter. It is not a spline defect and not a tuning subtlety — it is a proportional loop closed
around a noisy sensor with no meaningful filtering.

### The command model, validated

`cmd = setpoint + kp·(setpoint − measured)`, with `kp = 3` when `|setpoint| ≤ 0.5 ∧ |error| ≤ 3.5`, else `kp = 6`
(D is ignored: `_pid` gates D on `dt ≤ 2200 µs` and the loop runs ≈262 Hz, so D is discarded).

Validated against FIX-1's **logged** command, 124–164 s: **corr = 0.9923 (L) / 0.9757 (R), median |error| = 0.04 / 0.06 Nm.**

---

## 4. The gain scheduler is backwards during swing

The scheduler drops to `kp_zero = 3` when `|setpoint| ≤ 0.5` **and** `|error| ≤ 3.5`; otherwise it uses the
nominal `kp = 6`. During swing the setpoint is 0, so **`error` *is* the torque-sensor reading**. A noise spike
above 3.5 Nm therefore **doubles the gain at exactly the wrong instant**.

| Trial | Leg | GS engaged | swing rows >25 Nm | of those, GS **dis**engaged | peak \|cmd\| | peak if kp fixed at 3 |
|---|---|---|---|---|---|---|
| FIX-1 | L | 94.8% | 2.11% | **100.0%** | 35.8 Nm | 17.9 Nm |
| FIX-1 | R | 95.2% | 2.10% | **100.0%** | 39.0 Nm | 19.5 Nm |
| UW-1 | L | 97.4% | 0.90% | **100.0%** | 38.3 Nm | 19.1 Nm |
| UW-1 | R | 98.3% | 0.73% | **100.0%** | 36.7 Nm | 18.3 Nm |
| UW-2 | L | 95.2% | 1.77% | **100.0%** | 38.9 Nm | 19.4 Nm |
| UW-2 | R | 95.9% | 1.35% | **100.0%** | 38.6 Nm | 19.3 Nm |

Every single one of the largest swing commands in every trial comes from the escalation branch.
Pinning `kp = 3` during swing would **halve the peak** and cut command std ~30%.

---

## 5. Comparison against RAW (`upstream/main`) and the UW branch

`ExoCode` diff RAW → UW backup is only three files: `Board.h` (volt_sense pin), `Config.h` (battery divider),
`Side.cpp` (heel FSR forced false). **Nothing controller-related.** So RAW behaves like UW here.

| | RAW (`upstream/main`) | UW backup | `fix_spline_jitter` |
|---|---|---|---|
| `PJMC.csv` | `0,0,1,1,6,0,0.03,1,1,3,0,0.001` | identical | identical |
| `ankleUseTorqueSensor` | `yes` | `yes` | `yes` |
| PJMC PID active | **yes** | **yes** | yes |
| Output clamp `MAX_JOINT_TORQUE_NM` | **absent** | **absent** | 25 Nm (Motor.cpp) |
| SD logging | **absent** | **absent** | present, `sdLogEnabled = 0` |
| Spline gain scheduler | absent | absent | present |
| Spline EWMA alpha | 0.5 (lag in loop) | 0.5 | 1.0 (raw) |
| `percent_gait` type | `int` (quantized) | `int` | `float` |
| ZeroTorque writes `filtered_torque_reading` | no | **no** | yes |

Two consequences worth flagging:

**(a) RAW and UW send the chatter to the motor unclamped.** Applying the validated command model to the UW
trials (which do not log command):

| Trial | Leg | swing cmd std | peak \|cmd\| **unclamped** | rows >25 Nm |
|---|---|---|---|---|
| UW-1 | L / R | 5.43 / 4.93 | **38.3 / 36.7 Nm** | 0.90% / 0.73% |
| UW-2 | L / R | 6.81 / 6.65 | **38.9 / 38.6 Nm** | 1.77% / 1.35% |

The fix branch clips these at 25 Nm; UW and RAW do not. Under identical chatter the UW branch is the one
delivering the larger excursions.

**(b) UW's ZeroTorque freezes the GUI trace.** The UW branch's `ZeroTorque::calc_motor_cmd` never writes
`_controller_data->filtered_torque_reading`; the fix branch added that (`Controller.cpp:350`). Verified in
UW-2: from **57.0 s to 72.0 s** Measured Torque is a **constant −4.23 (L) / +2.72 (R)** — `nuniq = 1` across
869 distinct exo timestamps. That is a *stale value*, not a smooth signal. The PID itself uses the raw
reading, so control is unaffected — but **a zeroTorque segment on the UW branch will always look perfectly
flat on the GUI regardless of what the hardware is doing.** If a previous "UW is clean" impression came from
watching a zeroTorque segment, it was reading a frozen channel.

---

## 5b. THE JITTER IS MECHANICAL, NOT CONTROLLER-GENERATED (2026-08-12 evening logs)

Later trials that day added the decisive measurement. `trial_20260812_172544.csv` is **two experiments in one
file**: from t = 96 s to the end, `Commanded Torque` is **exactly 0.00** — the motor is free, no controller in
the loop. Comparing that against its own PID-on phase, walking swing rows, L leg:

| phase | mes std | **mes HF** | cmd std |
|---|---|---|---|
| 172544 PID ON (57–96 s) | 2.192 | **0.46** | 3.98 |
| 172544 **MOTOR OFF** (96–132 s) | 2.031 | **0.47** | 0.00 |

**Commanding the motor to exactly zero does not change the torque-sensor HF residual at all.** Therefore the
HF residual — the metric used throughout this document and in the July analyses — is **not** controller-
induced. It is real mechanical excitation. Three confirmations:

1. **It is generated by walking.** Standing still (`173721`, cadence 0): HF = **0.12** (the electrical noise
   floor). Walking at 30–45 steps/min: HF = **0.5–0.7**. ~5× rise from motion alone.
2. **It scales with cadence, with the motor off.** In the 172544 motor-off window, swing HF rises 0.21 → 0.45
   (L) and 0.25 → 0.77 (R) from cadence <20 to 20–32. Same within-trial pattern in every other trial
   (173807: 0.07 → 0.55). *Caveat:* the cross-trial version of this test is flat (log-log slope −0.09) but
   mixes wearers/donnings/setups — the within-trial version is the controlled one.
3. **It is a SWING phenomenon.** Motor off: stance HF 0.21 vs swing HF 0.41 (L); 0.22 vs 0.63 (R). ~2× worse
   in swing, i.e. foot off the ground, cable unloaded.

### Mechanism (user's physical observation + the data)

User reports seeing the joint, Bowden cables and waist gear chain visibly jitter **with the motor off**, and
that the quiet trial was the one where a **larger shoe** left the participant's foot loose. Load path is
motor (waist) → gear chain → Bowden cable → footplate → shoe → foot; shin cuff confirmed always tight.

Reflected motor-rotor inertia scales with the **square** of the gear ratio, so the transmission presents a
large effective mass at the footplate, behind a compliant high-friction cable with chain backlash. In stance
the foot is planted and the path is loaded and constrained — nothing rattles (HF 0.21). In swing the cable
tension collapses, limb angular acceleration peaks, and a **tightly laced shoe forces the footplate to track
the foot exactly**, so the swinging limb must accelerate that reflected inertia through a slack cable →
go-slack / snap-taut impacts and Bowden stick-slip. That is the visible rattle and it is where the sensor
says the energy is.

**A too-big shoe breaks the excitation path**: the foot moves inside the shoe rather than driving the
footplate, and the sloppy fit adds series compliance plus hysteretic damping. The tight shoe is the *input
port* for the disturbance. This also explains why the tight shin cuff does not help — the cuff is proximal to
the ankle joint and constrains the shank, not the footplate driving the cable.

### REFINEMENT: it is the FOOT-LANDING event, not the whole swing

User reports the visible rattle is worst "right at heel strike" — small jitter every time the foot lands, with
the motor off, and worse with PID on. Event-triggered RMS of the HF residual, aligned on **toe contact**
(t = 0; the only ground-truth event available, since `heelFsrPresent = 0`):

| trial | motor | peak HF | at | HF at toe contact |
|---|---|---|---|---|
| 172544 (L) | **OFF** | 1.90 | −0.50 s | 0.13 |
| 172544 (R) | **OFF** | 2.33 | −0.40 s | 0.20 |
| 172544 (L) | PID on | 1.19 | −0.55 s | 0.09 |
| 160338 (L) | PJMC smax=0 | 0.74 | −0.35 s | 0.13 |
| 174818 (R) | PJMC assist | 1.03 | −0.20 s | 0.15 |

**The disturbance is concentrated 0.2–0.55 s BEFORE toe contact and collapses 10–20× the moment the foot is
loaded.** With stride times of 3.3 s (cadence 18) to 1.5 s (cadence 39), and toe contact lagging heel strike
by ~10–15% of gait, that band is where heel strike falls. Present with the motor commanded to zero.

**The controller's largest command of the entire stride lands in that same window** — aligning `Commanded
Torque` on the same axis: rms **7.17 Nm @ −0.45 s** (174818 L), **5.29 @ −0.50 s** (R), **4.23 @ −0.40 s**
(172544 R). The P term reacts to the landing impact by kicking the motor hardest into a limb that is
mid-impact. That is drive-into-the-disturbance timing and directly confirms "PID makes it worse".

**Measurement limits (important):** no heel FSR, so heel-strike position is *inferred* from toe contact; and
the effective sample rate is ~50 Hz (Nyquist 25 Hz) while an impact transient carries energy well above that
— what is visible is smeared/aliased residue, not the impact itself. The energy is in the pre-contact window;
it cannot be timed to the millisecond from these logs.

**Revised mechanism.** Heel-strike-to-foot-flat is the fastest ankle motion in gait — rapid controlled
plantarflexion, a few hundred deg/s, over ~100 ms, driven by an impulsive ground collision. A rigidly laced
shoe forces the footplate to follow that rotation exactly, back-driving the Bowden cable into the reflected
motor inertia as a near-step velocity change; any slack becomes a snap-taut impact. Then the foot goes flat,
the path is loaded and constrained, and it goes quiet — exactly the shape in the table. **This explains the
oversized-shoe result better than the whole-swing version above:** the foot rotates *inside* the shoe during
the fast landing rotation, so the footplate never sees the step input.

**Consequences for the fix list:** cable free travel remains the top bench check, but specifically in the
**plantarflexion** direction, since that is the direction heel strike drives the footplate. Deliberate
compliance/damping at the footplate-to-shoe interface would reproduce what the oversized shoe did by accident
without giving up assist. On the software side, a well-targeted option is to **gate or reduce PID authority in
the ~150 ms around foot landing** — the event is already detected, and that is the one moment where high gain
is doing pure harm.

### Bench diagnostics (cheapest first, none require walking)

1. **Cable pretension / free travel** — off-body, move the footplate through its range, measure lost motion
   before the cable takes up. Slack in swing is what turns smooth motion into impacts. **Prime suspect.**
2. **Chain backlash** at the waist — rotate the output, measure lost motion. Load reverses every stride.
3. **Isolate the source** — shake the footplate by hand at ~2 Hz with the cable connected, then disconnected
   at the joint. Rattle surviving disconnection ⇒ footplate/joint; disappearing ⇒ cable/chain train.
4. **Footplate mass and shoe coupling stiffness** — anything lowering transmitted acceleration helps.

**The controller is not the source but it is not a bystander either:** the sensor faithfully reports this real
noise, the P term multiplies it by 3–6, and the motor injects that back into the same lightly damped,
backlash-y transmission. Mechanical fixes remove the source; §7's filtering/gain-scheduler fixes stop the loop
feeding it. Complementary — the mechanical lever is larger.

## 6. What is actually wrong, ranked

1. **A proportional loop is closed around the torque sensor with no filtering and no deadband.**
   `cmd = −P × measured` at r = −0.96. This is the jitter, on every branch, for PJMC. `torque_alpha = 1`
   in `PJMC.csv` means `filtered_torque_reading == raw` — there is literally no filter.
2. **The gain scheduler escalates on noise.** 100% of >25 Nm swing commands come from the `kp=6` branch.
3. **No output clamp on RAW/UW** — peaks ≈38 Nm vs 25 Nm on the fix branch.
4. **The D term is dead everywhere.** `_pid` gates D on `dt ≤ 2200 µs`; the loop runs ≈262 Hz (dt ≈ 3800 µs),
   so `d_gain = 0.03` does nothing. There is no damping term in the loop at all.
5. **Trial-to-trial bimodality in the swing-phase torque reading** (§2b) — the real open question. Some
   trials sit at std 0.69–0.99 (quiet, command p99 6.5–10.3), others at 1.08–1.81 (bad, p99 22–34), on the
   same day and the same firmware. On 08-12 no trial reached the quiet regime, on both branches. Not
   explained by cadence or time-since-boot, and no repo-tracked SD parameter changed after 07-23
   (`git log SDCard/` is empty since 8954667). Since command chatter is just ≈4× this reading, **whatever
   gates the quiet regime is the highest-value lever in the whole system** — likely donning/strap tightness,
   joint alignment, or the per-boot torque zero. A bench question, not a log question.

6. **The right toe FSR was dead for the entire 08-08 session.** All four trials show right-leg cadence 0.0
   and 99th-percentile right toe FSR of **0.00** — no signal at all, so PJMC never commanded right-leg
   torque (`act% = 0`). That is the "right leg not engaged" session. Worth separating from
   [[toe-fsr-asymmetry]]: this is not a weak sensor, it is no sensor. Compare against 08-12, where both
   FSRs read normally (peak ≈1.0–3.1).

## 7. Suggested next steps

Cheapest first, and **none of these have been done — all require your consent before anything drives a motor:**

1. **Single-variable test: set `torque_alpha` in `PJMC.csv` below 1** (e.g. 0.2) to actually low-pass the
   feedback. Costs nothing, no reflash — it is an SD-card CSV value.
2. **Test `kp_zero` fixed with no escalation** during swing (widen `ZERO_ERROR_BAND_NM` well past the noise,
   or clamp `kp` during zero setpoint). Predicted to halve peak swing command.
3. **Sanity test with `use PID = 0` in `PJMC.csv`** — pure feed-forward. If the jitter vanishes entirely,
   that confirms the whole chain above and gives a clean baseline to tune back up from.
4. **Investigate the common-mode rise (§6.5)** — torque-sensor zeroing, belt/transmission, battery sag under
   the new divider values. Requires bench inspection, not log analysis.

Note that (1)–(3) are card-only edits: `set_default_parameters()` reloads from SD on `trial_on` **and** on
controller change, so GUI-set values are wiped — edit the card, not the GUI, for a controlled test.

## Analysis scripts

In scratchpad: `char.py` (schema/characterisation), `jit.py` (jitter metrics), `seg.py` (5 s time buckets),
`mech.py` (cmd-vs-measured regression, sample-rate structure), `flat.py` (frozen-window verification),
`timeline.py` (all trials over time), `spl.py` (natural-cubic profile prediction per branch), `ident.py`
(controller identification by FSR proportionality), `drift.py` (torque-zero drift, tracking), `recon.py`
(validated command model), `gs.py` (gain-scheduler escalation).
