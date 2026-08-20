# Why the motor-current log is full of impossible values

**Date:** 2026-08-11
**Question:** `Current_A` in the SD motor logs reaches 19.5 A on a motor whose datasheet peak is 10.3 A, and the
big values sit next to tiny commands. Is the CAN decode wrong?

**Short answer: the decode is correct.** It matches the CubeMars manual byte-for-byte. The values are real
measurements. They are high because **the firmware is writing amps into a field the motor reads as a ±12
torque command**, and they look nonsensical because **the status frame is a free-running broadcast, so the
logged current is stale by an unknown amount.**

The `!! WARNING !!` block previously at the top of `SdLogger.h` — claiming the decode was "wrong by ~6x" —
was wrong. It has been corrected. See "Retraction" at the end.

---

## Practical status: this is archival, not actionable

**The SD logger is off.** It cost too much control-loop time (`Spline-Jitter-Round-2-SD-Logging-Regression.md`),
so `Current_A` and `Commanded_Torque_Nm` are not being produced at all. Nothing below changes any behaviour
today. This document exists so that (a) nobody re-derives a wrong conclusion from an archived log, and
(b) whoever re-enables the logger fixes the columns first.

**What replaced it is better for the question that matters.** The GUI CSV now carries, always-on over BLE
at ~100 Hz:

| ch | signal | source |
|---|---|---|
| 1 / 3 | Measured Torque (L/R) | ankle torque sensor |
| 8 / 9 | Commanded Torque (L/R) | `motor.t_ff × gearing` — post-PID, post-clamp, post-gain-schedule |

Both fresh, both synchronous, no staleness. Note `t_ff`, **not** `i_sat` — so channels 8/9 are genuine joint
N·m and do *not* inherit the amps/N·m confusion of §4a. (An earlier note in `SdLogger.h` said they came from
`i_sat`; that was wrong and is corrected.)

> **`t_ff` is a protocol name, not a control name — it is NOT feed-forward only.** It is named for the MIT
> frame's torque field, which is feed-forward with respect to the *motor's* internal kp/kd loop (kp = kd = 0
> here, so it carries the whole command). Traced: `Controller::calc_motor_cmd()` returns
> `cmd = torque_cmd + _pid(...)` → `controller.setpoint` (`Joint.cpp:652`) → `/ gearing` →
> `Motor::send_data()`, which applies the `MAX_JOINT_TORQUE_NM` clamp and only then assigns `t_ff` inside
> the transmit branches. **Post-PID, post-gain-schedule, post-clamp, and zero on zero-frame cycles.**
> The exo controller's actual feed-forward term is `ControllerData::ff_setpoint` / `desired_torque` —
> that is what channels 0/2 carry, which is why the gap between channel 0 and channel 8 is the PID's
> contribution. Documented at the declaration in `MotorData.h` and at the assignment in `Motor.cpp`.

**So the next trial settles §4f directly** — commanded vs delivered joint torque, measured live, which is the
number that actually matters for tuning and for how hard the clamp really bites. Plot channel 8 against
channel 1 and read the ratio; §4f predicts ~1.6–1.9 from the old SD data.

**It will not settle §2** (whether the ±12.0 field is N·m or IQ amps). Channel 8 is what the firmware
*intends* to send; the sensor is what arrives. The ratio between them lumps together the 1.165× wire error,
the Kt uncertainty, and transmission loss, and walking gives no way to separate them. Only the blocked-joint
static test in §5.2 — where transmission loss collapses to stiction and the motor's own current reading is
available — pulls those apart.

---

## 1. The decode is right (manual §4.3.1)

Source: *AK Series Module Product Manual v3.0.0*, §4.3.1 "CAN Upload Message Protocol", p.42.

| Byte | Field | Manual scale | `Motor.cpp:193-198` | |
|---|---|---|---|---|
| [0..1] | Position int16 | −32000..32000 → −3200°..3200° = **0.1 °/count** | `p_raw * 0.1f * PI/180` | ✅ |
| [2..3] | Speed int16 | −32000..32000 → ±320000 erpm = **10 erpm/count** | `v_raw * 10.0f / (14*6) * 2PI/60` | ✅ |
| [4..5] | Current int16 | −6000..6000 → **±60 A = 0.01 A/count** | `i_raw * 0.01f` | ✅ |
| [6] | Temperature int8 | −20..127 °C | *not read* | — |
| [7] | Error code uint8 | 0=none … 7=lock-up | *not read* | — |

The manual's own reference decode is character-identical to ours:

```c
int16_t cur_int = (rx_message)->Data[4]<<8 | (rx_message)->Data[5];
motor_cur = (float)(cur_int * 0.01f);   // Motor current
```

The transmit side matches §4.2 `pack_cmd` exactly too (`buf[0]=kp>>4`, … `buf[7]=t_int&0xff`), and the
extended ID `(8<<8)|id` matches the manual's MIT examples (`00000868`, p.52-53).

**Independent confirmation from our own data.** If we were decoding the wrong bytes, position would be
garbage. It isn't: reconstructed position counts trace a smooth ankle trajectory, and jumps >90° in <60 ms
occur in **0.08%** of frames (3 / 3705) — while currents above 10.3 A occur in **2.4%**. Thirty times more
"impossible" currents than corrupt frames. The frames are intact.

---

## 2. Why the numbers are large: `_I_MAX = 10.3` is the wrong constant

`Motor.cpp:798` sets `_I_MAX = 10.3f` for the AK60v3. That is the **peak current** off the AK60-6 V3.0
datasheet (10.3 A @24 V / 11.2 A @48 V). But `_I_MAX` is not used as a current limit — it is used as the
**full-scale of the MIT `t_ff` field**:

```cpp
float i_sat = constrain(direction_modifier * current, -_I_MAX, _I_MAX);
uint32_t i_int = _float_to_uint(i_sat, -_I_MAX, _I_MAX, 12);   // packs against +-10.3
```

Manual §4.2, "Parameter Ranges" table, **AK60-6** column:

| | AK10-9 | **AK60-6** | AK70-9 | AK80-9 |
|---|---|---|---|---|
| Motor position (rad) | ±12.56 | **±12.56** | ±12.56 | ±12.56 |
| Motor speed (rad/s) | ±28.0 | **±60.0** | ±30.0 | ±65.0 |
| Motor torque (N.m) | ±54.0 | **±12.0** | ±32.0 | ±18.0 |

The motor unpacks that 12-bit field against **±12**, not ±10.3. Both ranges are symmetric about zero, so
the error is a clean gain with no offset:

```
t_ff_motor  =  i_sat * (12.0 / 10.3)  =  i_sat * 1.165
```

**Every command leaves the Teensy 16.5% larger than the firmware believes.** That much is certain and does
not depend on what unit the field carries.

If the field is N·m (as the parameter table says), then "commanding 10.3 A" is really commanding **12.0 N·m
at the motor output shaft**. At the datasheet torque constant (0.135 Nm/A × 6:1 = 0.81 Nm/A at the output)
that draws **≈14.8 A**. So sustained currents to ~15 A are the *expected* behaviour at full-scale command,
and the 19.5 A outliers are current-loop overshoot on a step — about +32%, ordinary for an FOC loop into
676 µH / 595 mΩ.

Also mis-set, though currently inert: `_V_MAX = 48.0` vs the manual's **±60.0 rad/s** for AK60-6, and
`_P_MAX = 12.5` vs **±12.56**. Both fields are ignored while `kp = kd = 0` (they are never assigned
anywhere — `MotorData.cpp:284-285`), so they cost nothing today. They would matter the moment anyone
enables the motor's internal impedance loop.

### What the logs say

Regressing measured current on commanded current, fresh frames only:

| | slope | r² | n |
|---|---|---|---|
| Left, `i` vs `cmd` | **+1.358** | 0.578 | 3932 |
| Right, `i` vs `cmd` | **−1.318** | 0.561 | 959 |
| Right, `\|i\|` vs `\|cmd\|` | +1.266 | 0.526 | 959 |

The sign flip on the right is expected — `ankleFlipMotorDir = right`, and `cmd` is stored in the motor
frame while `i` has `direction_modifier` applied. Magnitudes agree across legs.

Predictions: the field-is-amps reading gives 1.165; the field-is-N·m reading with datasheet Kt gives 1.438.
Measured 1.32–1.36 is a **lower** bound (staleness attenuates the regression), and the peak-aligned
estimator — max |i| within 60 ms of each |cmd| ≥ 7 A — gives a median ratio of **1.62**, an upper bound.
The true value lies in ~1.34–1.62, which brackets 1.438 and sits above 1.165.

### What is NOT settled: is that ±12.0 in N·m or in amps?

**The manual contradicts itself.** §4.2's table header says "Motor torque (**N.M**)". §4.4.1's command
examples (p.53) label the very same field in amps:

```
MIT Torque Loop   00000868  0000007FFF7FF83F   2A IQ Current
                  00000868  0000007FFF7FF87E   4A IQ Current
```

Those examples **cannot** resolve it. Decoding them gives `t_int` = 2111 and 2174, and because the
encoding is linear and symmetric about zero, the implied constant is *exactly* self-consistent for any
full-scale value you assume — 0.18457 at ±12, 0.83057 at ±54. The 2A→4A doubling is guaranteed by the
arithmetic, so the example proves nothing about the scale. (I checked this specifically rather than
reading the agreement as confirmation.)

The log evidence above leans N·m, but only mildly, and it is partly circular too — the prediction depends
on which Kt you assume, and Kt is the other unknown.

**Why it matters:** it flips the sign of the net delivery error.

```
field is N·m   ->  delivered = requested × (12.0/10.3) × 4.5 / (4.5 × 1.11) = 1.049   (+5%)
field is amps  ->  delivered = requested × (12.0/10.3) × 0.81 / 1.11        = 0.850   (-15%)
```

The 1.165× range error is real in both cases. Only its interaction with Kt changes. §5.2 settles it.

### Alternative considered: is `Current_A` actually a torque reading?

Reasonable hypothesis — the values exceed the motor's current rating, so maybe they were never amps.

**First, the chain, traced rather than assumed.** `motor.i` is written in exactly two places in the whole
firmware (`Motor.cpp:198` AK60v3, `Motor.cpp:221` old AK) and mutated nowhere — no `*=`, no gearing, no Kt.
`_Motor::_Motor` binds `_motor_data = &(exo_data-><side>.<joint>.motor)`; `SdLogger::_used_joint` returns
`&side.ankle`; `SdLogger.cpp:300` prints `j->motor.i` with `%.3f`. Same object, no intermediary.

```
logged Current_A  =  int16 from bytes[4..5]  ×  0.01  ×  (−1 on the right leg)
```

**The logging code applies no conversion at all.** So a logged 19.480 is raw count 1948, and the only
question is what one count means.

**Second, the observation that motivated the hypothesis is real.** Over the whole trial, `motor.i` read
with *no* conversion tracks the joint torque sensor's distribution closely, on both legs:

| | median | 90th | max |
|---|---|---|---|
| `motor.i` raw, left | 1.03 | 5.64 | 19.48 |
| measured joint torque, left | 1.00 | 6.40 | 17.09 |
| `motor.i` raw, right | 0.62 | 3.91 | 13.65 |
| measured joint torque, right | 0.65 | 4.71 | 14.88 |

That is a good fit and it deserved checking.

**Third, it does not survive a control.** Median |desired torque| is **0.00** — half the trial commands
nothing, so those marginal distributions are dominated by swing phase, where both signals sit near zero for
unrelated reasons. Restricting to loaded samples (|desired| > 5 Nm):

| | left | right |
|---|---|---|
| mean measured joint torque | 10.08 Nm | 8.30 Nm |
| mean `motor.i` raw | 6.45 | 2.65 |
| **ratio raw / measured** | **0.64** | **0.32** |

Not 1.0, and **a factor of two apart between legs**. A physical torque cannot be leg-dependent. The
asymmetry is instead exactly what the staleness in §3 predicts: the right leg is 85% stale repeats, biased
toward the small values that dominate the trial, which drags its mean down. The hypothesis is rejected —
and the failure mode independently corroborates the staleness diagnosis.

Rejected on three further counts.

**1. Reading it as torque makes the physics worse, not better.** The only conversion available is the
4.5:1 joint gearing:

| reading of the max sample (19.48) | implied joint torque |
|---|---|
| N·m at the motor output shaft | **87.7 Nm** |
| amps, at datasheet Kt 0.81 | **71.0 Nm** |
| joint torque sensor, actual trial maximum | **17.09 Nm** |

Both readings imply a torque the joint never saw, so "I would have felt that" does not select between
them — and if anything it rejects the torque reading harder, since ×4.5 > ×0.81×4.5. There is no
interpretation that yields a comfortable number: the field would have to be *joint* torque (19.48 Nm), and
the motor cannot report that, because the 4.5:1 reduction is external to the actuator. It has no knowledge
of it.

**2. The manual is unambiguous on this field.** Unlike `t_ff`, where it genuinely contradicts itself,
§4.3.1 gives a range (−6000..6000 → **−60 to 60 A**), a unit, and reference code commented `//Motor
current`. Nothing elsewhere competes with it. That ±60 A range also explains why the tail looks
unremarkable to the encoder: 19.48 A is 32% of full scale, nowhere near a saturation artefact.

**3. Every excursion is a single isolated frame.** Collapsing the log to distinct CAN frames and counting
consecutive frames above 11.2 A:

```
Left  : 57 runs of length 1, 4 runs of length 2, none longer
Right :  2 runs of length 1
```

Not one sustained excursion in the entire trial. That is a transient signature. A real torque of 70–88 Nm
would persist across many frames and would appear on the joint torque sensor, which peaked at 17.09 Nm
(left) / 14.88 Nm (right) all trial.

So the excursions are brief current-loop overshoot on a step, snapshotted by the motor's ADC. The 10.3 A
"peak current" is a thermal/protection rating on a filtered value — note the datasheet's own peak:rated
ratio is already 2.7 (10.3 vs 3.8 A) — not a bound on an instantaneous q-axis sample. Mechanically these
are invisible: the ankle's inertia and series compliance filter a sub-millisecond electrical transient out
completely, which is exactly why nobody felt anything.

---

## 3. Why they look *nonsensical*: the status frame is a broadcast, not a reply

Manual §4.3.1, first line:

> The motor CAN message uses a **timed upload mode**, with an upload frequency that can be set from
> 1 to 500 Hz, and the upload byte is 8 bytes.

and §3.1.1.1 lists "**CAN feedback rate**" as a motor-side configuration parameter. The motor is *not*
answering our command frames. It is broadcasting on its own clock, at whatever rate is programmed into it.

Against that, `read_data()` performs exactly **one destructive pop per motor per 500 Hz cycle**, from a
single shared FlexCAN queue, routed by queue position:

```cpp
CAN_message_t msg = can->read();          // Can0.read(msg) — one frame, whoever is at the head
if ((msg.id & 0xFF) == uint8_t(_motor_data->id)) { ...decode... }
// else: frame silently discarded, and motor.p/v/i keep their previous values
```

Left runs before right every cycle. When one frame is queued and it belongs to the right motor, left pops
it, fails the ID test, drops it — and right then finds an empty queue. Right loses systematically:

| | rows with a **fresh** frame | stale |
|---|---|---|
| Left | 3946 / 6522 = **60.5%** | 39.5% |
| Right | 967 / 6522 = **14.8%** | **85.2%** |

So 85% of the right leg's `Current_A` column is a repeat of an older sample. That is exactly the pattern in
the log:

```
t=135.773  i=-11.09  cmd=+5.03   <- one real frame ...
t=135.794  i=-11.09  cmd=+3.29
t=135.822  i=-11.09  cmd=+2.07
   ... 11 rows, i frozen, command swinging through zero ...
t=135.956  i=-11.09  cmd=-0.32   <- ... still the same frame
```

The −11.09 A was real. It was measured at some earlier instant when the command *was* large. The logger
then printed it beside eleven successive commands it has nothing to do with.

This is what makes the column look like garbage: **a real peak current paired with the wrong row's
command.** Confirmed statistically — samples with |i| > 10.3 A have mean |cmd| = 5.04 A and mean |joint
torque| = 8.47 Nm, versus 1.09 A and 2.21 Nm for the rest. They land in genuine high-load moments, at
*lower* mean speed (7.10 vs 8.33 rad/s) — high load, low speed, i.e. near stall, exactly where current
peaks. And the magnitude histogram decays smoothly (…14 A:6, 16 A:2, 18 A:1, 19 A:1) with no second mode,
which a corruption process would not produce.

---

## 4. Consequences worth acting on

**a. `Commanded_Torque_Nm` is neither Nm nor amps.** It is `i_sat`, a number in fictitious "firmware amps".
True motor-shaft torque = `Commanded_Torque_Nm` × 1.165 N·m; true joint torque = **× 5.243**.

**b. The 25 Nm clamp actually permits 26.2 Nm.**

```
max_motor_torque = 25 / 4.5        = 5.556   ("motor Nm" as the firmware reckons)
i_sat            = 5.556 / 1.11    = 5.005   (firmware amps)
motor decodes    = 5.005 * 1.165   = 5.831   N·m at the motor shaft
joint            = 5.831 * 4.5     = 26.24   Nm
```

~5% loose. Not alarming, but the clamp is a safety limit and should mean what it says.

**c. The unclamped ceiling is 54.0 Nm, not 51.4 Nm.** `i_sat` saturates at 10.3 → the motor decodes 12.0
N·m → × 4.5 = **54.0 Nm** at the joint. The 51.4 Nm figure in `Motor.cpp` comments and in
`Fresh-Torque-Path-Safety-Audit.md` came from `I_MAX × Kt × gearing` = 10.3 × 1.11 × 4.5, which assumes the
firmware's own Kt and its own range. Both comments corrected.

**d. Net torque delivery is off by somewhere between +5% and −15%,** depending on the unresolved
unit question above — small either way, which is why the scaling has never looked obviously broken and
why closed-loop PID tracking measures ~90%. **Do not "fix" one constant without the other**: correcting
`_I_MAX` to 12.0 alone would leave every command 16.5% *smaller* than today, immediately and on both legs.

**e. Nothing here reaches the control path.** `motor.i` is consumed only by `check_response()`'s variance
test (`Motor.cpp:450`) and `error_types.h:66`. No controller reads it. This is a logging and
instrumentation defect, not a torque-path safety defect.

**f. Separate finding, decode-free: we command ~1.6–1.9× more joint torque than the sensor measures.**
On loaded samples (|desired| > 5 Nm), post-PID commanded joint torque averages 16.21 Nm (left) / 15.71 Nm
(right) against 10.08 / 8.30 Nm at the sensor. No CAN decoding is involved on either side of that
comparison, so it is not a current-decode issue. It is some combination of Bowden/transmission loss (38%
is unremarkable for an ankle exo), the Kt uncertainty in §2, and torque-sensor calibration. Worth
separating from everything above, and the §5.2 bench test measures all three at once.

---

## 5. To settle it on the bench

1. **Log bytes [6] and [7].** Temperature should read a plausible 25–60 °C and drift slowly; error should
   read 0. Two spare bytes we already receive and throw away, and they confirm the frame layout absolutely.
   They also give free over-current / over-temperature / lock-up detection (`2` = over-current,
   `7` = motor lock-up — directly relevant to the right-ankle failure).

2. **Static units test — motors powered, joint mechanically blocked, nobody wearing it.** This is the
   one that settles §2. Command a fixed `i_sat` and record BOTH readings in steady state (steady state
   matters — it removes the staleness problem, because a held command makes every frame equivalent).

   Read them as two independent, mutually exclusive checks:

   ```
   measured Current_A  ==  i_sat × 1.165           ->  the field is IQ AMPS
   joint torque / 4.5  ==  i_sat × 1.165  (N·m)    ->  the field is N·m
   ```

   Only one can hold. As a bonus the same two numbers give the true torque constant directly and
   without any protocol assumption: `Kt_output = joint_torque / (Current_A × 4.5)`, which resolves the
   1.11-vs-0.81 question that is currently entangled with this one. Do it at two or three command
   levels so a friction offset shows up as an intercept rather than corrupting the slope.

3. **Raise the motor's CAN feedback rate** in the CubeMars upper computer, and **drain the whole RX queue
   each cycle, routing by ID**, instead of one positional pop. That fixes the staleness and the 85% right-leg
   starvation together. Bus cost is negligible: 2 × 500 Hz × ~130 bits ≈ 130 kbit/s on a 1 Mbit bus, on top
   of the ~130 kbit/s we already send.

---

## Retraction

The `!! WARNING !!` block added to `SdLogger.h` earlier in this work claimed `Current_A` was "wrong by
roughly 6x" and "worthless", inferring ~1.0 Nm/A from a regression of joint torque on measured current in
trial 0009.

That inference was invalid. Torque was under **closed-loop PID control** in that trial, so `cmd` is not an
independent variable — it is driven by tracking error. Regressing the controlled output on the control
signal identifies the disturbance structure, not the plant gain. The same regression run against the
*commanded* current gives 0.463 Nm/A — nearly the same number — which should have been the tell: it is an
artefact of closed-loop identification, not a property of the current decode.

The decode matches the manual exactly. The warning has been rewritten to describe the real defects:
the ±10.3-vs-±12 range error and the staleness.
