# Ankle "Freeze" on Controller-Change and End-Trial (AK60v3 holds last command)

**Date:** 2026-07-22
**Scope:** Teensy 4.1 firmware (`ExoCode/`). No Nano, no GUI changes.
**Status:** ~~Edits made, pending on-device build/flash validation. Nothing committed.~~
**Updated 2026-08-12: both fixes were committed as `a6413c9`** ("Attempt to solve a bug where
changing controller or ending trial cause the motor to hang…"). Still not independently
bench-validated for the freeze symptom itself.
**Later correction:** a 2026-08-12 analysis initially claimed a large residual stall remained in
`set_controller_params()` (~2 s per joint from `Stream` EOF timeouts). That was **wrong** — a
`break` at `ParamsFromSD.cpp:648` ends the read loop after one pass, so no timeout occurs. The
residual cost is milliseconds of SD I/O, which suggests fix A below was the substantive one. See
`Spline-Run-Analysis-And-RT-Stream-Fix.md` §F1.
**Read alongside:** `Modification log with claude/SD-Card-Logging-and-End-Trial-Reset.md` (the
end-trial reset/shutdown handshake this builds on — if the reset misbehaves, cross-check both),
and `docs/superpowers/specs/2026-07-07-end-trial-shutdown-progress-design.md`.

---

## Symptom

The exo "freezes" — the ankle either **holds/stalls** or **suddenly resists** — in two situations:
1. **Whenever a new controller is selected** (GUI apply that switches the controller).
2. **Immediately after End Trial.**

## Root cause (one mechanism, two triggers)

The ankle motor is an **AK60v3** (`SDCard/config.ini: ankle = AK60v3`). It runs pure torque
(`kp=kd=0`), and it **holds its last torque command whenever the stream of fresh CAN frames stops.**
This is a known AK60v3 trait already noted in `Motor.cpp::send_data()` (the one-shot "final
zero-torque command" with the comment *"critical for AK60v3 which will otherwise hold the last
command"*). The control loop is 500 Hz, so the motor expects a fresh frame every 2 ms; when frames
stop while the motor is powered and holding a **non-zero** command, you feel a frozen/resisting joint.

Two code paths interrupted that frame stream:

**1. Controller change — a blocking SD read stalled the control loop.**
GUI apply (`updateTorqueValues` → BLE `'f'`) → Teensy `update_controller_param`
(`uart_commands.h`). When the controller actually changes, it calls
`set_default_parameters()` → `set_controller_params()` (`ParamsFromSD.cpp`), which ran
`SPI.begin()` + `SD.begin(SD_SELECT)` **every call**. `SD.begin()` re-mounts the card (re-runs the
card-init handshake) and, with the busy-wait line parse, blocks the loop for several ms. During that
stall `run_side()`/`send_data()` don't run → no CAN frames → the AK60v3 holds its last (walking)
torque → felt as a catch/stiffening at the switch.

**2. End Trial — the reboot happened before the zero frame was ever sent.**
End Trial sends `'Z'` → Teensy `get_system_reset()` (`uart_commands.h`). It set `motor.enabled = 0`
(a RAM flag only) then called `exo_system_reset()` **immediately**. But the AK60v3's final
zero-torque frame is emitted by `run_side()` → `send_data()` in the *control loop*, which never ran
again before the CPU restarted. So the (still-powered) motor held its last non-zero command through
the reboot, and after reboot the default is `zeroTorque` with motors disabled (no frames sent) — so
it held that stale torque indefinitely.

> Note: the ordinary `'G'` (stop) path already works — it sets `enabled = 0` and the *next* loop's
> `send_data()` emits the zero frame. The end-trial sequence's `'Z'`-first design (forced by BLE
> congestion — see the other log) rebooted before that could happen.

## The fixes

### A. Controller change — mount the SD card once (`ParamsFromSD.cpp`)
Added a file-local `_sd_ready()` that mounts the card **once** (caches success; retries only if the
first mount failed, e.g. no card yet), and pointed all six joint branches at it:

```cpp
static bool _sd_ready()
{
    static bool mounted = false;
    if (mounted) return true;
    SPI.begin();
    mounted = SD.begin(SD_SELECT);
    return mounted;
}
// each branch: `if (!SD.begin(SD_SELECT))`  ->  `if (!_sd_ready())`
```

This removes the per-call re-mount (the stall). The card is already mounted once at boot in
`ParseIni`, and `SdLogger` already treats `SD.begin()` as init-once (`_begin_if_needed`), so this
mirrors an established pattern. **Bonus safety:** changing a controller mid-trial no longer re-mounts
the volume while `SdLogger` has log files open (that was a latent FAT-corruption risk — the exact
scenario the logger's "Case C" self-test probes).
The per-branch `SPI.begin()` calls were left in place (cheap µs-level peripheral re-init; the
expensive part was `SD.begin()`).

### B. End Trial — defer the reboot a few control cycles so the zero frame goes out
`get_system_reset()` (`uart_commands.h`) now **arms** a deferred reset instead of rebooting inline:

- Still sends `reset_ack` to the Nano **first** (unchanged — the Nano's `WAIT_ACK`→`ACKED` timing is
  untouched).
- Still sets `motor.enabled = 0` and `trial_off`.
- Then sets `ExoData::reset_ticks = 0; ExoData::reset_pending = true;` and **returns**.
- Removed the inline `delay(10)`, `SdLogger::close_active()`, and `exo_system_reset()`.

The Teensy superloop (`ExoCode.ino`, right after `sd_logger.update(ran)`) does the reboot:

```cpp
if (exo_data.reset_pending && ran)
{
    static const uint8_t RESET_ZERO_TICKS = 3;
    if (++exo_data.reset_ticks >= RESET_ZERO_TICKS)
    {
        SdLogger::close_active();
        exo_system_reset();
    }
}
```

New `ExoData` fields: `bool reset_pending = false; uint8_t reset_ticks = 0;`.

**Timeline** (arm on tick N): tick N+1 `run_side()` transmits the zero frame (the enable→disable edge
in `send_data()`); tick N+2 `reset_ticks` reaches 3 → reboot. One full control tick (~2 ms) of margin
after the frame. `RESET_ZERO_TICKS` **must be ≥ 2** (the zero frame is sent on the *first* cycle after
`enabled` drops to 0; rebooting sooner would still hold the last command). Gated on `ran` so we count
real 500 Hz control cycles.

### Why the End-Trial fix preserves the hard-won invariants
- **Root cause #1 from the other log (reset abandoning open logs → FAT corruption): still fixed.**
  The log now closes on the `trial_off` edge inside `sd_logger.update()` (which runs during the
  deferral) **and** via the explicit `SdLogger::close_active()` immediately before the reboot
  (idempotent). Log is closed before `exo_system_reset()` either way.
- **`reset_ack` still sent first**, so the Nano state machine and the GUI shutdown dialog behave
  exactly as before. The reboot slips by ~6 ms (3 × 2 ms) — negligible against the Nano's multi-second
  `_reset_ack_timeout_ms` / `_reset_delay_ms` windows.
- **No BLE / Nano / GUI code touched.** The `'Z'`-first `send_end_trial_sequence()` and the Nano
  `_maybe_system_reset()` state machine are unchanged.

## What was deliberately NOT done
- **GUI pre-zero** (send motor-off before a controller-change apply, as belt-and-suspenders):
  **deferred** — the `_sd_ready()` fix should remove the stall on its own, so we test firmware first.
  If revisited: it must be **gated on an actual controller change** (controller-change and every
  parameter tweak share the `updateTorqueValues` path) and must **respect pause state** (don't
  auto-`motorOn` when motors were off) to avoid enabling the motor unexpectedly.

## How to validate on-device
1. **Controller change:** switch controllers mid-trial → ankle should no longer catch/stiffen at the
   switch (isolates fix A).
2. **End Trial:** end a trial mid-walk → ankle should go transparent, not hold its last torque
   (isolates fix B). Confirm the log files still close cleanly (openable, no FAT corruption) and the
   GUI shutdown dialog still shows `ACKED` → reboot as before.

## If it bugs out — where to look
- **Ankle still holds after End Trial:** the zero frame isn't landing. Bump `RESET_ZERO_TICKS`
  (e.g. 5); confirm `run_side()` runs with `enabled == 0` before the reboot; check `send_data()`'s
  final-zero branch (`_prev_motor_enabled` handling) still fires for the AK60v3.
- **Log corruption returns on End Trial:** the close ordering broke — verify `SdLogger::close_active()`
  runs before `exo_system_reset()` and that `sd_logger.update()` still closes on the `trial_off` edge.
- **Shutdown dialog no longer reaches ACKED / hangs:** the `reset_ack` timing regressed — confirm it's
  still the first thing `get_system_reset()` does. (Cross-check the other end-trial log.)
- **Controller change still stalls / SD errors:** `_sd_ready()` cached a failed mount — it retries on
  failure, but if the card was pulled mid-session, `SD.open()` will fail in each branch (existing
  behavior). A missing card still hangs at boot (`ParseIni`), as before.
