# Disabling the Heel FSR (runtime) + FSR-Refinement Status Visibility

**Date:** ~2026-07-06
**Scope:** Teensy 4.1 firmware (`ExoCode/`), Nano 33 BLE firmware (`ExoCode/` Nano branch), Python GUI.
**Status:** Working. Root-caused a real controller failure and fixed it.

This documents two related changes that came out of the same investigation:
(A) making the heel FSR optional at runtime (our hardware has **no heel FSR**), and
(B) surfacing the exo's status (cal / FSR-refinement / trial) on the GUI so you can tell when the
system is actually ready.

---

## Part A — Heel FSR made optional at runtime

### Why (the bug this fixed)
Our ankle hardware has **only toe FSRs, no heel FSRs**. But the firmware still read the heel-FSR
analog pins. Those pins were **floating / phantom** — unconnected inputs pick up noise — and that
noise fed the gait-timing/ground-contact logic. Concrete symptoms we chased:
- **Spline controller "a mess":** one leg never produced torque, the other worked then failed
  mid-trial. PJMC was less affected. Root cause: the phantom heel signal corrupted the gait-phase
  estimate (`percent_gait`), which the spline controller indexes into.
- **Exo stuck in FSR refinement:** refinement needs `_num_steps` (7) clean Schmitt low→high
  crossings **per FSR**. A floating heel FSR can never produce clean crossings, so refinement for the
  heel never completes → the exo sits in refinement forever and never reaches "ready".

Disabling all heel-FSR use removed the phantom signal and let refinement (toe-only) complete.

### The mechanism: `heel_fsr_present()`
A single runtime flag gates **all** heel-FSR use. It reads `config.ini` once and caches the result.

| File | Role |
|---|---|
| `ExoCode/src/HeelFsrConfig.h` | Declares `bool heel_fsr_present();` (platform-agnostic so it compiles on both MCUs). |
| `ExoCode/src/HeelFsrConfig.cpp` | Teensy: reads `[Sensors] heelFsrPresent` from `/config.ini` (cached). Nano: stub `return false;` (Nano has no SD; the flag only matters where the Teensy uses sensors). |
| `SDCard/config.ini` | `[Sensors] heelFsrPresent = 0` (0 = toe-only; 1 = heel installed). |

> Previously this was a **compile-time** `USE_HEEL_FSR` in `Config.h`. It was converted to a runtime
> `config.ini` flag to avoid reflashing when toggling. The `Config.h` define was removed (a comment
> points to `[Sensors] heelFsrPresent`).

### Where it's gated
- `ExoCode/src/Side.cpp` — every `_heel_fsr.*` call is wrapped in `if (heel_fsr_present())`:
  constructor thresholds, `read_data` (the passive analog read *and* thresholds), `reset_calibration`,
  `check_calibration` (heel cal/refine block), `_check_thresholds`, and crucially the ground-strike
  input:
  ```cpp
  bool heel_contact_state = heel_fsr_present() ? _heel_fsr.get_ground_contact() : false;
  ```
  When the flag is false the heel pin is **never read**, so a floating pin cannot inject noise.
- `ExoCode/src/uart_commands.h` — the FSR calibrate/refine handlers gate their heel flags with
  `if (heel_fsr_present())` so the GUI's "calibrate/refine FSR" doesn't wait on a nonexistent heel.
- `ExoCode/src/HeelFsrConfig.h` is `#include`d wherever the gate is used.

### Ground-strike detection — how toe vs heel interact (important context)
`Side::_check_ground_strike()` fires a ground strike on the **rising edge of heel OR toe** contact
while previously in full swing (`!prev_heel && !prev_toe`); the per-step gate opens once at first
contact and `percent_gait = 100*(now - strike_ts)/expected_step_duration`.
- **Toe** contact = `FSR_Regressed`; **heel** contact = `FSR`.
- With **both** FSRs present, heel-strike and toe-strike are two different events in a stride; if both
  are active the "OR" can retrigger/mistime the step and skew `percent_gait`. This is why a
  *phantom* heel was so damaging, and something to keep in mind if a real heel FSR is ever added.
- With **toe-only** (our case), strikes are toe-onset only — clean and sufficient for the spline's
  gait-phase indexing.

### How to re-enable a real heel FSR later
1. Set `[Sensors] heelFsrPresent = 1` in `SDCard/config.ini` (no reflash; the value is read once at
   boot / first use and cached).
2. Verify the heel FSR pin is actually wired (a floating pin at `=1` reintroduces the original bug).
3. Reconsider the ground-strike OR logic above if double-triggering appears.

### Gotchas
- `heel_fsr_present()` **caches** on first call, so changing `config.ini` requires a reboot.
- It's declared platform-agnostic on purpose: `uart_commands.h` compiles on **both** MCUs, so the
  Nano needs the symbol — hence the Nano stub. Don't Teensy-guard the declaration.

---

## Part B — FSR-refinement status visibility on the GUI

### Why
While debugging "stuck in refinement," there was no way to see the exo's state from the GUI — you
couldn't tell when calibration/refinement finished and the exo was actually ready. This adds that
visibility.

### How it works (the "Channel 8 hijack")
The bilateral-ankle real-time stream had an **unused** channel 8 (a placeholder constant). It's
repurposed to carry the exo status.
- `ExoCode/src/uart_commands.h` — in the bilateral-ankle real-time-data builder:
  ```cpp
  rx_msg.data[8] = (float)exo_data->get_status();   // HIJACK: was an unused placeholder
  ```
  (Marked with a `//HIJACK:` comment noting the status legend and how to revert.)
- `ExoCode/src/PlottingTitles.h` — bilateral-ankle `case 8: return "Status";`.
- **GUI** (`Python_GUI/pages/ActiveTrialPage.py`, `MainWindow.py`) — a status label maps the status
  code to a name/colour (e.g. 2 = "Trial On / Ready" green; 4/5/6/7 = cal/refine/etc. orange;
  ≥8 = error/warning red). `MainWindow._on_rt_update` calls `trial_page.update_exo_status(values[8])`.

### Status codes (`ExoCode/src/StatusDefs.h`, `status_defs::messages`)
`0` off · `1` trial_off · `2` trial_on · `3` test · `4` torque_calibration · `5` fsr_calibration ·
`6` fsr_refinement · `7` motor_start_up · error bit `1<<3`. "Active trial" (motors run) =
`trial_on || fsr_calibration || fsr_refinement`.

### Notes / gotchas
- Channel 8 is **logged/streamed** but is **not** one of the live-plottable channels in the GUI's
  4-channel plot blocks (those cover 0–7). It's read out of `values[8]` for the status label, not the
  plot. Don't expect to see it on the live graph.
- Clean transition detail: when toe-FSR refinement completes, `Side::check_calibration` sets status
  to `trial_on` (ready). It settles at `trial_on` once **both** sides finish. (Note: the "click Start
  Trial → cal → refine → ready" chain is driven by the existing `cal_fsr` BLE handler in
  `ble_commands.h`, which sends `update_cal_fsr` then `update_refine_fsr`.)
- To revert the hijack, restore `rx_msg.data[8]` to its old placeholder and `PlottingTitles` case 8
  to "Channel 8"; see the `//HIJACK:` comments.

### Related
The FSR-refinement stuck behavior and the heel-phantom root cause are the same investigation that
also produced the SD logger and end-trial work — see
`Modification log with claude/SD-Card-Logging-and-End-Trial-Reset.md`.
