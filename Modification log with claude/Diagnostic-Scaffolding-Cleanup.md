# Diagnostic scaffolding: what goes, what hides behind a flag

**Date:** 2026-09-17 · **Branch:** `fix_ack_failures`
**Part 1 (GUI logging): DONE, PC only, no flash.** **Part 2 (firmware): PLAN ONLY, nothing changed.**

Operator's rule for this cleanup, and the reason it exists: *delete what never produced information; keep
what did its job, but behind a debug flag, so a production session is quiet and a debugging session still
has everything.* Odd design choices accrete when temporary scaffolding is left in the normal path.

The verdicts come from `Nano-Hang-Watchdog-And-Breadcrumbs.md` §0.6b (2026-09-12), re-checked against the
code on 2026-09-17: none of its "discard" items had been removed yet. Its precondition - the interval fix
confirmed at torque and on a second laptop - was met on 2026-09-13/14.

## Part 1 - one log file per session (DONE 2026-09-17)

`GUI.py`, `services/QtExoDeviceManager.py`, `MainWindow.py`, `tests/test_logging_merge.py`.

- The BLE layer logged into its own `device_manager_*.log`. Reading a session meant correlating two files
  by timestamp - which is exactly what the ACK investigation had to do all day - and its exception hook
  chained to `GUI.py`'s, so every crash was written to both files.
- It now logs as `OpenExo.DeviceManager` into the one application logger. Its duplicate exception hook is
  gone; `GUI.py`'s hook already logged the same crash.
- **Nothing was trimmed.** `--verbose-log` (or `EXO_LOG_VERBOSE=1`) decides how much is kept:

  | | file | console |
  |---|---|---|
  | default | INFO | WARNING |
  | verbose | DEBUG | INFO |

  The chatty per-command BLE lines are DEBUG, so they are there when you ask for them. `CONN_PARAMS`,
  `Sending parameter update` and RtBridge's frame log are INFO and stay in every session.
- Constructing `QtExoDeviceManager` no longer creates a log file as a side effect, so tests stop
  littering `Saved_Data/logs`.
- The file is still named `app_crash_*.log`. It is really the session log; renaming is a separate
  decision (only write-ups reference the name, no code does).
- 4 new tests; 56/56 in `Python_GUI/tests` pass.

## Part 2 - firmware scaffolding (PROPOSED)

### The flag

One master switch, `EXO_DIAG` in `Config.h`, default **0**. Same idea as `--verbose-log`: the code stays,
the cost does not. `PARAM_ACK_DIAG` already works this way and would fold into it.

### Verdicts

**DELETE - never produced information**

| item | where | why |
|---|---|---|
| Stage breadcrumbs (`EXO_STAGE_*`, `exo_wdt_stage*`, the `.noinit` block) | `ExoCode.ino` (35 sites), `ExoBLE.cpp`, `SystemReset.h/.cpp` | Proven not to survive a watchdog reset, so every reading was meaningless |
| `RT_BLE_FORWARD` bisect switch | `Config.h`, `ComsMCU.cpp` | Experiment over; must stay 1 in any normal build |
| `SD_LOG_SELFTEST_TRIAL` block | `SdLogger.h`, `ExoCode.ino` | Self-test that halts in `setup()`; currently 0 |

**GATE behind `EXO_DIAG` - did their job, worth keeping for the next hunt**

| item | what it settled |
|---|---|
| Send-path diagnostics (`s_max_write_us`, `s_sends_since_rx`, `exo_ble_link_diag`, banner `_w`/`_s`/`_FAIL`) | Proved the bounded ArduinoBLE wait fires |
| Teensy RT-I2C counters + the link-stats UART command | Cleared I2C as a cause |
| Crash-trap self-test (`EXO_CRASH_TRAP_SELFTEST`) | Proves the trap works; currently 0 |
| `PARAM_ACK_DIAG` ACK counters (2026-09-16) | Proved the ACK is lost after the Nano sends it. Set to 0 after the worn session |

**KEEP ALWAYS - fixes and safety, not scaffolding**

Connection-interval fix and its selector; the hardware watchdog and boot-loop guard; the BLE stall
detector with the 2 s ping and TX-busy counter; the bounded `sendAclPkt` wait (ArduinoBLE); the
reset-reason banner itself (it is how a watchdog or stall event is reported); `EXO_FW_TAG` (cheap, and it
settled "which binary is on the board" repeatedly); the CSV exo-time unwrap; `ConnParamsMonitor`.

**DECIDED 2026-09-17: KEEP the Nano's own `CPi`/`UPi` connection-parameter readout.**

ArduinoBLE carries **three** local patches, not one (diffed against `HCI.cpp.orig-openexo-backup`):

| # | added | what | behaviour |
|---|---|---|---|
| 1 | 09-11 | `sendAclPkt` gives up after 50 ms instead of spinning on `_pendingPkt >= _maxPkt` | **changes behaviour** (drops a notification rather than hanging) |
| 2 | 09-12 | records the connection parameters the central chose, at LE Connection Complete | record-only |
| 3 | 09-12 | adds a case for LE Connection Update Complete (0x03), which upstream ignores | record-only |

Patches 2 and 3 feed the banner's `CPi`/`UPi`. An earlier draft proposed dropping them to reduce the
number of patches; on the evidence that is the wrong trade. They cost a few variables and two assignments
per connection event, they are the only firmware-side view of the interval (the GUI monitor needs
Windows 11 and reports what that host says), and the repo now vendors the patched library, so a library
update is a restore rather than a re-derivation. `UPi`'s known flaw - it shows only the first update - is a
documentation matter, already in the retraction ledger. If the banner is noisy, trim the text, not the
capture. They could not be `EXO_DIAG`-gated anyway: they live in the library, which does not see `Config.h`.

### Order, one step per flash and per bench trial

| Phase | Content | Risk |
|---|---|---|
| P1 | Add `EXO_DIAG` (default 0) and gate the four "did their job" items. No behaviour change at 1 | Low, but wide |
| P2 | Delete the breadcrumbs and the `.noinit` block | Medium: many sites, and it shares a latch with the send-path diagnostics, which must survive under the flag |
| P3 | Delete `RT_BLE_FORWARD` and the SD self-test block | Low |
| P4 | `PARAM_ACK_DIAG` to 0 after the worn session | Low |
| ~~P5~~ | ~~Decide on `CPi`/`UPi`~~ - **decided: keep, see above.** The plan ends at P4 | - |

### Risks and how they are handled

- **Every phase touches the live BLE path.** A mistake there looks exactly like the bug this whole
  investigation chased. So: compile both boards, then **one real bench trial per phase** (zero torque,
  trial on), not just a compile. Verification commands are in `Nano-Hang-…` §11.6.
- **The banner is parsed by the GUI** (`MainWindow._on_reset_reason`). When fields disappear, that text
  must follow, and the parser must tolerate missing fields.
- **Flashing habit:** the watchdog survives a warm reset and can kill an upload; power-cycle the Nano
  before flashing (`Nano-Hang-…` §7.0). Unchanged by this work.
- **Two copies of ArduinoBLE.** The build uses the sketchbook copy
  (`Documents\Arduino\libraries\ArduinoBLE`); `Libraries/ArduinoBLE` in the repo is a restore copy, today
  identical apart from line endings. Any library edit must be made in both, or they drift silently.
- **Stop condition:** if any phase's trial looks off, revert that phase (each is a separate commit-sized
  change) rather than debugging forward.
