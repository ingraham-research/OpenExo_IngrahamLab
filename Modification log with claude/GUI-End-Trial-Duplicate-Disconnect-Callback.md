# GUI: "Disconnected unexpectedly" after a normal End Trial (duplicate disconnect callback)

**Date:** 2026-09-13
**Branch:** `disconnection_troubleshooting`
**Scope:** `Python_GUI/services/QtExoDeviceManager.py` - `_mark_disconnected()` and the `_disc_cb` disconnect callback.
**Status:** **Found, NOT fixed.** Read from source and from the device-manager logs. No code changed. No data loss or
motor effect identified - the harm is a misleading message that sends the operator looking for a problem that is not there.

---

## 1. Symptom

After a normal End Trial, the GUI sometimes shows **"Disconnected unexpectedly"**, and the device-manager log shows two
disconnect lines a few milliseconds apart:

```
2026-09-13 17:27:49.837 | WARNING | _mark_disconnected | Device disconnected. Reason: link lost, Intentional: True
2026-09-13 17:27:49.838 | INFO    | _mark_disconnected | Intentional disconnect - no signal emitted
2026-09-13 17:27:49.839 | WARNING | _mark_disconnected | Device disconnected. Reason: link lost, Intentional: False
2026-09-13 17:27:49.842 | INFO    | _mark_disconnected | Disconnected signal emitted to UI
```

On 2026-09-13 this happened at the end of the first real-person trial on the B23 firmware. The operator had walked far from
the laptop just before, and reasonably read it as a range drop. **It was not.**

## 2. Mechanism

1. The GUI's End Trial path calls `disconnect()`, which sets `self._intentional_disconnect = True`.
2. Bleak's `disconnected_callback` (`_disc_cb` in `_connect_target()`) **sometimes fires twice**, 0-7 ms apart.
3. The first call goes through `_mark_disconnected()`, which correctly treats it as intentional - and then, at the end,
   **resets `self._intentional_disconnect = False`** ("Reset flag for next time").
4. The second call therefore sees `Intentional: False` and emits `disconnected`. `MainWindow._on_dev_disconnected()` shows
   the "Disconnected unexpectedly" warning and returns to the scan page.
5. Side effect seen on 2026-09-13: the external-control orchestrator reacted by calling `park_to_transparency()`, whose
   torqueScale=0 writes then failed with `Not connected` and timed out three times (17:27:49.881, 17:27:55.016,
   17:28:00.167). Harmless - the Nano was already rebooting from End Trial's `'Z'` - but noisy.

The trial CSV is not affected: End Trial closes it first (`CSV file closed` at 17:27:39.787, ten seconds before either
callback).

## 3. Evidence that it is not a range drop

- **The pattern is old.** Across all device-manager logs, an `Intentional: True` line followed within 0-7 ms by an
  `Intentional: False` line occurs **11 times in 10 logs**: 2026-07-06, 07-07, 07-21, 08-12, 08-20 (twice), 08-25, 08-26,
  09-09, 09-12 and 09-13.
- **The timing is identical to ordinary End Trials.** On 2026-09-13: `'Z'` delivered 17:27:39.800 -> link lost 17:27:49.837 =
  **10.04 s** (09-12 bench End Trials: 9.87 s, 9.91 s, 9.9 s). GUI `disconnect()` 17:27:45.895 -> callback = **3.94 s**
  (09-12: 3.83 s, 3.87 s). A range drop would not line up with the GUI's own disconnect call.
- **Distance cannot matter here.** `'Z'` reboots the Nano; the link ends whether the laptop is 1 m away or 20 m.

## 4. Fix shape - not built

Either of these, smallest first:

1. **Stop resetting the flag inside `_mark_disconnected()`.** Clear `_intentional_disconnect` at the start of `connect()`
   instead, so a duplicate callback for the same intentional disconnect still sees `True`.
2. **Ignore a callback for a client that is no longer current** - `_disc_cb` is a closure inside `_connect_target()`, so it
   can capture its own `client` and return early if that client is not the one being tracked.

Either should be checked against the duplicate-callback logs above and one real End Trial. The connection-parameter logger's
stop hook (`Windows-Connection-Parameter-Logger.md`) is already idempotent and needs no change.
