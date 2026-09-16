# Windows-side BLE connection-parameter logger (`ConnParamsMonitor`)

**Date:** 2026-09-13
**Branch:** `disconnection_troubleshooting`
**Scope:** `Python_GUI/services/ConnParamsMonitor.py` (new) and six hooks in `Python_GUI/services/QtExoDeviceManager.py`,
each tagged `# ConnParamsMonitor`. **GUI only** - no firmware change, no protocol change, nothing sent to the Nano.
**Status:** **In production use, committed in `a5542b4` (2026-09-14).** Both files parse and import. The monitor was exercised against fakes (a full
30 -> 15 -> 30 ms cycle, read failures, a non-WinRT backend, a Windows-10-like device) and against the real WinRT device
object for `EXOBLE_d1a71e`. Bench-validated on **9 real connections on 2026-09-13** (B23 and B20 firmware, including one
`(6,6)` link failure) and on a **second Windows 11 laptop on 2026-09-14**: a 20-minute bench trial, then a **26+ minute session worn by the
operator with the external controller driving torque**, heartbeat at 30 ms throughout (operator report). **Longest run so
far: 26+ minutes.** **No automated tests** - the operator waived them for this change.

Findings it produced: `Nano-Hang-Watchdog-And-Breadcrumbs.md` §15. Raw logs: `Nano-Disconnect-EVIDENCE-ONLY.md` §12.

---

## 1. What it does

It logs the BLE connection parameters **Windows actually applies**, for the whole life of every connection, into the
device-manager log (`Python_GUI/Saved_Data/logs/device_manager_*.log`) and the terminal:

```
CONN_PARAMS monitoring started
CONN_PARAMS [attach] no active link (Windows reports zeros) read=0.18ms
CONN_PARAMS [status] interval=30.00ms latency=0 timeout=9600ms read=0.04ms
CONN_PARAMS [event] interval=15.00ms latency=0 timeout=9600ms read=0.02ms CHANGED (was interval=30.00ms latency=0 timeout=9600ms)
CONN_PARAMS [event] interval=30.00ms latency=0 timeout=9600ms read=0.01ms CHANGED (was interval=15.00ms latency=0 timeout=9600ms)
CONN_PARAMS [heartbeat] interval=30.00ms latency=0 timeout=9600ms read=0.02ms
CONN_PARAMS monitoring stopped
```

| tag | when it is written |
|---|---|
| `[attach]` | once, as soon as Bleak has created the WinRT device object - usually before the link exists, so it reads zeros |
| `[status]` | Windows' `ConnectionStatusChanged` event: the link came up or went down |
| `[event]` | Windows' `ConnectionParametersChanged` event: the interval, latency or timeout changed |
| `[heartbeat]` | every 10 s while attached - a cross-check that the events really fire |

`CHANGED (was ...)` appears only when a real value differs from the previous real value. Units are converted from WinRT's
(interval in 1.25 ms steps, supervision timeout in 10 ms steps) - the same units as the firmware banner, so
`interval=30.00ms timeout=9600ms` is the same thing as `CPi24_t960`. `read=` is how long the WinRT call took.

Pull the lines out of a session:

```
grep CONN_PARAMS Python_GUI/Saved_Data/logs/device_manager_<timestamp>.log
```

## 2. Why it exists

The firmware's connect banner carries `CPi` (the interval the link opened at) and `UPi` (a connection update). On
2026-09-13 the banner read `UPi12` = 15 ms on a trial whose CSV was locked at 30 ms for its whole length. The reason
(`Nano-Hang-Watchdog-And-Breadcrumbs.md` §15.2) is that `UPi` only ever shows the **first** update, and the GUI reads the
banner **once**, ~2.5 s after connecting. Windows changes the interval during that window and again afterwards, and nothing
on our side could see it. The trust question behind the investigation - "did this trial really run at the interval the fix
asks for?" - had no direct answer. This gives one, for every trial.

## 3. How it works

- **Where the data comes from.** Windows 11 (10.0.22000+) exposes `BluetoothLEDevice.GetConnectionParameters()` plus the
  `ConnectionParametersChanged` and `ConnectionStatusChanged` events. Bleak 2.1.1 keeps the WinRT `BluetoothLEDevice` in the
  **private** attribute `BleakClient._backend._requester`.
- **When it starts.** `QtExoDeviceManager._connect_target()` calls `_start_conn_params_watch(client)` immediately after
  constructing the `BleakClient` and **before** `await client.connect()`. `watch_client()` polls every 50 ms for `_requester`
  to appear (Bleak creates it at the very start of `connect()`, before GATT service discovery), then attaches. Starting later
  would miss Windows' 15 ms step, which happens during the discovery that `connect()` performs.
- **When it stops.** `_stop_conn_params_watch()` runs from `_mark_disconnected()`, from `disconnect()`, and from the three
  failure branches of `_connect_target()` (not connected / cancelled / exception). It detaches and cancels the task, is safe
  from any thread and safe to call repeatedly (the duplicate disconnect callback in
  `GUI-End-Trial-Duplicate-Disconnect-Callback.md` calls it twice).
- **Threads.** WinRT events arrive on a WinRT thread-pool thread; the heartbeat runs on the GUI's BLE asyncio thread. State is
  guarded by an `RLock` held only around the WinRT read itself.
- **Zeros.** With no live link Windows returns all zeros instead of raising (found on the real API, not in any doc). Those
  reads log `no active link (Windows reports zeros)` and are never recorded as the previous value, so link-up and link-drop
  never show up as fake `CHANGED` lines.
- **Read failures** log once per failure streak, then stay quiet until a read succeeds again.

## 4. Reading the log

**A normal B23 connection** (2026-09-13 19:17, Windows had 7.5 ms stored from a `(6,6)` session):

```
19:17:59.629 [event]  interval=7.50ms                         <- link opens at the stored value
19:17:59.807 [event]  interval=30.00ms CHANGED (was 7.50ms)   <- firmware asked once, Windows granted the top of (20,24)
19:18:00.227 [event]  interval=15.00ms CHANGED (was 30.00ms)  <- Windows' own step during service discovery
19:18:01.967 [event]  interval=30.00ms CHANGED (was 15.00ms)  <- step over, ~1.7 s
19:18:09.610 [heartbeat] interval=30.00ms                     <- and every 10 s from here on
```

**Cross-checks worth making:**
- The first non-zero value (`[status]` or the first `[event]`) must equal the banner's `CPi`.
- The first `CHANGED` line must equal the banner's `UPi`. Everything after it is what the banner cannot show.

**What matters for a trial:** any `CHANGED` line **after** the connect sequence. None has been seen in any logged
connection so far. If one appears, its timestamp says exactly when the trial left the interval it was running at.

**At a link failure** (2026-09-13 19:00, `(6,6)`): the heartbeat kept reporting 7.5 ms up to 19:00:06, 3.4 s after data had
already stopped, then `[status]` and `[event]` both read zeros at 19:00:12.48, 6 ms before Bleak's `link lost`. Windows
treats the parameters as valid until the 9.6 s supervision timeout expires, so **the last heartbeat before a disconnect
is the interval the link died on.**

## 5. Safety and cost

- **Nothing reaches the exo.** These are read-only queries of Windows' own Bluetooth stack: no GATT traffic, no writes to
  the Nano, so it cannot collide with parameter writes the way the 2 s ping could (`Nano-Hang-Watchdog-And-Breadcrumbs.md`
  §10.1). The WinRT call is synchronous (the Python bindings mark asynchronous calls with an `_async` suffix; this has none).
- **It cannot break the connection.** Every WinRT call is wrapped; nothing it does can raise into the connect or disconnect
  paths.
- **Measured cost:** a read takes <= 0.2 ms at a 30 ms interval (up to 0.8 ms at attach), and up to 2.3 ms at 7.5 ms while
  the RT stream is running. **On the second laptop, reads reached 15 ms while streaming** (11 of ~134 heartbeats above
  3 ms). The heartbeat read runs on the GUI's BLE thread, so a slow read delays incoming notifications by that long - it does
  not drop them. If that ever matters, move the heartbeat read off that thread. Once every 10 s plus a handful of events per
  connection. About 6 log lines a minute - fewer than
  the ping's existing DEBUG line every 2 s.

## 6. Limits

| limit | what you see |
|---|---|
| Needs Windows 11 (10.0.22000+) | `CONN_PARAMS not available on this host (needs Windows 11)` once; GUI unaffected |
| Relies on Bleak's private `_backend._requester` (checked on bleak 2.1.1) | `CONN_PARAMS off: WinRT device object never appeared` or `not a WinRT Bleak backend`; GUI unaffected |
| macOS / Linux backends | `CONN_PARAMS off: not a WinRT Bleak backend` |
| It reports what **Windows** believes | It cannot see the Nano's side of the link. A BLE sniffer remains the arbiter for anything below that |
| The link-up event is not flagged `CHANGED` | By design - the value before it was "no link", not a real interval |
| 10 s heartbeat resolution | Only matters if an event were ever missed; events have fired every time so far |

## 7. How to modify or remove

- **Heartbeat period:** `heartbeat_s` default in `watch_client()` (`ConnParamsMonitor.py`); `QtExoDeviceManager` uses the
  defaults.
- **Remove it entirely:** delete `Python_GUI/services/ConnParamsMonitor.py`, then every line tagged `# ConnParamsMonitor` in
  `QtExoDeviceManager.py` - the import, the two lines in `__init__`, the `_start_conn_params_watch` /
  `_stop_conn_params_watch` pair, the one start call in `_connect_target()`, and the five stop calls. `grep -n
  ConnParamsMonitor Python_GUI/services/QtExoDeviceManager.py` finds them all. The GUI then behaves exactly as before.

## 8. What it has shown so far

In short (details in `Nano-Hang-Watchdog-And-Breadcrumbs.md` §15.3-§15.4):

1. Windows opens a connection at the interval it stored for this device last time.
2. If that is outside `(20,24)`, the firmware asks once and Windows grants **30 ms** (the top of the range) within ~0.2-0.35 s.
3. During service discovery Windows switches to a fixed **15 ms for ~1.6-1.9 s**, then restores the value in force just
   before the step. This is why the firmware banner said `UPi12`.
4. After that the interval **never changed** in any logged connection - not while streaming, not before a failure.
5. The two 2026-09-12 "15 ms" failures were really 7.5 ms failures.
