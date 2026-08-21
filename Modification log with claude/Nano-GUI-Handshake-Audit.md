# Nano ↔ PC GUI Handshake — Full Pipeline Audit

**Date:** 2026-08-19
**Branch:** `fix_nano_GUI_handshaking`
**Status:** **AUDIT ONLY — no code was changed.**
**Supersedes/extends:** `BLE-Handshake-Controller-List-Loss.md` (2026-07-23). That document's
detection work still stands; this one re-opens the *mechanism* question, and corrects one of its
conclusions.

---

## 0. What this session was asked to do

> "Take a comprehensive look at how this codebase does handshaking between nano and PC GUI …
> I would like to have a better idea of WHY it is happening and how we can solve it. Pay special
> attention to stuff like spelling mismatch, case sensitivity, etc."

So: the whole path, and specifically the *deterministic* failure classes (naming, case, parsing)
that the earlier RF-focused investigation did not look for.

**Headline result: there are two completely independent bug classes here, and they produce the
same symptom.**

* **Class A — transport loss.** Real, currently active, ~20% of connections. Now pinned down much
  more precisely than before: the loss quantum is **exactly one 19-byte BLE notification**.
* **Class B — naming / parsing traps.** Not currently firing, but several are one typo away from
  silently deleting an *entire joint* from the dropdown. These are the ones you asked about, and
  they are genuinely dangerous because no check in the system would flag them.

---

## 1. The pipeline, end to end

Five hops. Every one of them is a place a controller can vanish.

```
 [1] SD card CSVs        SDCard/ankleControllers/*.csv  (+ config.ini)
       |                 ParseIni.cpp   — config.ini name -> uint8_t enum   (CASE-SENSITIVE)
       v
 [2] Teensy build        ListCtrlParams.cpp
       |                 - gates joints on config.ini defaults
       |                 - reads CSV line 5 (param names) + line 6 (values)
       |                 - truncates EVERY cell to 9 chars (MAX_STRING_LENGTH=10)
       |                 - flattens into txBuffer_bulkStr: "f,<row>\n<row>\n...,??"
       v
 [3] Teensy -> Nano      UART Serial1 @115200, GetBulkChar.cpp
       |                 blocking state machine, framed 'f' ',' ... ",??"
       |                 NO checksum, NO retry
       v
 [4] Nano -> GUI         ExoBLE.cpp send_handshake_payload()
       |                 - "READY", then "n,<rows>|" header, then payload
       |                 - '\n' -> '|', trailing '\n' terminates
       |                 - 19-byte chunks, delay(20) each  => ~170 chunks, ~3.4 s
       |                 - ONCE per connection, no retransmit, no re-request command
       v
 [5] GUI parse           RtBridge.feed_bytes() -> controller matrix
                         MainWindow -> ActiveTrialSettingsPage -> dropdown
```

Two facts about the shape of this design that matter more than any individual bug:

1. **The controller list is fetched exactly once per BLE connection.** `ExoBLE.cpp` gates on
   `_handshake_sent_this_connection`, and there is **no command anywhere that asks the exo to send
   it again**. A controller missing from the dropdown has been missing since the moment you
   connected — starting a trial, or changing a controller, cannot cause it. You just *notice* it
   then. (This reframe was correct in the July doc and is re-confirmed here.)
2. **There is no integrity check on the payload — only heuristics after the fact.** No CRC, no
   byte count, no per-row checksum. The `n,<rows>` header is the only integrity signal, and
   §3.2 shows it is blind to the most common damage pattern.

---

## 2. What the logs actually show

96 connections across `Python_GUI/Saved_Data/logs/app_crash_*.log`:

| Matrix entries | Connections | |
|---|---|---|
| 20 (complete) | 77 | |
| 19 | 9 | |
| 18 | 5 | |
| 17 | 1 | |
| 15 | 1 | |
| 13 | 2 | |
| **21** | 1 | more than possible — a phantom row (pre-guard, 2026-07-23) |

**~20% of connections deliver a damaged controller list.** That is worse than the 11% measured in
July, on a larger sample.

Every handshake produced a matrix (handshake count == matrix count in all 96 logs), so the payload
never fails to arrive *entirely* — it arrives with holes.

---

## 3. Class A — transport loss

### 3.1 The loss quantum is exactly one BLE notification (proven)

On 2026-08-12 17:47:48 the GUI logged a list with a **full 20 entries**, no row loss, but this
asymmetry warning:

```
Ankle(L) (68) missing constantT; Ankle(R) (36) missing constrection
```

`constrection` is 12 characters. The firmware truncates every cell to **9** characters, so this
string cannot have been produced by the Teensy. It was manufactured in transit.

Reconstructing the true row from `SDCard/ankleControllers/constantTorque.csv`:

```
Ankle(L),68,constantT,5,Amplitude,Direction,Alpha,PID Flag,P Gain,I Gain,D Gain
```

Deleting **exactly 19 bytes** starting at offset 17 — `antT,5,Amplitude,Di` — yields:

```
Ankle(L),68,constrection,Alpha,PID Flag,P Gain,I Gain,D Gain
        ^^ joint id still numeric      ^^^^^ "Alpha" is now the CONTROLLER ID
```

which is byte-for-byte the string in the log. Brute-forcing every other mangled name found across
all logs against a payload reconstructed from the real CSVs:

| Logged fragment | Explained by |
|---|---|
| `constrection` | drop of **19** bytes (1 notification) |
| `Timing Th` | drop of **19** bytes |
| `zhe` | drop of **38** bytes (2 notifications) |
| `zha` | drop of **38** bytes |
| `pjTh` | drop of **38** bytes |
| `pjl A` | drop of **57** bytes (3 notifications) |
| `Timing T A` | not reproduced — from the 2026-07-23 payload, whose parameter set differs |

**Every reproducible case is a contiguous deletion of an exact multiple of 19 bytes.** 19 is
`kHandshakeChunkSize`. So the hole is created at *notification granularity*, and whole
notifications go missing while everything before and after arrives intact and in order.

### 3.1.1 Independent corroboration from the trial CSV headers (found 2026-08-19)

The controller rows are not the only thing in the handshake payload. The plot/CSV channel titles
arrive in the same transfer as the `t` row (from `PlottingTitles.h`), and they land in the header of
every trial CSV — a completely separate artefact from the log messages above.

`Saved_Data/trial_20260812_174818.csv` ends its header with:

```
... ,Commanded Torque (R),Status,Econds)
```

against a reference of `Exoskeleton time (seconds)`. Decomposing it: prefix `E` survives, suffix
`conds)` survives, and the deleted run is **`'xoskeleton time (se'` — exactly 19 bytes.**

This matters for three reasons:

1. **It is independent confirmation of the 19-byte quantum**, from a different file, a different
   part of the payload, and a different code path than the mangled controller names.
2. **It was the same connection as `constrection`.** The log shows the handshake at 17:47:48, the
   previous disconnect at 17:40:46, and the CSV opening at 17:48:18 — one connection in between.
   So that single handshake lost **two separate notifications**. Losses therefore **cluster within
   a connection** rather than being independent per-packet coin flips, which is what you would
   expect if the cause is a sustained condition (buffer saturation) rather than random on-air luck.
3. **The measured damage rate in §2 is an undercount.** None of the four completeness checks looks
   at the `t` row at all, so this connection's second hole was never flagged — and the field count
   was unchanged (the deletion contained no comma), so even the "13 parameter names" log line
   looked normal.

**And it has a silent functional consequence.** `ActiveTrialPage.set_channel_labels` does:

```python
self._exo_time_idx = self._param_names.index("Exoskeleton time (seconds)")
except ValueError: self._exo_time_idx = None
```

An exact string match. With the title corrupted the lookup raises, `_exo_time_idx` becomes `None`,
and `_x_for_sample` falls back to **wall-clock arrival time** for the whole trial instead of the
exo's own clock. The sample *data* is fine (channel count unchanged), but that trial's x-axis is the
jittery BLE-arrival timebase the exo-clock channel exists to avoid — with nothing on screen saying
so.

Sweeping all 27 trial CSVs: 13 differ from the modal header only by channel count (a genuinely
different exo config, not damage), 1 shows this 19-byte deletion, and 1 more is discussed in §5.4.

### 3.2 Why the current checks miss it

The `constrection` case is the important one because it slipped through almost everything:

- The lost bytes stayed **inside one row**, so no `|` was destroyed → **row count unchanged** →
  check 0 (`n,<rows>` header) silent.
- The joint ID field survived and is still numeric → **passes the phantom guard** → not counted
  malformed → check 1 silent.
- 20 entries, same as every good connection → check 3 silent.
- **Only the left/right asymmetry check (check 2) caught it.**

Consequence: **on a unilateral exo, or if the same chunk were lost on both sides, this damage is
completely invisible.** The user gets a dropdown that looks perfectly normal and is wrong.

Worse, in the damaged row the **controller ID slot now holds `"Alpha"`**. In
`ActiveTrialSettingsPage._on_apply` (line ~453):

```python
controller_id = controller_local_idx   # Default to local index if parsing fails
```

So applying a parameter on a corrupted row does not error — it **silently falls back to the
dropdown position** and writes the parameter into *some other controller's* parameter set on the
exo. That is a correctness bug with real experimental consequences, not just a cosmetic one.

### 3.3 Re-opening the "it's just RF" conclusion

The July document closed with *"the cause is radio, not code"*, based on the user's (entirely
valid) experiment that connecting close to the laptop gives a clean list every time.

The distance correlation is real. **But "RF marginality" is not a sufficient mechanism**, and it is
worth being precise about why, because it changes what fixes are available:

> BLE's Link Layer is an acknowledged, retransmitting, in-order protocol. A corrupted or lost
> over-the-air packet fails CRC and is **retransmitted by the controller** until it lands. Bad RF
> makes a link *slow*, and eventually makes it *drop* (supervision timeout). It does not punch a
> clean 19-byte hole in the middle of an ATT stream and then deliver everything after it perfectly.

Two supporting observations from this audit:

- **The bytes that do arrive are never corrupt.** `RtBridge.feed_bytes` logs
  `"Failed to decode received data"` on any non-UTF-8 chunk. Across all 96 connections there are
  **zero** such log lines. Damage is always whole-chunk deletion, never bit corruption — which is
  what you would expect if the LL is doing its job and the loss is happening *above* it.
- **The Nano's own TX path applies backpressure and does not drop.** Confirmed by reading the
  vendored library at `Libraries/ArduinoBLE/src/utility/HCI.cpp:416`:
  ```cpp
  int HCIClass::sendAclPkt(...) {
    while (_pendingPkt >= _maxPkt) { poll(); }   // blocks, does not discard
  ```

So where does a whole notification go? The single most suspicious thing found in this audit:

**The entire 3.4-second payload is transmitted from inside a BLE event callback.**

`ExoBLE::_handle_tx_subscribed()` is the `BLESubscribed` handler for the TX characteristic, and it
calls `send_handshake_payload()` directly — ~170 × `writeValue()` + `delay(20)`. The Nano's BLE
stack is therefore blocked inside an event handler for ~3.4 s on every connection.

This is measurable in the GUI logs. `QtExoDeviceManager` does
`await start_notify(UART_RX)` → `await start_notify(ERROR_CHAR)` → `connected.emit(...)`. Across
**94/94** successful connections, the full controller matrix was parsed *before* `_on_dev_connected`
ran (median gap 52 ms, max 1.56 s). The second `start_notify` — a GATT descriptor write — cannot be
serviced until the Nano escapes its notify loop. **The GUI's connect sequence is being stalled by
the Nano's blocking send.**

And the loop is not as "dumb" as the comment above it claims. The comment warns:

```
// pumping BLE.poll() inside this loop is worse than dead, because it lets other BLE events
// dispatch mid-payload and interleave bytes into the stream. Keep this loop dumb.
```

But `writeValue()` → `ATT.handleNotify()` → `HCI.sendAclPkt()` → **`poll()` whenever the controller
buffer is full**. Buffers fill precisely when the link is slow — i.e. **at range**. So the exact
re-entrancy the comment forbids is being triggered automatically by the library, and more often the
farther away you are.

### 3.3.1 The specific defect: a credit-accounting race in `HCI::sendAclPkt`

> **STATUS: REFUTED by the 2026-08-20 experiment — see §7.6.** The mechanism described below is
> real code and the race is genuinely possible, but it is **not** what is causing the loss here.
> The section is kept because ruling it out was informative and because the re-entrancy is still a
> latent hazard worth knowing about. The 19-byte quantum it was built to explain **was confirmed**.

`Libraries/ArduinoBLE/src/utility/HCI.cpp:416`:

```cpp
int HCIClass::sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data)
{
  while (_pendingPkt >= _maxPkt) {
    poll();              // <-- can re-enter sendAclPkt and consume the credit we just waited for
  }
  ... build txBuffer ...
  _pendingPkt++;         // <-- condition is NEVER re-checked after poll() returns
  HCITransport.write(txBuffer, sizeof(aclHdr) + plen);
}
```

`_pendingPkt` is the host's count of ACL packets handed to the controller but not yet acknowledged
by a *Number Of Completed Packets* event; `_maxPkt` is the controller's buffer count from
*LE Read Buffer Size*. Sending when `_pendingPkt == _maxPkt` overruns the controller's buffer, and
the controller **silently discards the packet**.

The re-entrancy is confirmed by reading the chain: `poll()` (HCI.cpp:113) reads inbound ACL data →
`handleAclDataPkt` → `ATT.handleData` → and every ATT response path calls `HCI.sendAclPkt()` again
(ATT.cpp:628, 723, 800, 868, 999, 1115, 1230, 1317).

So:

1. Outer `sendAclPkt` (our notification chunk) spins because `_pendingPkt == _maxPkt`.
2. A *Number Of Completed Packets* event arrives during `poll()`; `_pendingPkt` drops to
   `_maxPkt - 1`; the `while` is now satisfiable.
3. **Still inside that same `poll()`**, an inbound ATT request is dispatched and its response calls
   `sendAclPkt` nested. The nested call sees `_pendingPkt < _maxPkt`, passes, and takes the credit
   (`_pendingPkt` back to `_maxPkt`).
4. `poll()` returns. The outer call **does not re-evaluate its condition** — it already left the
   loop — so it does `_pendingPkt++` (→ `_maxPkt + 1`) and writes.
5. The controller has no buffer. **That notification is dropped.** Everything before and after is
   unaffected.

That is a one-whole-notification loss, no corruption, and it needs **both** (a) buffer saturation
— which is what a slow link at range produces — and (b) an inbound ATT request mid-payload.

**And condition (b) is guaranteed on every single connection.** `QtExoDeviceManager` does
`await start_notify(UART_RX)` — which triggers the Nano's `BLESubscribed` and starts the 3.4 s blast
— and *then* `await start_notify(ERROR_CHAR)`. That second call is a CCCD **Write Request**, and it
therefore lands while the Nano is mid-payload, generating a Write Response from inside the notify
loop. The GUI's own connect sequence supplies the trigger.

**This is a hypothesis, not a proven root cause.** But unlike "the radio is marginal" it is sharply
falsifiable, it explains every observation (notification granularity, clean surviving bytes,
distance dependence), and the first test needs **no firmware flash at all** — see §7.

**What I checked and ruled out while forming it:** ATT's response and notification buffers are all
stack-allocated VLAs (`uint8_t notification[_peers[i].mtu]`, `uint8_t response[mtu]`), not shared
statics, so there is **no shared-buffer clobber** — plain "interleaving" does not itself corrupt the
stream. The credit race above is the only path found that actually loses a whole packet.

*(Separate latent hazard, same code, not implicated here: `poll()` assembles inbound packets into
the shared members `_recvBuffer` / `_recvIndex`. Re-entering `poll()` from inside `sendAclPkt`
therefore corrupts any partially-assembled inbound packet. That would damage commands **to** the
exo, not the handshake stream — worth remembering if RX-side flakiness ever shows up.)*

### 3.4 Confirmed dead ends (do not retry)

Carried forward from the July doc, still valid:

- **Retrying on `writeValue()`'s return value.** Implemented, flashed, did nothing.
  `BLELocalCharacteristic::writeValue()` returns `ATT.handleNotify()`, which discards
  `HCI.sendAclPkt()`'s result and reports success whenever any peer is connected.
- **Calling `BLE.poll()` explicitly in the chunk loop.** Actively harmful; reverted.
- **Raising `kHandshakeChunkSize` above 19** without first confirming a negotiated MTU > 23.
  Over-MTU writes silently truncate *every* notification.

---

## 4. Class B — naming, case, and parsing traps

These are what you asked me to look for. **None of them is currently firing** — the SD card and
`config.ini` in the repo are consistent — but each one is a single typo away from silently
removing controllers, and **not one of them would produce a warning.**

### B1. `config.ini` controller name → enum is case-sensitive with a silent zero fallback ⚠️ **worst one**

`ParseIni.h:398` defines `config_map` as `std::map<std::string, uint8_t>` — exact, case-sensitive
match. `ParseIni.cpp:295`:

```cpp
config_to_send[config_defs::exo_ankle_default_controller_idx] =
    config_map::ankle_controllers[data.exo_ankle_default_controller];
```

`std::map::operator[]` **inserts a default-constructed `0` for a key that does not exist**. The
header even documents the contract:

> *"If you see a uint8_t that is zero it indicates the field didn't exist."*

Verified locally (the maps are `const`, which compiles only because Teensyduino builds with
`-fpermissive`):

```
spline -> 12      splin -> 0      pjmc -> 0
```

Now follow that zero into `ListCtrlParams.cpp:75`:

```cpp
if ((config_to_send[config_defs::exo_ankle_default_controller_idx] > 1) && ...) { ... }
else { continue; }     // <-- the WHOLE JOINT is skipped
```

**A misspelled or miscased `ankleDefaultController` in `config.ini` silently removes both ankles
and all 10 of their controllers from the handshake.** No error, no warning, no log line. The GUI
would just show an empty or hip-only dropdown. Same for `sides` — `Bilateral` instead of
`bilateral` maps to 0 and kills every joint on both sides.

**And `config.ini` actively documents the wrong spelling.** Line 38:

```
;Ankle Controllers: zeroTorque, PJMC, zhangCollins, constantTorque, TREC, calibrManager,
;                   chirp, step, splin, SPV2, PJMC_PLUS
                                  ^^^^^ typo — the map key is "spline"
```

A user following the file's own instructions and writing `ankleDefaultController = splin` gets a
silently disabled ankle. The valid keys are also inconsistently cased (`TREC`, `SPV2` and
`PJMC_PLUS` uppercase; `zeroTorque`/`chirp`/`step`/`spline` not; `phmc` lowercase for hip while the
file on disk is `PHMC.csv`), which makes this very easy to get wrong.

### B2. Controller name extraction is case-sensitive on `.csv` and needs a forward slash

`ListCtrlParams.cpp` (`retrieveJointAndController`):

```cpp
const char* joint_end_ptr      = strchr(start_ptr, '/');                // must be '/', not '\'
const char* controller_end_ptr = strstr(controller_start_ptr, ".csv");  // case-sensitive
if (!controller_end_ptr) return false;
```

On failure the caller does **not** skip the row — it emits it with the name `"UNKNOWN_CTRL"`, which
is then truncated to 9 chars → **`UNKNOWN_C`**.

So a map entry written as `ankleControllers/spline.CSV` (or with a backslash — and note the
function's own doc comment says `"\Joint\Controller.csv"`, i.e. backslashes, which would fail)
produces a dropdown row named `UNKNOWN_C`. If two entries fail, you get **two identical
`UNKNOWN_C` rows**, the left/right asymmetry check stays happy, and it reads exactly like
"my controller is missing".

### B3. Every cell is truncated to 9 characters — collisions are silent

`MAX_STRING_LENGTH = 10`, and the copy is `strncpy(dst, src, maxLen - 1)` → **9 usable chars**.
Current ankle names after truncation:

```
zeroTorqu  PJMC  zhangColl  spline  constantT  trec  chirp  step  spv2  pjmc_plus
```

All unique today. But add `constantTorque2.csv` and it also becomes `constantT`. The dropdown then
shows two identical `constantT` rows, and the asymmetry check compares *sets of names* —
`per_joint.setdefault(...).add(str(row[2]))` in `RtBridge._emit_matrix_completeness` — so the
duplicate collapses and the check reports everything is fine. **Any new controller whose name
matches an existing one in the first 9 characters will look like it failed to appear.**

### B4. The CSV parser has no quote handling, and `zhangCollins.csv` relies on quotes

`SDCard/ankleControllers/zhangCollins.csv` line 5:

```
Torque (Nm),Peak Time (% Gait),Rise Time (% Gait),Fall Time (% Gait),"Direciton (0 = PF, >0 = DF)",use_pid,sim %gait,p_gain,i_gain,d_gain
```

That quoted field **contains a comma**. Both parsers — `readAndParseFifthRow` on the Teensy and
`row.split(",")` in `RtBridge` — are naive comma splitters with no quote awareness. So this one
field becomes **two columns**, and the names row for zhangCollins carries 11 parameter names while
its values row (line 6) carries 10 values. **Parameter names and values are misaligned by one from
that column onward for this controller.**

(Also note the field itself is misspelled — `Direciton` — which is cosmetic, but it is the kind of
thing this audit was asked to surface.)

### B5. An empty cell truncates the rest of the row

`create_csv_message` (`ListCtrlParams.cpp`):

```cpp
if (stringArray[i][j][0] == '\0') { break; }   // stops the row at the first empty cell
```

A blank cell anywhere in CSV line 5 or 6 — a stray `,,` while editing — **silently drops every
parameter after it** for that controller. The controller still appears; its parameter list is just
short. Nothing warns.

### B6. The GUI drops empty fields, which shifts every later field left

`RtBridge.feed_bytes`:

```python
parts = [part.strip() for part in parts_raw if part.strip()]
```

Empty fields are discarded rather than preserved as empty strings. This is the same left-shift that
makes chunk loss so destructive, and it means a legitimately empty cell is indistinguishable from a
missing one. Positional fields (`row[1]` = joint ID, `row[3]` = controller ID) are read *after* this
filtering, so any empty cell mis-assigns them.

---

## 5. Structural issues worth knowing about

### 5.1 The `n,<rows>` check is off by one — quantified

Known from 2026-08-12; here is the exact arithmetic.

The firmware counts `'|'` separators and declares `row_count + 1`. The GUI splits the payload on
`'|'` and counts **the header line and the `,??` sentinel as rows too**. So:

```
declared_rows = R + 1
actual_rows   = R + 2        (header + R data rows + ",??")
```

`actual_rows` is normally **`declared_rows + 1`**, and the test is `actual_rows < declared_rows`.
**A single lost row does not trip it — you need two.** This is directly visible in the logs:

```
"1 of 42 rows lost in transit" … "Received controller matrix with 18 entries"
```

18 of 20 entries means **2** controllers were lost, while the check reported 1. Every "N rows lost"
message in the logs understates the damage by exactly one row.

### 5.2 `reset_for_new_ble_session()` can wipe an in-flight handshake (latent, not currently firing)

`MainWindow._on_dev_connected` calls `rt_bridge.reset_for_new_ble_session()`, which sets
`_collecting_handshake_payload = False` and clears `_handshake_payload_buf`. If that runs while the
payload is still streaming, the buffer is discarded and every remaining chunk falls through to the
stream parser and is dropped — **total loss of the controller list**, not partial.

MainWindow's own comment acknowledges the ordering hazard ("handshake can arrive before this slot
runs"). Measured: **94/94 connections had the matrix fully parsed before `_on_dev_connected` ran**,
so the race is currently always won — but *only because* the Nano blocks for 3.4 s and stalls the
GUI's connect sequence (§3.3). **If the handshake is ever made faster, or if
`start_notify(ERROR_CHAR)` fails** (it logs "Error characteristic not found" and continues —
0 occurrences so far), **the ordering inverts and this becomes a live bug.** Any fix to §3.3 must
fix this at the same time.

### 5.3 Buffer size mismatch between the two boards (latent)

| | value |
|---|---|
| Teensy `MAX_MESSAGE_SIZE` (`ListCtrlParams.h`) | `(10+1)·30·188 + 188 + 1` = **62 229** |
| Nano `MAX_MESSAGE_SIZE` (`GetBulkChar.h`) | **25 000** (hard-coded) |

Today's ankle-bilateral payload is ~3.2 KB, so there is plenty of headroom. But the Teensy will
happily build a payload up to 62 KB (`MAX_SNAPSHOTS` = 188 rows), and the Nano's receiver would
overflow at 25 KB → `currentState = WAITING_FOR_F; rxIndex = 0` → **the whole payload is
discarded** and the GUI gets nothing. Enabling more joints bilaterally is what would trigger this.
The two constants should be derived from one place.

### 5.4 A second, distinct failure mode: the payload never arrives at all

`GetBulkChar.cpp` frames on `'f'` `','` … `",??"` with no checksum and no retry. On timeout it does
`rxBuffer_bulkStr[0] = '\0'`. `ExoBLE::send_handshake_payload` then sends **`READY` and nothing
else** — because the empty-payload check happens *after* `READY` has already gone out. The GUI sets
`_collecting_handshake_payload = True` and waits forever for a newline that never comes.

**This has happened.** On 2026-08-12 at 17:01:49 the log shows `_on_dev_connected` with **no**
"Handshake payload received" before it and `has_matrix=False` after it, and the resulting
`trial_20260812_170154.csv` is headed `data0 … data11` — the GUI's placeholder names when no channel
titles ever arrived.

**It is not the §5.2 race.** If the GUI had wiped an in-flight payload, the Nano would still have
blocked ~3.4 s sending it, which delays `connected.emit` — and the reset only runs inside
`_on_dev_connected`, i.e. *after* that delay, by which point the matrix is already parsed. The only
shape that fits is: **the Nano had nothing to send.** Consistent with that, `_on_dev_connected` at
17:01:49 shows no stall at all.

So this is upstream of everything in §3 — a UART-hop (or Teensy-side) failure, not chunk loss. It
should be counted separately, which is what the probe's analyzer now does.

Note also that the July doc's elimination of the UART is weaker than it reads: the Nano computes the
`n,<rows>` header **after** the UART hop, so **a row lost on the UART is invisible to that check** —
the Nano would simply declare a lower number and the GUI would agree with it.

---

## 6. Recommended fixes, ranked

Nothing below has been implemented. Roughly best-value-first.

| # | Fix | Class | Effort / risk |
|---|---|---|---|
| 1 | **Add a re-request command.** One BLE command that clears `_handshake_sent_this_connection` and re-sends the payload. Today the *only* recovery from a 20%-likely event is a full disconnect/reconnect cycle. This alone converts the failure from "ruined session" to "click a button". | A | Low risk, high value |
| 2 | **Send a byte count and a checksum in the header**, not just a row count. A deficit that is an exact multiple of 19 confirms whole-notification loss; a checksum catches the within-row damage (`constrection`) that all four current checks miss. | A | Low |
| 3 | **Fix the off-by-one in check 0** (§5.1) so a *single* lost row is caught. Either stop counting the header and `,??` as rows GUI-side, or have the firmware declare the same thing the GUI counts. | A | Trivial |
| 4 | **Stop sending the payload from inside the `BLESubscribed` callback.** Set a pending flag and send from the main loop instead (the `handle_updates()` path already has the plumbing). This removes the 3.4 s stack block, and with it the re-entrant `poll()` that is the leading suspect for the dropped notifications. **Must be done together with §5.2**, because it changes the connect ordering the GUI currently relies on. | A | Medium — Nano flash, needs bench validation |
| 5 | **Validate `config.ini` names instead of defaulting to 0.** Use `.count()` / `.at()`, and print a loud error naming the unrecognised value and the valid keys. Also fix the `splin` typo in `config.ini`'s comment and consider case-insensitive matching. | B | Low, and it removes the scariest silent failure |
| 6 | **Reject rows whose controller name is `UNKNOWN_C`, or whose controller ID is non-numeric**, instead of showing them. Also refuse to apply a parameter when `row[3]` will not parse, rather than silently falling back to the dropdown index (§3.2). | A+B | Low |
| 7 | **Warn on duplicate controller names within a joint** in `_emit_matrix_completeness`. Catches B2 and B3, which the set-based asymmetry check structurally cannot see. | B | Trivial |
| 8 | **Halve the payload:** send the ~20 `v` value rows lazily, on demand. Fewer chunks = proportionally fewer chances to lose one, and it shortens the 3.4 s window. | A | Medium |
| 9 | **Derive the Nano's `MAX_MESSAGE_SIZE` from the Teensy's** (or at minimum assert they match) (§5.3). | — | Low |
| 10 | **Add quote handling to both CSV parsers**, or rename `zhangCollins.csv`'s `"Direciton (0 = PF, >0 = DF)"` field to remove the comma (§B4). Renaming is the cheap correct fix. | B | Trivial |

**The operating rule from July still applies and is still the best zero-code mitigation:**
**connect with the exo next to the laptop, then walk away.** Only the handshake is fragile.
If the warning appears, disconnect and reconnect.

---

## 7. Test plan for the §3.3.1 hypothesis

**Safety note: none of this runs a trial or moves a motor.** Every tier is connect / observe /
disconnect. Tier 0 and Tier 1 are **GUI-only — no Nano flash, no Teensy flash.**

### Falsifiable predictions

If the credit race is the cause:

| | Prediction |
|---|---|
| **P1** | Every hole is a contiguous run of exactly `N × 19` bytes, aligned to a chunk boundary. |
| **P2** | Losses occur only on connections where `_pendingPkt` reached `_maxPkt` at least once. |
| **P3** | The lost chunk index coincides with a chunk during which an inbound ATT packet was dispatched. |
| **P4** | **Moving `start_notify(ERROR_CHAR)` so it does not land mid-payload eliminates the loss** — with total RF traffic unchanged. |

P4 is the cheap one and it is nearly decisive, so do it first.

---

### Tier 0 — Prospective chunk logging (GUI only, ~30 min, no flash)

Right now every conclusion about hole size is *reconstructed backwards* from mangled names. Log the
raw stream instead. In `RtBridge.feed_bytes`, while `_collecting_handshake_payload`, append to a
per-connection file: `(timestamp, running_byte_offset, len(data), repr(chunk))`. Also log a
timestamp when `start_notify(ERROR_CHAR)` returns in `QtExoDeviceManager`.

Diff the reassembled payload against the payload predicted from the SD card CSVs (the generator
used in §3.1 works — it reproduced the real rows byte-for-byte).

**This answers three things at once:**

- Is the hole size always a multiple of 19, **and is it chunk-aligned**? (P1.) If a hole is
  *not* aligned to a 19-byte boundary, the credit-race hypothesis is dead on the spot and the
  problem is somewhere else entirely.
- Where in the stream does it land — and is it near the ERROR-subscribe timestamp? (P3.)
- Are chunk arrival intervals uniform (~20 ms) or is there a stall at the loss point? A stall is the
  signature of the controller buffer filling.

Note the arrival timestamps are host-side and coarse, so treat the timing as corroborating, not
proof.

### Tier 1 — The A/B/C experiment (GUI only, ~45 min, no flash) ← **highest value**

Three variants of the connect sequence in `QtExoDeviceManager`:

| Arm | Sequence | Inbound request mid-payload? | RF traffic |
|---|---|---|---|
| **A** (control) | `start_notify(UART_RX)` → `start_notify(ERROR_CHAR)` | **yes** | baseline |
| **B** | `start_notify(ERROR_CHAR)` → `start_notify(UART_RX)` | no | **identical** |
| **C** | `start_notify(UART_RX)` → wait for matrix → `start_notify(ERROR_CHAR)` | no | identical |

**Arm B is the important control.** It moves the same GATT write earlier rather than removing it,
so total on-air traffic is unchanged — which separates "an inbound request landed mid-payload"
(the hypothesis) from "there was simply less radio traffic" (a confound). If we only tested C, a
positive result would be ambiguous.

**Protocol:** fix the exo at one position that reliably reproduces the fault (see Tier 3), then
**interleave the arms — A, B, C, A, B, C, …** rather than running 20 of each in a block. Battery
voltage, laptop position and 2.4 GHz neighbours all drift over an hour, and blocking the arms would
confound that drift with the treatment. Do **20 connections per arm** (~60 total, a connection is
~10 s). At the observed p ≈ 0.2, seeing zero failures in 20 has probability 0.8²⁰ ≈ 1.2% if the arm
were really unchanged, so a clean sweep in B and C is a strong result.

Score each connection straight from the existing log line — `Controller list looks complete (20
entries)` vs the warning — but **also compare full payloads via Tier 0 logging**, because §3.2
showed the current checks can miss a within-row loss entirely. Scoring on the warning alone would
undercount failures in every arm.

**Read-out:**
- A fails ~20%, B and C ~0% → hypothesis strongly supported, and the fix is a GUI reorder plus the
  firmware guard in Tier 2. **This is also, by itself, a shippable low-risk mitigation.**
- All three fail at similar rates → hypothesis is wrong. The inbound request is not the trigger;
  go to Tier 2 to find out whether the Nano even emitted the missing chunk.
- All three *pass* → the fault did not reproduce at that distance; fix the geometry (Tier 3) and
  repeat. Do not conclude anything from this.

### Tier 2 — Firmware instrumentation (needs Nano flash + a vendored-library edit)

Only if Tier 1 is inconclusive, or to confirm the mechanism before changing firmware.

1. Add read-only accessors to `HCIClass` (`Libraries/ArduinoBLE` is vendored in this repo, so this
   is a local edit): `uint8_t pendingPkt() const`, `uint8_t maxPkt() const`, and a counter
   `_nestedSendCount` incremented on re-entry into `sendAclPkt`.
2. In `ExoBLE::send_chunked`, record per chunk into a **RAM array** (not `Serial.print` — printing
   170 lines inside the send loop changes the timing you are trying to measure): chunk index,
   `_pendingPkt` before the call, whether the `while` loop spun, and whether `_nestedSendCount`
   increased during that chunk.
3. Dump the array over USB serial **after** the payload completes.

**Read-out:** cross-reference the missing chunk index from Tier 0 against the array.
- Missing chunk sits on an entry where the loop spun **and** a nested send occurred → mechanism
  confirmed; apply the guard.
- All ~170 chunks logged as sent with `_pendingPkt` never reaching `_maxPkt` → the Nano did its job
  and the loss is **host-side (Windows/Bleak)**. Fixes #1 and #2 still apply; fix #4 does not.

The candidate firmware fix is one line — re-check the credit after `poll()` returns instead of
trusting the loop exit:

```cpp
while (_pendingPkt >= _maxPkt) { poll(); }   // before
for (;;) { if (_pendingPkt < _maxPkt) break; poll(); }   // ...then re-check right before _pendingPkt++
```

i.e. make the guard immediately precede `_pendingPkt++` so a nested send cannot slip in between.

### Tier 3 — Getting a marginal link on the bench

The nuisance: the fault needs distance, but Tier 2 wants a USB serial tether. Options, best first:

1. **Do Tier 0 and Tier 1 untethered.** They are GUI-side only, so distance is free. This is why
   they come first.
2. **Attenuate instead of moving.** Wrap the Nano end in foil, put it in a metal enclosure, or place
   a body/laptop lid in the line of sight. Reproduces a marginal link at 1 m with USB attached — the
   cleanest way to run Tier 2.
3. **Active USB extension** (5 m) if attenuation proves too coarse to control.

Whichever is used, **record the geometry and keep it fixed for the whole session** — it is the
dominant nuisance variable, and a shifted exo between arms invalidates the comparison.

### What would make me abandon the hypothesis

- A hole that is **not** a multiple of 19, or not chunk-aligned (Tier 0).
- Arms B and C failing at the same rate as A (Tier 1).
- The Nano logging all chunks sent with credits never saturating (Tier 2).

Any of these points at the host stack instead, and the answer becomes fix #1 (re-request) plus
fix #2 (checksum) — which are worth doing regardless of how this resolves.

---

## 7.5 What is implemented right now (2026-08-19)

Tier 0 and Tier 1 are **built and self-tested**. GUI-side only — **no firmware was touched, and
nothing here sends a new command to the exo or moves a motor.** The branch is disposable, so this
instrumentation is deliberately left in place rather than made reversible; switching back to the old
branch removes it.

| File | Change |
|---|---|
| `Python_GUI/services/HandshakeProbe.py` | **new** — chunk logger + arm selector, self-contained |
| `Python_GUI/services/RtBridge.py` | logs each handshake notification, the reassembled payload, and the verdict; sets the arm-C gate |
| `Python_GUI/services/QtExoDeviceManager.py` | A/B/C subscribe ordering; closes the log on disconnect |
| `Python_GUI/tests/analyze_handshake_probe.py` | **new** — scores the experiment |

**Running it:** just use the GUI normally — connect, look at Update Controller, disconnect, repeat.
Arms rotate **A, B, C, A, B, C…** automatically per connection attempt, so interleaving happens
without the operator tracking it. One log per connection lands in
`Python_GUI/Saved_Data/logs/handshake_probe/`. Aim for **20 connections per arm (~60 total)**, all
at the same exo position — see Tier 3.

Env overrides: `EXO_HANDSHAKE_ARM=A|B|C` pins one arm; `EXO_HANDSHAKE_PROBE=0` disables the probe
entirely and restores stock behaviour.

**Scoring:**

```
cd Python_GUI
python tests/analyze_handshake_probe.py
```

It takes the **modal payload** across all logged connections as the reference — self-calibrating, so
it needs no model of the SD card — then decomposes every other payload as a contiguous deletion and
reports hole offset, hole size, `size % 19`, and chunk alignment relative to the start of the body
(the `n,<rows>|` header is sent as its own notification, so alignment is measured after it). Finally
it summarises damage rate per arm.

**Verified end-to-end against synthetic damage** (clean, aligned 19-byte drop, aligned 38-byte drop,
and a deliberately *un*aligned 19-byte drop). All were classified correctly. Note what the aligned
single-chunk case produced:

```
>> ... arm=A  DAMAGED off=404 size=19 multiple_of_19=yes chunk_aligned=yes
      GUI verdict: CLEAN entries=20
```

The payload diff caught damage that the GUI's own four completeness checks called clean — which is
§3.2's blind spot, and precisely why the experiment must be scored on payload diffs rather than on
the warning banner.

**Caveat on arm balance:** the arm advances per *connection attempt*, so a failed attempt that gets
far enough to subscribe will consume an arm. The analyzer prints `n` per arm, so any imbalance is
visible; ignore it unless the arms end up badly uneven.

---

## 7.6 RESULTS — first experimental run, 2026-08-20

15 connections (arms A/B/C rotating), real hardware, trials run between connections.

### P1 — the 19-byte quantum: **CONFIRMED**

```
121 holes across all connections
119 (98%) an exact multiple of 19 bytes
120 (99%) on a chunk boundary
```

The two exceptions are the lost *tails* of truncated captures, not mid-stream holes. This is now
measured prospectively rather than reconstructed, across ~120 independent events. **Whole BLE
notifications go missing; nothing else does.** Every chunk that arrives is exactly 19 bytes
(plus the 5-byte header and 14-byte final chunk) — no coalescing, no corruption, no partial chunks.

### P3 / P4 — the CCCD write as trigger: **REFUTED**

Two independent lines kill it:

1. **The inbound request is never dispatched mid-payload.** `ERROR_SUBSCRIBE_DONE` (when the Write
   Response reaches the host) lands at 3874–4228 ms, always *after* the payload span of
   3749–4006 ms. So on every connection — including the damaged ones — the Nano only serviced that
   CCCD write after finishing the payload. The race in §3.3.1 needs it dispatched *during*.
2. **Arm C, which defers the subscribe entirely, still loses chunks** (1, 18, 9, 15 holes). And a
   single connection had **33 holes** — there is only ever *one* inbound CCCD write, so it cannot
   account for 33 dropped notifications.

The hypothesis was wrong. The quantum it was built to explain survives; the trigger does not.

### The arm comparison itself: **VOID for this run** (saturated and confounded)

| arm | n | damaged | truncated | fail rate |
|---|---|---|---|---|
| A | 6 | 4 | 0 | 67% |
| B | 5 | 2 | 1 | 60% |
| C | 4 | 3 | 1 | 100% |

Two problems, both methodological:

- **Saturation.** This ran at ~60–100% failure with up to 33 holes and 868 bytes lost per
  connection, against a historical ~20% single-hole regime. Well past where an arm effect could
  show.
- **A monotonic session trend swamps everything.** In connection order the hole counts are
  `0, 0, 0, 1, 1, 11, 18, 5, 33, 9, 11, 16, 15` — the first three connections are perfectly clean
  and then every arm degrades together over ~14 minutes. That trend, not the treatment, explains
  the table.

A re-run needs a **milder position** (target ~20–30% failure, mostly single-hole) and a stable link
across the session.

### Weak stall correlation

Holes preceded by a >60 ms inter-chunk gap: 19%, against a 9% baseline — about 2× enrichment, so
congestion matters somewhat, but most holes are *not* at a stall (median gap before a hole 22 ms vs
21 ms baseline). Not a mechanism on its own.

### NEW — losing the tail discards the entire payload

Two connections received **136 and 154 chunks** — nearly the whole list — but the terminating
newline never arrived. `RtBridge` only parses when `"\n" in self._handshake_payload_buf`, so the
buffer was never parsed and **100% of a 96%-complete payload was silently thrown away.** The GUI
shows no controller list at all.

This is distinct from §5.4 (where the Nano genuinely has nothing to send) and it is trivially
fixable GUI-side: parse on a short idle timeout after chunks stop arriving, instead of requiring a
byte that may never come.

### NEW — damaged handshakes silently MISLABEL trial data ⚠️ **worst practical consequence found**

The channel titles ride the same payload. `MainWindow._csv_channel_indices` builds CSV columns as
positions in `_param_names`, but the real-time `values` array is indexed by the **firmware's** fixed
channel numbering. Lose a chunk from the `t` row and the two indexings silently diverge.

From this run, `trial_20260820_155929.csv`:

```
epoch,mark,Desired Torque (L),Measured Torque (L),Desired Torque (R),Measured Torque (R),
Toe FSR (L),In Stance (L),rque (L),Commanded Torque (R),Status,Exoskeleton time (seconds)
                          ^^^^^^^^ "Toe FSR (R),In Stance (R),Commanded To" deleted
```

Header and data both have 12 columns, so **nothing looks wrong**. But every column from index 6 on
carries the wrong channel. Verified against the data:

| file | last column, labelled `Exoskeleton time (seconds)` |
|---|---|
| `trial_20260820_155800.csv` (clean) | `41.79, 41.81, 41.81, 41.83` — monotonic time ✔ |
| `trial_20260820_155929.csv` (damaged) | `0.00, -4.96, -4.78, -5.21` — **torque** |
| `trial_20260820_155855.csv` (damaged) | `0.00, 0.00, 0.00, 0.00` — **In Stance** |

`trial_20260820_155855.csv` lost four channel names, so it has 10 columns instead of 14: Commanded
Torque L/R, Status and the exo clock are **absent**, while their labels sit on FSR data.

**Three trial CSVs from 2026-08-20 are affected** (`155703` has the `Econds)` corruption, `155855`
and `155929` are mislabeled). Any analysis of those files is wrong, and nothing in the GUI or the
file says so. This alone justifies a validity check on `_param_names` before a trial is allowed to
start.

### Where this leaves the root cause

Confirmed: loss is **notification-granular**, contiguous, uncorrupted, and scales with link quality.
Refuted: the CCCD-write trigger. Still open: whether the drop happens in the nRF52 controller, in
ArduinoBLE below `writeValue`, or in the Windows/Bleak host stack. That is exactly what **Tier 2**
was designed to separate, and it is now the next step — but the robustness fixes above are worth
doing regardless of how it resolves.

---

## 8. What was ruled out this session

- **The GUI's parser corrupting rows.** Zero `"Failed to decode received data"` lines in 96 logs;
  arriving bytes are always clean ASCII.
- **The UART hop as the source of the *current* damage.** The loss quantum is an exact multiple of
  19 = the BLE chunk size, which the UART knows nothing about. (But see §5.4 — it is untested, not
  proven safe.)
- **Phantom joints.** All `Ankle(L) (Dorsi Sca)`-style entries in the logs are dated **2026-07-23**,
  i.e. before the numeric-joint-ID guard shipped in `53079b5`. **The guard works — no phantom has
  recurred since.**
- **Starting a trial / changing a controller as a cause.** Structurally impossible: the list is
  fetched once per connection and `_on_update_controller` only re-displays the cached copy.
- **`writeValue()` return-value retry.** Cannot work; see §3.4.
