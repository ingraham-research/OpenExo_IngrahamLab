# Controllers Missing from the GUI Dropdown — BLE Handshake Row Loss

**Date:** 2026-07-23
**Scope:** `Python_GUI/services/RtBridge.py`, `Python_GUI/MainWindow.py` (GUI), `ExoCode/src/ExoBLE.cpp` (Nano).
**Status:** **Root cause identified (RF link quality) and detection shipped. The transport is deliberately
NOT "fixed" in software** — see *Why we stopped here*. GUI warning code committed as `ac1c88e`;
~~the row-count header, phantom rejection, and exact row check are working-tree changes on top of
it~~ — **updated 2026-08-12: those were committed as `53079b5`.**

> **Known gap in the exact row check (found 2026-08-12, not yet fixed).** The firmware's `n,<rows>`
> header counts `row_count + 1`, but the GUI parses the header line itself as an extra row, so
> `actual_rows` is normally `declared_rows + 1`. The `actual_rows < declared_rows` test therefore
> only trips once **two or more** rows are lost — a single lost row slips through it, and is caught
> only by the weaker malformed-row and left/right-asymmetry heuristics. See
> `Spline-Run-Analysis-And-RT-Stream-Fix.md` §1.1, precondition 1.
**Read alongside:** `Modification log with claude/Remote-Control-UDP.md` (the same `'f'` /
`update_controller_param` path) and `Spline-Jitter-Round-2-SD-Logging-Regression.md` (the session this
was found during).

---

## Symptom

Controllers went missing from the GUI's Update-Controller dropdown. Initially reported as *"after
setting PJMC, the spline controller disappears"* — and later *"even PJMC sometimes drops"*, plus a
phantom joint appearing: **`Ankle(L) (Dorsi Sca)`**.

## The reframe that unlocked it

**Setting a controller has nothing to do with it.** The controller list is fetched **once per BLE
connection**, during the handshake; `_on_update_controller` only re-displays the cached copy. So a
controller missing from the dropdown **was missing from the moment you connected** — you just notice it
when you open that page, which happens to be right after changing a controller.

Evidence from `python_gui/saved_data/logs/` across 63 logged connections (full list = 10 ankle CSVs x 2
legs = **20 entries**):

| Matrix entries | Connections |
|---|---|
| 20 (complete) | 56 |
| 19 | 4 |
| 18 | 2 |
| 15 | 1 |

Same firmware, same SD card, different result per connect.

## Root cause — RF link quality at connection time

**Found by direct experiment (user, not analysis): the farther the exo is from the laptop *when the
handshake runs*, the more corruption. Start the connection next to the laptop and the list is intact
every time.**

The handshake payload is ~3-4 KB pushed as roughly **180 unacknowledged 19-byte notifications over
~3.7 s**, sent **once, with no retransmission and no integrity check**. On a marginal link that is ~180
independent chances to lose a packet. That single design choice — a large one-shot unverified transfer —
is what turns ordinary RF marginality into a silently wrong UI.

### Signature of the damage

A lost chunk cuts **mid-row**, so surviving fields shift left:

- Truncated fragments become fake controller names: **`zha`** (from `zhangColl`), **`Timing Th`**.
- Parameter names land in the joint-ID slot, inventing phantom joints: **`Ankle(L) (Dorsi Sca)`** —
  that is `Dorsi Scaling [Nm]` from `pjmc_plus.csv`, cut to 10 chars by `ListCtrlParams`
  `MAX_STRING_LENGTH = 10`.

So losses land **anywhere** in the stream, not just the tail (PJMC is 2nd in the list and has dropped).

## What was proven, and what was ruled out

- **Loss is Nano -> GUI, not Teensy -> Nano.** Proven by the row-count header: the Nano's count is
  computed *after* the UART hop. Nano said 42 rows, GUI received 41.
- **Not characteristic truncation.** `GattDb BUFFER_SIZE = 255`, chunks are 19.
- **Not an HCI drop.** `HCI::sendAclPkt` *blocks* on `while (_pendingPkt >= _maxPkt) { poll(); }` —
  it applies backpressure rather than discarding.
- **Not caused by changing controllers**, per the reframe above.

## Dead ends — do not repeat these

### 1. Retrying on `writeValue()`'s return value. Implemented, flashed, did nothing.

The assumption was that a failed notification returns 0 and could be retried. It cannot:

```
BLELocalCharacteristic::writeValue()
  -> ATT.handleNotify()
       -> HCI.sendAclPkt(...)          // return value DISCARDED
       -> return (numNotifications > 0) ? length : 0;
```

`writeValue()` reports success whenever **any peer is connected**, regardless of whether the
notification landed. The retry loop was dead code. *Lesson: read the library before proposing a fix
built on its return contract.*

### 2. `BLE.poll()` inside the chunk-send loop. Actively harmful; reverted.

It lets other BLE events dispatch **mid-payload**, which can interleave bytes into the stream. It was
added alongside the retry and removed with it. The transport loop is now byte-for-byte equivalent to
the original.

Both dead ends are recorded as comments in `ExoBLE.cpp` so they are not re-attempted.

## What shipped: detection, not correction

**`RtBridge.py`** — `_emit_matrix_completeness()` runs four checks and emits a human-readable reason:

0. **Exact:** firmware's `n,<rows>` header vs rows actually parsed -> `"1 of 42 rows lost in transit"`.
   The only check that catches a *symmetric* loss on a *first* connection.
1. **Malformed rows** — direct evidence of a mangled payload.
2. **Left/right asymmetry** — a bilateral setup must expose the same controllers on both sides; names
   which controller is missing from which joint. Catches in-row *corruption* (a mangled name), which
   the row count cannot see.
3. **Fewer entries than earlier this run** — catches symmetric losses after one good connection.

**Phantom rejection:** a row is only accepted if `row[1]` is a **numeric joint ID**. Corrupted rows are
counted as damage instead of becoming fake joints.

**`MainWindow.py`** — logs a warning, shows it on the settings page, and **re-shows it when Update
Controller opens** (the connect-time banner auto-clears after 8 s, and that page is where a missing
controller is noticed).

**`ExoBLE.cpp`** — sends the `n,<rows>|` header; flags buffer overrun as `TRUNCATED(buffer full)`
instead of a bare `break`; prints `sent=OK`/`sent=FAILED`. **No transport behaviour changed.**

### Verification (no hardware)

Replayed real firmware output through the real GUI parser using a stubbed ArduinoBLE:

- Sweeping **all 74 single-notification drops**: **0 phantom joints** reached the matrix, **0 cases
  damaged-but-silent**, 62 raised a warning (the other 12 lost a chunk inside a `v` value row, which
  harms no controller — correctly reported clean).
- Compatible both directions: old firmware -> no header -> falls back to checks 1-3; new firmware ->
  old GUI -> the `n,42` row fails the `len >= 3` test and is ignored.

## Why we stopped here

The cause is **radio**, not code. Software mitigations (longer inter-chunk delay, smaller payload,
auto-resend) would reduce exposure but never eliminate it, and each costs firmware risk on the Nano.
Detection plus a manual reconnect gives the same practical outcome for far less risk — the user's call,
and the right one.

## Operating rule (this is the actual fix)

**Connect with the exo next to the laptop, then walk away.** Only the handshake is fragile; once the
list is loaded, distance no longer matters. If the warning appears, **disconnect and reconnect** —
a clean connect logs `Controller list looks complete (20 entries)`.

## If it bugs out — where to look

- **A controller is missing but no warning appeared:** the Nano is running firmware without the
  `n,<rows>` header, so check 0 is inert (`declared_rows` stays `None`) and only checks 1-3 apply. A
  symmetric loss on the first connection of an app run is the one case those three cannot see.
- **Phantom joints reappear in the dropdown:** the numeric-joint-ID guard in `RtBridge` was lost or
  bypassed.
- **Warnings on every connect even up close:** suspect the payload outgrew its buffer — look for
  `TRUNCATED(buffer full)` on the Nano's serial (`SIMPLE_DEBUG` is enabled in `Config.h`).
- **Tempted to raise `kHandshakeChunkSize` above 19:** don't, unless MTU negotiation is confirmed
  first. Default ATT MTU is 23, and an over-MTU write silently truncates **every** notification.

## Files changed

| File | Change | State |
|---|---|---|
| `Python_GUI/services/RtBridge.py` | 4 completeness checks; numeric-joint-ID guard; `n,` header parsing | checks 1-3 in `ac1c88e`; check 0 + guard uncommitted |
| `Python_GUI/MainWindow.py` | warning wiring, log line, re-show on page open | committed `ac1c88e` |
| `ExoCode/src/ExoBLE.cpp` | `n,<rows>` header, truncation flag, `sent=` status, dead-end comments | uncommitted — **Nano flash** |

`ExoBLE.cpp` is guarded to `ARDUINO_ARDUINO_NANO33BLE`, so it is a **Nano** flash, not a Teensy one.
