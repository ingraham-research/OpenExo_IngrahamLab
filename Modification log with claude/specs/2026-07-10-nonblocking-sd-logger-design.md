# Non-Blocking SD Logger — Design

**Date:** 2026-07-10
**Branch:** `fix_zerotorque_pid`
**Status:** Approved design, pending implementation
**Related:** [[2026-07-10-zerotorque-transparency-design]] (the logger stall is the dominant cause of
the ankle transparency oscillation)

## Problem

The current `SdLogger` (`ExoCode/src/SdLogger.cpp`) uses the stock Arduino `SD` library with
blocking writes. Measured from bench logs (sessions 0008/0009/0002), each log-write cycle stalls the
single-threaded control loop ~20 ms, dropping the 500 Hz control loop to ~156-177 Hz with heavy
jitter (2.4-14 ms/cycle). That loss of loop bandwidth collapses the torque-null feedback loop's phase
margin and is the dominant driver of the ankle "zero torque" oscillation (`corr(tau,cmd) = -0.94..-1.00`,
a pure feedback limit cycle). Disabling logging made the chatter "a lot better"; raising the flush
interval 300 ms -> 5000 ms did NOT help (loop 156 -> 177 Hz), proving the stall is the **per-write
path**, not the flush sync.

The blocking is inherent to the stock `SD` API: `File::write()` waits synchronously for the card,
whose internal write/erase latency is high and variable (occasional tens of ms). DMA (which the
Teensy SDIO path already uses) offloads the byte transfer but does not make the call non-blocking.

## Goal

Make SD logging never block the control loop, so the loop holds ~500 Hz with logging enabled, while
preserving the existing 3-file text log format and the crash/reset integrity guarantees.

Non-goal (this spec): perfectly deterministic 500 Hz timing. Near-500 Hz with rare sub-millisecond
sector writes is sufficient for torque-loop stability; bit-perfect timing (control-in-ISR) is out of
scope.

## Approach (chosen: A)

**RAM ring buffer + `isBusy()`-gated single-sector draining, staying in the superloop.** This is the
SdFat `ExFatLogger`/`LowLatencyLogger` pattern. Rejected alternatives: (B) periodic bulk write — still
blocks on card latency spikes, only lowers stall frequency; (C) control loop in an `IntervalTimer`
ISR — strongest guarantee but a large, risky restructure (CAN/UART/control in interrupt context),
disproportionate to the need. C remains a documented future option if A proves insufficient.

## Design

### Interface (unchanged)

`SdLogger`'s public API is preserved exactly: constructor, `update(bool ran)`, `close_active()`,
`self_test()`. `ExoCode.ino:690-693` (`bool ran = exo.run(); sd_logger.update(ran);`) does not
change. All changes are internal to `SdLogger`.

The logger uses the SdFat object underlying the stock `SD` library (`SD.sdfs`) for the low-level
`FsFile` / `preAllocate` / `card()->isBusy()` calls, so it shares the single already-mounted
filesystem instance with the existing config reads (`IniFile`, `ParamsFromSD`) — no second mount.

### Components

- **Producer** (`update`): on active trial and decimation hit, `snprintf` the motor rows (identical
  format string to today) and copy the bytes into that file's RAM ring buffer. Ground-strike events
  are pushed to the GS ring buffer as they occur. Never touches the card. O(row length) memcpy.
- **Drainer** (`_service_writes`, called every loop iteration from `update`, independent of `ran`):
  if `SD.sdfs.card()->isBusy()` is false, write exactly one 512-byte sector from one file's buffer
  (round-robin across the 3 files) to its pre-allocated contiguous `FsFile`. Only writes when the
  card is ready, so it returns fast and never waits on card latency.
- **Three files**: `Motor_L_log.txt`, `Motor_R_log.txt`, `Ground_strike_log.txt` — same filenames,
  header lines, and columns as the current logger. Three ring buffers, three contiguous `FsFile`s.

### Data flow

- **Open (trial-active edge, `_open_session`):** `mkdir` the session folder, create the 3 files as
  `FsFile`, `preAllocate()` an **adaptively sized** amount each, push the header line into each buffer.
  Pre-allocation reserves a contiguous cluster run, eliminating mid-trial FAT-allocation stalls.
  This one-time cost lands at the trial-active edge (trial start, before walking) and is accepted.
  **Adaptive size:** computed from `sdLogDecimation` at open so it always covers ~90 min at the
  actual rate: `bytes_per_s = (500 / decimation) * ROW_BYTES_EST` (ROW_BYTES_EST ~= 72), reserve
  `bytes_per_s * 5400 s`, clamped to `[1 MB, 128 MB]` per motor file. Ground-strike file uses a small
  fixed reservation (event-driven, low volume; e.g. 1 MB). Examples: 100 Hz -> ~37 MB/file,
  500 Hz -> clamped 128 MB/file, 50 Hz -> ~19 MB/file. If a contiguous run that large is unavailable
  (fragmented card) or the trial outruns it, the file falls back to on-demand growth for the
  remainder (allocation stalls return but the ring buffer absorbs them) — graceful degradation, never
  a hard failure.
- **Per control cycle:** `update(ran)` -> `_handle_session` (open/close edges) -> if logging & ran:
  ground-strike check (push to GS buffer), decimation counter, push motor rows to L/R buffers ->
  `_service_writes()`.
- **Between control cycles:** `loop()` spins fast while `Exo::run()` only executes its body every
  ~2 ms, so `_service_writes()` runs many times per control cycle — many drain opportunities whenever
  the card is idle.
- **Close (trial-off edge or `close_active()` on reset):** flush remaining buffered bytes (final
  partial sector), `truncate()` each file to its real byte length (so the pre-allocated tail is
  discarded and the file is a normal-length text file), `sync()`, `close()`.

### Durability & integrity

- Pre-allocated contiguous files avoid mid-trial cluster-allocation stalls.
- A periodic `sync()` (updates the directory file-size) runs on the `sdLogFlushMs` cadence but
  **gated on `!isBusy()`**, keeping it off the control-critical path. It bounds worst-case data loss
  on an abnormal reset to ~one interval without stalling control.
- Normal trial-end does a full flush+truncate+sync+close; `close_active()` still finalizes on the
  reset path (`SdLogger.h:58-61`). Same integrity guarantees as today, now non-blocking.

### Overflow policy

If the card falls so far behind that a ring buffer fills (very unlikely: card sustains MB/s, the
logger produces ~11 KB/s), the producer drops the oldest sector, increments a dropped-byte counter,
and emits a one-line `# GAP <n> bytes` marker into that stream so the gap is visible when parsing.
**Control never stalls** — a full buffer degrades the log, never the loop.

### RAM

Per-file ring buffers in `DMAMEM` (Teensy 4.1 OCRAM, 512 KB total): ~8 KB per motor buffer + ~2 KB
for ground-strike ≈ 18 KB. At the ~5.5 KB/s per-motor-file rate that rides out a ~1.4 s card stall —
far beyond any realistic GC spike. Comfortable alongside existing `DMAMEM` use (e.g. ListCtrlParams
`stringArray` ~49 KB).

### Config

Same `[Logging]` keys in `/config.ini`, read at boot (no reflash to change):
- `sdLogEnabled` — unchanged.
- `sdLogDecimation` — unchanged (rows per Nth 500 Hz cycle).
- `sdLogFlushMs` — repurposed as the durability-sync cadence; no longer causes control stalls.

## Testing

- **Fake-trial path:** reuse `SD_LOG_SELFTEST_TRIAL` (`ExoCode.ino:148`) to drive the new
  producer/drain path with a faked active trial (no motors enabled), then read back the 3 files and
  verify row counts and that headers/format are intact.
- **Loop-rate proof:** enable `USE_SPEED_CHECK` (`Exo.cpp`) and confirm the control loop holds
  ~500 Hz with logging ON (vs ~156-177 Hz before). This is the direct evidence the fix works.
- **Integrity:** power-cut / reset mid-trial and confirm the log is a valid, readable, correctly
  sized text file up to ~the last sync (user-run).
- **Field:** re-run the ankle transparency test with logging ON and confirm the chatter matches the
  logging-OFF case (i.e. logging no longer degrades control).

## Out of scope (this spec)

- Control loop in an ISR (approach C).
- The residual transparency chatter that remains even with logging off — separate control-law work,
  to be diagnosed on clean 500 Hz logs once this lands (see [[zerotorque-transparency]]).
- The AK60v3 CAN velocity/current decode bug (~37-42% garbage) — separate, latent.
- Consolidating or changing the log file format (explicitly keeping the 3-file layout for downstream
  parsing).

## Risks

- **Pre-allocation at trial start** may cost tens of ms once at the trial-active edge; accepted (before
  walking). If it proves disruptive, pre-create/pre-allocate at boot instead.
- **`SD.sdfs` low-level use** must not conflict with the stock `SD` calls elsewhere (config reads
  happen at boot, before trials; logging happens during trials) — verify no concurrent access.
- **`truncate()`/`preAllocate` availability** depends on the Teensy SdFat version bundled with the
  installed Teensyduino; verify the API at implementation time and fall back to write-without-
  preallocate if unavailable (still non-blocking via isBusy gating, just with possible allocation
  stalls that the buffer absorbs).
