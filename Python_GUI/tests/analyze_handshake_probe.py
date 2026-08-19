"""Score the Tier 0 / Tier 1 handshake experiment.

TEMPORARY — companion to services/HandshakeProbe.py, for the `fix_nano_GUI_handshaking`
investigation. See `Modification log with claude/Nano-GUI-Handshake-Audit.md` §7.

Usage (from Python_GUI/):

    python tests/analyze_handshake_probe.py
    python tests/analyze_handshake_probe.py --dir Saved_Data/logs/handshake_probe

What it does:

* Reads every per-connection chunk log written by HandshakeProbe.
* Picks the **modal payload** — the byte string seen most often — as the reference "clean" payload.
  This self-calibrates from the data, so no model of the SD card CSVs is needed and no assumption
  about which controllers should be present.
* Diffs every other payload against it. The expected damage is a *contiguous deletion*, so a
  common-prefix / common-suffix decomposition recovers the hole exactly. Anything that does not
  decompose that way is reported as such, which is itself a finding.
* Reports, per damaged connection: hole offset, hole size, whether the size is a multiple of the
  19-byte chunk, and whether the hole is aligned to a chunk boundary measured from the start of
  the payload body (i.e. after the `n,<rows>|` header, which is sent as its own notification).
* Summarises pass/fail per arm.

Scoring uses the payload diff, NOT the GUI's own completeness warning: §3.2 of the audit showed a
within-row loss can leave all four completeness checks silent, so warning-based scoring would
undercount failures in every arm equally and blunt the comparison.
"""

import argparse
import ast
import os
import re
from collections import Counter, defaultdict

CHUNK_SIZE = 19  # ExoBLE.cpp kHandshakeChunkSize
HEADER_RE = re.compile(r"^n,\d+\|")


def parse_log(path):
    """Return a dict describing one connection, or None if it carried no payload."""
    rec = {
        "path": path,
        "arm": "?",
        "payload": None,
        "tail": "",
        "chunks": [],
        "marks": [],
        "verdict": "",
    }
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if line.startswith("# arm="):
                rec["arm"] = line.split("=", 1)[1].strip()
            elif line.startswith("PAYLOAD\t"):
                _, _n, blob = line.split("\t", 2)
                try:
                    rec["payload"] = ast.literal_eval(blob)
                except Exception:
                    pass
            elif line.startswith("DISCARDED_TAIL\t"):
                _, _n, blob = line.split("\t", 2)
                try:
                    rec["tail"] = ast.literal_eval(blob)
                except Exception:
                    pass
            elif line.startswith("CHUNK\t"):
                parts = line.split("\t")
                if len(parts) >= 6:
                    try:
                        rec["chunks"].append(
                            {"t_ms": float(parts[1]), "idx": int(parts[2]),
                             "off": int(parts[3]), "len": int(parts[4])}
                        )
                    except ValueError:
                        pass
            elif line.startswith("MARK\t"):
                parts = line.split("\t")
                if len(parts) >= 4:
                    rec["marks"].append((float(parts[1]), parts[2], parts[3] if len(parts) > 3 else ""))
                    if parts[2] == "VERDICT":
                        rec["verdict"] = parts[3] if len(parts) > 3 else ""
    # A record with no payload is NOT noise - it is the "payload never arrived" failure mode
    # (Nano sends READY, then has nothing to send because the Teensy->Nano UART transfer failed;
    # audit §5.4). Seen in the wild on 2026-08-12 17:01:49, which produced a trial CSV headed
    # data0..data11. Keep it and count it.
    return rec


def diff_contiguous(ref, got):
    """Decompose `got` as `ref` with one contiguous run deleted.

    Returns (offset, size) or None if it is not a simple contiguous deletion.
    """
    if got == ref:
        return (0, 0)
    if len(got) >= len(ref):
        return None
    p = 0
    while p < len(got) and got[p] == ref[p]:
        p += 1
    s = 0
    while s < (len(got) - p) and got[len(got) - 1 - s] == ref[len(ref) - 1 - s]:
        s += 1
    if p + s != len(got):
        return None
    return (p, len(ref) - len(got))


def body_start(payload):
    """Byte offset where the chunked body begins (the `n,<rows>|` header is its own notification)."""
    m = HEADER_RE.match(payload)
    return m.end() if m else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "Saved_Data", "logs", "handshake_probe"))
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        print(f"No probe directory at {args.dir}")
        return

    allrecs = []
    for name in sorted(os.listdir(args.dir)):
        if not name.endswith(".log"):
            continue
        r = parse_log(os.path.join(args.dir, name))
        if r:
            allrecs.append(r)

    recs = [r for r in allrecs if r["payload"]]
    nopayload = [r for r in allrecs if not r["payload"]]

    if not recs:
        print(f"No connections with a payload found in {args.dir}")
        if nopayload:
            print(f"({len(nopayload)} connection(s) received no payload at all.)")
        return

    counts = Counter(r["payload"] for r in recs)
    ref, ref_n = counts.most_common(1)[0]
    print(f"Connections with a payload: {len(recs)}")
    print(f"Reference payload: {len(ref)} bytes, seen {ref_n}/{len(recs)} times, "
          f"{len(set(counts))} distinct payload(s) overall")
    if ref_n <= len(recs) // 2:
        print("  !! WARNING: the modal payload is not a clear majority. Either the exo's controller "
              "set changed mid-session, or damage is more common than the reference assumption. "
              "Check the distinct payloads before trusting the per-arm numbers.")
    print()

    per_arm = defaultdict(lambda: {"n": 0, "bad": 0, "none": 0})
    rows = []
    for r in nopayload:
        per_arm[r["arm"]]["n"] += 1
        per_arm[r["arm"]]["none"] += 1
        saw_ready = any(m[1] == "READY" for m in r["marks"])
        rows.append((os.path.basename(r["path"]), r["arm"],
                     "NO PAYLOAD AT ALL (READY seen: %s, chunks: %d) - Nano had nothing to send, "
                     "or the link died mid-handshake" % ("yes" if saw_ready else "no", len(r["chunks"])),
                     r["verdict"]))
    for r in recs:
        arm = r["arm"]
        per_arm[arm]["n"] += 1
        d = diff_contiguous(ref, r["payload"])
        if d == (0, 0):
            status = "clean"
        else:
            per_arm[arm]["bad"] += 1
            if d is None:
                status = "DAMAGED (not a simple contiguous deletion)"
            else:
                off, size = d
                bs = body_start(ref)
                rel = off - bs
                mult = "yes" if size % CHUNK_SIZE == 0 else f"NO ({size} bytes)"
                if rel >= 0:
                    aligned = "yes" if rel % CHUNK_SIZE == 0 else f"NO (rel={rel}, rel%19={rel % CHUNK_SIZE})"
                else:
                    aligned = "n/a (hole is inside the header)"
                status = (f"DAMAGED off={off} size={size} "
                          f"multiple_of_19={mult} chunk_aligned={aligned}")
        if r["tail"]:
            status += f" | DISCARDED TAIL {len(r['tail'])} bytes"
        rows.append((os.path.basename(r["path"]), arm, status, r["verdict"]))

    for name, arm, status, verdict in rows:
        flag = "   " if status == "clean" else ">> "
        print(f"{flag}{name}  arm={arm}  {status}")
        if status != "clean" and verdict:
            print(f"      GUI verdict: {verdict}")

    print()
    print("Per-arm summary (scored on payload diff, not the GUI warning):")
    print(f"  {'arm':<5}{'n':>5}{'damaged':>9}{'no-payld':>10}{'bad rate':>10}")
    for arm in sorted(per_arm):
        st = per_arm[arm]
        rate = ((st["bad"] + st["none"]) / st["n"] * 100.0) if st["n"] else 0.0
        print(f"  {arm:<5}{st['n']:>5}{st['bad']:>9}{st['none']:>10}{rate:>9.1f}%")
    print()
    print("'no-payld' is a separate failure mode (audit sec 5.4): the payload never arrived at")
    print("all, so the GUI shows no controller list and names CSV columns data0..dataN. It is")
    print("upstream of chunk loss - do not mix it into the chunk-loss comparison.")
    print()
    print("Reading it: arm A is the control. If A shows damage and B and C do not, the inbound")
    print("CCCD write landing mid-payload is implicated (audit sec 3.3.1). If a hole is ever NOT")
    print("a multiple of 19 or NOT chunk-aligned, the credit-race hypothesis is dead.")


if __name__ == "__main__":
    main()
