"""Score the Tier 0 / Tier 1 handshake experiment.

TEMPORARY — companion to services/HandshakeProbe.py, for the `fix_nano_GUI_handshaking`
investigation. See `Modification log with claude/Nano-GUI-Handshake-Audit.md` sec 7.

Usage (from Python_GUI/):

    python tests/analyze_handshake_probe.py
    python tests/analyze_handshake_probe.py --dir Saved_Data/logs/handshake_probe
    python tests/analyze_handshake_probe.py --holes     # list every hole individually

What it does:

* Reads every per-connection chunk log written by HandshakeProbe.
* Picks the **modal payload** as the reference "clean" payload. Self-calibrating, so no model of
  the SD card CSVs is needed. (Damage is random, so N identical payloads are the undamaged one.)
* Diffs each connection against it with difflib, which finds **all** holes — the first version
  assumed a single contiguous deletion, and real captures routinely have 10-30 separate holes.
* For every hole: size, whether it is a multiple of the 19-byte notification, and whether it is
  aligned to a chunk boundary measured from the start of the payload body (the `n,<rows>|` header
  is sent as its own notification, so alignment is measured after it).
* Classifies three outcomes: clean / damaged / truncated-tail.

Scoring uses the payload diff, NOT the GUI's own completeness warning: a real capture had a
connection lose 133 bytes across 5 holes while the GUI reported "CLEAN entries=20".

**Alignment caveat:** difflib reports a minimal edit, and when the bytes bordering a hole match the
bytes ending it the reported offset can slide by a byte or two. Holes reported as off-by-1 or -2
from a boundary (rel%19 of 17 or 18) are alignment artefacts, not genuine misalignment. Only treat
a hole as unaligned if it is far from a boundary.
"""

import argparse
import ast
import difflib
import os
import re
from collections import Counter, defaultdict

CHUNK = 19  # ExoBLE.cpp kHandshakeChunkSize
HEADER_RE = re.compile(r"^n,\d+\|")


def parse_log(path):
    rec = {"path": path, "arm": "?", "condition": "unlabelled", "payload": None, "tail": "",
           "chunks": [], "marks": [], "verdict": ""}
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if line.startswith("# arm="):
                rec["arm"] = line.split("=", 1)[1].strip()
            elif line.startswith("# condition="):
                rec["condition"] = line.split("=", 1)[1].strip() or "unlabelled"
            elif line.startswith("PAYLOAD\t"):
                try:
                    rec["payload"] = ast.literal_eval(line.split("\t", 2)[2])
                except Exception:
                    pass
            elif line.startswith("DISCARDED_TAIL\t"):
                try:
                    rec["tail"] = ast.literal_eval(line.split("\t", 2)[2])
                except Exception:
                    pass
            elif line.startswith("CHUNK\t"):
                f = line.split("\t")
                if len(f) >= 6:
                    try:
                        rec["chunks"].append((float(f[1]), int(f[3]), int(f[4]),
                                              ast.literal_eval(f[5])))
                    except Exception:
                        pass
            elif line.startswith("MARK\t"):
                f = line.split("\t")
                if len(f) >= 3:
                    detail = f[3] if len(f) > 3 else ""
                    rec["marks"].append((float(f[1]), f[2], detail))
                    if f[2] == "VERDICT":
                        rec["verdict"] = detail
    return rec


def received_text(rec):
    """What actually arrived. Falls back to concatenated chunks when the terminating newline
    was lost, in which case the GUI parser never fired and discarded everything."""
    if rec["payload"] is not None:
        return rec["payload"], False
    return "".join(c[3] for c in rec["chunks"]), True


def holes_of(ref, got):
    sm = difflib.SequenceMatcher(None, ref, got, autojunk=False)
    holes, inserted = [], 0
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "delete":
            holes.append((i1, i2 - i1))
        elif tag == "replace":
            holes.append((i1, i2 - i1))
            inserted += j2 - j1
        elif tag == "insert":
            inserted += j2 - j1
    return holes, inserted


def body_start(payload):
    m = HEADER_RE.match(payload)
    return m.end() if m else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "Saved_Data", "logs", "handshake_probe"))
    ap.add_argument("--holes", action="store_true", help="list every hole individually")
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        print(f"No probe directory at {args.dir}")
        return

    recs = [parse_log(os.path.join(args.dir, n))
            for n in sorted(os.listdir(args.dir)) if n.endswith(".log")]
    recs = [r for r in recs if r["chunks"] or r["payload"]]
    if not recs:
        print(f"No usable connection logs in {args.dir}")
        return

    full = [r["payload"] for r in recs if r["payload"]]
    if not full:
        print("No connection completed a payload; cannot establish a reference.")
        return
    counts = Counter(full)
    ref, ref_n = counts.most_common(1)[0]
    hdr = body_start(ref)

    print(f"Connections: {len(recs)}   reference payload: {len(ref)} bytes "
          f"(seen {ref_n}x, {len(counts)} distinct)   header: {hdr} bytes")
    if ref_n < 2:
        print("  !! Only one connection produced this payload. With no repeat there is no "
              "independent confirmation it is the undamaged one - treat results as provisional.")
    print()

    per_arm = defaultdict(lambda: {"n": 0, "bad": 0, "trunc": 0, "lost": 0})
    per_cond = defaultdict(lambda: {"n": 0, "bad": 0, "trunc": 0, "lost": 0, "holes": 0})
    tot_holes = aligned = mult19 = 0

    for r in recs:
        got, truncated = received_text(r)
        hs, inserted = holes_of(ref, got)
        arm = r["arm"]
        st = per_arm[arm]
        st["n"] += 1
        missing = len(ref) - len(got)
        st["lost"] += max(0, missing)
        cst = per_cond[r["condition"]]
        cst["n"] += 1
        cst["lost"] += max(0, missing)
        cst["holes"] += len(hs)

        if not hs and not inserted and not truncated:
            print(f"    {os.path.basename(r['path'])}  arm={arm}  clean")
            continue
        cst["bad" if not truncated else "trunc"] += 1

        if truncated:
            st["trunc"] += 1
            kind = (f"TRUNCATED - {len(r['chunks'])} chunks arrived but the terminating newline "
                    f"never did, so the GUI parser never fired and discarded ALL of it")
        else:
            st["bad"] += 1
            kind = "DAMAGED"

        for _off, sz in hs:
            tot_holes += 1
            if sz % CHUNK == 0:
                mult19 += 1
        for off, _sz in hs:
            rel = (off - hdr) % CHUNK
            if rel in (0, 1, CHUNK - 1, CHUNK - 2):  # allow difflib's 1-2 byte slide
                aligned += 1

        print(f" >> {os.path.basename(r['path'])}  arm={arm}  {kind}")
        print(f"      {len(hs)} hole(s), {missing} bytes missing"
              + (f", {inserted} inserted/replaced (NOT a pure deletion)" if inserted else ""))
        if r["verdict"]:
            print(f"      GUI verdict: {r['verdict']}")
        if args.holes:
            for off, sz in hs:
                rel = off - hdr
                print(f"        off={off} (rel {rel}) size={sz} "
                      f"{'x19' if sz % CHUNK == 0 else 'NOT-x19'} "
                      f"{'aligned' if rel % CHUNK in (0, 1, CHUNK-1, CHUNK-2) else 'UNALIGNED(%d)' % (rel % CHUNK)}")

    print()
    print("Per-arm summary (scored on payload diff, not the GUI warning):")
    print(f"  {'arm':<5}{'n':>4}{'damaged':>9}{'truncated':>11}{'bytes lost':>12}{'fail rate':>11}")
    for arm in sorted(per_arm):
        st = per_arm[arm]
        rate = ((st["bad"] + st["trunc"]) / st["n"] * 100.0) if st["n"] else 0.0
        print(f"  {arm:<5}{st['n']:>4}{st['bad']:>9}{st['trunc']:>11}{st['lost']:>12}{rate:>10.1f}%")

    if len(per_cond) > 1 or "unlabelled" not in per_cond:
        print()
        print("Per-CONDITION summary (set via CONDITION.txt in the probe dir):")
        print(f"  {'condition':<24}{'n':>4}{'damaged':>9}{'truncated':>11}{'holes':>8}{'bytes lost':>12}{'fail rate':>11}")
        for c in sorted(per_cond):
            st = per_cond[c]
            rate = ((st["bad"] + st["trunc"]) / st["n"] * 100.0) if st["n"] else 0.0
            print(f"  {c[:23]:<24}{st['n']:>4}{st['bad']:>9}{st['trunc']:>11}{st['holes']:>8}{st['lost']:>12}{rate:>10.1f}%")

    if tot_holes:
        print()
        print(f"Hole quantum across ALL connections: {tot_holes} holes, "
              f"{mult19} ({100*mult19//tot_holes}%) an exact multiple of {CHUNK} bytes, "
              f"{aligned} ({100*aligned//tot_holes}%) on a chunk boundary.")
        print("Near-100% on both is the signature of whole BLE notifications going missing.")
        print("Holes that are neither are usually the lost TAIL of a truncated capture, not a")
        print("mid-stream hole - check with --holes before concluding anything from them.")


if __name__ == "__main__":
    main()
