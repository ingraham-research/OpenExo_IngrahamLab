"""BLE link stress test for the OpenExo.

Same shape as main_external_control.py, stripped down to one job: write TorqScale = 0 at a fixed
interval and measure how long the link survives. Nothing else is ever written, so the exo sits at
zero torque for the whole run and this is safe to leave running on a benchtop.

Why this exists: on 2026-09-09 a UDP sender stuck at 200 Hz drove main_external_control into writing
40 parameters/second (20 Hz loop x 2 ankles, the zero-torque path being exempt from the rate limiter),
and the Nano died far faster than its ~107 s spontaneous-freeze mean. That made the failure
REPRODUCIBLE for the first time. This file turns that accident into a dial: set write_interval_s,
run, and record time-to-failure. Sweep the interval and you get a dose-response curve, which is what
separates "the link dies from packet volume" from "the link dies for an unrelated reason".

What it deliberately does NOT do, matching main_external_control: it never starts a trial, never
enables motors, never calibrates. A human drives the GUI. Run the GUI first, connect it to the exo,
start the trial there, then from Python_GUI/external_control:  python stress_test_ble.py
"""

import os
import time

from Utilities.OpenExoLink_utilities import OpenExoLink, OpenExoLinkError
from Utilities.ActionMap_utilities import SPLINEALT_TORQUE_SCALE


def main():

    #################### Config ####################

    #The dial. Seconds between write BURSTS. One burst writes TorqScale to every joint below, so the
    #actual BLE round trips per second is (1 / write_interval_s) * len(joints_to_drive).
    #0.05 with two joints reproduces the 40 writes/s that killed it on 2026-09-09.
    write_interval_s = 0.05

    joints_to_drive = ("Ankle(L)", "Ankle(R)")   #Use a single joint to halve the load without touching the interval
    controller_name = "splineAlt"                #Whatever we write to becomes the ACTIVE controller - see the warning below

    run_duration_s = 900          #Give up and call it a pass after this long
    report_every_s = 10           #How often to print a progress line

    #Connection to the OpenExo GUI. Same defaults as main_external_control
    gui_host = "127.0.0.1"
    gui_port = 9750
    ack_timeout = 5.0             #How long we wait for the exo to acknowledge a write
    max_write_retries = 3         #Resends before a write is declared lost

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Logs", "Stress test")
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(log_dir, f"stress_{stamp}.txt")   #Timestamped, so a run never overwrites the last one

    #################### Connect ####################

    exo_link = OpenExoLink(host=gui_host, port=gui_port, ack_timeout=ack_timeout,
                           max_retries=max_write_retries, verbose=0)   #verbose 0: a write every 50 ms would drown the terminal
    try:
        exo_link.connect()
    except OpenExoLinkError as e:
        print(f"{e}")
        input("Start the GUI, connect it to the exo, then restart this code - hit enter to exit.\n")
        return

    if not exo_link.connected:
        input("Connect the GUI to the exo first, then restart this code - hit enter to exit.\n")
        exo_link.close()
        return

    #Resolve up front so a name mismatch fails here rather than mid-run
    try:
        addresses = [exo_link.resolve(j, controller_name, SPLINEALT_TORQUE_SCALE) for j in joints_to_drive]
    except OpenExoLinkError as e:
        print(f"Could not resolve TorqScale: {e}")
        exo_link.close()
        return

    #SAFETY GUARD. resolve() falls back to PREFIX matching when the exact controller name is not in the
    #handshake matrix (BLE truncates names to 9 characters). "splinealt".startswith("spline") is True,
    #so a truncated name could silently resolve to the 12-node `spline` controller instead - where
    #index 10 is node6_x, NOT TorqScale. Writing 0 there would change the spline shape rather than
    #zero the torque, and this script's whole safety claim rests on index 10 meaning TorqScale.
    #So verify what we actually resolved to, by name, and refuse to run if it is anything else.
    for (joint_id, controller_id, param_index), joint in zip(addresses, joints_to_drive):
        row = next((r for r in exo_link.matrix
                    if len(r) >= 4 and str(r[1]) == str(joint_id) and str(r[3]) == str(controller_id)), None)
        resolved_name = str(row[2]).strip() if row else "<not in matrix>"
        if not resolved_name.lower().startswith("splineal"):
            print(f"REFUSING TO RUN: {joint} resolved to controller '{resolved_name}' "
                  f"(id {controller_id}), not splineAlt.")
            print(f"  Index {param_index} is only guaranteed to be TorqScale on splineAlt. On any other")
            print(f"  controller it is a different parameter and writing 0 to it may NOT mean zero torque.")
            exo_link.close()
            return
        print(f"  {joint}: joint_id={joint_id}, controller='{resolved_name}' (id {controller_id}), "
              f"param_index={param_index} (TorqScale)")

    writes_per_s = len(joints_to_drive) / write_interval_s
    print("\n=== BLE LINK STRESS TEST ===")
    print(f"  {len(joints_to_drive)} joint(s) x every {write_interval_s:.3f} s = {writes_per_s:.0f} writes/s")
    print(f"  Each write is one BLE write out plus one ack notification back, on top of the ~100 Hz")
    print(f"  real-time stream the exo is already sending.")
    print(f"  Only value ever written: TorqScale = 0. The exo stays at ZERO torque throughout.")
    print(f"  Log: {log_path}")

    #The first write SWITCHES the joint to this controller and loads that controller's SD defaults,
    #whose TorqScale is non-zero (95 in splineAlt.csv). Writing TorqScale = 0 first is what keeps the
    #switch harmless - same reasoning as engage_controller_safely(). Do not reorder this.
    print(f"\n=== The next step CHANGES THE ACTIVE CONTROLLER to {controller_name} on {', '.join(joints_to_drive)} ===")
    print("    It goes straight to zero torque scale, and zero is the ONLY value this script writes.")
    print("")
    print("    Trial running or not, your choice, and it changes what you are testing:")
    print("      NO trial  - motors are not enabled, so nothing can command torque at all. But the")
    print("                  real-time stream is gated on trial status, so you test only the")
    print("                  write/ack path. Best for a first smoke test of this harness.")
    print("      Trial ON  - the ~100 Hz real-time stream flows, which is the traffic actually")
    print("                  present during a normal freeze. Motors are enabled, but commanded")
    print("                  torque is 0 and the PID holds measured torque at 0 (transparency) -")
    print("                  the same state engage_controller_safely() leaves the exo in.")
    input("Press enter to begin, or Ctrl-C to abort:\n")

    #################### Run ####################

    log = open(log_path, "w", buffering=1)   #Line-buffered, so a hard failure still leaves the history on disk
    print("python_time,elapsed_s,event,detail", file=log)
    print(f"{time.time()},0.00,config,"
          f"{len(joints_to_drive)} joints @ {write_interval_s}s = {writes_per_s:.0f} writes/s", file=log)

    writes_ok = 0
    writes_failed = 0
    writes_rejected = 0       #Corrupted frames the exo refused. The link is alive - these do NOT end the run
    start_time = time.perf_counter()
    next_write = start_time
    next_report = start_time + report_every_s
    outcome = "completed"

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            if elapsed >= run_duration_s:
                break

            exo_link.pump()                      #Notice a GUI-reported disconnect between writes
            if exo_link.link_down:
                outcome = "LINK DOWN (GUI reported disconnect)"
                print(f"{time.time()},{elapsed:.2f},link_down,after {writes_ok} writes", file=log)
                break

            if now >= next_write:
                next_write += write_interval_s
                if next_write < now:             #We fell behind; resync rather than burst to catch up
                    next_write = now + write_interval_s

                for addr, joint in zip(addresses, joints_to_drive):
                    try:
                        exo_link.set_param_confirmed(addr, 0.0)   #Idempotent: already 0, so nothing on the exo changes
                        writes_ok += 1
                    except OpenExoLinkError as e:
                        #TWO different failures arrive as the same exception type, and only one of them
                        #means the link died. Telling them apart is the whole point of this loop:
                        #
                        #  reason is not None -> the exo ACKed with accepted=false. It received a frame
                        #      and refused it, so the link is ALIVE. In practice this is the corrupted
                        #      inbound frame we see logged as "controller=0 index=0, invalid message":
                        #      the joint field survives the BLE hop and the rest does not. Transient,
                        #      the exo's validation rejects it safely, and the next write goes through.
                        #      COUNT IT AND CARRY ON - treating this as death ends the run early and
                        #      reports a survival time that is really just "time to first bad frame".
                        #
                        #  reason is None -> set_param_confirmed exhausted its retries against SILENCE.
                        #      That is the exo having gone quiet, which is the event being measured.
                        if getattr(e, "reason", None) is not None:
                            writes_rejected += 1
                            print(f"{time.time()},{time.perf_counter() - start_time:.2f},rejected,"
                                  f"{joint}: {e}", file=log)
                            continue

                        #Silence on a write is the FAST detector. The GUI's own disconnect lags the exo
                        #going quiet by ~9.6 s (the BLE supervision timeout), so this fires first.
                        writes_failed += 1
                        elapsed = time.perf_counter() - start_time
                        outcome = f"WRITE FAILED (silence) on {joint}: {e}"
                        print(f"{time.time()},{elapsed:.2f},write_failed,{joint}: {e}", file=log)
                        raise KeyboardInterrupt   #Reuse the clean-shutdown path below

            if now >= next_report:
                next_report += report_every_s
                rate = writes_ok / elapsed if elapsed > 0 else 0
                print(f"  {elapsed:6.1f} s   {writes_ok:6d} ok   {writes_rejected:4d} rejected   "
                      f"{rate:5.1f} writes/s actual")
                print(f"{time.time()},{elapsed:.2f},progress,"
                      f"{writes_ok} ok / {writes_rejected} rejected / {writes_failed} failed", file=log)

            time.sleep(0.002)    #Keep the poll loop off a hard spin without blurring the interval

    except KeyboardInterrupt:
        if outcome == "completed":
            outcome = "stopped by user"

    #################### Report ####################

    elapsed = time.perf_counter() - start_time
    actual_rate = writes_ok / elapsed if elapsed > 0 else 0
    print("\n=== RESULT ===")
    print(f"  Outcome            : {outcome}")
    print(f"  Time to failure    : {elapsed:.1f} s")
    print(f"  Writes acknowledged: {writes_ok}")
    print(f"  Frames rejected    : {writes_rejected}   (corrupted in flight, exo refused them, link stayed up)")
    if writes_ok + writes_rejected > 0:
        print(f"  Corruption rate    : {100.0 * writes_rejected / (writes_ok + writes_rejected):.2f} %")
    print(f"  Writes failed      : {writes_failed}   (silence - this is what ends the run)")
    print(f"  Requested rate     : {writes_per_s:.1f} writes/s")
    print(f"  Achieved rate      : {actual_rate:.1f} writes/s")
    print(f"  Total BLE round trips before failure: {writes_ok}")
    print(f"\n  Log written to {log_path}")

    print(f"{time.time()},{elapsed:.2f},result,{outcome}", file=log)
    print(f"{time.time()},{elapsed:.2f},summary,"
          f"{writes_ok} ok / {writes_rejected} rejected / {writes_failed} failed / "
          f"{actual_rate:.1f} per s", file=log)
    log.close()

    #Park at zero and let go. The controller is already at TorqScale 0, so there is nothing to undo -
    #this is only here so the exo is left in a state the operator expects.
    exo_link.close()
    print("\n  The exo is left on "
          f"{controller_name} at TorqScale 0 (transparency). Stop the trial from the GUI.")


if __name__ == "__main__":
    main()
