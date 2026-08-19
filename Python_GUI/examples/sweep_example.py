"""Example: sweep a controller parameter over the UDP remote control.

Run the GUI first (python GUI.py) and connect to the exo. Then, from the
Python_GUI folder:  python examples/sweep_example.py

This only SETS parameters and reads telemetry; it never starts a trial or a
motor on its own. Nothing here moves a motor that a GUI user hasn't already
enabled.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "remote"))
from client import ExoRemote, RemoteError  # noqa: E402


def main():
    with ExoRemote() as exo:
        matrix = exo.wait_for_matrix(timeout=5)
        if not matrix:
            print("No controller matrix yet - is the GUI connected to an exo?")
            return
        print("Controllers on Ankle(L):", exo.controllers("Ankle(L)"))

        for gain in (1.0, 2.0, 3.0):
            try:
                exo.set_param("Ankle(L)", "zeroTorque", "p_gain", gain, bilateral=True)
                print(f"set p_gain = {gain} (GUI accepted)")
            except RemoteError as e:
                print(f"rejected: {e} (code={e.code})")
            # Watch the firmware's own accept/reject for this write.
            time.sleep(0.2)
            ack = exo.last_ack()
            if ack:
                print("  firmware ack:", ack["accepted"], ack["reason"])
            time.sleep(2.0)


if __name__ == "__main__":
    main()
