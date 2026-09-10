"""
External control code for the OpenExo ankle exoskeleton.

This is the OpenExo counterpart of main_control_code.py on the hip exo. The difference is where the
control loop lives. On the hip exo, that file OWNS the motors: it builds torque splines in Python, runs
them at 200 Hz, and writes CAN frames. Here the Teensy owns all of that. This code is supervisory - it
owns an optimization backend, computes machine actions, and applies them by writing controller
parameters through the GUI's UDP remote service, then confirming the exo acknowledged them.

Consequences of that split, all of them deliberate:
    1) This code does NO signal processing. It never derives an observation from raw exo telemetry. If an
       exo-derived input is ever needed, the Teensy computes it and sends it up.
    2) External instruments (the self-paced treadmill here, a metabolic cart later) connect DIRECTLY to
       this code on their own sockets. They are not exo data and do not go through the GUI.
    3) For the game theory protocol the exo is therefore WRITE-ONLY. The human action is treadmill speed,
       and nothing needs to come back from the exo except acknowledgements.
    4) There is no trial start/stop, motor enable or calibration over UDP, by design. A human operator
       still drives the GUI. This code refuses to start anything on its own.

Run the GUI first (python GUI.py), connect it to the exo, and start the trial from the GUI. Then, from
the Python_GUI/external_control folder:  python main_external_control.py
"""

import numpy as np
import os
import sys
import time

from Utilities.OpenExoLink_utilities import OpenExoLink, OpenExoLinkError
from Utilities.ActionMap_utilities import SplineAlt_action_map


def _load_module_from_file(module_name, file_path):
    '''Load one .py file as a module, by its full path.

    We do this instead of putting the hip exo's folder on sys.path because BOTH code bases have a folder
    called "Utilities", and having both on the path makes "import Utilities.something" ambiguous in a way
    that fails confusingly. Loading by path keeps the algorithm single-source (we never copy the backend
    into this repo) without the collision.'''
    import importlib.util
    if not os.path.isfile(file_path):
        raise ImportError(f"Could not find {module_name} at {file_path}")
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# Main external control code
def main():

    #Where the hip exo's code base lives. We import the optimization backend and the UDP receiver from there rather than duplicating them. 
    #NOTE:YOU NEED TO EDIT THIS TO YOUR OWN SETTING
    hippo_control_code_path = r"E:\Research\Exoskeletons\Max Shepherd's exo\Exo control codes"

    #Options
    udp_in_use = 0  # By default, don't listen on UDP unless user decided to
    game_theory_mode = 0  # Default to no backend; only toggles if the user picks the game theory mode

    #For debugging
    use_loop_time = 1  #If 1, all log timestamps within one loop iteration come from the loop start time, so everything logged in a cycle shares one timestamp
    
    #Data logging
    log_m_action_enabled = 1      #Whether we log every machine action (torque percentage and peak timing)
    log_param_write_enabled = 1   #Whether we log every parameter write and how the exo answered it
    log_h_action_enabled = 1    #Whether we log every human action collection and its average

    #Connection to the OpenExo GUI. Localhost only unless the GUI's RemoteConfig was deliberately widened
    gui_host = "127.0.0.1"
    gui_port = 9750
    ack_timeout = 5.0       #How long we wait for the exo to acknowledge a parameter write, in seconds
    max_write_retries = 3   #How many times we resend a write that was met with silence

    #Controller and safety
    controller_name = "splineAlt"    #The ankle controller we drive. Its TorqScale is the current machine action of choice. 
    #No other controllers can be easily modified by any backend since only 1 parameter can be modified per update.
    
    joints_to_drive = ("Ankle(L)", "Ankle(R)")
    max_torque_scaling = 100.0  #Claude has some concerns about the actual "requested torque" being larger than CSV demanded, but in the end we have PiD to correct for stuff like that   
    plantar_peak_torque = 15.0   #Peak plantarflexion magnitude in Nm at 100% scale, before body mass scaling
    dorsi_peak_torque = 8.0      #Peak dorsiflexion magnitude in Nm at 100% scale, before body mass scaling

    #Timing
    operating_rate = 20             #Main loop rate in Hz. Nothing here needs to be fast - the exo runs its own loop at 100x this, and we only write a parameter now and then. Can go even lower

    h_action_strides_to_average = 10  #How many STRIDES of treadmill speed we average into one human action.
    #The treadmill sends exactly one speed per step, so each successful read IS one stride - we never need stride detection of our own.
    #NOTE: we hand this to the backend and then read it back off it, because GUI doesn't broadcast HS and we can only trust treadmill as source of this
    #If eventually this becomes a problem, we can ask teensy+nano/GUI to broadcast HS events   

    treadmill_udp_port = 5004       #Where MATLAB / the self-paced treadmill sends the current speed
    timing_udp_port = 5003          #Where a timing value arrives in UDP mode. Same port as the hip exo
    udp_min_write_interval = 0.5    #Shortest gap between two UDP-driven writes, in seconds. Every UDP update costs a BLE round trip here (unlike the hip exo, where it was just a local variable), so a chatty sender must not saturate the link

    body_mass_standard = 75         #In kg, the body mass the torque magnitudes above correspond to

    #Peak timing is a percent of gait, and the firmware bounds-checks it to 0-100 (ExoCode/src/ControllerData.cpp, spline_alt_bounds)
    peak_timing_min = 0
    peak_timing_max = 100

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Logs", "Active Test")
    os.makedirs(log_dir, exist_ok=True)   # create if not exist

    if log_m_action_enabled:
        action_log_file = open(os.path.join(log_dir, "Machine_action_log.txt"), "w", buffering=1)  # Line-buffered
        print(f"Python time,Elapsed time,Source,Value requested,Value applied", file=action_log_file)
    else:
        action_log_file = None

    #Param_write_log keeps track of what THIS code tried to write into GUI (so all commands from backend)
    if log_param_write_enabled:
        write_log_file = open(os.path.join(log_dir, "Param_write_log.txt"), "w", buffering=1)  # Line-buffered
        print(f"Python time,Target,Value,Result,Attempts", file=write_log_file)
    else:
        write_log_file = None

    if log_h_action_enabled:
        h_action_log_file = open(os.path.join(log_dir, "Human_action_log.txt"), "w", buffering=1)  # Line-buffered
        print(f"Python time,Elapsed time,Strides collected,Averaged human action", file=h_action_log_file)
    else:
        h_action_log_file = None

    #Connect to the GUI's remote service. This does not touch the exo - it only checks the GUI is there
    OpenExo_link = OpenExoLink(host=gui_host, port=gui_port, ack_timeout=ack_timeout, max_retries=max_write_retries, verbose=1, logfile=write_log_file)
    try:
        OpenExo_link.connect(matrix_timeout=10.0)
    except OpenExoLinkError as e:
        print(f"{e}")
        input("Start the GUI, connect it to the exo, then restart this code - hit enter to exit.\n")
        return

    if not OpenExo_link.matrix:
        print("No controller matrix, so parameter names cannot be resolved.")
        input("Connect the GUI to the exo first, then restart this code - hit enter to exit.\n")
        return

    #Show the user exactly what the exo advertised, so a name mismatch is caught before anything is written
    show_matrix = input("Print the controller matrix the exo advertised? (y / Press enter to skip):\n")
    if show_matrix.lower().startswith("y"):
        OpenExo_link.print_matrix()

    #These get filled in by whichever mode the user picks below
    zero_torque_mode = 0        #If 1, we hold at zero scale no matter what percentage the user typed
    peak_timing_value = None    #Peak timing for the lobe we are shaping, if the mode sets one
    timing_lobe = None          #Which lobe that timing belongs to, "plantar" or "dorsi"
    plantar_nm = plantar_peak_torque   #Magnitude for each lobe. Setting one to 0 disables that lobe
    dorsi_nm = dorsi_peak_torque       #entirely (splineAlt emits no nodes for a zero-magnitude lobe)
    new_torque_percentage = None

    while True:  #Mode selection loop - decide what drives the torque percentage and the timing
        exo_mode = input("Please input 2 for running game-theory backend, 1 for anything else:\n")
        if exo_mode in ("1", "2"):
            break
        print("Invalid input")

    if exo_mode == "2":
        game_theory_mode = 1
        #OpenExo GUI doesn't broadcast cadence, so right now, self-paced treadmill speed is the only human action available
        print("Entering game-theory mode. Human action is self-paced treadmill speed (the only option here), and machine action is the torque percentage.")
        GameTheory_module = _load_module_from_file("GameTheory_backend",os.path.join(hippo_control_code_path, "Controller_backend", "GameTheory_backend.py"))
        GameTheory_1D_Backend = GameTheory_module.GameTheory_1D_Backend

    if exo_mode == "1":
        game_theory_mode = 0
        print("Not running game-theory backend")
        exo_mode_sub = input("Torque profile selection: please input OP, plantar, dorsi or udp. "
                             "Hit enter for zero torque and IGNORE UDP input:\n")
        #Note the ankle's two lobes are plantarflexion and dorsiflexion, so these replace the hip exo's
        #'flexion' and 'extension'. They behave the same way: pick a lobe, place its peak, disable the other
        if exo_mode_sub.upper() == "OP":  #Here OP would refer to what the SD card natively carries, which should be "optimized"
            peak_timing_value = None
            print("Optimized torque profile selected - the timings on the SD card are used unchanged.")
        elif exo_mode_sub.lower() == "plantar":  #Plantarflexion-only torque profile mode
            exo_mode_sub = input("Plantarflexion-only mode. Please input the peak plantarflexion torque timing:\n")
            try:
                peak_timing_value = float(exo_mode_sub)
            except ValueError:
                peak_timing_value = None
            if peak_timing_value is not None and peak_timing_min < peak_timing_value < peak_timing_max:
                timing_lobe = "plantar"
                dorsi_nm = 0.0  #A zero-magnitude lobe emits no nodes at all, which is how splineAlt disables one
                print(f"Fixed peak plantarflexion assistance timing mode at {peak_timing_value}% of gait cycle")
                print('Dorsiflexion assistance disabled')
            else:
                #Out of range
                peak_timing_value = None
                plantar_nm = 0.0
                dorsi_nm = 0.0
                zero_torque_mode = 1
                print(f"Timing out of range of {peak_timing_min}% to {peak_timing_max}% gait cycle. Zero torque.")
        elif exo_mode_sub.lower() == "dorsi":  #Dorsiflexion-only torque profile mode
            exo_mode_sub = input("Dorsiflexion-only mode. Please input the peak dorsiflexion torque timing:\n")
            try:
                peak_timing_value = float(exo_mode_sub)
            except ValueError:
                peak_timing_value = None
            if peak_timing_value is not None and peak_timing_min < peak_timing_value < peak_timing_max:
                timing_lobe = "dorsi"
                plantar_nm = 0.0
                print(f"Fixed peak dorsiflexion assistance timing mode at {peak_timing_value}% of gait cycle")
                print('Plantarflexion assistance disabled')
            else:
                #Out of range
                peak_timing_value = None
                plantar_nm = 0.0
                dorsi_nm = 0.0
                zero_torque_mode = 1
                print(f"Timing out of range of {peak_timing_min}% to {peak_timing_max}% gait cycle. Zero torque.")
        elif exo_mode_sub.lower() == "udp":
            # In this case we initiate UDP
            udp_in_use = 1
            while True:  #Which lobe does the incoming timing move? The hip exo only had one answer, we have two
                timing_lobe = input("Which lobe should the UDP timing move? Please input plantar or dorsi:\n").lower()
                if timing_lobe in ("plantar", "dorsi"):
                    break
                print("Invalid input")
            if timing_lobe == "plantar":
                dorsi_nm = 0.0
                print("Dorsiflexion assistance disabled")
            else:
                plantar_nm = 0.0
                print("Plantarflexion assistance disabled")
            peak_timing_value = None
            print(f"Zero torque profile waiting for UDP input on port {timing_udp_port}, "
                  f"moving the {timing_lobe} peak timing.")
        elif exo_mode_sub.strip() == "":
            #Zero torque profile
            plantar_nm = 0.0
            dorsi_nm = 0.0
            zero_torque_mode = 1
            print("Zero torque profile selected. Ignoring UDP")
        else:
            input("Invalid input. Please restart and retry - code exiting.")
            OpenExo_link.close()
            return

    #Body mass, used to scale the peak torque magnitudes. Same convention as the hip exo
    body_mass_participant = input(f"Please input participant's body mass in kg (enter for {body_mass_standard}):\n")
    try:
        body_mass_participant = float(body_mass_participant)
    except ValueError:
        print(f"No input. Defaulting to {body_mass_standard} with body mass scaling factor 1")
        body_mass_participant = body_mass_standard

    if zero_torque_mode:
        #Both lobes are already at 0 Nm, so a percentage would scale nothing. Leave the scale at 0 as well,
        #so the controller reports what the user actually asked for rather than "60% of nothing"
        new_torque_percentage = 0.0
        print("Holding at zero torque scale.")
    elif not game_theory_mode:
        #Allow the user to create a familiarization session which outputs a scaled portion of the profile.
        #In game theory mode this is the machine action, so the backend owns it instead
        new_torque_percentage = input("If familiarization session, input percentage of peak torque. "
                                      "Else, hit enter for maximum peak torque:\n")
        if new_torque_percentage.strip() == "":
            # User just pressed Enter
            new_torque_percentage = 100.0
        else:
            new_torque_percentage = float(new_torque_percentage)
        print(f"Torque percentage set to {new_torque_percentage}% ")

    #Build the action map. This resolves every parameter address up front, so a name mismatch fails here
    #rather than half way through an experiment
    try:
        action_map = SplineAlt_action_map(OpenExo_link, body_mass_standard, joints=joints_to_drive, controller=controller_name, torque_percentage_max=max_torque_scaling, verbose=1)
    except OpenExoLinkError as e:
        print(f"Could not resolve the controller parameters: {e}")
        input("Check the controller name against the printed matrix, then restart - hit enter to exit.\n")
        OpenExo_link.close()
        return

    #Everything below this line writes to the exo
    print("\n=== The next step CHANGES THE ACTIVE CONTROLLER on both ankles ===")
    print(f"    Both ankles will switch to {controller_name} at ZERO torque scale (transparency), then")
    print(f"    the peak torque magnitudes will be set for a {body_mass_participant:.0f} kg participant.")
    print("    Make sure the trial is running in the GUI and the participant is ready.")
    input("Press enter to engage the controller, or Ctrl-C to abort:\n")

    try:
        #Call engage safety to overwrite any pre-existing torque scale on CSV to 0, so we start at zero torque
        action_map.engage_controller_safely()
        #Apply scaling. This is the ONLY place we call apply_torque_magnitude. Anywhere else, it's apply_machine_action
        scaling = action_map.apply_torque_magnitudes(plantar_nm, dorsi_nm, body_mass=body_mass_participant)
        if peak_timing_value is not None and timing_lobe is not None:
            action_map.apply_peak_timing(timing_lobe, peak_timing_value)
        print(f"Controller engaged. Torque magnitudes scaled by {scaling:.2f}, torque scale still at 0%.")
    except OpenExoLinkError as e:
        print(f"Session setup FAILED: {e}")
        print("Nothing further will be sent. Stop the trial from the GUI.")
        OpenExo_link.close()
        return

    input("Press enter to start running the exo:\n")
    start_time = time.perf_counter()  # start a timer

    #Loop state every mode needs
    new_timing_value = None             #A peak timing waiting to be deployed, if any

    if udp_in_use:
        # Initialize UDP receiver
        UDP_module = _load_module_from_file("UDP_utilities", os.path.join(hippo_control_code_path, "Utilities", "UDP_utilities.py"))
        udp_receiver = UDP_module.UDPReceiver(port=timing_udp_port)
        udp_receiver.start_listening()
        print(f"Listening for timing values on UDP port {timing_udp_port}")

        #Loop state only UDP mode uses
        pending_udp_timing = None           #Newest UDP timing we have not been allowed to write yet
        pending_udp_zero = False            #A disable-assistance request waiting on the rate limiter
        last_udp_write_time = 0             #When we last let a UDP update through to the exo
        working_torque_percentage = new_torque_percentage  #What we restore to when assistance resumes
        new_torque_percentage = None        #Hold at zero until a timing value arrives, like the hip exo

    if game_theory_mode:
        #Game theory mode requires getting treadmill speed from Bertec, which goes through UDP as well
        UDP_module = _load_module_from_file("UDP_utilities", os.path.join(hippo_control_code_path, "Utilities", "UDP_utilities.py"))
        treadmill_receiver = UDP_module.UDPReceiver(port=treadmill_udp_port, fmt='f')
        treadmill_receiver.start_listening()
        print(f"Listening for self-paced treadmill speed on UDP port {treadmill_udp_port}")

        #These hyperparameters are carried over from the hip exo's SPT game theory session. Review them
        #for this study rather than assuming they transfer
        game_theory_backend = GameTheory_1D_Backend(start_time=time.perf_counter(), max_iterations=32,
                                                    est_hstar_init=1.4, est_mstar_init=30,
                                                    L_values=[200, -200], L_decay_ON=1,
                                                    max_repeats_per_trial=30, repeats_to_average=10,
                                                    estimate_update_decay_ON=0, human_lag_time=1,
                                                    _h_action_max_stride_count=h_action_strides_to_average,
                                                    log_dir=log_dir)
        game_theory_backend.start()  # Start the backend thread

        #Loop state only game theory mode uses
        prev_human_action_time = start_time
        start_human_action_recording = 0    #Whether we are inside a round of human action collection
        _h_action_max_stride_count = game_theory_backend._h_action_max_stride_count
        _h_action_current_stride_count = 0  #How many strides we have collected in this round
        _h_action_average = np.zeros(_h_action_max_stride_count)  #Buffer of speeds, one entry per stride
        new_torque_percentage = None        #The backend owns this from here on

    try:
        while True:
            loop_start_time = time.perf_counter()

            #Notice a dropped link between writes. This is a backstop, not a fast watchdog - the GUI only
            #reports a disconnect once BLE gives up, which is several seconds after the exo goes quiet
            OpenExo_link.pump()
            if OpenExo_link.link_down:
                print("The GUI reports the exo is no longer connected. Stopping - no further actions will be sent.")
                break

            if game_theory_mode:
                #Check for game-theory backend finish
                if game_theory_backend.finished:
                    print("Completed - maximum game theory algorithm iterations reached")
                    break

                #If we passed the wait time, then start logging human actions for averaging and eventually
                #register as actual human action.
                if loop_start_time - prev_human_action_time > game_theory_backend.human_lag_time:
                    if not start_human_action_recording:
                        _h_action_current_stride_count = 0
                        _h_action_average = np.zeros(_h_action_max_stride_count)
                        start_human_action_recording = 1
                        print(f"Start collecting {_h_action_max_stride_count} strides data for human action!")

                    treadmill_speed = treadmill_receiver.get_latest_timing_value()
                    #UDPReceiver maps any non-positive payload to -1, a leftover from its timing-value use
                    #case. Guard against it rather than averaging a -1 into the human action
                    if (treadmill_speed is not None) and (treadmill_speed > 0):
                        #Only update the counts if we succeeded
                        _h_action_average[_h_action_current_stride_count] = treadmill_speed
                        _h_action_current_stride_count = _h_action_current_stride_count + 1
                        print(f"Collected {_h_action_current_stride_count} data point!")

                    #Did we log enough steps?
                    if _h_action_current_stride_count >= _h_action_max_stride_count:
                        game_theory_backend.read_human_action(np.mean(_h_action_average), loop_start_time)
                        if log_h_action_enabled:
                            print(f"{loop_start_time},{loop_start_time - start_time:.2f},"
                                  f"{_h_action_current_stride_count},{np.mean(_h_action_average)}", file=h_action_log_file)
                        prev_human_action_time = loop_start_time
                        _h_action_current_stride_count = 0  #Reset this flag
                        _h_action_average = np.zeros(_h_action_max_stride_count)  #Reset the buffer
                        start_human_action_recording = 0    #Ready for the next round of collection

                #If the backend finished calculations, fetch it and deploy
                if game_theory_backend.output_ready is True:
                    new_torque_percentage = game_theory_backend.m_action_next
                    game_theory_backend.output_fetched = True  #Set this flag once we finish fetching data from backend

            elif udp_in_use:  #Listen to UDP server if user requested
                # Check for new UDP timing value
                _incoming_timing_value = udp_receiver.get_latest_timing_value()
                if _incoming_timing_value is not None:
                    if _incoming_timing_value > 0:  #So we received a real timing value
                        #Hold on to the newest timing rather than acting on it right here. If the rate
                        #limiter below is not ready yet we KEEP it instead of dropping it, and a fresher
                        #value simply replaces it - what we want is the latest commanded state, not a
                        #replay of every value the sender ever produced
                        pending_udp_timing = _incoming_timing_value
                        pending_udp_zero = False    #A real timing supersedes an unsent disable request.
                                                    #Without this a queued zero would fire in the same
                                                    #pass that restores assistance, and the two would
                                                    #fight over new_torque_percentage.
                    else:  #A negative timing value means we want to disable assistance for now
                        pending_udp_timing = None       #A stale timing must not outlive a disable command
                        peak_timing_value = _incoming_timing_value
                        #Ask for zero, but do NOT write it here. Two guards below decide whether it is
                        #actually worth a BLE round trip. This used to write unconditionally on every
                        #loop pass, which is how a sender stuck at 200 Hz turned into 20 writes/s of
                        #a value the exo was already holding.
                        pending_udp_zero = True

                #Zero-torque request. Two conditions have to be met before this costs a BLE round trip.
                #
                #  1) NOT ALREADY ZERO. action_map.last_applied_action is only assigned after EVERY joint
                #     has acknowledged (ActionMap_utilities.apply_torque_percentage), so `== 0` means zero
                #     is confirmed on BOTH legs, not just requested. It starts as None, so the first zero
                #     is never skipped, and a half-applied write leaves it at its previous value - in both
                #     of those cases we correctly fall through and write.
                #  2) RATE LIMIT. Same 0.5 s budget as a timing update, because it costs the link exactly
                #     the same. The request is HELD rather than dropped, so a disable command is never
                #     lost - it just waits its turn, at most udp_min_write_interval.
                if pending_udp_zero:
                    if action_map.last_applied_action == 0:
                        pending_udp_zero = False        #Already there on both legs. Nothing to send.
                    elif loop_start_time - last_udp_write_time >= udp_min_write_interval:
                        print("Received command from UDP to temporarily disable assistance")
                        new_torque_percentage = 0.0
                        last_udp_write_time = loop_start_time
                        pending_udp_zero = False

                #Every UDP update costs a BLE round trip here (on the hip exo it was just a local variable),
                #so a chatty sender must not queue up writes faster than the link can retire them
                if (pending_udp_timing is not None) and (loop_start_time - last_udp_write_time >= udp_min_write_interval):
                    if pending_udp_timing != peak_timing_value:
                        peak_timing_value = pending_udp_timing
                        last_udp_write_time = loop_start_time
                        print(f"Updated timing value from UDP: {peak_timing_value:.2f}")
                        new_timing_value = peak_timing_value
                        #Bring assistance back up if we had been parked by an earlier negative value
                        if action_map.last_applied_action == 0:
                            new_torque_percentage = working_torque_percentage
                    pending_udp_timing = None

            #Deploy a new peak timing, if one is waiting
            if new_timing_value is not None:
                try:
                    action_map.apply_peak_timing(timing_lobe, new_timing_value)
                except OpenExoLinkError as e:
                    print(f"FAILED to apply the peak timing: {e}")
                    print("The two legs may now differ. Stopping and attempting to park to transparency.")
                    break
                if log_m_action_enabled:
                    if use_loop_time:
                        _timestamp = loop_start_time
                    else:
                        _timestamp = time.perf_counter()
                    print(f"{_timestamp},{_timestamp - start_time:.2f},{timing_lobe} timing,"
                          f"{new_timing_value},{new_timing_value}", file=action_log_file)
                new_timing_value = None  #Wipe the buffer

            #Deploy a new torque percentage, if one is waiting
            if new_torque_percentage is not None:
                print(f"Applying torque percentage: {new_torque_percentage:.2f}%")
                try:
                    applied = action_map.apply_torque_percentage(new_torque_percentage)
                except OpenExoLinkError as e:
                    #A half-applied action means the two legs are assisting differently. Stop rather than
                    #carry on with an unknown state on the exo
                    print(f"FAILED to apply the torque percentage: {e}")
                    print("The two legs may now differ. Stopping and attempting to park to transparency.")
                    break
                if log_m_action_enabled:
                    if use_loop_time:
                        _timestamp = loop_start_time
                    else:
                        _timestamp = time.perf_counter()
                    print(f"{_timestamp},{_timestamp - start_time:.2f},torque percentage,"
                          f"{new_torque_percentage},{applied}", file=action_log_file)
                new_torque_percentage = None  #Wipe the buffer

            # End of loop operations
            loop_duration = time.perf_counter() - loop_start_time
            sleep_time = (1 / operating_rate) - loop_duration
            if sleep_time < 0:
                sleep_time = 0
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n Keyboard interrupted. Stopping backend and parking the exo...")

    finally:  #No matter how we reached here, safely exit all things
        #Park to transparency FIRST, while the link is most likely still alive
        action_map.park_to_transparency()

        if game_theory_mode:
            game_theory_backend.stop_and_finalize()
            treadmill_receiver.stop()
        if udp_in_use:
            udp_receiver.stop()

        OpenExo_link.close()

        if log_m_action_enabled:
            action_log_file.close()
        if log_param_write_enabled:
            write_log_file.close()
        if log_h_action_enabled:
            h_action_log_file.close()

        print("NOTE: this code never started the trial and cannot end it. End the trial from the GUI.")

        #Finally, ask the user if they'd want to rename the logs folder - helpful to avoid accidentally overwriting data...
        if os.path.isdir(log_dir):
            try:
                new_log_dir_name = input(
                    f"Rename this run's folder? (blank = not renaming and overwrite next run): "
                ).strip()
            except (KeyboardInterrupt, EOFError):
                new_log_dir_name = ""
                print()

            if new_log_dir_name:
                parent = os.path.dirname(os.path.abspath(log_dir))
                target = os.path.join(parent, new_log_dir_name)
                i = 1
                base = target
                while os.path.exists(target):  #Handles the case if we type an existing name, it just rename the whole folder
                    print('Warning! Folder name already exists. Auto renaming')
                    target = f"{base}_{i}"
                    i += 1
                try:
                    os.rename(log_dir, target)
                    print(f"Saved as: {target}")
                except OSError as e:
                    print(f"Rename failed ({e}). Logs still in {log_dir}.")
            else:
                print(f"Logs left in {log_dir} (will be overwritten on next run).")

        print("Shutdown complete.")


if __name__ == "__main__":
    main()
