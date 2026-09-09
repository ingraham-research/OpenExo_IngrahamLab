# Pseudocode — external control code for the OpenExo ankle

This is the whole design written out as pseudocode, so it can be hand-typed rather than copied. It
follows the three real files exactly:

| Pseudocode section | File |
|---|---|
| 1. `ExoLink` | `Utilities/ExoLink_utilities.py` |
| 2. `SplineAlt_action_map` | `Utilities/ActionMap_utilities.py` |
| 3. `main` | `main_external_control.py` |

---

## 0. The idea in six lines

```
The GUI owns the exo. The Teensy owns the control loop.
This code owns an optimization backend and ONE number: the torque percentage.

    human action  <- self-paced treadmill, straight into this code over UDP
    machine action -> splineAlt TorqScale, written through the GUI, one parameter, both legs

It does no signal processing, and it cannot start or stop a trial.
```

## 0.1 The five traps this code exists to handle

Everything non-obvious below traces back to one of these. They are all real and all cost a session if
missed.

1. **Silence is the only failure signal.** A dropped BLE write produces no negative acknowledgement.
   The GUI's `ok` reply only means it *queued* a write — it returns `ok` with no exo connected at all.
   Only the exo's ack proves the value was stored.
2. **Acks carry no request id.** They carry `joint_id`, `controller_id`, `param_index` and nothing
   else. So you must resolve names to numbers *yourself* and match on that triple.
3. **A fast ack gets swallowed.** `ExoRemote._command` drains stream frames while waiting for its own
   `ok`, so an ack arriving in that window lands in `last_ack()` and is never re-delivered by
   `stream()`. Check `last_ack()` *first*, then fall back to the stream.
4. **Writing a parameter to a non-active controller switches the joint to it and loads its SD-card
   defaults.** `splineAlt.csv` ships `TorqScale = 95`. So the first write must be `TorqScale = 0`, or
   the leg starts assisting at 95 % the instant you touch it.
5. **`bilateral=True` acks twice and only the last is kept.** Send two unilateral writes instead —
   identical on the wire, unambiguous to match.

---

## 1. `ExoLink` — a confirmed parameter write

```
CONSTANTS
    LEFT_ANKLE_JOINT_ID  = 68        # ExoCode/src/ParseIni.h: left = 0b01000000
    RIGHT_ANKLE_JOINT_ID = 36        #                          right = 0b00100000
    # NEVER use Python_GUI/utils/config.py JointConfig.ID_TO_NUM - all 8 entries have L/R inverted
    DEFAULT_ACK_TIMEOUT = 5.0
    DEFAULT_MAX_RETRIES = 3
    ACK_POLL_INTERVAL   = 0.2

CLASS ExoLink(host, port, timeout, ack_timeout, max_retries, verbose, logfile)

    __init__:
        put ../../remote on sys.path, import ExoRemote and RemoteError from client
        self.exo = ExoRemote(host, port, timeout)
        self.matrix = []            # the handshake controller matrix
        self.link_down = 0          # set by pump() when the GUI says the exo went away
        self._ack_snapshot = None   # what last_ack() held just before our most recent write


    ### Connection ###

    connect(matrix_timeout):
        exo.ping()                      -> if it raises, the GUI is not running: raise ExoLinkError
        exo.subscribe(["ack","status"]) # NOT "rt". We have no use for telemetry and it is opt-in
        loop until matrix_timeout:
            matrix = exo.get_matrix()
            if matrix: break
            sleep 0.5
        if no matrix: warn "the GUI is running but is not connected to an exo"
        return matrix

    print_matrix():
        for each row: print joint name + id, controller name + id, and every param with its index
        # row layout is [display_name, joint_id, controller_name, controller_id, param0, param1, ...]


    ### Name -> number ###

    resolve(joint, controller, param) -> (joint_id, controller_id, param_index):
        joint_id   = _resolve_joint(joint)
        joint_rows = rows of matrix whose row[1] == joint_id
        if controller and param are BOTH integers:
            return (joint_id, controller, param)      # works with no matrix at all, for testing
        if joint_rows is empty: raise ExoLinkError
        controller_row = _resolve_controller_row(controller, joint_rows)
        return (joint_id, int(controller_row[3]), _resolve_param_index(param, controller_row))

    _resolve_joint(j):
        if j is an int (and NOT a bool): return it
        strip " (<id>)" off row[0] and compare case-insensitively; also accept the id as a string
        else raise, listing the joint names the exo actually advertised

    _resolve_controller_row(c, joint_rows):
        if int: find the row whose row[3] == c
        else: case-insensitive match on row[2]
              THEN fall back to a prefix match  # the BLE handshake truncates long names
        else raise, listing the controller names on this joint

    _resolve_param_index(p, row):
        params = row[4:]
        if int: bounds-check against len(params) and return
        else: case-insensitive match, then prefix match  # same truncation problem
        else raise, listing the parameter names


    ### The write ###

    set_param_confirmed(address, value, label) -> ack:
        (joint_id, controller_id, param_index) = address

        for attempt in 1..max_retries:

            self._ack_snapshot = exo.last_ack()      # so we can tell a NEW ack from a stale one

            try:
                exo.set_param(joint_id, controller_id, param_index, value, bilateral=False)
                # ^ ALWAYS unilateral. See trap 5.
            except RemoteError as e:
                raise ExoLinkError          # the GUI itself refused it - resending cannot help,
                                            # so fail now rather than burning the retries

            ack = _wait_for_ack(joint_id, controller_id, param_index, ack_timeout)

            if ack is None:
                print "no ack, resending"
                continue                    # silence is the ONE case worth retrying (trap 1)

            if ack.accepted:
                log it and return ack

            raise ExoLinkError(reason=ack.reason)
            # the firmware got it and said no. Out of bounds / wrong type / wrong controller.
            # Resending the identical value gets the identical rejection

        raise ExoLinkError("no acknowledgement after N attempts - the write never reached the exo")

    _wait_for_ack(joint_id, controller_id, param_index, ack_timeout) -> ack or None:

        # FIRST, and this is the important bit (trap 3):
        ack = exo.last_ack()
        if ack is not None AND ack is not self._ack_snapshot AND _ack_matches(ack, ...):
            return ack

        # only then listen on the socket for the rest of the window
        save the socket timeout; set it to ACK_POLL_INTERVAL
        while now < deadline:
            msg = _recv_one()                       # returns None on timeout / junk
            if msg is None: continue
            if msg.stream != "ack": continue
            if _ack_matches(msg, ...): return msg
        restore the socket timeout
        return None

    _ack_matches(ack, joint_id, controller_id, param_index):
        int-compare all three fields; return False on any None or non-numeric   # trap 2


    ### Link health ###

    pump(budget):
        set socket non-blocking, drain whatever is waiting, restore the timeout
        on a "disconnected" or "device_error" status frame: link_down = 1 and say so
        on "connected": link_down = 0
        # Honest limit: the GUI only reports a disconnect once BLE gives up, which for the Nano
        # radio-silence failure is ~9.6 s after the exo actually went quiet. This is a backstop.
        # The real detection of a dead link is a set_param_confirmed that comes back silent.

    close():
        unsubscribe and close the socket, both wrapped so a dead peer cannot raise on shutdown
```

---

## 2. `SplineAlt_action_map` — machine action to parameter writes

```
CONSTANTS  # from ExoCode/src/ControllerData.h, namespace controller_defs::spline_alt
    SPLINEALT_MAX_PLANTAR_TORQUE = 0     SPLINEALT_DORSI_RISE_TIME    = 7
    SPLINEALT_MAX_DORSI_TORQUE   = 1     SPLINEALT_DORSI_DWELL_TIME   = 8
    SPLINEALT_PEAK_PLANTAR_TIME  = 2     SPLINEALT_DORSI_FALL_TIME    = 9
    SPLINEALT_PEAK_DORSI_TIME    = 3     SPLINEALT_TORQUE_SCALE       = 10   <-- the machine action
    SPLINEALT_PLANTAR_RISE_TIME  = 4     SPLINEALT_SIM_GAIT           = 11
    SPLINEALT_PLANTAR_DWELL_TIME = 5     SPLINEALT_USE_PERCENT_GAIT   = 12
    SPLINEALT_PLANTAR_FALL_TIME  = 6     SPLINEALT_USE_PID            = 13
    BODY_MASS_STANDARD = 75              SPLINEALT_P/I/D_GAIN         = 14/15/16

    # firmware bounds (ControllerData.cpp, spline_alt_bounds): magnitudes +-50 Nm, every
    # time/percent parameter 0-100, the three flags integer-only 0 or 1

CLASS SplineAlt_action_map(exo_link, joints, controller, m_max, verbose)

    __init__:
        resolve EVERY address up front - TorqScale, PlantarNm, DorsiNm, and BOTH peak times, per joint
        # so a name typo fails here at setup, not half way through an experiment

    engage_controller_safely():
        write TorqScale = 0 to both legs, FIRST, before anything else
        # This is trap 4. Writing any parameter to a controller that is not active switches the joint
        # to it AND loads its SD defaults, and splineAlt.csv ships TorqScale = 95. Writing the scale
        # first means the controller comes up transparent.
        # Scale 0 is a real mode, not a hack: every node collapses to zero and the gain scheduler
        # holds the tuned zero-torque gains all stride (Controller.cpp, SplineAlt::_build_nodes).

    apply_torque_magnitudes(plantar_nm, dorsi_nm, body_mass) -> scaling:
        scaling = body_mass / BODY_MASS_STANDARD      (1.0 if no body mass given)
        write PlantarNm = plantar_nm * scaling to both legs
        write DorsiNm   = dorsi_nm   * scaling to both legs
        return scaling
        # Session setup ONLY. The experiment loop never touches these.
        # Passing 0 for one of them DISABLES that lobe outright - splineAlt emits no nodes at all for
        # a zero-magnitude lobe, so its timing parameters cannot then collide with the other lobe's.

    apply_peak_timing(lobe, percent_gait):          # lobe is "plantar" or "dorsi"
        write PlantarPk (or DorsiPk) = percent_gait to both legs
        # This is what a UDP timing value drives. It is the ankle equivalent of the hip exo's
        # create_flexion_only_torque_profile(udp_timing_value) - except that there, a new timing meant
        # rebuilding the whole spline in Python, and here it is ONE parameter write per leg. The
        # Teensy rebuilds its own nodes from it.

    apply_timing(param_index, value, label):
        write any other shape parameter (a rise/dwell/fall duration) to both legs. Session setup only.
        # WARNING: a rise or fall duration of 0 is NOT safe and is not repaired by the firmware -
        # it trips the interpolator's monotonicity guard and zeroes the whole profile.
        # A dwell of 0 is fine and is the normal default.

    apply_machine_action(m) -> applied:
        m_clamped = clamp(m, 0, m_max)             # our own limit, on top of the firmware's
        write TorqScale = m_clamped to both legs
        return m_clamped
        # Do NOT catch a failed write here. A half-applied action means the two legs are assisting
        # differently, which is worse than stopping. Let it raise; the caller decides.

    park_to_transparency():
        try apply_machine_action(0)
        on failure say plainly that the exo is STILL RUNNING whatever it had last, and that the
        operator must end the trial from the GUI. Return 0. Never pretend it was parked.
```

---

## 3. `main` — the mode menu and the supervisory loop

### 3.1 The menu

Kept as close to the hip exo's as the hardware allows. Two deliberate differences:

- **Game theory has only one option now.** The hip asked `cadence` or `SPT`. Cadence needs stride
  timing, which only exists on the Teensy, and this code derives nothing from raw telemetry — so
  self-paced treadmill speed is the only human action available.
- **`flexion` / `extension` become `plantar` / `dorsi`.** Those are the ankle's two lobes and the names
  the CSV uses. They behave the same way: pick a lobe, place its peak, disable the other.

```
LOOP until valid:
    "Please input 2 for running game-theory backend, 1 for anything else:"

IF 2:   game_theory_mode = 1
        say human action is treadmill speed (the ONLY option here), machine action is torque percent
        load GameTheory_backend.py BY FULL PATH
        # not via sys.path: BOTH code bases have a folder called "Utilities", and putting both on the
        # path makes "import Utilities.x" ambiguous in a way that fails confusingly

IF 1:   game_theory_mode = 0
        "Torque profile selection: please input OP, plantar, dorsi or udp.
         Hit enter for zero torque and IGNORE UDP input:"

        OP       -> keep the SD card timings unchanged, both lobes live
        plantar  -> ask peak plantarflexion timing.  in range -> dorsi_nm = 0 (disable the other lobe)
                                                     out of range -> both magnitudes 0, zero_torque_mode = 1
        dorsi    -> ask peak dorsiflexion timing.    in range -> plantar_nm = 0
                                                     out of range -> both magnitudes 0, zero_torque_mode = 1
        udp      -> udp_in_use = 1
                    ask which lobe the incoming timing should move (plantar / dorsi), disable the other
                    hold at zero torque until a timing value actually arrives
        ""       -> both magnitudes 0, zero_torque_mode = 1
        anything else -> "Invalid input. Please restart and retry", exit

ask body mass (enter -> 75, scaling factor 1)

IF zero_torque_mode:  torque percentage = 0, skip the prompt
                      # both lobes are already 0 Nm, so a percentage would scale nothing. Holding the
                      # scale at 0 too means the controller reports what the user actually asked for
                      # rather than "60% of nothing"
ELIF not game_theory_mode:
    "If familiarization session, input percentage of peak torque. Else, hit enter for maximum peak torque:"
```

### 3.2 Session setup — the one destructive step, gated on a human

```
print loudly that this CHANGES THE ACTIVE CONTROLLER on both ankles
input("press enter to engage, Ctrl-C to abort")

engage_controller_safely()                                  # TorqScale = 0 first. Always.
apply_torque_magnitudes(plantar_nm, dorsi_nm, body_mass)    # a 0 here disables that lobe
if this mode chose a peak timing: apply_peak_timing(lobe, value)

on any failure: say nothing further will be sent, and exit

input("press enter to start running the exo")
start_time = perf_counter()

new_timing_value = None            # the only loop state EVERY mode needs

IF udp_in_use:                                  # each mode declares its own loop state HERE, next to
    udp_receiver = UDPReceiver(port=5003)       # the thing that feeds it, rather than all of it in one
    udp_receiver.start_listening()              # block at the top of the file
    pending_udp_timing        = None
    last_udp_write_time       = 0
    working_torque_percentage = new_torque_percentage   # what we restore to when assistance resumes
    new_torque_percentage     = None                    # hold at zero until a timing actually arrives

IF game_theory_mode:
    treadmill_receiver = UDPReceiver(port=5004, fmt='f'); start_listening()
    backend = GameTheory_1D_Backend(..., _h_action_max_stride_count=N); backend.start()
    prev_human_action_time       = start_time
    start_human_action_recording = 0
    _h_action_samples            = []
    _h_action_max_stride_count   = backend._h_action_max_stride_count
        # read N back OFF the backend, the way the hip exo does, so there is one source of truth and
        # the backend's own final log matches what we actually collected
    new_torque_percentage        = None    # the backend owns this from here on
```

### 3.3 The loop

```
TRY:
  LOOP:
    loop_start_time = perf_counter()

    exo_link.pump()
    if exo_link.link_down: say so and BREAK

    IF game theory mode:

        if backend.finished: say so and BREAK

        # --- collect the human action ---
        # The treadmill sends exactly ONE speed per step, so every successful read here IS one stride.
        # That is why this needs no heel strike gate and no stride detection of its own: the hip exo
        # checked HS_count before fetching, but with one packet per step that check only told us what
        # the packet itself already told us.
        if now - prev_human_action_time > backend.human_lag_time:
            if not recording: samples = []; recording = 1
                              say "Start collecting <N> strides data for human action!"

            speed = treadmill_receiver.get_latest_timing_value()
            if speed is not None AND speed > 0:
                samples.append(speed)                       # only count it if we succeeded
                say "Collected <len(samples)> data point!"
            # the > 0 guard matters: UDPReceiver maps any non-positive payload to -1, a leftover from
            # its timing-value use case. Averaging a -1 corrupts the human action.

            if len(samples) >= _h_action_max_stride_count:   # did we log enough steps?
                backend.read_human_action(mean(samples), now)
                log it; prev_human_action_time = now; samples = []; recording = 0

        # --- fetch a finished machine action ---
        if backend.output_ready:
            new_torque_percentage = backend.m_action_next
            backend.output_fetched = True

    ELSE IF udp_in_use:

        incoming = udp_receiver.get_latest_timing_value()
        if incoming is not None:
            if incoming > 0:
                pending_udp_timing = incoming
                # HOLD it rather than acting now. If the rate limiter below is not ready we KEEP the
                # value instead of dropping it, and a fresher one simply replaces it - what we want is
                # the latest commanded state, not a replay of every value the sender ever produced.
            else:
                # "stop assisting" is a SAFETY command: straight through, no rate limiting, and no
                # chance of a later timing coalescing it away while it sits pending
                say "Received command from UDP to temporarily disable assistance"
                pending_udp_timing = None
                peak_timing_value  = incoming
                new_torque_percentage = 0

        if pending_udp_timing is not None AND now - last_udp_write_time >= udp_min_write_interval:
            if pending_udp_timing != peak_timing_value:      # same dedupe as the hip exo
                peak_timing_value   = pending_udp_timing
                last_udp_write_time = now
                new_timing_value    = peak_timing_value
                if action_map.last_applied_action == 0:      # we were parked by an earlier negative
                    new_torque_percentage = working_torque_percentage
            pending_udp_timing = None
        # Why rate limit at all: on the hip exo a UDP update was just a local variable. Here every one
        # costs a BLE round trip, so a chatty sender must not queue writes faster than the link can
        # retire them. udp_min_write_interval = 0.5 s.

    # --- deploy a new peak timing ---
    if new_timing_value is not None:
        try apply_peak_timing(timing_lobe, new_timing_value)
        on ExoLinkError: say the two legs may now differ, and BREAK
        log it; new_timing_value = None

    # --- deploy a new torque percentage ---
    if new_torque_percentage is not None:
        try applied = apply_machine_action(new_torque_percentage)
        on ExoLinkError: say the two legs may now differ, and BREAK
        log (requested, applied); new_torque_percentage = None

    sleep out the remainder of 1/operating_rate

EXCEPT KeyboardInterrupt:
    say we are stopping and parking

FINALLY:
    park_to_transparency()          # FIRST, while the link is most likely still alive
    if game_theory_mode: backend.stop_and_finalize()   # writes the backend's own final log
                         treadmill_receiver.stop()
    if udp_in_use:       udp_receiver.stop()
    # guard on the MODE FLAGS, the way the hip exo does - not on None sentinels. Anything a mode
    # created is guaranteed to exist whenever that mode's flag is set
    exo_link.close(); close the log files
    remind the operator that this code never started the trial and cannot end it
    offer to rename the log folder, auto-suffixing if the name is taken
```

---

## 4. What is deliberately absent

Worth knowing so none of it gets "helpfully" added back:

- **No stride detection.** Not because strides do not matter, but because the treadmill already sends
  one packet per step — the stride signal arrives from outside the exo, fully formed. If stride
  *duration* is ever wanted, the Teensy computes it and sends it up; this code never derives an
  observation from raw telemetry.
- **No `rt` subscription.** Telemetry is opt-in per subscriber and nothing here needs it.
- **No background thread, no second socket.** At these rates a single blocking socket is fine. The
  hazard was never throughput, it was trap 3, and that is a correctness fix, not a concurrency one.
- **No trial start/stop, motor enable, or calibration.** Not exposed over UDP, by design. The operator
  drives the GUI.
