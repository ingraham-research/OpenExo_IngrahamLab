# Pseudocode — external control code for the OpenExo ankle

This is the whole design written out as pseudocode, so it can be hand-typed rather than copied. It
follows the four real files exactly:

| Pseudocode section | File |
|---|---|
| 1. `OpenExoLink` | `Utilities/OpenExoLink_utilities.py` |
| 1b. `WriteScheduler` | `Utilities/WriteScheduler_utilities.py` |
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

1. **Silence is the common failure signal — and it almost always means the ACK was lost, not the
   write.** A dropped BLE write produces no negative acknowledgement. The GUI's `ok` reply only means
   it *queued* a write — it returns `ok` with no exo connected at all. Only the exo's ack proves the
   value was stored. What the diagnostic firmware settled (2026-09-16/17/18) is what a silent write
   actually is: about 17 % of writes go unanswered, and the ack counters put nearly all of those on the
   Nano → PC hop, i.e. the Teensy had applied the value. That is why nothing here exits on silence.
2. **Acks carry no request id.** They carry `joint_id`, `controller_id`, `param_index` and nothing
   else. So you must resolve names to numbers *yourself* and match on that triple.
3. **A fast ack gets swallowed.** `ExoRemote._command` drains stream frames while waiting for its own
   `ok`, so an ack arriving in that window is never re-delivered by `stream()` — and a status frame
   read there would hide a disconnect. Fixed at the source since V0.2: `_AckTrackingRemote` overrides
   `_absorb`, so every ack and status frame reaches the scheduler the moment it is read, whichever
   receive call read it. The old "snapshot `last_ack()` before every write" dance is gone.
4. **Writing a parameter to a non-active controller switches the joint to it and loads its SD-card
   defaults.** `splineAlt.csv` ships `TorqScale = 95`. So the first write must be `TorqScale = 0`, or
   the leg starts assisting at 95 % the instant you touch it.
5. **`bilateral=True` acks twice and only the last is kept.** Send two unilateral writes instead —
   identical on the wire, unambiguous to match.

---

## 1. `OpenExoLink` — names in, acknowledged writes out

```
CONSTANTS
    LEFT_ANKLE_JOINT_ID  = 68        # ExoCode/src/ParseIni.h: left = 0b01000000
    RIGHT_ANKLE_JOINT_ID = 36        #                          right = 0b00100000
    # NEVER use Python_GUI/utils/config.py JointConfig.ID_TO_NUM - all 8 entries have L/R inverted
    DEFAULT_ACK_TIMEOUT = 1.0        # an unanswered write is a FAILED ATTEMPT, not a failure
    DEFAULT_MAX_RETRIES = 5          # only the stress test uses it. Setup passes None = never give up
    ACK_POLL_INTERVAL   = 0.05       # how long a blocking wait spends inside each service() call

CLASS OpenExoLink(host, port, timeout, ack_timeout, max_retries, verbose, logfile, action_logfile)

    __init__:
        put ../../remote on sys.path, import ExoRemote and RemoteError from client
        self.exo       = _AckTrackingRemote(host, port, timeout, on_ack=..., on_status=...)
            # a subclass of ExoRemote that overrides _absorb, so EVERY ack and status frame reaches us
            # the moment it is read, whichever receive call read it. That is trap 3, fixed at the source
        self.scheduler = WriteScheduler(ack_timeout, reporter)      # section 1b
        self.matrix    = []         # the handshake controller matrix
        self.link_down = 0          # set by _on_status when the GUI says the exo went away


    ### Connection ###

    connect(matrix_timeout):
        exo.ping()                      -> if it raises, the GUI is not running: raise OpenExoLinkError
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
        if joint_rows is empty: raise OpenExoLinkError
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


    ### The write - two ways in, one engine ###

    request(address, value, label, requested, stamp, force) -> Command or None:
        # The experiment loop's way. Returns AT ONCE - nothing is sent here
        hand it to the scheduler, stamped on the caller's perf_counter() clock
        returns None if `value` just repeats the latest request for this address (unless force)
        # Its fate - accepted / superseded (no ACK) / replaced before sending / rejected /
        # unconfirmed at exit - goes in the machine action log once it is known, one row per command

    service(budget, until):
        # Called once per loop pass INSTEAD of sleeping, with whatever idle time is left over
        loop:
            drain every frame already waiting            # acks and statuses arrive via _absorb
            scheduler.poll(now)                          # has the on-air command timed out?
            if not link_down: send the next command the scheduler offers
            scheduler.emit_warnings(now)
            if until() is true: return                   # blocking waits return AT the ack
            wait on the socket for the shortest of: the budget left, the on-air timeout,
                                                    the next warning tick
        # so an ack is acted on the moment it arrives and the next write goes straight out, and the
        # loop spends its idle time listening rather than sleeping through it

    set_param_confirmed(address, value, label, max_retries) -> ack:
        # Session setup and the stress test only. BLOCKS. The experiment loop uses request()
        cmd = scheduler.request(..., max_attempts=max_retries, force=True, log_action=False)
        wait_for([cmd])
        accepted    -> return the ack
        gui_refused -> raise OpenExoLinkError(code=...)    # the GUI refused it: resending cannot help
        gave_up     -> raise OpenExoLinkError             # only reachable with a CAPPED max_retries
        rejected    -> raise OpenExoLinkError(reason=...) # the firmware got it and said no
        # max_retries=None means NEVER give up, and that is what session setup passes
        # KNOWN LIMIT: an ack slower than ack_timeout, arriving after the same parameter was sent
        # again, confirms the RESEND. None slower than 0.57 s has ever been seen. The real fix is the
        # firmware echoing the value back in the ack

    wait_for(commands):
        until every one of them is resolved: service(ACK_POLL_INTERVAL, until=all_resolved)
        if link_down meanwhile: raise OpenExoLinkError   # nothing can reach the exo any more
        # Ctrl-C propagates to the caller - that is how a second Ctrl-C abandons the park

    latest(address) / confirmed(address):
        the last value REQUESTED for this address / the last value the exo ACKNOWLEDGED, or None

    begin_exit(keep):
        from now on only the `keep` addresses may send. Anything else still waiting is dropped and
        logged "unconfirmed at exit"; a command already on the air elsewhere gets its ack or its
        timeout first. This is how park_to_transparency makes sure ONLY the park goes out

    pump():
        service(0.0) - one non-blocking pass. Kept for stress_test_ble.py


    ### Link health ###

    _on_status(msg):
        "disconnected" -> link_down = 1, and say so. The GUI lost the exo, and the Nano cannot be
                          reconnected mid-trial without side effects, so the caller should stop
        "connected"    -> link_down = 0
        "device_error" -> DELIBERATELY IGNORED. The write it hit gets no ack, and the ack check
                          already reports that. Two warnings for one event is noise
        # Honest limit: the GUI reports a disconnect only once BLE gives up, which for the Nano
        # radio-silence failure is ~9.6 s after the exo actually went quiet. A backstop, not a watchdog

    close():
        unsubscribe and close the socket, both wrapped so a dead peer cannot raise on shutdown
```

---

## 1b. `WriteScheduler` — one command on the air

A pure state machine: no sockets, no sleeping, no clock of its own. Every method that cares about time
takes `now` from its caller, which is what lets the tests drive the whole thing with a fake clock.

**Why only one command on the air.** An ack carries `(joint, controller, parameter, accepted, reason)`
and nothing else — no value, no sequence number (trap 2). With exactly one command out, an ack can only
be that command's. The one time two were ever out at once — the GUI's July/August bilateral writes — is
also the only time an ack ever took longer than 1 s.

**Why a newer value replaces a resend instead of queueing behind it.** On the exo, 250 silent attempts
turned out to be acks the Nano had sent and the PC never received, against 1 command that may not have
arrived. So a resend is almost always a value the exo already holds, and sending it only delays the
fresh one.

```
STATE
    _on_air         the one command currently out, or none
    one _slot per address, holding:
        wanted      the command waiting to go out for this address - at most one. A newer value
                    OVERWRITES it, keeping the older one's place in line
        tab         seconds THIS address has spent on the air unanswered since its last ack
        fails       failed attempts since its last ack

    request(address, value, ...):
        if value repeats the latest request for this address and not force: return None, send nothing
        if the slot already holds one waiting: the new value REPLACES it, logged
           "replaced before sending"
        remember it as this address's wanted command

    next_send(now) -> the command to send now, or None:
        nothing while a command is on the air. Otherwise round-robin over the addresses that have one
        waiting, so a busy address cannot starve the other leg

    on_ack(ack, now):
        does it answer the on-air command's address? (trap 2: match on the triple, nothing else)
            garbled (reason code 1) -> a FAILED ATTEMPT, not a refusal. The Nano could not parse the
                Teensy's ack, so the Teensy may well have applied the value: unknown, not no.
                A garbled ack keeps only the fields parsed before the damage, so a 0 there means
                "unknown" and matches anything
            accepted -> resolve it, and clear this address's tab and fail count
            rejected -> resolve it as rejected. Resending the identical value gets the identical answer
        an ack that answers nothing on the air is logged and dropped (a late ack for a command we
        already wrote off)

    poll(now) -> _fail(result):                          # the timeout path
        add at most ONE ack_timeout to this address's tab, count the failed attempt, then:
            exiting, and this is not a kept address   -> resolve "unconfirmed at exit"
            a capped write out of attempts            -> resolve "gave_up"  (setup passes no cap)
            a NEWER value is waiting for this address -> resolve the old one "superseded (no ACK)"
                                                         and let the newer value go instead
            otherwise                                 -> put it back as this address's wanted command,
                                                         i.e. a plain resend

    emit_warnings(now):
        while an address that still has something to deliver has a tab >= 5 s, print one line per
        second: "no ACK for <label> for <n> s (<k> failed attempts), wanted <value>"
        # The tab pauses while the address is off the air, survives a newer value replacing an older
        # one, and clears ONLY on an ack. NOTHING here exits or parks - a human is always watching the
        # terminal. For scale: the worst run of consecutive failures in ~2,100 worn writes was 4

    begin_exit(keep) / abandon_all():
        the shutdown path - stop sending anything but the park, then give up waiting on a second Ctrl-C
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

CLASS SplineAlt_action_map(exo_link, body_mass_standard, joints, controller,
                           torque_percentage_max, verbose)

    __init__:
        resolve EVERY address up front - TorqScale, PlantarNm, DorsiNm, and BOTH peak times, per joint
        # so a name typo fails here at setup, not half way through an experiment

    engage_controller_safely():
        write TorqScale = 0 to both legs, FIRST, before anything else - blocking, and NEVER giving up
        (set_param_confirmed with max_retries=None), because nothing else may be written until both
        joints are actually on splineAlt at zero
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

    apply_peak_timing(lobe, percent_gait, stamp, wait):     # lobe is "plantar" or "dorsi"
        write PlantarPk (or DorsiPk) = percent_gait to both legs
        wait=True  -> session setup: BLOCK until both legs acked, never giving up
        wait=False -> the loop: REQUEST it and return at once. service() delivers it
        # This is what a UDP timing value drives. It is the ankle equivalent of the hip exo's
        # create_flexion_only_torque_profile(udp_timing_value) - except that there, a new timing meant
        # rebuilding the whole spline in Python, and here it is ONE parameter write per leg. The
        # Teensy rebuilds its own nodes from it.

    # NOTE: there is deliberately NO generic "write any other shape parameter" method. The
    # rise/dwell/fall durations come off the SD card and are not written from here. If one is ever
    # added: a rise or fall duration of 0 is NOT safe and is not repaired by the firmware - it trips
    # the interpolator's monotonicity guard and zeroes the whole profile. A dwell of 0 is fine and is
    # the normal default.

    apply_torque_percentage(scale, stamp) -> applied:
        clamped = clamp(scale, 0, torque_percentage_max)   # our own limit, on top of the firmware's
        REQUEST TorqScale = clamped on both legs, and return at once     # request(), not a blocking write
        return clamped                                     # log THIS, not the caller's own number
        # The floor at 0 matters as much as the cap: a negative scale would flip both lobes.
        # Nothing raises here any more. The link delivers both legs - resending, or superseding with a
        # newer value - and warns if it cannot. A half-applied action is now a warning on the terminal
        # and a "superseded (no ACK)" row in the machine action log, not a reason to end the session.

    latest_torque_request() -> value or None:
        the torque percentage most recently REQUESTED on both legs, if they agree, else None
        # REQUESTED, not confirmed: once zero has been asked for, the link keeps delivering it, so this
        # is the right test for "do I need to ask for zero again?" before spending a BLE round trip

    park_to_transparency() -> 1 if both legs acknowledged it:
        begin_exit(the two TorqScale addresses)   # everything else still waiting is dropped and logged
                                                  # "unconfirmed at exit". Only the park goes out
        request TorqScale = 0 on both legs, force=True     # force: send it even if 0 was the last
                                                           # value requested
        wait_for(those two commands)              # however many attempts that takes
        on a second Ctrl-C, on any exception, or on any result that is not "accepted": say plainly that
        the exo is STILL RUNNING whatever it had last and that the operator must end the trial from the
        GUI, and return 0. Never pretend it was parked.
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
if this mode chose a peak timing: apply_peak_timing(lobe, value, wait=True)   # setup: block

on any failure: say nothing further will be sent, and exit

input("press enter to start running the exo")
start_time = perf_counter()
exo_link.elapsed_origin = start_time        # the machine action log's Elapsed column counts from here

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

    # The GUI reporting the exo disconnected is the ONE thing that ends this loop on its own: the
    # Nano cannot be reconnected mid-trial without side effects. ACK trouble NEVER ends it - the link
    # prints warnings instead. link_down is kept current by the service() call at the bottom
    if exo_link.link_down: say so, link_lost = 1, and BREAK

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
                pending_udp_zero   = False
                # HOLD it rather than acting now. If the rate limiter below is not ready we KEEP the
                # value instead of dropping it, and a fresher one simply replaces it - what we want is
                # the latest commanded state, not a replay of every value the sender ever produced.
                # A real timing also supersedes an UNSENT disable: without that, a queued zero would
                # fire in the same pass that restores assistance, and the two would fight over
                # new_torque_percentage.
            else:
                pending_udp_timing = None     # a stale timing must not outlive a disable command
                peak_timing_value  = incoming
                pending_udp_zero   = True     # ask for zero, but do NOT write it here. The two guards
                                              # below decide whether it is worth a BLE round trip.
                                              # This used to write unconditionally on every loop pass,
                                              # which is how a sender stuck at 200 Hz turned into
                                              # 20 writes/s of a value the exo was already holding

        # A zero-torque request has to clear two guards before it costs a BLE round trip
        if pending_udp_zero:
            if action_map.latest_torque_request() == 0:
                pending_udp_zero = False          # 1) ALREADY ZERO on both legs. Nothing to send: once
                                                  #    zero is requested the link delivers it, so asking
                                                  #    again adds nothing. None (legs disagreeing, or a
                                                  #    half-made zero) falls through to the write
            else if now - last_udp_write_time >= udp_min_write_interval:
                say "Received command from UDP to temporarily disable assistance"
                new_torque_percentage = 0         # 2) RATE LIMIT, the same 0.5 s budget as a timing
                last_udp_write_time   = now       #    update, because it costs the link exactly the
                pending_udp_zero      = False     #    same. The request is HELD rather than dropped,
                                                  #    so a disable is never lost - it just waits its
                                                  #    turn, at most udp_min_write_interval

        if pending_udp_timing is not None AND now - last_udp_write_time >= udp_min_write_interval:
            if pending_udp_timing != peak_timing_value:      # same dedupe as the hip exo
                peak_timing_value   = pending_udp_timing
                last_udp_write_time = now
                new_timing_value    = peak_timing_value
                if action_map.latest_torque_request() == 0:  # we were parked by an earlier negative
                    new_torque_percentage = working_torque_percentage
            pending_udp_timing = None
        # Why rate limit at all: on the hip exo a UDP update was just a local variable. Here every one
        # costs a BLE round trip, so a chatty sender must not queue writes faster than the link can
        # retire them. udp_min_write_interval = 0.5 s.

    # --- deploy a new peak timing ---
    if new_timing_value is not None:
        apply_peak_timing(timing_lobe, new_timing_value, stamp=now)
        new_timing_value = None
        # This only REQUESTS it. service() below sends it, resends it if its ack goes missing, and
        # lets a newer timing replace a pending resend. Its fate goes in the machine action log

    # --- deploy a new torque percentage ---
    if new_torque_percentage is not None:
        applied = apply_torque_percentage(new_torque_percentage, stamp=now)
        new_torque_percentage = None
        # Same: requested here, delivered by service(). NEITHER of these can raise any more, so
        # neither of them can end the loop - that is the V0.3 rule, never destructive on ack trouble

    # --- end of the pass ---
    exo_link.service(budget = max(1/operating_rate - how long this pass took, 0))
    # The idle time is spent INSIDE service(), waiting on the socket instead of sleeping, so an ack is
    # acted on the moment it arrives and the next write goes straight out

EXCEPT KeyboardInterrupt:
    say we are stopping and parking

FINALLY:
    if link_lost: skip the park and say so - nothing can reach the exo, and the park would only retry
                  forever. It keeps whatever it last accepted; the operator ends the trial from the GUI
    else:         park_to_transparency()    # FIRST, while the link is most likely still alive
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
- **No background thread, no second socket.** At these rates one socket is fine. The hazard was never
  throughput, it was trap 3, and that is a correctness fix, not a concurrency one. V0.3 did not change
  that: `service()` simply blocks on the same socket with a computed timeout instead of sleeping.
- **No giving up, and no automatic park, on ACK trouble.** Warnings only — a human is always watching
  this terminal, and a silent write is nearly always one the exo applied. The ONLY thing that ends the
  loop by itself is the GUI reporting the exo disconnected.
- **No trial start/stop, motor enable, or calibration.** Not exposed over UDP, by design. The operator
  drives the GUI.
