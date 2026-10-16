#This utility wraps the OpenExo GUI's UDP remote client into something an external "main" control code
#can use. The GUI already owns the exo: it holds the BLE link, it holds the handshake controller matrix,
#and it is the only thing allowed to start a trial. All this utility does is push controller PARAMETER
#values into it and keep track of whether the exo acknowledged them.
#
#Things here that are not obvious, and are the reason this file exists at all:
#   1) The firmware acknowledges a parameter write with NUMERIC ids only (joint id, controller id,
#      parameter index) - no request id, no value. So to know which write an ack answers, we resolve names to
#      numbers ourselves, match on that triple, and keep ONE command on the air at a time (WriteScheduler_utilities.py).
#   2) SILENCE IS THE COMMON FAILURE SIGNAL, and it almost never means the command was lost: on the exo nearly every
#      silent attempt was an ack lost on its way back (2026-09-16/17). The GUI's "ok" to a set_param only means it
#      queued a BLE write, even with no exo connected. Only the ack means the firmware stored the value.
#   3) Some acks arrive GARBLED. When the Nano cannot parse the Teensy's UART ack it sends up a fake
#      "rejected, reason 1 (invalid message)" instead, keeping only the fields it parsed before the damage
#      (ExoCode/src/ComsMCU.cpp _send_param_update_ack). The Teensy may well have applied the value, so such an
#      ack means "unknown", not "refused": it counts as a failed attempt. See "Modification log with claude/ACK-Loss-Investigation.md".
#   4) Two ways to write, one engine. request() is the experiment loop's: it returns at once, and service() - called
#      every loop pass instead of sleeping - sends, retries and lets a newer value replace a pending resend.
#      set_param_confirmed() blocks until the ack, for session setup and the stress test. KNOWN LIMIT: an ack slower
#      than ack_timeout, arriving after the same parameter was sent again, confirms the re-send. None slower than
#      0.57 s has been seen (2026-09-17). The real fix is echoing the value in the ack (firmware). See
#      "Modification log with claude/specs/2026-09-17-ack-aware-write-scheduler-design.md".
#V0.1 2026 Sep
#V0.2 2026 Sep: garbled-ack resend, per-attempt logging, attempt bookkeeping, 1 s / 5 attempt defaults
#V0.3 2026 Sep 18: one command on the air, non-blocking request()/service(), newer values replace resends, no-ack
#   warnings instead of giving up, machine action log per command, status frames caught during set_param replies

import os
import socket
import sys
import time

#The remote client lives with the GUI, not in this folder. Same path trick as Python_GUI/examples/remote_console.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "remote"))
from client import ExoRemote, RemoteError  # noqa: E402

#The scheduler sits next to this file. Imported by path as well, because this file is itself loaded by path in the tests
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from WriteScheduler_utilities import WriteScheduler  # noqa: E402

#Ankle joint ids, from ExoCode/src/ParseIni.h:125-137 (left = 0b01000000, right = 0b00100000).
#WARNING: do NOT use Python_GUI/utils/config.py JointConfig.ID_TO_NUM for this - all 8 of its entries
#have their left/right labels inverted, and it would silently command the wrong leg. The handshake
#matrix (and these two constants) are the real source of truth.
LEFT_ANKLE_JOINT_ID = 68
RIGHT_ANKLE_JOINT_ID = 36

#How long a command waits for its ack before it counts as a failed attempt. Every ack seen on the exo up to
#2026-09-17 came within 0.57 s (the slowest when the Nano's outgoing notification queue was backed up)
DEFAULT_ACK_TIMEOUT = 1.0
#Attempts before set_param_confirmed gives up, when the caller does not say. Only stress_test_ble.py relies on it:
#session setup passes max_retries=None, and the experiment loop's request() never gives up
DEFAULT_MAX_RETRIES = 5

#How long the blocking waits (set_param_confirmed, wait_for) spend in each service() call
ACK_POLL_INTERVAL = 0.05

#"max_retries not given" must differ from max_retries=None, which means "never give up"
_USE_DEFAULT = object()


class OpenExoLinkError(Exception):
    """A parameter write did not make it to the exo, or was refused by the GUI or the firmware."""

    def __init__(self, message, code=None, reason=None):
        super().__init__(message)
        self.code = code        #The GUI's machine-readable rejection code, if it was the GUI that refused
        self.reason = reason    #The firmware's rejection reason string, if it was the exo that refused


class _AckTrackingRemote(ExoRemote):
    """ExoRemote that hands EVERY ack and status frame to a callback the moment it is read, whichever receive call read
    it (set_param's own wait for the GUI's ok reply, or our polling). The stock client keeps only the most recent ack,
    and drops status frames read during that reply wait - which would hide a disconnect."""

    def __init__(self, *args, on_ack=None, on_status=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.on_ack = on_ack
        self.on_status = on_status

    def _absorb(self, msg):
        super()._absorb(msg)
        stream = msg.get("stream")
        if stream == "ack" and self.on_ack is not None:
            self.on_ack(msg)
        elif stream == "status" and self.on_status is not None:
            self.on_status(msg)


class _LinkReporter:
    """Turns what the scheduler reports into log rows and terminal lines"""

    def __init__(self, link):
        self.link = link

    def attempt(self, cmd, result):
        '''One Param_write_log row per ATTEMPT. Result is one of: accepted, no_ack, garbled_ack, gui_no_reply,
        rejected (<reason>), gui_refused, or gave_up (which closes a capped write whose every attempt failed, and
        repeats the last attempt number). Filtering on "accepted" gives one row per confirmed write'''
        if self.link.logfile is not None:
            result = str(result).replace(",", ";")   #The log is comma separated
            print(f"{time.time()}, {cmd.label}, {cmd.value}, {result}, {cmd.attempts}", file=self.link.logfile)

    def command(self, cmd):
        '''One Machine_action_log row per COMMAND, once its fate is known. Python time is when it was REQUESTED.
        Result is one of: accepted, superseded (no ACK), replaced before sending, rejected (<reason>), gui_refused,
        unconfirmed at exit. A superseded command was almost certainly applied - the label says what we observed'''
        logfile = self.link.action_logfile
        if logfile is None:
            return
        stamp = "" if cmd.stamp is None else cmd.stamp
        origin = self.link.elapsed_origin
        elapsed = "" if (origin is None or cmd.stamp is None) else f"{cmd.stamp - origin:.2f}"
        result = str(cmd.result).replace(",", ";")
        print(f"{stamp},{elapsed},{cmd.label},{cmd.requested},{cmd.value},{result},{cmd.attempts}", file=logfile)

    def say(self, line, always=False):
        if always or self.link.verbose:
            print(line)


class OpenExoLink:
    """This class owns the UDP connection to the GUI and turns "set this parameter to this value" into writes the
    exo acknowledges. It resolves human-readable names into the numeric ids the firmware acks with, and runs the
    one-command-on-the-air WriteScheduler over the GUI socket.

    Inputs:
    host, port: where the GUI's remote service is listening. Localhost only by default (see
        Python_GUI/utils/config.py RemoteConfig)
    timeout: how long we wait for the GUI's own ok/error reply to a command
    ack_timeout: how long a command waits for the EXO's acknowledgement before it counts as a failed attempt
    max_retries: attempts before set_param_confirmed gives up, when its caller does not say (None = never)
    verbose: if 1, print every send and every accepted ack. Failures and warnings always print
    logfile: an already-open, line-buffered file for the per-ATTEMPT write log (Param_write_log). Optional
    action_logfile: an already-open, line-buffered file for the per-COMMAND machine action log. Optional. Only
        request() writes go there. Set .elapsed_origin to the loop's start perf_counter() for its Elapsed column
    """

    def __init__(self, host="127.0.0.1", port=9750, timeout=2.0, ack_timeout=DEFAULT_ACK_TIMEOUT,
                 max_retries=DEFAULT_MAX_RETRIES, verbose=1, logfile=None, action_logfile=None):
        self.host = host
        self.port = port
        self.ack_timeout = ack_timeout
        self.max_retries = max_retries
        self.verbose = verbose
        self.logfile = logfile
        self.action_logfile = action_logfile
        self.elapsed_origin = None      #perf_counter() the experiment loop started at. The machine log's Elapsed counts from it

        #How long set_param waits for the GUI's ok. Restored before every send, because service() keeps changing the
        #socket timeout to wait for acks
        self._reply_timeout = timeout
        self.exo = _AckTrackingRemote(host, port, timeout=timeout, on_ack=self._on_ack, on_status=self._on_status)
        self.matrix = []            #The handshake controller matrix, once the GUI has one
        self.connected = 0          #Whether we have seen a live GUI and a non-empty matrix
        self.link_down = 0          #Set when the GUI reports the exo disconnected
        self.last_status = None     #Most recent status frame seen, for diagnostics

        self.scheduler = WriteScheduler(ack_timeout, reporter=_LinkReporter(self))

    ############################### Connection and handshake #############################################

    def connect(self, matrix_timeout=10.0):
        '''Check the GUI is listening, subscribe to the streams we care about, and wait for the exo
        handshake matrix to exist. Returns the matrix (empty list if it never arrived).

        We deliberately subscribe to ack and status ONLY. The rt stream is opt-in per subscriber and
        we have no use for it - this code does no signal processing, it only writes parameters.'''
        try:
            self.exo.ping()
        except RemoteError as e:
            raise OpenExoLinkError(f"No reply from the GUI at udp://{self.host}:{self.port}. Is it running? ({e})")
        if self.verbose:
            print(f"Connected to the GUI remote service at udp://{self.host}:{self.port}")

        self.exo.subscribe(["ack", "status"])

        #The matrix only exists once the GUI has completed a BLE handshake with an exo
        deadline = time.time() + matrix_timeout
        while time.time() < deadline:
            self.matrix = self.exo.get_matrix()
            if self.matrix:
                break
            time.sleep(0.5)

        if self.matrix:
            self.connected = 1
            if self.verbose:
                print(f"Controller matrix received: {len(self.matrix)} rows")
        else:
            print("WARNING: no controller matrix yet - the GUI is running but is not connected to an exo.")
        return self.matrix

    def refresh_matrix(self):
        '''Re-fetch the controller matrix. Worth calling after a reconnection, since a new BLE session
        rebuilds it from scratch'''
        self.matrix = self.exo.get_matrix()
        return self.matrix

    def print_matrix(self):
        '''Print every joint, controller and parameter the exo advertised. Use this to confirm the exact
        names and indices before trusting anything below'''
        if not self.matrix:
            print("  (matrix empty - the GUI is not connected to an exo yet)")
            return
        last_joint = None
        for row in self.matrix:
            if len(row) < 4:
                continue
            jname = _joint_display_name(row)
            if jname != last_joint:
                print(f"\n  {jname}  (id {row[1]})")
                last_joint = jname
            params = ", ".join(f"[{i}] {p}" for i, p in enumerate(row[4:])) or "(no params)"
            print(f"      {row[2]:<14} (id {row[3]}): {params}")
        print()

    ############################### Name to id resolution #############################################

    def resolve(self, joint, controller, param):
        '''Turn a (joint, controller, parameter) triple into the numeric (joint_id, controller_id,
        param_index) address the firmware acks with. Any of the three may be given as a name or as an
        integer id/index; integers pass straight through.

        We resolve here rather than letting the GUI do it, because the GUI does not tell us which ids it
        picked - and without them we cannot match the ack that comes back.'''
        joint_id = self._resolve_joint(joint)
        joint_rows = [row for row in self.matrix if len(row) >= 4 and str(row[1]) == str(joint_id)]

        #Integer ids work even with no matrix at all, which is how you drive a disconnected GUI for testing
        if _is_int(controller) and _is_int(param):
            return (int(joint_id), int(controller), int(param))

        if not joint_rows:
            raise OpenExoLinkError(f"Joint {joint!r} (id {joint_id}) has no controllers in the matrix. "
                               f"Either the exo is not connected, or use integer ids instead of names.")

        controller_row = self._resolve_controller_row(controller, joint_rows)
        controller_id = int(controller_row[3])
        param_index = self._resolve_param_index(param, controller_row)
        return (int(joint_id), controller_id, param_index)

    def _resolve_joint(self, joint):
        if _is_int(joint):
            return int(joint)
        target = str(joint).strip().lower()
        names = []
        for row in self.matrix:
            if len(row) < 2:
                continue
            name = _joint_display_name(row)
            names.append(name)
            if name.strip().lower() == target or str(row[1]) == str(joint).strip():
                return int(row[1])
        raise OpenExoLinkError(f"Unknown joint {joint!r}; the exo advertised: {sorted(set(names))}")

    def _resolve_controller_row(self, controller, joint_rows):
        if _is_int(controller):
            for row in joint_rows:
                if int(row[3]) == int(controller):
                    return row
            raise OpenExoLinkError(f"Controller id {controller} is not present on this joint")
        target = str(controller).strip().lower()
        names = [str(row[2]) for row in joint_rows]
        for row in joint_rows:
            if str(row[2]).strip().lower() == target:
                return row
        #The BLE handshake truncates long names, so fall back to a prefix match before giving up
        for row in joint_rows:
            if target.startswith(str(row[2]).strip().lower()):
                return row
        raise OpenExoLinkError(f"Unknown controller {controller!r}; this joint advertised: {names}")

    def _resolve_param_index(self, param, controller_row):
        params = controller_row[4:] if len(controller_row) > 4 else []
        if _is_int(param):
            if 0 <= int(param) < len(params):
                return int(param)
            raise OpenExoLinkError(f"Parameter index {param} is out of range 0..{len(params) - 1} "
                               f"for controller {controller_row[2]!r}")
        target = str(param).strip().lower()
        for i, name in enumerate(params):
            if str(name).strip().lower() == target:
                return i
        #Same truncation problem as controller names
        for i, name in enumerate(params):
            if target.startswith(str(name).strip().lower()):
                return i
        raise OpenExoLinkError(f"Unknown parameter {param!r}; controller {controller_row[2]!r} advertised: {list(params)}")

    ############################### Writing parameters #############################################

    def request(self, address, value, label="", requested=None, stamp=None, force=False):
        '''Ask for one parameter to hold `value`, WITHOUT waiting. The experiment loop's way to write.

        address: the (joint_id, controller_id, param_index) tuple from resolve()
        value: the value to write
        label: a human-readable name for terminal and log lines
        requested: the value before any clamp, for the machine action log only (defaults to value)
        stamp: when it was requested, on the caller's time.perf_counter() clock (defaults to now)
        force: send even if `value` repeats the latest request for this address

        It goes out on a later service() call: one command on the air at a time, resent until acked, and replaced by a
        newer value for the same address if one arrives first. Its fate goes in the machine action log.
        Returns the Command (its .result stays None until resolved), or None if nothing needs sending.'''
        address = _as_address(address)
        return self.scheduler.request(address, value, label or _tag(address), requested=requested,
                                      stamp=time.perf_counter() if stamp is None else stamp, force=force)

    def set_param_confirmed(self, address, value, label="", max_retries=_USE_DEFAULT):
        '''Write one parameter and do not return until the exo has acknowledged it. For session setup and the stress
        test - the experiment loop uses request() instead.

        max_retries: attempts before giving up. Not given = this link's max_retries. None = never give up.

        Raises OpenExoLinkError if the GUI refuses the write (.code set), if the firmware rejects the value (.reason
        set), if every allowed attempt went unanswered (.reason None), or if the GUI reports the exo disconnected.
        Returns the ack dict on success. Every attempt is logged; the machine action log is not written.

        Note we always send UNILATERALLY, one side at a time, never with bilateral=True. A bilateral write is expanded
        GUI-side into two BLE writes anyway (Python_GUI/services/QtExoDeviceManager.py build_parameter_updates), and one
        write per ack keeps every ack unambiguous.'''
        cap = self.max_retries if max_retries is _USE_DEFAULT else max_retries
        address = _as_address(address)
        tag = label if label else _tag(address)
        cmd = self.scheduler.request(address, value, tag, max_attempts=cap, log_action=False, force=True)
        self.wait_for([cmd])
        if cmd.result == "accepted":
            return cmd.ack
        if cmd.result == "gui_refused":
            raise OpenExoLinkError(f"GUI refused the write of {tag} = {value}: {cmd.error}",
                                   code=getattr(cmd.error, "code", None))
        if cmd.result == "gave_up":
            raise OpenExoLinkError(f"No usable acknowledgement for {tag} = {value} after {cap} attempts. "
                                   f"The exo may or may not have applied it.")
        if cmd.result.startswith("rejected"):
            reason = cmd.ack.get("reason")
            raise OpenExoLinkError(f"Exo REJECTED {tag} = {value}: {reason}", reason=reason)
        raise OpenExoLinkError(f"The write of {tag} = {value} ended as '{cmd.result}'")

    def wait_for(self, commands):
        '''Block, keeping the link serviced, until every given Command is resolved (None entries are skipped).
        Raises OpenExoLinkError if the GUI reports the exo disconnected meanwhile - nothing can reach it any more.
        Ctrl-C propagates to the caller.'''
        pending = [c for c in commands if c is not None]

        def all_resolved():
            return all(c.result is not None for c in pending)

        while not all_resolved():
            if self.link_down:
                raise OpenExoLinkError("The GUI reports the exo disconnected")
            self.service(ACK_POLL_INTERVAL, until=all_resolved)

    def latest(self, address):
        '''The last value requested for this address (confirmed or not), or None'''
        return self.scheduler.latest(_as_address(address))

    def confirmed(self, address):
        '''The last value the exo acknowledged for this address, or None'''
        return self.scheduler.confirmed(_as_address(address))

    def begin_exit(self, keep):
        '''From now on only the addresses in `keep` (the park) may send. Values waiting elsewhere are dropped and logged
        "unconfirmed at exit"; a command already on the air elsewhere gets its ack or its timeout first.'''
        self.scheduler.begin_exit([_as_address(a) for a in keep])

    ############################### Servicing the link #############################################

    def service(self, budget=0.0, until=None):
        '''Run the write scheduler. Read every waiting frame, resolve the command on the air (its ack, or its timeout),
        send the next command, print due warnings - then wait on the socket for up to `budget` seconds, doing the same
        the moment each frame arrives. The experiment loop calls this once per pass with its idle time as the budget,
        in place of sleeping, so an ack is acted on at once. budget=0 is one non-blocking pass.

        until: optional callable. Return early, after a pass, as soon as it is true - the blocking waits use this so a
        confirmed write returns the moment its ack arrives, not at the end of the budget.

        Nothing is sent once the GUI has reported the exo disconnected (link_down).'''
        deadline = time.monotonic() + budget
        original_timeout = self.exo._sock.gettimeout()
        try:
            while True:
                self._drain()
                self.scheduler.poll(time.monotonic())
                if not self.link_down:
                    self._send_next()
                self.scheduler.emit_warnings(time.monotonic())
                if until is not None and until():
                    return
                now = time.monotonic()
                remaining = deadline - now
                if remaining <= 0:
                    return
                #Wake for the next frame, the on-air timeout or the next warning tick, whichever comes first
                wait = min(remaining, self.scheduler.warn_every)
                until_timeout = self.scheduler.time_to_timeout(now)
                if until_timeout is not None:
                    wait = min(wait, until_timeout)
                self.exo._sock.settimeout(max(wait, 0.001))
                self._recv_one()
        finally:
            self.exo._sock.settimeout(original_timeout)

    def pump(self):
        '''One non-blocking service() pass: handle whatever is already waiting, never wait for more. Kept for
        stress_test_ble.py, which calls it every loop between writes to notice a disconnect.

        Be honest about what this can see: the GUI reports a disconnect only after BLE gives up, about 9.6 seconds
        after the exo actually went quiet. A backstop, not a fast watchdog.'''
        self.service(0.0)

    def _send_next(self):
        '''Hand the next command, if any, to the GUI. set_param blocks only for the GUI's ok reply (milliseconds)'''
        cmd = self.scheduler.next_send(time.monotonic())
        if cmd is None:
            return
        joint_id, controller_id, param_index = cmd.address
        self.exo._sock.settimeout(self._reply_timeout)
        try:
            self.exo.set_param(joint_id, controller_id, param_index, cmd.value, bilateral=False)
        except RemoteError as e:
            #A GUI refusal always carries a code (remote/service.py). No code = the client's own timeout: no reply at all
            if e.code is None:
                self.scheduler.gui_no_reply(cmd, time.monotonic())
            else:
                self.scheduler.gui_refused(cmd, e, time.monotonic())
            return
        except BaseException:
            #Interrupted (Ctrl-C) while waiting on the GUI's reply. The request has most likely gone out, so treat it as
            #sent: its ack or its timeout resolves it as usual. Left half-sent, it would never time out, and the park
            #that follows a Ctrl-C would wait behind it forever
            self.scheduler.mark_sent(cmd, time.monotonic())
            raise
        self.scheduler.mark_sent(cmd, time.monotonic())

    def _drain(self):
        '''Read every frame already waiting, without blocking'''
        self.exo._sock.settimeout(0.0)
        while self._recv_one() is not None:
            pass

    def _on_ack(self, ack):
        '''Called for every ack frame the moment it is read, whichever receive call read it'''
        self.scheduler.on_ack(ack, time.monotonic())

    def _on_status(self, msg):
        '''Called for every status frame the moment it is read. "disconnected" sets link_down: the GUI lost the exo, and
        the Nano cannot be reconnected mid-trial without side effects, so the caller should stop.
        "device_error" is deliberately ignored - the write it hit gets no ack, and the ack check reports that.'''
        event = msg.get("event")
        self.last_status = msg
        if event == "disconnected":
            if not self.link_down:
                print("  LINK DOWN: the GUI reports the exo disconnected")
            self.link_down = 1
        elif event == "connected":
            self.link_down = 0
            print(f"  Link up: the GUI connected to {msg.get('name')} ({msg.get('address')})")

    def _recv_one(self):
        '''One non-fatal receive. Returns the message dict, or None if nothing was waiting or it was junk'''
        try:
            msg = self.exo._recv()
        except (socket.timeout, BlockingIOError):
            return None
        except (ValueError, OSError):
            return None
        self.exo._absorb(msg)
        return msg

    ############################### Shutdown #############################################

    def close(self):
        '''Log every command never answered as "unconfirmed at exit", then unsubscribe and close the socket.
        Safe to call more than once'''
        self.scheduler.abandon_all()
        try:
            self.exo.unsubscribe()
        except Exception:
            pass
        try:
            self.exo._sock.close()
        except Exception:
            pass
        if self.verbose:
            print("Exo link closed")


############################### Module-level helpers #############################################

def _is_int(v):
    '''bool is a subclass of int, so True/False must not be mistaken for an id'''
    return isinstance(v, int) and not isinstance(v, bool)


def _joint_display_name(row):
    '''"Ankle(L) (68)" -> "Ankle(L)". Strips the trailing " (<id>)" the GUI appends'''
    disp = str(row[0])
    cut = disp.rfind(" (")
    return disp[:cut] if cut > 0 else disp


def _as_address(address):
    '''(joint_id, controller_id, param_index) as plain ints, so addresses compare equal however they were built'''
    joint_id, controller_id, param_index = address
    return (int(joint_id), int(controller_id), int(param_index))


def _tag(address):
    '''Fallback label when the caller gave none'''
    joint_id, controller_id, param_index = address
    return f"joint {joint_id} / controller {controller_id} / param {param_index}"
