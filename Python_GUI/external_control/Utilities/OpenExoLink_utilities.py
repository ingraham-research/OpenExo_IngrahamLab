#This utility wraps the OpenExo GUI's UDP remote client into something an external "main" control code
#can use. The GUI already owns the exo: it holds the BLE link, it holds the handshake controller matrix,
#and it is the only thing allowed to start a trial. All this utility does is push controller PARAMETER
#values into it and then wait for the exo to say it actually stored them.
#
#Two things here are not obvious and are the reason this file exists at all:
#   1) The firmware acknowledges a parameter write with NUMERIC ids only (joint id, controller id,
#      parameter index) and carries no request id. So to know whether OUR write landed, we have to
#      resolve names to numbers ourselves and match on that triple.
#   2) SILENCE IS NOT THE ONLY FAILURE SIGNAL, but it is the most common one. A dropped BLE write produces no
#      negative acknowledgement, and the GUI returns "ok" to a set_param even with no exo connected - "ok" only
#      means the GUI queued a BLE write. Only the ack means the firmware stored the value.
#   3) Some acks arrive GARBLED. When the Nano cannot parse the Teensy's UART ack it sends up a fake
#      "rejected, reason 1 (invalid message)" instead, keeping only the fields it parsed before the damage
#      (ExoCode/src/ComsMCU.cpp _send_param_update_ack). The Teensy may well have applied the value, so such an
#      ack means "unknown", not "refused" - we resend. See "Modification log with claude/ACK-Loss-Investigation.md".
#   4) Acks carry no value, so an ack can only be matched to a write by address and timing. Every attempt is
#      tracked until an ack pays it off (oldest first), and is written off just before it is resent. KNOWN
#      LIMIT: an ack arriving later than ack_timeout can still be taken as the confirmation of the next write
#      to the same parameter. Keeping attempts open longer would close that gap, but on a lossy link every lost
#      ack would then tax every later write, until writes start giving up. No ack later than 0.48 s has been
#      seen (2026-09-16). The real fix is echoing the value in the ack (firmware).
#V0.1 2026 Sep
#V0.2 2026 Sep: garbled-ack resend, per-attempt logging, attempt bookkeeping, 1 s / 5 attempt defaults

import os
import socket
import sys
import time

#The remote client lives with the GUI, not in this folder. Same path trick as Python_GUI/examples/remote_console.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "remote"))
from client import ExoRemote, RemoteError  # noqa: E402

#Ankle joint ids, from ExoCode/src/ParseIni.h:125-137 (left = 0b01000000, right = 0b00100000).
#WARNING: do NOT use Python_GUI/utils/config.py JointConfig.ID_TO_NUM for this - all 8 of its entries
#have their left/right labels inverted, and it would silently command the wrong leg. The handshake
#matrix (and these two constants) are the real source of truth.
LEFT_ANKLE_JOINT_ID = 68
RIGHT_ANKLE_JOINT_ID = 36

#How long we listen for an ack before resending, and how many attempts we make in total.
#1 s: every unilateral write in the GUI logs up to 2026-09-16 (2215 of them) was acked within 453 ms
DEFAULT_ACK_TIMEOUT = 1.0
DEFAULT_MAX_RETRIES = 5

#Poll granularity while waiting on the socket. Small enough to feel instant, large enough not to spin
ACK_POLL_INTERVAL = 0.05

#An unanswered attempt is written off this long BEFORE it is resent, so an attempt can never still be open
#when its successor goes out. Capped at half the timeout, so it stays strictly shorter for any ack_timeout.
#An ack landing inside this margin is not credited and the write is resent as usual - with every ack seen so
#far under 0.48 s, the 0.9-1.0 s band is empty in practice
ACK_WRITE_OFF_MARGIN = 0.1

#The reason code the firmware uses for "invalid message". See point 3 at the top of this file
GARBLED_REASON_CODE = 1


class OpenExoLinkError(Exception):
    """A parameter write did not make it to the exo, or was refused by the GUI or the firmware."""

    def __init__(self, message, code=None, reason=None):
        super().__init__(message)
        self.code = code        #The GUI's machine-readable rejection code, if it was the GUI that refused
        self.reason = reason    #The firmware's rejection reason string, if it was the exo that refused


class _AckTrackingRemote(ExoRemote):
    """ExoRemote that hands EVERY ack frame to a callback the moment it is read, whichever receive call read it
    (set_param's own wait for the GUI's ok reply, or our polling). The stock client only keeps the most recent
    ack, so two acks landing inside one of its waits would silently lose the first."""

    def __init__(self, *args, on_ack=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.on_ack = on_ack

    def _absorb(self, msg):
        super()._absorb(msg)
        if msg.get("stream") == "ack" and self.on_ack is not None:
            self.on_ack(msg)


class OpenExoLink:
    """This class owns the UDP connection to the GUI and turns "set this parameter to this value" into
    a confirmed write. It resolves human-readable names into the numeric ids the firmware acks with,
    sends the write, waits for the matching ack, and retries on silence or on a garbled ack.

    Inputs:
    host, port: where the GUI's remote service is listening. Localhost only by default (see
        Python_GUI/utils/config.py RemoteConfig)
    timeout: how long we wait for the GUI's own ok/error reply to a command
    ack_timeout: how long we wait for the EXO's acknowledgement of a parameter write
    max_retries: how many attempts we make at a write before giving up (silence and garbled acks both use one up)
    verbose: if 1, print every write and every ack to the terminal
    logfile: an already-open (by main_external_control), line-buffered file to mirror the write/ack history into,
        one row per ATTEMPT. Optional
    """

    def __init__(self, host="127.0.0.1", port=9750, timeout=2.0, ack_timeout=DEFAULT_ACK_TIMEOUT,
                 max_retries=DEFAULT_MAX_RETRIES, verbose=1, logfile=None):
        self.host = host
        self.port = port
        self.ack_timeout = ack_timeout
        self.max_retries = max_retries
        self.verbose = verbose
        self.logfile = logfile

        self.exo = _AckTrackingRemote(host, port, timeout=timeout, on_ack=self._on_ack)
        self.matrix = []            #The handshake controller matrix, once the GUI has one
        self.connected = 0          #Whether we have seen a live GUI and a non-empty matrix
        self.link_down = 0          #Set by pump() when the GUI reports the exo disconnected or errored
        self.last_status = None     #Most recent status frame seen, for diagnostics

        self._outstanding = []      #Attempts sent but not yet acked, oldest first: [sent_at, write_id, address]
        self._write_id = 0          #Counts logical writes. All attempts at one write share its id
        self._answers = []          #Acks that paid off an attempt of the write in progress
        #How long an attempt stays payable. Strictly shorter than ack_timeout - see ACK_WRITE_OFF_MARGIN
        self._write_off_age = ack_timeout - min(ACK_WRITE_OFF_MARGIN, ack_timeout / 2)

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

    def set_param_confirmed(self, address, value, label=""):
        '''Write one parameter and do not return until the exo has acknowledged it.

        address: the (joint_id, controller_id, param_index) tuple from resolve()
        value: the value to write
        label: a human-readable name for the terminal/log line only

        Raises OpenExoLinkError if the GUI refuses the write, if the firmware rejects the value, or if every
        attempt is met with silence or a garbled ack. Returns the ack dict on success.

        Every attempt is logged, whatever its outcome (see _log_attempt). All attempts carry the same value, so
        an ack for ANY of them confirms this write - resending is always safe.

        Note we always send this UNILATERALLY, one side at a time, never with bilateral=True. A bilateral
        write is expanded GUI-side into two sequential BLE writes anyway (see
        Python_GUI/services/QtExoDeviceManager.py build_parameter_updates), so it costs nothing extra to
        send them ourselves - and one write per ack keeps the attempt bookkeeping below unambiguous.'''
        joint_id, controller_id, param_index = address
        tag = label if label else f"joint {joint_id} / controller {controller_id} / param {param_index}"

        #A new logical write. Acks for attempts of earlier writes no longer count as answers
        self._write_id += 1
        self._answers = []

        for attempt in range(1, self.max_retries + 1):
            try:
                self.exo.set_param(joint_id, controller_id, param_index, value, bilateral=False)
            except RemoteError as e:
                #The GUI itself refused this - a bad name, a bad index, a malformed value. Resending
                #the identical message cannot help, so fail immediately rather than burning the retries
                self._log_attempt(tag, value, "gui_refused", attempt)
                raise OpenExoLinkError(f"GUI refused the write of {tag} = {value}: {e}", code=e.code)

            #Only now does this attempt become payable. The GUI replies "ok" before it even starts the BLE write,
            #so no ack for THIS attempt can have been read yet - but a stale ack read during that reply wait could
            #otherwise have paid it off.
            #ONE timestamp drives both the write-off (sent_at + _write_off_age) and the resend (sent_at +
            #ack_timeout), so the write-off is guaranteed to come first. Monotonic, so a wall-clock adjustment
            #cannot reorder them either
            sent_at = time.monotonic()
            self._outstanding.append([sent_at, self._write_id, (int(joint_id), int(controller_id), int(param_index))])

            ack = self._wait_for_ack(sent_at + self.ack_timeout)

            if ack is None:
                #Silence. A dropped BLE write looks like this - there is no negative ack
                self._log_attempt(tag, value, "no_ack", attempt)
                print(f"  No ack for {tag} = {value} (attempt {attempt} of {self.max_retries}).{_next_step(attempt, self.max_retries)}")
                continue

            if _is_garbled(ack):
                #Damaged in transit, so we cannot tell whether the exo applied it. Resend straight away instead
                #of sitting out the timeout, and wait for a fresh answer
                self._answers = []
                self._log_attempt(tag, value, "garbled_ack", attempt)
                print(f"  Garbled ack for {tag} = {value} (attempt {attempt} of {self.max_retries}).{_next_step(attempt, self.max_retries)}")
                continue

            if ack.get("accepted"):
                self._log_attempt(tag, value, "accepted", attempt)
                self._report_write(tag, value, attempt)
                return ack

            #The firmware got it and said no. The value is out of bounds, or the wrong type, or aimed at
            #the wrong controller. Resending the same value will be rejected the same way
            self._log_attempt(tag, value, f"rejected ({ack.get('reason')})", attempt)
            raise OpenExoLinkError(f"Exo REJECTED {tag} = {value}: {ack.get('reason')}",
                               reason=ack.get("reason"))

        self._log_attempt(tag, value, "gave_up", self.max_retries)
        raise OpenExoLinkError(f"No usable acknowledgement for {tag} = {value} after {self.max_retries} attempts. "
                           f"The exo may or may not have applied it.")

    def _wait_for_ack(self, deadline):
        '''Wait until the write in progress has an answer, or until `deadline` (time.monotonic()). Returns the ack
        dict, or None on silence.

        Answers are collected by _on_ack, which runs for EVERY ack frame no matter which receive call read it -
        including set_param's own wait for the GUI's ok reply, which is where a fast ack usually lands. So the
        answer may already be here before we listen at all.'''
        original_timeout = self.exo._sock.gettimeout()
        try:
            while True:
                answer = self._pick_answer()
                if answer is not None:
                    return answer
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.exo._sock.settimeout(min(ACK_POLL_INTERVAL, remaining))
                self._recv_one()
        finally:
            self.exo._sock.settimeout(original_timeout)

    def _pick_answer(self):
        '''The answer to the write in progress, or None if we should keep waiting.

        A real answer (accepted, or a genuine rejection) wins over a garbled one. A garbled ack cannot tell us
        WHICH attempt it answers, so we do not try to guess whether another attempt is still coming: resending
        wrongly costs one quick extra write, waiting wrongly costs a whole timeout. Either way the bookkeeping
        stays safe, because attempts of one write are interchangeable.'''
        for ack in self._answers:
            if not _is_garbled(ack):
                return ack
        if self._answers:
            return self._answers[-1]
        return None

    def _on_ack(self, ack):
        '''Called for every ack frame the moment it is read. Pays off the OLDEST attempt it could be answering.

        An attempt is written off just BEFORE we give up waiting on it and resend (_write_off_age, strictly less
        than ack_timeout), so it can never still be open when its successor goes out. Keeping it open longer
        (MainWindow keeps its own for 5 s) looks safer but is not: when that attempt's ack was LOST, the next
        write's ack pays it off instead, that write needs an extra attempt, and its own leftover attempt carries
        the debt forward to the write after. At heel-strike write rates the debt never expires, and every further
        lost ack adds to it. See point 4 at the top of this file for what this leaves open.'''
        now = time.monotonic()
        #Write off attempts whose window has closed, so an ack arriving now is not matched to them
        self._outstanding = [entry for entry in self._outstanding if now - entry[0] < self._write_off_age]
        for i, (sent_at, write_id, address) in enumerate(self._outstanding):
            if _ack_answers(ack, address):
                del self._outstanding[i]
                if write_id == self._write_id:
                    self._answers.append(ack)
                return
        #No attempt of ours is waiting for this one: a GUI-button write, or an attempt already written off. Ignore

    ############################### Link health #############################################

    def pump(self, budget=0.01):
        '''Drain whatever is waiting on the socket without blocking, so status frames are noticed between
        writes. Call this once per loop in the main code.

        Be honest about what this can and cannot see: the GUI reports a disconnect only after BLE gives up,
        which for the Nano radio-silence failure mode is about 9.6 seconds after the exo actually went
        quiet. This is a backstop, not a fast watchdog. The real detection of a dead link is a
        set_param_confirmed that comes back silent.

        Any ack read here still goes through _on_ack, so a late ack arriving between writes pays off its
        attempt now rather than lingering until the next write.'''
        original_timeout = self.exo._sock.gettimeout()
        self.exo._sock.settimeout(0.0)
        deadline = time.time() + budget
        try:
            while time.time() < deadline:
                msg = self._recv_one()
                if msg is None:
                    break
                if msg.get("stream") == "status":
                    self._handle_status(msg)
        finally:
            self.exo._sock.settimeout(original_timeout)

    def _handle_status(self, msg):
        event = msg.get("event")
        self.last_status = msg
        if event in ("disconnected", "device_error"):
            self.link_down = 1
            print(f"  LINK DOWN: the GUI reports '{event}' {msg.get('message', '')}".rstrip())
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

    ############################### Reporting and shutdown #############################################

    def _report_write(self, tag, value, attempt):
        retried = f" (after {attempt} attempts)" if attempt > 1 else ""
        if self.verbose:
            print(f"  Exo ACCEPTED {tag} = {value}{retried}")

    def _log_attempt(self, tag, value, result, attempt):
        '''One Param_write_log row per ATTEMPT. Result is one of: accepted, no_ack, garbled_ack,
        rejected (<reason>), gui_refused, or gave_up (which closes a write whose every attempt failed, and
        repeats the last attempt number). Filtering on "accepted" gives one row per confirmed write'''
        if self.logfile is not None:
            result = str(result).replace(",", ";")   #The log is comma separated
            print(f"{time.time()}, {tag}, {value}, {result}, {attempt}", file=self.logfile)

    def close(self):
        '''Unsubscribe and close the socket. Safe to call more than once'''
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


def _next_step(attempt, max_retries):
    '''Tail of the per-attempt terminal line: only promise a resend if one is actually coming'''
    return " Resending..." if attempt < max_retries else " Giving up."


def _is_garbled(ack):
    '''Reason code 1 ("invalid message") is what the Nano sends when it could not parse the Teensy's ack, so the
    frame was damaged in transit and none of its fields can be trusted - even an "accepted" one'''
    try:
        return int(ack.get("reason_code")) == GARBLED_REASON_CODE
    except (TypeError, ValueError):
        return False


def _ack_answers(ack, address):
    '''Could this ack be the answer to an attempt at `address` = (joint_id, controller_id, param_index)?

    Acks carry no request id, so the (joint, controller, parameter) triple is all we have to match on. A garbled
    ack only kept the fields the Nano parsed before the damage - joint first, then controller, then parameter -
    and the rest come through as 0. So there a 0 means "unknown" and matches anything'''
    joint_id, controller_id, param_index = address
    try:
        ack_joint = int(ack.get("joint_id"))
        ack_controller = int(ack.get("controller_id"))
        ack_param = int(ack.get("param_index"))
    except (TypeError, ValueError):
        return False
    if ack_joint != joint_id:
        return False
    if not _is_garbled(ack):
        return (ack_controller == controller_id) and (ack_param == param_index)
    if ack_controller == 0:
        return True
    return (ack_controller == controller_id) and (ack_param in (0, param_index))
