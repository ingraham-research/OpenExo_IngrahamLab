#This utility wraps the OpenExo GUI's UDP remote client into something an external "main" control code
#can use. The GUI already owns the exo: it holds the BLE link, it holds the handshake controller matrix,
#and it is the only thing allowed to start a trial. All this utility does is push controller PARAMETER
#values into it and then wait for the exo to say it actually stored them.
#
#Two things here are not obvious and are the reason this file exists at all:
#   1) The firmware acknowledges a parameter write with NUMERIC ids only (joint id, controller id,
#      parameter index) and carries no request id. So to know whether OUR write landed, we have to
#      resolve names to numbers ourselves and match on that triple.
#   2) SILENCE IS THE ONLY FAILURE SIGNAL. A dropped BLE write produces no negative acknowledgement,
#      and the GUI returns "ok" to a set_param even with no exo connected - "ok" only means the GUI
#      queued a BLE write. Only the ack means the firmware stored the value.
#V0.1 2026 Sep

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

#How long we listen for an ack before calling the write lost, and how many times we resend
DEFAULT_ACK_TIMEOUT = 5.0
DEFAULT_MAX_RETRIES = 3

#Poll granularity while waiting on the socket. Small enough to feel instant, large enough not to spin
ACK_POLL_INTERVAL = 0.2


class OpenExoLinkError(Exception):
    """A parameter write did not make it to the exo, or was refused by the GUI or the firmware."""

    def __init__(self, message, code=None, reason=None):
        super().__init__(message)
        self.code = code        #The GUI's machine-readable rejection code, if it was the GUI that refused
        self.reason = reason    #The firmware's rejection reason string, if it was the exo that refused


class OpenExoLink:
    """This class owns the UDP connection to the GUI and turns "set this parameter to this value" into
    a confirmed write. It resolves human-readable names into the numeric ids the firmware acks with,
    sends the write, waits for the matching ack, and retries on silence.

    Inputs:
    host, port: where the GUI's remote service is listening. Localhost only by default (see
        Python_GUI/utils/config.py RemoteConfig)
    timeout: how long we wait for the GUI's own ok/error reply to a command
    ack_timeout: how long we wait for the EXO's acknowledgement of a parameter write
    max_retries: how many times we resend a write that was met with silence
    verbose: if 1, print every write and every ack to the terminal
    logfile: an already-open (by main_external_control), line-buffered file to mirror the write/ack history into. Optional
    """

    def __init__(self, host="127.0.0.1", port=9750, timeout=2.0, ack_timeout=DEFAULT_ACK_TIMEOUT,
                 max_retries=DEFAULT_MAX_RETRIES, verbose=1, logfile=None):
        self.host = host
        self.port = port
        self.ack_timeout = ack_timeout
        self.max_retries = max_retries
        self.verbose = verbose
        self.logfile = logfile

        self.exo = ExoRemote(host, port, timeout=timeout)
        self.matrix = []            #The handshake controller matrix, once the GUI has one
        self.connected = 0          #Whether we have seen a live GUI and a non-empty matrix
        self.link_down = 0          #Set by pump() when the GUI reports the exo disconnected or errored
        self.last_status = None     #Most recent status frame seen, for diagnostics

        self._ack_snapshot = None   #What last_ack() held immediately before our most recent write

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

        Raises ExoLinkError if the GUI refuses the write, if the firmware rejects the value, or if all
        retries are met with silence. Returns the ack dict on success.

        Note we always send this UNILATERALLY, one side at a time, never with bilateral=True. A bilateral
        write is expanded GUI-side into two sequential BLE writes anyway (see
        Python_GUI/services/QtExoDeviceManager.py build_parameter_updates), so it costs nothing extra to
        send them ourselves - and because the client only ever holds the MOST RECENT ack, a bilateral
        write can have its first ack overwritten before we read it. One write, one ack, no ambiguity.'''
        joint_id, controller_id, param_index = address
        tag = label if label else f"joint {joint_id} / controller {controller_id} / param {param_index}"

        for attempt in range(1, self.max_retries + 1):
            #Snapshot what the client is already holding, so we can tell a NEW ack from a stale one
            self._ack_snapshot = self.exo.last_ack()

            try:
                self.exo.set_param(joint_id, controller_id, param_index, value, bilateral=False)
            except RemoteError as e:
                #The GUI itself refused this - a bad name, a bad index, a malformed value. Resending
                #the identical message cannot help, so fail immediately rather than burning the retries
                raise OpenExoLinkError(f"GUI refused the write of {tag} = {value}: {e}", code=e.code)

            ack = self._wait_for_ack(joint_id, controller_id, param_index, self.ack_timeout)

            if ack is None:
                #Silence. This is what a dropped BLE write looks like - there is no negative ack - so
                #this is the one case worth retrying
                print(f"  No ack for {tag} = {value} (attempt {attempt} of {self.max_retries}). Resending...")
                continue

            if ack.get("accepted"):
                self._report_write(tag, value, ack, attempt)
                return ack

            #The firmware got it and said no. The value is out of bounds, or the wrong type, or aimed at
            #the wrong controller. Resending the same value will be rejected the same way
            raise OpenExoLinkError(f"Exo REJECTED {tag} = {value}: {ack.get('reason')}",
                               reason=ack.get("reason"))

        raise OpenExoLinkError(f"No acknowledgement for {tag} = {value} after {self.max_retries} attempts. "
                           f"The write never reached the exo.")

    def _wait_for_ack(self, joint_id, controller_id, param_index, ack_timeout):
        '''Wait for the ack that matches this exact write. Returns the ack dict, or None on silence.

        The first check here is the important one. ExoRemote._command drains stream frames while it waits
        for its own ok reply, so an ack that arrives inside that window is absorbed into last_ack() and is
        NEVER re-delivered by stream(). Listening on the stream first would therefore block for the full
        timeout on a write the exo already acknowledged.'''
        ack = self.exo.last_ack()
        if (ack is not None) and (ack is not self._ack_snapshot) and _ack_matches(ack, joint_id, controller_id, param_index):
            return ack

        #Nothing matching was absorbed, so listen on the socket for the rest of the window
        deadline = time.time() + ack_timeout
        original_timeout = self.exo._sock.gettimeout()
        self.exo._sock.settimeout(ACK_POLL_INTERVAL)
        try:
            while time.time() < deadline:
                msg = self._recv_one()
                if msg is None:
                    continue
                if msg.get("stream") != "ack":
                    continue
                if _ack_matches(msg, joint_id, controller_id, param_index):
                    return msg
        finally:
            self.exo._sock.settimeout(original_timeout)
        return None

    ############################### Link health #############################################

    def pump(self, budget=0.01):
        '''Drain whatever is waiting on the socket without blocking, so status frames are noticed between
        writes. Call this once per loop in the main code.

        Be honest about what this can and cannot see: the GUI reports a disconnect only after BLE gives up,
        which for the Nano radio-silence failure mode is about 9.6 seconds after the exo actually went
        quiet. This is a backstop, not a fast watchdog. The real detection of a dead link is a
        set_param_confirmed that comes back silent.'''
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

    def _report_write(self, tag, value, ack, attempt):
        retried = f" (after {attempt} attempts)" if attempt > 1 else ""
        if self.verbose:
            print(f"  Exo ACCEPTED {tag} = {value}{retried}")
        if self.logfile is not None:
            print(f"{time.time()}, {tag}, {value}, accepted, {attempt}", file=self.logfile)

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


def _ack_matches(ack, joint_id, controller_id, param_index):
    '''Acks carry no request id, so the (joint, controller, parameter) triple is all we have to match on'''
    try:
        return (int(ack.get("joint_id")) == int(joint_id)
                and int(ack.get("controller_id")) == int(controller_id)
                and int(ack.get("param_index")) == int(param_index))
    except (TypeError, ValueError):
        return False
