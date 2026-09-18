#The write scheduler behind OpenExoLink (V0.3): ONE command on the air at a time, a retry only when it is needed, and a
#newer value always beats a resend of an older one. Design, and the measurements behind every rule here:
#"Modification log with claude/specs/2026-09-17-ack-aware-write-scheduler-design.md"
#
#This file is a pure state machine: no sockets, no sleeping, no clock of its own. Every method that cares about time
#takes `now` (time.monotonic() seconds) from its caller. OpenExoLink does the talking; the tests drive it with a fake clock.
#
#Why only one command on the air: an ack carries (joint, controller, parameter, accepted, reason) and NOTHING else - no
#value, no sequence number. With one command out, an ack can only be that command's. And the one time we did have two
#out at once (the GUI's July/August bilateral writes) is also the only time acks ever took longer than 1 s.
#
#Why a newer value replaces a resend instead of queueing behind it: on the exo (2026-09-16/17, diag firmware), 250
#silent attempts were acks the Nano sent and the PC never got, against 1 command that may not have arrived. So a resend
#is almost always a value the exo already holds, and resending a stale value only delays the fresh one.
#V0.1 2026 Sep 17

GARBLED_REASON_CODE = 1     #What the Nano sends when it could not parse the Teensy's ack (ExoCode/src/ComsMCU.cpp _send_param_update_ack)
NO_ACK_WARN_S = 5.0         #Warn once an address has spent this long on the air without an ack: about 5 failed attempts in a row. The most seen in ~2,100 writes is 4
NO_ACK_WARN_EVERY_S = 1.0   #And repeat the warning this often, for as long as it lasts


def is_garbled(ack):
    '''Reason code 1 ("invalid message") is what the Nano sends when it could not parse the Teensy's ack, so the frame
    was damaged in transit and none of its fields can be trusted - even an "accepted" one'''
    try:
        return int(ack.get("reason_code")) == GARBLED_REASON_CODE
    except (TypeError, ValueError):
        return False


def ack_answers(ack, address):
    '''Could this ack be the answer to a command at `address` = (joint_id, controller_id, param_index)?

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
    if not is_garbled(ack):
        return (ack_controller == controller_id) and (ack_param == param_index)
    if ack_controller == 0:
        return True
    return (ack_controller == controller_id) and (ack_param in (0, param_index))


class Command:
    '''One value for one address, from the request to its fate. A retry is the SAME Command sent again (attempts + 1).
    A newer value is a NEW Command.'''

    def __init__(self, address, value, label, requested=None, stamp=None, max_attempts=None, log_action=True):
        self.address = address              #(joint_id, controller_id, param_index)
        self.value = value                  #What goes on the wire
        self.label = label                  #Human-readable name for terminal and log lines
        self.requested = value if requested is None else requested   #Before ActionMap's clamp. Machine action log only
        self.stamp = stamp                  #When it was requested, in the caller's clock. Machine action log only
        self.max_attempts = max_attempts    #None = keep going until an ack. A number = give up after that many
        self.log_action = log_action        #Whether its fate goes in the machine action log (loop writes do, setup writes do not)
        self.attempts = 0                   #How many times it has been handed out for sending
        self.replaces = None                #The unconfirmed value this one superseded, for its send line
        self.result = None                  #None until resolved - see the _resolve() callers for the possible results
        self.ack = None                     #The ack that accepted or refused it
        self.error = None                   #The GUI's RemoteError, when the GUI refused it


class _Slot:
    '''Everything the scheduler keeps per address'''

    def __init__(self):
        self.wanted = None      #The Command to send on this address's next turn (a new value, or a retry), or None
        self.latest = None      #The last value requested, so a repeat can be ignored
        self.confirmed = None   #The last value an accepted ack confirmed
        self.tab = 0.0          #No-ack time from FINISHED attempts since this address's last ack. See live_tab()
        self.fails = 0          #Failed attempts since this address's last ack
        self.label = ""         #The latest command's label, for warnings
        self.last_warn = None   #When the last no-ack warning for this address was printed


class WriteScheduler:
    """Decides what goes on the air next, and what every ack, timeout and GUI error means.

    Inputs:
    ack_timeout: how long a command waits for its ack before it counts as a failed attempt, in seconds
    reporter: what the scheduler tells the outside world. Needs three methods:
        attempt(cmd, result) - one attempt finished (a Param_write_log row)
        command(cmd) - a command's fate is known, cmd.result is set (a Machine_action_log row, loop writes only)
        say(line, always=False) - a terminal line; always=False lines are for verbose mode only
    warn_after, warn_every: the no-ack warning threshold and repeat period, in seconds
    """

    def __init__(self, ack_timeout, reporter, warn_after=NO_ACK_WARN_S, warn_every=NO_ACK_WARN_EVERY_S):
        self.ack_timeout = ack_timeout
        self.reporter = reporter
        self.warn_after = warn_after
        self.warn_every = warn_every

        self._slots = {}            #address -> _Slot
        self._order = []            #Addresses in first-request order: the round-robin cycle
        self._last_sent = None      #The address sent most recently. The round-robin search starts just after it
        self._on_air = None         #The one Command handed out and not resolved yet
        self._handed_at = None      #When it was handed out, before the GUI replied
        self._sent_at = None        #When the GUI replied ok. None = still waiting on the GUI, so no ack can be its yet
        self._keep_on_exit = None   #None normally. At exit: the only addresses still allowed to send (the park)

    ############################### Requests #############################################

    def request(self, address, value, label="", requested=None, stamp=None, max_attempts=None,
                log_action=True, force=False):
        '''Ask for `address` to hold `value`. Returns the new Command, or None if nothing needs sending: the value
        repeats the latest request (unless force), or we are exiting and this is not a park address.

        If an older value for this address is still waiting, the new one replaces it. The older one is logged
        "replaced before sending" if it never went out, or "superseded (no ACK)" if it went out, got no ack, and was
        waiting for its resend.'''
        if self._keep_on_exit is not None and address not in self._keep_on_exit:
            return None
        slot = self._slot(address)
        if not force and slot.latest == value:
            return None
        cmd = Command(address, value, label, requested, stamp, max_attempts, log_action)
        old = slot.wanted
        if old is not None:
            if old.attempts == 0:
                self._resolve(old, "replaced before sending")
            else:
                cmd.replaces = old.value
                self._resolve(old, "superseded (no ACK)")
        slot.wanted = cmd
        slot.latest = value
        slot.label = label
        return cmd

    ############################### Sending #############################################

    def next_send(self, now):
        '''Hand out the next command to send, or None. Nothing is handed out while another command is on the air.
        Addresses take turns in first-request order, starting just after the one sent last - so after a timeout on
        one leg the other leg goes next, and the first leg's next turn sends its newest value or its retry.

        The caller sends it, then calls mark_sent(), gui_refused() or gui_no_reply().'''
        if self._on_air is not None or not self._order:
            return None
        start = 0
        if self._last_sent in self._order:
            start = self._order.index(self._last_sent) + 1
        count = len(self._order)
        for k in range(count):
            address = self._order[(start + k) % count]
            slot = self._slots[address]
            if slot.wanted is None:
                continue
            cmd = slot.wanted
            slot.wanted = None
            cmd.attempts += 1
            self._on_air = cmd
            self._handed_at = now
            self._sent_at = None
            self._last_sent = address
            if cmd.attempts > 1:
                kind = f"resend, attempt {cmd.attempts}"
            elif cmd.replaces is not None:
                kind = f"replaces unconfirmed {cmd.replaces}"
            else:
                kind = "first send"
            self.reporter.say(f"  -> {cmd.label} = {cmd.value} ({kind})")
            return cmd
        return None

    def mark_sent(self, cmd, now):
        '''The GUI replied ok to the set_param. Only from now on can an ack answer this command: the GUI replies before
        it even starts the BLE write, so an ack read while we waited for that reply is a late one for an earlier command.'''
        if cmd is self._on_air:
            self._sent_at = now

    ############################### Answers #############################################

    def on_ack(self, ack, now):
        '''Every ack frame, the moment it is read. Only an ack for the command on the air counts. Anything else - a late
        ack for an earlier command, a GUI-button write, one read during the GUI's reply wait - is ignored.'''
        cmd = self._on_air
        if cmd is None or self._sent_at is None or not ack_answers(ack, cmd.address):
            return
        if is_garbled(ack):
            #Damaged in transit, so we cannot tell whether the exo applied it: a failed attempt, without sitting out the timeout
            self._fail(now, "garbled_ack")
            return
        slot = self._slots[cmd.address]
        self._clear_on_air()
        #Any real ack - accepted or refused - proves the link delivered. That, and only that, clears the no-ack tab
        slot.tab = 0.0
        slot.fails = 0
        slot.last_warn = None
        cmd.ack = ack
        if ack.get("accepted"):
            slot.confirmed = cmd.value
            self.reporter.attempt(cmd, "accepted")
            self.reporter.say(f"  Exo ACCEPTED {cmd.label} = {cmd.value} (attempt {cmd.attempts})")
            self._resolve(cmd, "accepted")
        else:
            #The firmware got it and said no (bounds, wrong controller...). Resending the same value is refused the same way
            reason = ack.get("reason")
            self.reporter.attempt(cmd, f"rejected ({reason})")
            self.reporter.say(f"  Exo REJECTED {cmd.label} = {cmd.value}: {reason}. Value dropped.", always=True)
            self._resolve(cmd, f"rejected ({reason})")

    def poll(self, now):
        '''Call every service pass: a command whose ack_timeout has run out becomes a failed attempt'''
        if self._on_air is not None and self._sent_at is not None and now - self._sent_at >= self.ack_timeout:
            self._fail(now, "no_ack")

    def gui_refused(self, cmd, error, now):
        '''The GUI rejected the set_param itself (bad address or value, never the exo's doing). Resending the identical
        message cannot help, so the value is dropped. The tab is left alone: this was not an ack'''
        if cmd is not self._on_air:
            return
        self._clear_on_air()
        cmd.error = error
        self.reporter.attempt(cmd, "gui_refused")
        self.reporter.say(f"  GUI REFUSED {cmd.label} = {cmd.value}: {error}. Value dropped.", always=True)
        self._resolve(cmd, "gui_refused")

    def gui_no_reply(self, cmd, now):
        '''The GUI never replied to the set_param (a frozen GUI, not the exo). A failed attempt, like a timeout'''
        if cmd is not self._on_air:
            return
        self._sent_at = self._handed_at     #So the tab gets this attempt's wait
        self._fail(now, "gui_no_reply")

    def _fail(self, now, result):
        '''The command on the air failed (timeout, garbled ack, or no GUI reply). Log the attempt, then either resend it,
        let a newer value for the same address take its place, or - capped writes and exit only - stop.'''
        cmd = self._on_air
        slot = self._slots[cmd.address]
        slot.tab += min(now - self._sent_at, self.ack_timeout)     #At most +1 timeout per attempt
        slot.fails += 1
        self._clear_on_air()
        self.reporter.attempt(cmd, result)
        what = {"no_ack": "No ACK", "garbled_ack": "Garbled ACK"}.get(result, "No reply from the GUI")
        head = f"  {what} for {cmd.label} = {cmd.value} (attempt {cmd.attempts})"
        if self._keep_on_exit is not None and cmd.address not in self._keep_on_exit:
            self.reporter.say(f"{head}, not resent (exiting)", always=True)
            self._resolve(cmd, "unconfirmed at exit")
        elif cmd.max_attempts is not None and cmd.attempts >= cmd.max_attempts:
            self.reporter.attempt(cmd, "gave_up")
            self.reporter.say(f"{head}. Giving up.", always=True)
            self._resolve(cmd, "gave_up")
        elif slot.wanted is not None:
            #A newer value arrived while this one was out. It goes instead of a resend
            slot.wanted.replaces = cmd.value
            self.reporter.say(f"{head}, newer value {slot.wanted.value} waiting", always=True)
            self._resolve(cmd, "superseded (no ACK)")
        else:
            slot.wanted = cmd
            self.reporter.say(f"{head}, pending resend", always=True)

    ############################### Warnings #############################################

    def live_tab(self, address, now):
        '''Seconds this address's own commands have spent on the air without an ack since its last ack. It only grows
        while this address has a command on the air (at most +1 timeout per attempt), pauses while other addresses are
        out or while it waits its turn, survives a newer value replacing an older one, and clears only on an ack.'''
        slot = self._slots.get(address)
        if slot is None:
            return 0.0
        tab = slot.tab
        if self._on_air is not None and self._on_air.address == address and self._sent_at is not None:
            tab += min(now - self._sent_at, self.ack_timeout)
        return tab

    def pending(self, address):
        '''Whether this address still has something to deliver: a command waiting, or one on the air'''
        slot = self._slots.get(address)
        if slot is None:
            return False
        return slot.wanted is not None or (self._on_air is not None and self._on_air.address == address)

    def emit_warnings(self, now):
        '''Call every service pass. While an address with something to deliver has a tab of warn_after or more, print
        one warning line per warn_every seconds'''
        for address in self._order:
            slot = self._slots[address]
            if not self.pending(address):
                continue
            tab = self.live_tab(address, now)
            if tab < self.warn_after:
                continue
            if slot.last_warn is not None and now - slot.last_warn < self.warn_every:
                continue
            slot.last_warn = now
            want = slot.wanted.value if slot.wanted is not None else self._on_air.value
            self.reporter.say(f"  WARNING: no ACK for {slot.label} for {tab:.1f} s "
                              f"({slot.fails} failed attempts), wanted {want}", always=True)

    def time_to_timeout(self, now):
        '''Seconds until the command on the air times out, or None if nothing is waiting on an ack'''
        if self._on_air is None or self._sent_at is None:
            return None
        return max(self.ack_timeout - (now - self._sent_at), 0.0)

    ############################### State #############################################

    def latest(self, address):
        slot = self._slots.get(address)
        return None if slot is None else slot.latest

    def confirmed(self, address):
        slot = self._slots.get(address)
        return None if slot is None else slot.confirmed

    def idle(self):
        '''Nothing on the air and nothing waiting to go'''
        return self._on_air is None and all(slot.wanted is None for slot in self._slots.values())

    ############################### Exit #############################################

    def begin_exit(self, keep):
        '''From now on only the addresses in `keep` (the park) may send. Values waiting elsewhere are dropped and logged
        "unconfirmed at exit". A command already on the air elsewhere still gets its ack or its timeout, but is not resent.'''
        self._keep_on_exit = set(keep)
        for address in self._order:
            slot = self._slots[address]
            if address not in self._keep_on_exit and slot.wanted is not None:
                cmd = slot.wanted
                slot.wanted = None
                self._resolve(cmd, "unconfirmed at exit")

    def abandon_all(self):
        '''Closing down: every command never answered, on the air or waiting, is logged "unconfirmed at exit"'''
        if self._on_air is not None:
            cmd = self._on_air
            self._clear_on_air()
            self._resolve(cmd, "unconfirmed at exit")
        for address in self._order:
            slot = self._slots[address]
            if slot.wanted is not None:
                cmd = slot.wanted
                slot.wanted = None
                self._resolve(cmd, "unconfirmed at exit")

    ############################### Internals #############################################

    def _slot(self, address):
        if address not in self._slots:
            self._slots[address] = _Slot()
            self._order.append(address)
        return self._slots[address]

    def _clear_on_air(self):
        self._on_air = None
        self._handed_at = None
        self._sent_at = None

    def _resolve(self, cmd, result):
        cmd.result = result
        if cmd.log_action:
            self.reporter.command(cmd)
