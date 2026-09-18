# ACK-aware write scheduler Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the external control loop's blocking confirm-or-exit writes with a one-command-on-the-air scheduler in which newer values replace pending resends, missing ACKs raise warnings instead of ending the session, and every command's fate is logged.

**Architecture:** A new pure state machine (`WriteScheduler_utilities.py`) owns all write timing and takes `now` from its caller, so it is unit-tested with a fake clock. `OpenExoLink` (V0.3) connects it to the GUI's UDP socket: `request()` is non-blocking, `service()` runs once per loop pass, and `set_param_confirmed()` blocks on the same engine. `ActionMap` and `main_external_control` switch to the new calls.

**Tech Stack:** Python 3.13, stdlib sockets, pytest (`biomotum` conda env), the GUI's `remote/client.py`.

**Spec:** `Modification log with claude/specs/2026-09-17-ack-aware-write-scheduler-design.md`

## Global Constraints

- **Never commit.** The operator reviews and commits. Every task ends with "leave uncommitted".
- PC side only: files under `Python_GUI/external_control/` and `Python_GUI/tests/`. No firmware, no GUI change.
- Test command (the base MiniConda has no pytest), from the repo root: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests`. Baseline: 56 passed.
- `ack_timeout` 1.0 s. `NO_ACK_WARN_S` 5.0. `NO_ACK_WARN_EVERY_S` 1.0. `DEFAULT_MAX_RETRIES` 5 (capped wrapper only).
- One command on the air at a time, globally.
- Style: match the surrounding files. `#Comment` with no space after `#`, explanatory comments on the why, snake_case.
- Do not write or edit `Modification log with claude/` docs beyond this plan and its spec unless the operator asks.
- Keep terminal and log output verbose (operator preference). Don't strip existing diagnostic prints.

---

### Task 1: WriteScheduler state machine

**Files:**
- Create: `Python_GUI/external_control/Utilities/WriteScheduler_utilities.py`
- Test: `Python_GUI/tests/test_write_scheduler.py`

**Interfaces:**
- Produces (used by Task 2):
  - `is_garbled(ack) -> bool`, `ack_answers(ack, address) -> bool`
  - `class Command` with attributes `address, value, label, requested, stamp, max_attempts, log_action, attempts, replaces, result, ack, error`
  - `class WriteScheduler(ack_timeout, reporter, warn_after=5.0, warn_every=1.0)` with methods
    `request(address, value, label="", requested=None, stamp=None, max_attempts=None, log_action=True, force=False) -> Command | None`,
    `next_send(now) -> Command | None`, `mark_sent(cmd, now)`, `on_ack(ack, now)`, `poll(now)`,
    `gui_refused(cmd, error, now)`, `gui_no_reply(cmd, now)`, `emit_warnings(now)`, `time_to_timeout(now) -> float | None`,
    `live_tab(address, now) -> float`, `pending(address) -> bool`, `latest(address)`, `confirmed(address)`, `idle() -> bool`,
    `begin_exit(keep)`, `abandon_all()`, and attribute `warn_every`.
  - Reporter protocol (the caller supplies it): `attempt(cmd, result)`, `command(cmd)`, `say(line, always=False)`.

- [ ] **Step 1: Write the failing tests** in `Python_GUI/tests/test_write_scheduler.py`:

```python
"""WriteScheduler (external_control) as a pure state machine: fake clock, recording reporter, no sockets.

Every timing rule of the one-command-on-the-air scheduler is checked here with exact times. The socket side is
covered by test_openexolink.py. Design: Modification log with claude/specs/2026-09-17-ack-aware-write-scheduler-design.md
"""
import importlib.util
import os

import pytest

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "external_control", "Utilities", "WriteScheduler_utilities.py")
_spec = importlib.util.spec_from_file_location("WriteScheduler_utilities_under_test", _PATH)
ws = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ws)

L_TIME, R_TIME = (68, 13, 2), (36, 13, 2)      # splineAlt plantar peak time, left and right ankle
L_SCALE, R_SCALE = (68, 13, 10), (36, 13, 10)  # splineAlt TorqScale


def ack(address, accepted=True, reason_code=0, reason=None):
    """An ack frame exactly as RemoteControlService.publish_ack broadcasts it."""
    joint, ctrl, idx = address
    return {"stream": "ack", "joint_id": joint, "controller_id": ctrl, "param_index": idx,
            "accepted": accepted, "reason_code": reason_code, "reason": reason}


class Rec:
    """Records everything the scheduler reports."""

    def __init__(self):
        self.attempts, self.commands, self.lines = [], [], []

    def attempt(self, cmd, result):
        self.attempts.append((cmd.address, cmd.value, result, cmd.attempts))

    def command(self, cmd):
        self.commands.append((cmd.address, cmd.value, cmd.result, cmd.attempts))

    def say(self, line, always=False):
        self.lines.append((line.strip(), always))

    def said(self, fragment):
        return [line for line, _ in self.lines if fragment in line]


@pytest.fixture
def sched():
    rec = Rec()
    return ws.WriteScheduler(ack_timeout=1.0, reporter=rec), rec


def send(s, now):
    """Hand out the next command and have the GUI reply ok at the same instant."""
    cmd = s.next_send(now)
    if cmd is not None:
        s.mark_sent(cmd, now)
    return cmd


def test_only_one_command_is_on_the_air(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    assert send(s, 0.0).address == L_TIME
    assert rec.said("L time = 43.0 (first send)")
    assert s.next_send(0.1) is None                 # R waits while L is out
    s.on_ack(ack(L_TIME), 0.2)
    assert send(s, 0.2).address == R_TIME


def test_after_a_timeout_the_other_leg_goes_first_then_the_resend(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)                                    # L1
    s.poll(1.0)                                     # L1 timed out
    assert rec.attempts[-1] == (L_TIME, 43.0, "no_ack", 1)
    assert rec.said("No ACK for L time = 43.0 (attempt 1), pending resend")
    assert send(s, 1.0).address == R_TIME           # the other leg, not the resend
    s.on_ack(ack(R_TIME), 1.1)
    resend = send(s, 1.1)
    assert (resend.address, resend.value, resend.attempts) == (L_TIME, 43.0, 2)
    assert rec.said("L time = 43.0 (resend, attempt 2)")


def test_newer_value_waiting_at_the_timeout_goes_out_instead_of_a_resend(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    send(s, 0.0)
    s.request(L_TIME, 29.0, "L time")               # arrives while 43 is on the air
    s.poll(1.0)
    assert rec.commands == [(L_TIME, 43.0, "superseded (no ACK)", 1)]
    assert rec.said("No ACK for L time = 43.0 (attempt 1), newer value 29.0 waiting")
    nxt = send(s, 1.0)
    assert (nxt.value, nxt.attempts) == (29.0, 1)
    assert rec.said("L time = 29.0 (replaces unconfirmed 43.0)")


def test_newer_value_replaces_a_pending_resend(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)
    s.poll(1.0)                                     # L1 lost, resend pending
    send(s, 1.0)                                    # R goes out meanwhile
    s.request(L_TIME, 29.0, "L time")               # the newer L value arrives before L's turn
    assert rec.commands == [(L_TIME, 43.0, "superseded (no ACK)", 1)]
    s.on_ack(ack(R_TIME), 1.1)
    assert send(s, 1.1).value == 29.0
    assert rec.said("L time = 29.0 (replaces unconfirmed 43.0)")


def test_a_value_replaced_before_it_went_out_is_logged_as_such(sched):
    s, rec = sched
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)                                    # R on the air
    s.request(L_TIME, 43.0, "L time")
    s.request(L_TIME, 29.0, "L time")
    assert rec.commands == [(L_TIME, 43.0, "replaced before sending", 0)]


def test_a_repeat_of_the_latest_request_is_a_no_op_unless_forced(sched):
    s, _ = sched
    assert s.request(L_TIME, 43.0, "L time") is not None
    assert s.request(L_TIME, 43.0, "L time") is None
    assert s.request(L_TIME, 43.0, "L time", force=True) is not None


def test_an_ack_read_during_the_gui_reply_wait_is_not_this_commands(sched):
    s, _ = sched
    s.request(L_TIME, 43.0, "L time")
    cmd = s.next_send(0.0)                          # handed out, the GUI has not replied yet
    s.on_ack(ack(L_TIME), 0.01)                     # so this is a late ack for an earlier command
    assert cmd.result is None
    s.mark_sent(cmd, 0.02)
    s.on_ack(ack(L_TIME), 0.2)
    assert cmd.result == "accepted"


def test_an_ack_for_another_address_is_ignored(sched):
    s, _ = sched
    s.request(L_TIME, 43.0, "L time")
    cmd = send(s, 0.0)
    s.on_ack(ack(R_TIME), 0.1)
    assert cmd.result is None


def test_a_garbled_ack_is_an_immediate_timeout(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    send(s, 0.0)
    s.on_ack(ack(L_TIME, accepted=True, reason_code=1), 0.2)   # "accepted" but garbled: not trusted
    assert rec.attempts[-1] == (L_TIME, 43.0, "garbled_ack", 1)
    assert s.live_tab(L_TIME, 0.2) == pytest.approx(0.2)
    assert rec.said("Garbled ACK for L time = 43.0 (attempt 1), pending resend")
    assert send(s, 0.2).attempts == 2


def test_a_firmware_refusal_drops_the_value_and_clears_the_tab(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    send(s, 0.0)
    s.poll(1.0)                                     # tab 1.0
    send(s, 1.0)
    s.on_ack(ack(L_TIME, accepted=False, reason_code=5, reason="value out of bounds"), 1.1)
    assert rec.commands == [(L_TIME, 43.0, "rejected (value out of bounds)", 2)]
    assert s.live_tab(L_TIME, 1.1) == 0.0
    assert s.idle()
    assert ("Exo REJECTED L time = 43.0: value out of bounds. Value dropped.", True) in rec.lines


def test_the_tab_counts_only_this_addresss_own_time_on_the_air(sched):
    s, _ = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)
    s.poll(1.3)                                     # polled late: still +1.0 at most per attempt
    assert s.live_tab(L_TIME, 1.3) == pytest.approx(1.0)
    send(s, 1.3)                                    # R on the air: L's tab pauses
    assert s.live_tab(L_TIME, 2.0) == pytest.approx(1.0)
    s.on_ack(ack(R_TIME), 2.0)
    send(s, 2.0)                                    # L's resend on the air: L's tab runs again
    assert s.live_tab(L_TIME, 2.5) == pytest.approx(1.5)


def test_the_tab_survives_a_supersede_and_clears_only_on_an_ack(sched):
    s, _ = sched
    s.request(L_TIME, 43.0, "L time")
    send(s, 0.0)
    s.poll(1.0)                                     # 43 lost
    s.request(L_TIME, 29.0, "L time")               # supersedes the pending resend
    assert s.live_tab(L_TIME, 1.0) == pytest.approx(1.0)
    send(s, 1.0)
    s.poll(2.0)                                     # 29 lost too
    assert s.live_tab(L_TIME, 2.0) == pytest.approx(2.0)
    send(s, 2.0)
    s.on_ack(ack(L_TIME), 2.3)
    assert s.live_tab(L_TIME, 2.3) == 0.0


def test_warning_at_5_s_repeats_each_second_and_stops_after_an_ack(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    for k in range(4):                              # four lost attempts, back to back
        send(s, float(k))
        s.poll(float(k + 1))
    send(s, 4.0)                                    # the fifth on the air: closed tab 4.0
    s.emit_warnings(4.9)
    assert not rec.said("WARNING")
    s.poll(5.0)                                     # the fifth lost too: tab 5.0
    s.emit_warnings(5.0)
    assert rec.said("WARNING: no ACK for L time for 5.0 s (5 failed attempts), wanted 43.0")
    send(s, 5.0)
    s.emit_warnings(5.5)
    assert len(rec.said("WARNING")) == 1            # not again within the second
    s.emit_warnings(6.0)
    assert len(rec.said("WARNING")) == 2
    s.on_ack(ack(L_TIME), 6.2)
    s.emit_warnings(8.0)
    assert len(rec.said("WARNING")) == 2            # the ack cleared it


def test_no_warning_once_nothing_is_pending(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    for k in range(5):
        send(s, float(k))
        s.poll(float(k + 1))
    cmd = s.next_send(5.0)
    s.gui_refused(cmd, "bad_name", 5.0)             # the value is dropped: nothing left to deliver
    s.emit_warnings(6.0)
    assert not rec.said("WARNING")
    assert ("GUI REFUSED L time = 43.0: bad_name. Value dropped.", True) in rec.lines


def test_a_capped_command_gives_up(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time", max_attempts=3)
    for k in range(3):
        send(s, float(k))
        s.poll(float(k + 1))
    assert [a[2] for a in rec.attempts] == ["no_ack", "no_ack", "no_ack", "gave_up"]
    assert s.idle()


def test_no_reply_from_the_gui_is_a_failed_attempt(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    cmd = s.next_send(0.0)
    s.gui_no_reply(cmd, 2.0)                        # the client waited 2 s for the GUI's reply
    assert rec.attempts[-1] == (L_TIME, 43.0, "gui_no_reply", 1)
    assert s.live_tab(L_TIME, 2.0) == pytest.approx(1.0)
    assert send(s, 2.0).attempts == 2


def test_exit_keeps_only_the_park_and_lets_the_on_air_command_resolve(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)                                    # L timing on the air
    s.begin_exit([L_SCALE, R_SCALE])
    assert (R_TIME, 43.0, "unconfirmed at exit", 0) in rec.commands
    assert s.request(R_TIME, 50.0, "R time") is None          # nothing but the park is taken now
    s.request(L_SCALE, 0.0, "L scale")
    s.request(R_SCALE, 0.0, "R scale")
    s.poll(1.0)                                     # L timing lost: not resent
    assert (L_TIME, 43.0, "unconfirmed at exit", 1) in rec.commands
    assert send(s, 1.0).address == L_SCALE


def test_abandon_all_logs_every_unresolved_command(sched):
    s, rec = sched
    s.request(L_TIME, 43.0, "L time")
    s.request(R_TIME, 43.0, "R time")
    send(s, 0.0)
    s.abandon_all()
    assert sorted(rec.commands) == sorted([(L_TIME, 43.0, "unconfirmed at exit", 1),
                                           (R_TIME, 43.0, "unconfirmed at exit", 0)])
    assert s.idle()


def test_only_loop_commands_reach_the_machine_action_log(sched):
    s, rec = sched
    s.request(L_SCALE, 0.0, "L scale", log_action=False)       # a setup write
    send(s, 0.0)
    s.on_ack(ack(L_SCALE), 0.1)
    assert rec.commands == []
    assert rec.attempts == [(L_SCALE, 0.0, "accepted", 1)]


def test_latest_and_confirmed(sched):
    s, _ = sched
    s.request(L_TIME, 43.0, "L time")
    assert (s.latest(L_TIME), s.confirmed(L_TIME)) == (43.0, None)
    send(s, 0.0)
    s.on_ack(ack(L_TIME), 0.1)
    assert s.confirmed(L_TIME) == 43.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_write_scheduler.py`
Expected: an error at collection, because `WriteScheduler_utilities.py` does not exist yet.

- [ ] **Step 3: Write `Python_GUI/external_control/Utilities/WriteScheduler_utilities.py`:**

```python
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
        self.result = None                  #None until resolved - see _resolve() callers for the possible results
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_write_scheduler.py`
Expected: 20 passed.

- [ ] **Step 5: Leave uncommitted** (the operator commits).

---

### Task 2: OpenExoLink V0.3 on the scheduler

**Files:**
- Modify: `Python_GUI/external_control/Utilities/OpenExoLink_utilities.py`. The header comment (lines 1-24), the constants (lines 42-57), `_AckTrackingRemote` (lines 69-81) and `OpenExoLink.__init__` (lines 100-119) are replaced. `connect`, `refresh_matrix`, `print_matrix` and the resolve section (lines 121-249) stay unchanged. Everything from `############################### Writing parameters` (line 251) to the end of the file is replaced.
- Modify: `Python_GUI/tests/test_openexolink.py`

**Interfaces:**
- Consumes (Task 1): `WriteScheduler`, `Command` attributes, the Reporter protocol.
- Produces (used by Tasks 3-4):
  - `OpenExoLink(host, port, timeout=2.0, ack_timeout=1.0, max_retries=5, verbose=1, logfile=None, action_logfile=None)`, with attributes `elapsed_origin`, `link_down`, `scheduler`.
  - `request(address, value, label="", requested=None, stamp=None, force=False) -> Command | None`
  - `set_param_confirmed(address, value, label="", max_retries=<link default>) -> ack dict`. `max_retries=None` means unbounded.
  - `wait_for(commands)`, `service(budget=0.0)`, `pump(budget=0.01)`, `latest(address)`, `confirmed(address)`, `begin_exit(keep)`, `close()`.
  - Unchanged: `connect`, `resolve`, `print_matrix`, `matrix`, `connected`.

- [ ] **Step 1: Update the tests** in `Python_GUI/tests/test_openexolink.py`.

Replace the `FakeGui` class with:

```python
class FakeGui:
    """script[n] is the list of (delay_s, ack_frame) to push after the n-th set_param (0-based). With auto_ack_delay
    set, a set_param with no script entry is answered with a correct ack for its own address after that delay, unless
    n is in `lose`. Otherwise a missing entry means "no ack at all"."""

    def __init__(self, script=None, refuse=False, auto_ack_delay=None, lose=()):
        self.script = script or {}
        self.refuse = refuse
        self.auto_ack_delay = auto_ack_delay
        self.lose = set(lose)
        self.set_params = []                      # (receive time, message)
        self.pushes = []                          # (push time, n, frame) for every ack actually sent
        self.client = None                        # where the link listens, for status frames
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _send(self, obj, addr):
        try:
            self.sock.sendto(json.dumps(obj).encode("utf-8"), addr)
        except OSError:
            pass

    def _push(self, n, frame, addr):
        self.pushes.append((time.time(), n, frame))
        self._send(frame, addr)

    def push_status(self, event, **extra):
        """A status frame exactly as RemoteControlService.publish_status broadcasts it."""
        self._send({"stream": "status", "event": event, **extra}, self.client)

    def _run(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            self.client = addr
            msg = json.loads(data.decode("utf-8"))
            reply = {"ok": True, "id": msg.get("id")}
            if msg.get("cmd") == "get_matrix":
                reply["matrix"] = [["Ankle(L) (68)", "68", "splineAlt", "13"] + [f"p{i}" for i in range(17)]]
                reply["names"] = []
            if msg.get("cmd") == "set_param":
                if self.refuse:
                    self._send({"ok": False, "id": msg.get("id"), "error": "nope", "code": "bad_name"}, addr)
                    continue
                n = len(self.set_params)
                self.set_params.append((time.time(), msg))
                frames = self.script.get(n)
                if frames is None and self.auto_ack_delay is not None and n not in self.lose:
                    frames = [(self.auto_ack_delay, ack(int(msg["joint"]), int(msg["controller"]), int(msg["param"])))]
                for delay, frame in frames or []:
                    t = threading.Timer(delay, self._push, args=(n, frame, addr))
                    t.daemon = True
                    t.start()
            self._send(reply, addr)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.sock.close()
```

Replace the `make_link` fixture with:

```python
@pytest.fixture
def make_link():
    made = []

    def _make(script=None, ack_timeout=1.0, max_retries=5, refuse=False, auto_ack_delay=None, lose=(),
              verbose=0, action_log=False):
        gui = FakeGui(script, refuse=refuse, auto_ack_delay=auto_ack_delay, lose=lose)
        log = io.StringIO()
        link = link_mod.OpenExoLink(port=gui.port, ack_timeout=ack_timeout, max_retries=max_retries,
                                    verbose=verbose, logfile=log,
                                    action_logfile=io.StringIO() if action_log else None)
        link.connect(matrix_timeout=2.0)
        made.append((gui, link))
        return gui, link, log

    yield _make
    for gui, link in made:
        link.close()
        gui.close()
```

After `log_rows`, add these helpers:

```python
def service_until(link, done, limit=5.0):
    """Drive the link the way the experiment loop does, until done() or the limit."""
    end = time.time() + limit
    while not done() and time.time() < end:
        link.service(0.05)
    assert done(), "timed out waiting on the link"


def assert_one_on_the_air(gui, ack_timeout):
    """Every set_param after the first must come after the previous one was answered, or had timed out."""
    for n in range(1, len(gui.set_params)):
        prev_sent = gui.set_params[n - 1][0]
        answered = [t for t, k, _ in gui.pushes if k == n - 1]
        free_at = min(answered + [prev_sent + ack_timeout])
        assert gui.set_params[n][0] >= free_at - 0.02, f"set_param {n} went out while {n - 1} was on the air"
```

Replace `test_attempt_is_written_off_before_it_is_resent` (and its `monkeypatch` of the removed `ACK_WRITE_OFF_MARGIN`) with:

```python
def test_ack_just_before_the_timeout_is_credited(make_link):
    # V0.2 wrote an attempt off 0.1 s before its resend. With one command on the air the timeout itself closes it,
    # so an ack at 0.85 s of a 1.0 s timeout now confirms the write instead of being thrown away.
    gui, link, log = make_link({0: [(0.85, ack())]}, ack_timeout=1.0)
    link.set_param_confirmed(ADDR, 50.0)
    assert len(gui.set_params) == 1
    assert log_rows(log) == [("accepted", "1")]
```

At the end of the file, append:

```python
def test_loop_writes_keep_one_command_on_the_air_and_alternate_legs(make_link):
    gui, link, _ = make_link(auto_ack_delay=0.05, lose={0}, ack_timeout=0.4)
    left, right = (68, 13, 2), (36, 13, 2)
    a = link.request(left, 43.0, "L time")
    b = link.request(right, 43.0, "R time")
    service_until(link, lambda: a.result and b.result)
    sent = [(m["joint"], m["value"]) for _, m in gui.set_params]
    assert sent == [(68, 43.0), (36, 43.0), (68, 43.0)]     # L lost -> R goes next -> then L's resend
    assert (a.result, a.attempts, b.result) == ("accepted", 2, "accepted")
    assert_one_on_the_air(gui, 0.4)


def test_a_newer_value_replaces_the_resend_on_the_wire(make_link, capsys):
    gui, link, _ = make_link(auto_ack_delay=0.05, lose={0}, ack_timeout=0.4, verbose=1, action_log=True)
    left = (68, 13, 2)
    old = link.request(left, 43.0, "L time")
    service_until(link, lambda: len(gui.set_params) == 1)
    new = link.request(left, 29.0, "L time")      # arrives while 43 is on the air, and 43's ack is lost
    service_until(link, lambda: new.result is not None)
    assert [m["value"] for _, m in gui.set_params] == [43.0, 29.0]
    assert (old.result, new.result) == ("superseded (no ACK)", "accepted")
    out = capsys.readouterr().out
    assert "L time = 43.0 (first send)" in out
    assert "No ACK for L time = 43.0 (attempt 1), newer value 29.0 waiting" in out
    assert "L time = 29.0 (replaces unconfirmed 43.0)" in out
    rows = [line.split(",") for line in link.action_logfile.getvalue().splitlines()]
    assert [(r[2], r[4], r[5], r[6]) for r in rows] == [("L time", "43.0", "superseded (no ACK)", "1"),
                                                        ("L time", "29.0", "accepted", "1")]


def test_gui_disconnect_sets_link_down_and_stops_a_blocking_write(make_link):
    gui, link, _ = make_link({}, ack_timeout=5.0)
    threading.Timer(0.2, gui.push_status, args=("disconnected",)).start()
    with pytest.raises(link_mod.OpenExoLinkError):
        link.set_param_confirmed(ADDR, 50.0, max_retries=None)
    assert link.link_down == 1


def test_a_device_error_is_ignored(make_link, capsys):
    gui, link, _ = make_link({0: [(0.3, ack())]})
    threading.Timer(0.1, gui.push_status, args=("device_error",), kwargs={"message": "Not connected"}).start()
    link.set_param_confirmed(ADDR, 50.0)
    assert link.link_down == 0
    assert "Not connected" not in capsys.readouterr().out
```

- [ ] **Step 2: Run the tests to verify the new ones fail**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_openexolink.py`
Expected: FAIL. The fixture passes `action_logfile`, which V0.2's constructor does not accept.

- [ ] **Step 3: Rewrite `OpenExoLink_utilities.py`.**

Replace lines 1-24 (the header comment) with:

```python
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
#   3) Some acks arrive GARBLED ("rejected, reason 1") when the Nano could not parse the Teensy's reply. The Teensy
#      may well have applied the value, so such an ack means "unknown", not "refused": it counts as a failed attempt.
#   4) Two ways to write, one engine. request() is the experiment loop's: it returns at once, and service() - called
#      every loop pass instead of sleeping - sends, retries and lets a newer value replace a pending resend.
#      set_param_confirmed() blocks until the ack, for session setup and the stress test. See
#      "Modification log with claude/specs/2026-09-17-ack-aware-write-scheduler-design.md".
#V0.1 2026 Sep
#V0.2 2026 Sep: garbled-ack resend, per-attempt logging, attempt bookkeeping, 1 s / 5 attempt defaults
#V0.3 2026 Sep 18: one command on the air, non-blocking request()/service(), newer values replace resends, no-ack
#   warnings instead of giving up, machine action log per command, status frames caught during set_param replies
```

After the existing `from client import ExoRemote, RemoteError  # noqa: E402` line, add:

```python

#The scheduler sits next to this file. Imported by path as well, because this file is itself loaded by path in the tests
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from WriteScheduler_utilities import WriteScheduler  # noqa: E402
```

Replace lines 42-57 (from `#How long we listen for an ack` through `GARBLED_REASON_CODE = 1`) with:

```python
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
```

Replace `_AckTrackingRemote` (lines 69-81) with:

```python
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
```

In the `OpenExoLink` class docstring, replace the paragraph starting `This class owns the UDP connection` and its `Inputs:` list with:

```python
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
```

Replace `__init__` (lines 100-119) with:

```python
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
```

Replace everything from `    ############################### Writing parameters` to the end of the file with:

```python
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
        while any(c.result is None for c in pending):
            if self.link_down:
                raise OpenExoLinkError("The GUI reports the exo disconnected")
            self.service(ACK_POLL_INTERVAL)

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

    def service(self, budget=0.0):
        '''Run the write scheduler. Read every waiting frame, resolve the command on the air (its ack, or its timeout),
        send the next command, print due warnings - then wait on the socket for up to `budget` seconds, doing the same
        the moment each frame arrives. The experiment loop calls this once per pass with its idle time as the budget,
        in place of sleeping, so an ack is acted on at once. budget=0 is one non-blocking pass.

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

    def pump(self, budget=0.01):
        '''A short service() pass. Kept for stress_test_ble.py, which calls it between writes to notice a disconnect.

        Be honest about what this can see: the GUI reports a disconnect only after BLE gives up, about 9.6 seconds
        after the exo actually went quiet. A backstop, not a fast watchdog.'''
        self.service(budget)

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
```

- [ ] **Step 4: Run the link tests and the scheduler tests**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_openexolink.py Python_GUI/tests/test_write_scheduler.py`
Expected: all pass. That's 15 in `test_openexolink.py` (the V0.2 set, one rewritten, plus 4 new) and 20 in `test_write_scheduler.py`.

- [ ] **Step 5: Leave uncommitted.**

---

### Task 3: ActionMap on the new calls

**Files:**
- Modify: `Python_GUI/external_control/Utilities/ActionMap_utilities.py:58-168`
- Create: `Python_GUI/tests/test_actionmap.py`

**Interfaces:**
- Consumes (Task 2): `set_param_confirmed(..., max_retries=None)`, `request(...)`, `latest(address)`, `begin_exit(keep)`, `wait_for(commands)`, `resolve(...)`.
- Produces (used by Task 4): `apply_torque_percentage(_scale, stamp=None) -> float` (non-blocking), `apply_peak_timing(lobe, percent_gait, stamp=None, wait=False) -> float`, `latest_torque_request() -> float | None`, `park_to_transparency() -> 1 | 0`. `last_applied_action` is removed.

- [ ] **Step 1: Write the failing tests** in `Python_GUI/tests/test_actionmap.py`:

```python
"""SplineAlt_action_map on OpenExoLink V0.3, against a fake link: which writes block, which only request, the park.
Also main_external_control's park-or-skip exit decision."""
import importlib.util
import os
import sys
import types

import pytest

_EXT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "external_control")
_spec = importlib.util.spec_from_file_location("ActionMap_utilities_under_test",
                                               os.path.join(_EXT, "Utilities", "ActionMap_utilities.py"))
am_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(am_mod)

L_SCALE, R_SCALE = (68, 13, 10), (36, 13, 10)


class FakeLink:
    """Records every call. wait_for resolves the park commands with park_result, or raises what it is told to."""

    def __init__(self):
        self.calls = []
        self._latest = {}
        self.exit_keep = None
        self.park_result = "accepted"
        self.wait_raises = None

    def resolve(self, joint, controller, param):
        return ({"Ankle(L)": 68, "Ankle(R)": 36}[joint], 13, param)

    def set_param_confirmed(self, address, value, label="", max_retries="link default"):
        self.calls.append(("confirmed", address, value, label, max_retries))
        self._latest[address] = value

    def request(self, address, value, label="", requested=None, stamp=None, force=False):
        self.calls.append(("request", address, value, label, requested, stamp, force))
        self._latest[address] = value
        return types.SimpleNamespace(result=None, address=address)

    def latest(self, address):
        return self._latest.get(address)

    def begin_exit(self, keep):
        self.exit_keep = list(keep)

    def wait_for(self, commands):
        if self.wait_raises is not None:
            raise self.wait_raises
        for c in commands:
            c.result = self.park_result


@pytest.fixture
def amap():
    link = FakeLink()
    return am_mod.SplineAlt_action_map(link, body_mass_standard=75, verbose=0), link


def test_engage_blocks_until_confirmed_with_no_cap(amap):
    m, link = amap
    m.engage_controller_safely()
    assert link.calls == [("confirmed", L_SCALE, 0.0, "Ankle(L) TorqScale", None),
                          ("confirmed", R_SCALE, 0.0, "Ankle(R) TorqScale", None)]


def test_magnitudes_block_until_confirmed_with_no_cap(amap):
    m, link = amap
    assert m.apply_torque_magnitudes(15.0, 8.0, body_mass=150) == 2.0
    assert [(c[1][2], c[2], c[4]) for c in link.calls] == [(0, 30.0, None), (0, 30.0, None),
                                                           (1, 16.0, None), (1, 16.0, None)]


def test_torque_percentage_only_requests_and_keeps_the_raw_value(amap):
    m, link = amap
    assert m.apply_torque_percentage(130.0, stamp=12.5) == 100.0
    assert link.calls == [("request", L_SCALE, 100.0, "Ankle(L) TorqScale", 130.0, 12.5, False),
                          ("request", R_SCALE, 100.0, "Ankle(R) TorqScale", 130.0, 12.5, False)]


def test_peak_timing_requests_in_the_loop_and_blocks_at_setup(amap):
    m, link = amap
    m.apply_peak_timing("plantar", 43, stamp=1.0)
    assert [c[0] for c in link.calls] == ["request", "request"]
    assert link.calls[0][3] == "Ankle(L) plantar peak time"
    link.calls.clear()
    m.apply_peak_timing("plantar", 43, wait=True)
    assert [(c[0], c[4]) for c in link.calls] == [("confirmed", None), ("confirmed", None)]
    with pytest.raises(ValueError):
        m.apply_peak_timing("heel", 43)


def test_latest_torque_request(amap):
    m, link = amap
    assert m.latest_torque_request() is None
    m.apply_torque_percentage(0.0)
    assert m.latest_torque_request() == 0.0
    link._latest[R_SCALE] = 30.0
    assert m.latest_torque_request() is None             # the legs disagree


def test_park_keeps_only_torque_scale_and_waits_for_both(amap, capsys):
    m, link = amap
    assert m.park_to_transparency() == 1
    assert link.exit_keep == [L_SCALE, R_SCALE]
    assert [(c[1], c[2], c[6]) for c in link.calls] == [(L_SCALE, 0.0, True), (R_SCALE, 0.0, True)]
    assert "Parked both ankles" in capsys.readouterr().out


def test_second_ctrl_c_abandons_the_park(amap, capsys):
    m, link = amap
    link.wait_raises = KeyboardInterrupt()
    assert m.park_to_transparency() == 0
    assert "park NOT confirmed" in capsys.readouterr().out


def test_park_on_a_dead_link_reports_not_confirmed(amap, capsys):
    m, link = amap
    link.wait_raises = RuntimeError("The GUI reports the exo disconnected")
    assert m.park_to_transparency() == 0
    assert "park NOT confirmed" in capsys.readouterr().out


############################### main_external_control's exit decision #############################################

sys.path.insert(0, _EXT)      # main_external_control imports Utilities.* relative to its own folder
_main_spec = importlib.util.spec_from_file_location("main_external_control_under_test",
                                                    os.path.join(_EXT, "main_external_control.py"))
main_mod = importlib.util.module_from_spec(_main_spec)
_main_spec.loader.exec_module(main_mod)


class ParkRecorder:
    def __init__(self):
        self.parked = 0

    def park_to_transparency(self):
        self.parked += 1
        return 1


def test_a_normal_exit_parks():
    rec = ParkRecorder()
    assert main_mod._park_or_skip(rec, link_lost=0) == 1
    assert rec.parked == 1


def test_parking_is_skipped_when_the_gui_lost_the_exo(capsys):
    rec = ParkRecorder()
    assert main_mod._park_or_skip(rec, link_lost=1) == 0
    assert rec.parked == 0
    assert "parking is skipped" in capsys.readouterr().out
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_actionmap.py`
Expected: FAIL. ActionMap still blocks on every write and has no `latest_torque_request`, and `_park_or_skip` doesn't exist yet.

- [ ] **Step 3: Update `ActionMap_utilities.py`.**

In `__init__`, delete these two lines:

```python
        self.last_applied_action = None     #The last torque percentage we successfully wrote to BOTH sides

```

In `engage_controller_safely`, replace the loop body and the `last_applied_action` line:

```python
        for addr, joint in zip(self._torque_scale_addr, self.joints):   #Walk every joint we were given, one write each
            self.exo_link.set_param_confirmed(addr, 0.0, label=f"{joint} TorqScale")
        self.last_applied_action = 0.0  #This should read as "we just deliberately applied zero"
```

with:

```python
        #Setup waits for the ack and never gives up (max_retries=None): nothing else may be written until the joint is
        #on splineAlt at zero, or a later write would do the switch itself and bring up the SD card's non-zero TorqScale
        for addr, joint in zip(self._torque_scale_addr, self.joints):   #Walk every joint we were given, one write each
            self.exo_link.set_param_confirmed(addr, 0.0, label=f"{joint} TorqScale", max_retries=None)
```

In `apply_torque_magnitudes`, add `max_retries=None` to both `set_param_confirmed` calls:

```python
        for addr, joint in zip(self._plantar_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_plantar, label=f"{joint} PlantarNm", max_retries=None)
        for addr, joint in zip(self._dorsi_nm_addr, self.joints):
            self.exo_link.set_param_confirmed(addr, scaled_dorsi, label=f"{joint} DorsiNm", max_retries=None)
```

Replace `apply_torque_percentage`, `apply_peak_timing` and `park_to_transparency` (lines 129-168) with:

```python
    def apply_torque_percentage(self, _scale, stamp=None):
        '''Request a torque scaling percentage on both ankles. Returns at once with the value that will be sent, after
        clamping. The link delivers it (resent until acked, or replaced by a newer value) and logs its fate per leg in
        the machine action log.

        stamp: when the action was produced, on the caller's time.perf_counter() clock, for that log'''
        _scale_clamped = min(max(float(_scale), 0.0), self.torque_percentage_max)  #Floor at 0 as well as cap at m_max - a negative scale would flip both lobes
        if _scale_clamped != float(_scale) and self.verbose:      #Only say something when we actually changed the backend's number
            print(f"  Machine action {_scale:.2f}% clamped to {_scale_clamped:.2f}% by our own safety limit")

        for addr, joint in zip(self._torque_scale_addr, self.joints):   #Every joint gets the same scale, so both legs assist identically
            self.exo_link.request(addr, _scale_clamped, label=f"{joint} TorqScale", requested=float(_scale), stamp=stamp)
        return _scale_clamped    #The caller should log THIS, not its own scale - the two differ whenever the clamp fired

    def apply_peak_timing(self, lobe, percent_gait, stamp=None, wait=False):
        '''Move the peak of one lobe ("plantar" or "dorsi") to a new percent of the gait cycle, on both sides.
            This is what a UDP timing value drives, and it is the ankle equivalent of the hip exo's create_flexion_only_torque_profile(udp_timing_value).

            Unlike the hip exo, where a new timing meant rebuilding the whole spline in Python, here it is one parameter write per leg - the Teensy rebuilds its own nodes from it.

            In the loop this only REQUESTS the write and returns at once. wait=True is for session setup (the fixed
            plantar/dorsi modes): block until both legs acked, never giving up.'''
        if lobe not in self._peak_time_addr:    #Catch a mistyped lobe name here, before we have written anything to the exo
            raise ValueError(f"lobe must be 'plantar' or 'dorsi', got {lobe!r}")
        for addr, joint in zip(self._peak_time_addr[lobe], self.joints):    #Every joint, same new peak time
            label = f"{joint} {lobe} peak time"
            if wait:
                self.exo_link.set_param_confirmed(addr, float(percent_gait), label=label, max_retries=None)
            else:
                self.exo_link.request(addr, float(percent_gait), label=label, stamp=stamp)
        return float(percent_gait)  #Hand back what we actually wrote, so the caller logs the value the exo has rather than the one it asked for

    def latest_torque_request(self):
        '''The torque percentage most recently REQUESTED on both legs, if they agree, else None. Requested rather than
        confirmed, because the link delivers every request (or keeps warning that it cannot) - so "already asked for
        zero" is the right test before asking for zero again'''
        values = [self.exo_link.latest(addr) for addr in self._torque_scale_addr]
        if values and all(v is not None and v == values[0] for v in values):
            return values[0]
        return None

    def park_to_transparency(self):
        '''Drop both sides to zero torque scale on the way out, and wait until both legs acknowledged it - however many
        attempts that takes. Everything else still waiting to be written is dropped first, so only the park goes out.
        A second Ctrl-C abandons the wait. Returns 1 if both sides acknowledged the park, 0 if not.'''
        try:
            self.exo_link.begin_exit(self._torque_scale_addr)
            commands = [self.exo_link.request(addr, 0.0, label=f"{joint} TorqScale", force=True)
                        for addr, joint in zip(self._torque_scale_addr, self.joints)]
            print("Parking both ankles to zero torque scale (transparency). Ctrl-C again to abandon.")
            self.exo_link.wait_for(commands)
        except KeyboardInterrupt:
            print("park NOT confirmed - abandoned by the operator. The exo keeps whatever it last accepted. "
                  "Stop the trial from the GUI.")
            return 0
        except Exception as e:  #Catch everything else on purpose - this runs in shutdown paths, so it must never raise on its own
            print(f"WARNING: park NOT confirmed: {e}")
            print("The exo keeps whatever it last accepted. Stop the trial from the GUI.")
            return 0
        if all(c.result == "accepted" for c in commands):
            print("Parked both ankles to zero torque scale (transparency)")
            return 1
        print(f"WARNING: park NOT confirmed: {[c.result for c in commands]}. Stop the trial from the GUI.")
        return 0
```

- [ ] **Step 4: Run the ActionMap tests.** Its `_park_or_skip` tests still fail until Task 4.

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests/test_actionmap.py`
Expected: the 8 ActionMap tests pass. The 2 main tests fail with `AttributeError: ... has no attribute '_park_or_skip'`, which is added in Task 4.

- [ ] **Step 5: Leave uncommitted.**

---

### Task 4: main_external_control on the new loop

**Files:**
- Modify: `Python_GUI/external_control/main_external_control.py`
- Test: `Python_GUI/tests/test_actionmap.py` (its two `_park_or_skip` tests, written in Task 3)

**Interfaces:**
- Consumes: `OpenExoLink(..., action_logfile=...)`, `.elapsed_origin`, `.link_down`, `.service(budget)`; ActionMap's `apply_torque_percentage(_, stamp)`, `apply_peak_timing(_, _, stamp, wait)`, `latest_torque_request()`, `park_to_transparency()`.
- Produces: `_park_or_skip(action_map, link_lost) -> int`.

- [ ] **Step 1: Add `_park_or_skip`** right after `_load_module_from_file`:

```python
def _park_or_skip(action_map, link_lost):
    '''On the way out, park to transparency - unless the GUI lost the exo. Then nothing can reach it, and the park
    would only retry forever. Returns what park_to_transparency returned, or 0 when skipped.'''
    if link_lost:
        print("The exo is unreachable, so parking is skipped. It keeps whatever it last accepted - "
              "stop the trial from the GUI.")
        return 0
    return action_map.park_to_transparency()
```

- [ ] **Step 2: Config and link construction.**

Replace:

```python
    ack_timeout = 1.0       #How long we wait for the exo to acknowledge a parameter write before resending, in seconds. Every unilateral write in the GUI logs up to 2026-09-16 was acked within 0.45 s
    max_write_retries = 5   #How many attempts a write gets before we give up. Silence and garbled acks both use one up
```

with:

```python
    ack_timeout = 1.0       #How long a write waits for the exo's acknowledgement before it is resent (or replaced by a newer value), in seconds. Every ack seen on the exo up to 2026-09-17 came within 0.57 s
    #There is no retry cap: setup writes wait until acked, and loop writes are resent until acked or replaced. A long
    #silence prints a warning instead - a human is always watching this terminal
```

Replace the machine action log header line:

```python
        print(f"Python time,Elapsed time,Source,Value requested,Value applied", file=action_log_file)
```

with:

```python
        #One row per COMMAND (one leg, one parameter), written by OpenExoLink once its fate is known. Python time is when
        #it was requested. Result: accepted, superseded (no ACK), replaced before sending, rejected (<reason>),
        #gui_refused, unconfirmed at exit
        print(f"Python time,Elapsed time,Target,Value requested,Value sent,Result,Attempts", file=action_log_file)
```

Replace:

```python
    OpenExo_link = OpenExoLink(host=gui_host, port=gui_port, ack_timeout=ack_timeout, max_retries=max_write_retries, verbose=1, logfile=write_log_file)
```

with:

```python
    OpenExo_link = OpenExoLink(host=gui_host, port=gui_port, ack_timeout=ack_timeout, verbose=1,
                               logfile=write_log_file, action_logfile=action_log_file)
```

- [ ] **Step 3: Setup peak timing blocks. Stamp the loop start.**

Replace:

```python
            action_map.apply_peak_timing(timing_lobe, peak_timing_value)
```

with:

```python
            action_map.apply_peak_timing(timing_lobe, peak_timing_value, wait=True)   #Setup: wait for both acks
```

Replace:

```python
    input("Press enter to start running the exo:\n")
    start_time = time.perf_counter()  # start a timer
```

with:

```python
    input("Press enter to start running the exo:\n")
    start_time = time.perf_counter()  # start a timer
    OpenExo_link.elapsed_origin = start_time    #The machine action log's Elapsed column counts from here
```

- [ ] **Step 4: The loop.**

Replace:

```python
    try:
        while True:
            loop_start_time = time.perf_counter()

            #Notice a dropped link between writes. This is a backstop, not a fast watchdog - the GUI only
            #reports a disconnect once BLE gives up, which is several seconds after the exo goes quiet
            OpenExo_link.pump()
            if OpenExo_link.link_down:
                print("The GUI reports the exo is no longer connected. Stopping - no further actions will be sent.")
                break
```

with:

```python
    link_lost = 0   #Set when the GUI reports the exo disconnected. Parking is skipped then - nothing can reach the exo
    try:
        while True:
            loop_start_time = time.perf_counter()

            #The GUI reporting the exo disconnected is the ONE thing that ends this loop on its own: the Nano cannot be
            #reconnected mid-trial without side effects. Ack trouble never ends it - the link prints warnings instead.
            #The GUI reports a disconnect about 9.6 s after the exo goes quiet (the BLE supervision timeout)
            if OpenExo_link.link_down:
                print("The GUI reports the exo is no longer connected. Stopping - no further actions will be sent.")
                link_lost = 1
                break
```

Replace the two UDP-mode `action_map.last_applied_action == 0` checks. First:

```python
                #  1) NOT ALREADY ZERO. action_map.last_applied_action is only assigned after EVERY joint
                #     has acknowledged (ActionMap_utilities.apply_torque_percentage), so `== 0` means zero
                #     is confirmed on BOTH legs, not just requested. It starts as None, so the first zero
                #     is never skipped, and a half-applied write leaves it at its previous value - in both
                #     of those cases we correctly fall through and write.
```

becomes:

```python
                #  1) NOT ALREADY ZERO. latest_torque_request() is what we last REQUESTED on both legs. Once zero
                #     is requested the link delivers it (resending until acked, warning if it cannot), so asking
                #     again adds nothing. It is None until both legs agree, so a first or half-made zero falls through.
```

and:

```python
                    if action_map.last_applied_action == 0:
                        pending_udp_zero = False        #Already there on both legs. Nothing to send.
```

becomes:

```python
                    if action_map.latest_torque_request() == 0:
                        pending_udp_zero = False        #Already asked for on both legs. Nothing to send.
```

Second:

```python
                        if action_map.last_applied_action == 0:
                            new_torque_percentage = working_torque_percentage
```

becomes:

```python
                        if action_map.latest_torque_request() == 0:
                            new_torque_percentage = working_torque_percentage
```

Replace the two deploy blocks, from `            #Deploy a new peak timing, if one is waiting` through `                new_torque_percentage = None  #Wipe the buffer`, with:

```python
            #Deploy a new peak timing, if one is waiting. This only REQUESTS it: service() below sends it, resends it if
            #its ack goes missing, and lets a newer timing replace a pending resend. Its fate goes in the machine action log
            if new_timing_value is not None:
                _timestamp = loop_start_time if use_loop_time else time.perf_counter()
                action_map.apply_peak_timing(timing_lobe, new_timing_value, stamp=_timestamp)
                new_timing_value = None  #Wipe the buffer

            #Deploy a new torque percentage, if one is waiting. Same: requested here, delivered by service()
            if new_torque_percentage is not None:
                print(f"Applying torque percentage: {new_torque_percentage:.2f}%")
                _timestamp = loop_start_time if use_loop_time else time.perf_counter()
                action_map.apply_torque_percentage(new_torque_percentage, stamp=_timestamp)
                new_torque_percentage = None  #Wipe the buffer
```

Replace the end-of-loop sleep:

```python
            # End of loop operations
            loop_duration = time.perf_counter() - loop_start_time
            sleep_time = (1 / operating_rate) - loop_duration
            if sleep_time < 0:
                sleep_time = 0
            time.sleep(sleep_time)
```

with:

```python
            # End of loop operations. The idle time is spent inside service(), waiting on the socket instead of
            # sleeping, so an ack is acted on the moment it arrives and the next write goes straight out
            loop_duration = time.perf_counter() - loop_start_time
            OpenExo_link.service(budget=max((1 / operating_rate) - loop_duration, 0.0))
```

Replace, in the `finally:` block:

```python
        #Park to transparency FIRST, while the link is most likely still alive
        action_map.park_to_transparency()
```

with:

```python
        #Park to transparency FIRST, while the link is most likely still alive - unless the GUI already lost the exo
        _park_or_skip(action_map, link_lost)
```

- [ ] **Step 5: Check no stale references remain**

Run: `grep -rn "last_applied_action\|max_write_retries\|ACK_WRITE_OFF_MARGIN\|_write_off_age\|_outstanding" Python_GUI/external_control Python_GUI/tests`
Expected: only `stress_test_ble.py`'s own `max_write_retries` (its config, passed to `OpenExoLink(max_retries=...)`). Nothing else.

- [ ] **Step 6: Run the full suite**

Run: `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests`
Expected: all pass. That's 56 at baseline, minus the one rewritten test counted once, plus 4 new link tests, 20 scheduler tests and 10 ActionMap/main tests, so 90 passed.

- [ ] **Step 7: Leave uncommitted.**

---

### Task 5: Verification and handover

**Files:** none new.

- [ ] **Step 1: Import check of both entry points**, from `Python_GUI/external_control`:

Run: `E:\MiniConda\envs\biomotum\python.exe -c "import main_external_control, stress_test_ble; print('ok')"`
Expected: `ok`.

- [ ] **Step 2: Smoke-run `stress_test_ble.py` against a FakeGui on port 9750.** It's the one caller left on the capped blocking write. Write a scratchpad script (outside the repo) that:
  - starts `FakeGui` from `tests/test_openexolink.py` bound to 127.0.0.1:9750 with `auto_ack_delay=0.05` and `lose={3}`;
  - runs `stress_test_ble.py` as a subprocess with `"\n"` on stdin;
  - sends Ctrl-C (`CTRL_BREAK_EVENT` on Windows, or terminates it) after ~5 s;
  - prints the tail of its output and its `param_write_*.txt`.

  Expected:
  - writes are reported ok;
  - the lost one shows a `no_ack` row followed by `accepted` on attempt 2;
  - the run shuts down cleanly.

  If the GUI is running on 9750, skip this step and say so.
- [ ] **Step 3: Review the diff.** Run `git diff --stat` and read `git diff` for the four modified files, checking against the spec's sections 2-5.
- [ ] **Step 4: Leave everything uncommitted.** Report what changed, the test count, any deviation from this plan, and the on-exo tests left for the operator (spec §6: stress test at zero torque, then a worn UDP session).

---

## Execution notes (2026-09-18)

Executed inline, uncommitted. Final suite: **92 passed**. The plan predicted 90; the 2 extra tests come from deviations 1 and 2 below.

Deviations from the plan, each found while executing:
1. **Ctrl-C during a send.** `_send_next` now treats a `BaseException` raised inside `set_param` (Ctrl-C while
   waiting on the GUI's ok) as "sent", then re-raises. Before this, the command stayed on the air with no send time,
   so it never timed out, and the park after Ctrl-C waited behind it forever. Test:
   `test_ctrl_c_during_a_send_does_not_leave_it_stuck_on_the_air`. It was confirmed to fail without the guard.
2. **Blocking writes return at the ack.** `service(budget, until=None)` returns early once `until()` is true, and
   `wait_for` passes it. Without it a confirmed write returned only at the end of its 50 ms slice: the stress-test
   smoke run dropped to 11 writes/s. Test: `test_a_blocking_write_returns_as_soon_as_its_ack_arrives`.
3. **`pump()` never waits** (one `service(0.0)` pass), matching V0.2. The planned 10 ms budget added a wait to every
   stress-test loop.
4. **`stress_test_ble.py`:** `gui_no_reply` added to its per-result summary list (new result type). Nothing else
   changed.

Smoke run of the unchanged stress test against a fake GUI on 9750 (30 ms acks, 3rd ack lost, Ctrl-C at 5 s):
V0.3 got 19.2 writes/s and V0.2 (HEAD) got 20.0. The lost ack became `no_ack`, then a resend was accepted.
Clean shutdown. Its log files were deleted afterwards.
