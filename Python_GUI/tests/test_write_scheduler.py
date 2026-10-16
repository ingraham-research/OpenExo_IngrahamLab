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
