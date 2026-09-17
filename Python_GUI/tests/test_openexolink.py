"""OpenExoLink (external_control) against a fake GUI speaking the real UDP wire protocol.

No Qt, no BLE, no exo. FakeGui answers every command "ok" the way remote/service.py does, and for
each set_param it pushes back whatever ack frames the test's script says, after the scripted delay.
That exercises the real OpenExoLink + real ExoRemote over real localhost UDP.
"""
import importlib.util
import io
import json
import os
import socket
import threading
import time

import pytest

_LINK_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "external_control", "Utilities", "OpenExoLink_utilities.py")
_spec = importlib.util.spec_from_file_location("OpenExoLink_utilities_under_test", _LINK_PATH)
link_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(link_mod)

J, C, P = 68, 13, 10          # left ankle, splineAlt, TorqScale
ADDR = (J, C, P)


def ack(joint=J, ctrl=C, idx=P, accepted=True, reason_code=0):
    """An ack frame exactly as RemoteControlService.publish_ack broadcasts it."""
    return {"stream": "ack", "joint_id": joint, "controller_id": ctrl, "param_index": idx,
            "accepted": accepted, "reason_code": reason_code,
            "reason": {0: "accepted", 1: "invalid message", 5: "value out of bounds"}.get(reason_code)}


GARBLED_ZEROED = ack(ctrl=0, idx=0, accepted=False, reason_code=1)      # (J,0,0,0,1)
GARBLED_FULL = ack(accepted=False, reason_code=1)                       # (J,c,p,0,1)
OUT_OF_BOUNDS = ack(accepted=False, reason_code=5)


class FakeGui:
    """script[n] is the list of (delay_s, ack_frame) to push after the n-th set_param (0-based).
    Missing entries mean "no ack at all"."""

    def __init__(self, script=None, refuse=False):
        self.script = script or {}
        self.refuse = refuse
        self.set_params = []                      # (receive time, message)
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

    def _run(self):
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
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
                for delay, frame in self.script.get(n, []):
                    t = threading.Timer(delay, self._send, args=(frame, addr))
                    t.daemon = True
                    t.start()
            self._send(reply, addr)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.sock.close()


@pytest.fixture
def make_link():
    made = []

    def _make(script=None, ack_timeout=1.0, max_retries=5, refuse=False):
        gui = FakeGui(script, refuse=refuse)
        log = io.StringIO()
        link = link_mod.OpenExoLink(port=gui.port, ack_timeout=ack_timeout, max_retries=max_retries,
                                    verbose=0, logfile=log)
        link.connect(matrix_timeout=2.0)
        made.append((gui, link))
        return gui, link, log

    yield _make
    for gui, link in made:
        link.close()
        gui.close()


def log_rows(log):
    """Param_write_log rows as (Result, Attempts) pairs."""
    return [tuple(field.strip() for field in line.split(",")[3:5])
            for line in log.getvalue().splitlines() if line.strip()]


def test_zeroed_garbled_ack_triggers_an_immediate_resend(make_link):
    gui, link, _ = make_link({0: [(0.05, GARBLED_ZEROED)], 1: [(0.05, ack())]}, ack_timeout=5.0)
    start = time.time()
    result = link.set_param_confirmed(ADDR, 50.0)
    assert result["accepted"] is True
    assert len(gui.set_params) == 2
    assert time.time() - start < 1.0      # resent on the garbled ack, not after the 5 s timeout


def test_full_triple_garbled_ack_is_resent_not_raised(make_link):
    gui, link, _ = make_link({0: [(0.05, GARBLED_FULL)], 1: [(0.05, ack())]})
    result = link.set_param_confirmed(ADDR, 50.0)
    assert result["accepted"] is True
    assert len(gui.set_params) == 2


def test_accepted_ack_with_garbled_reason_is_not_trusted(make_link):
    # (J,c,p,accepted=1,reason=1) cannot come from the Teensy - the frame was damaged in transit, so any
    # of its fields may be wrong. Resending costs one round trip; trusting it could confirm a wrong write.
    gui, link, _ = make_link({0: [(0.05, ack(accepted=True, reason_code=1))], 1: [(0.05, ack())]})
    result = link.set_param_confirmed(ADDR, 50.0)
    assert result["reason_code"] == 0
    assert len(gui.set_params) == 2


def test_real_rejection_still_raises_without_resending(make_link):
    gui, link, _ = make_link({0: [(0.05, OUT_OF_BOUNDS)]})
    with pytest.raises(link_mod.OpenExoLinkError) as err:
        link.set_param_confirmed(ADDR, 500.0)
    assert err.value.reason == "value out of bounds"
    assert len(gui.set_params) == 1


def test_a_lost_ack_does_not_tax_the_following_writes(make_link):
    # A1's ack is lost on a bad link and A2 confirms A. The unanswered A1 must be written off when it times
    # out - otherwise every later write to the same parameter, sent at heel-strike rate, needs an extra
    # attempt, and each further lost ack adds another, until writes start giving up.
    script = {1: [(0.05, ack())],     # A1 (n=0) lost, A2 answered
              2: [(0.05, ack())],     # B1
              3: [(0.05, ack())],     # C1
              4: [(0.05, ack())]}     # D1
    gui, link, log = make_link(script, ack_timeout=0.5)
    for value in (50.0, 60.0, 70.0, 80.0):
        link.set_param_confirmed(ADDR, value)
    assert len(gui.set_params) == 5
    assert [row for row in log_rows(log) if row[0] == "accepted"] == [("accepted", "2"), ("accepted", "1"),
                                                                     ("accepted", "1"), ("accepted", "1")]


def test_attempt_is_written_off_before_it_is_resent(make_link, monkeypatch):
    # Write-off must come strictly BEFORE the resend, so no attempt is ever still open when its successor goes
    # out. An ack landing in that gap (after write-off, before the timeout) is therefore not credited, and the
    # write is resent at the timeout as usual. Margin widened here only to keep the test's timing robust.
    monkeypatch.setattr(link_mod, "ACK_WRITE_OFF_MARGIN", 0.3)      # timeout 1.0 -> written off at 0.7 s
    script = {0: [(0.85, ack())],     # A1 -> 0.85 s: after write-off, before the 1.0 s resend
              1: [(0.05, ack())]}     # A2
    gui, link, log = make_link(script, ack_timeout=1.0)
    link.set_param_confirmed(ADDR, 50.0)
    assert len(gui.set_params) == 2
    assert log_rows(log) == [("no_ack", "1"), ("accepted", "2")]


def test_ack_arriving_between_writes_is_not_taken_by_the_next_write(make_link):
    # A1 times out and A2 confirms A. A1's own ack then turns up after A has returned but before B is sent.
    # It must be dropped, not held over to confirm B - whose own first attempt is lost here.
    script = {0: [(0.75, ack())],     # A1 -> ~0.75 s, after A2 (sent ~0.5 s) already confirmed A, before B (~0.85 s)
              1: [(0.05, ack())],     # A2 -> ~0.55 s
              # 2: B1 lost
              3: [(0.05, ack())]}     # B2
    gui, link, log = make_link(script, ack_timeout=0.5)
    link.set_param_confirmed(ADDR, 50.0)          # A
    time.sleep(0.3)                               # A1's late ack lands in this gap
    link.set_param_confirmed(ADDR, 60.0)          # B
    assert len(gui.set_params) == 4               # B needed its own second attempt
    assert log_rows(log)[-2:] == [("no_ack", "1"), ("accepted", "2")]


def test_clean_ack_from_an_earlier_attempt_confirms_after_a_garbled_resend(make_link):
    # A1 times out at 1.0 s and is resent. A1's garbled ack arrives at 1.4 s, so A3 goes out - but A2's clean
    # ack at 1.8 s already confirms the write (same value), so we must not sit out A3's timeout (2.4 s).
    script = {0: [(1.4, GARBLED_ZEROED)], 1: [(0.8, ack())]}      # A3 (n=2) never answered
    gui, link, log = make_link(script, ack_timeout=1.0)
    start = time.time()
    result = link.set_param_confirmed(ADDR, 50.0)
    assert result["accepted"] is True
    assert time.time() - start < 2.2
    assert len(gui.set_params) == 3
    assert log_rows(log) == [("no_ack", "1"), ("garbled_ack", "2"), ("accepted", "3")]


def test_every_attempt_is_logged(make_link):
    script = {1: [(0.05, GARBLED_ZEROED)], 2: [(0.05, ack())]}      # attempt 1 silent
    _, link, log = make_link(script, ack_timeout=0.3)
    link.set_param_confirmed(ADDR, 50.0, label="Ankle(L) TorqScale")
    assert log_rows(log) == [("no_ack", "1"), ("garbled_ack", "2"), ("accepted", "3")]
    assert "Ankle(L) TorqScale" in log.getvalue()


def test_giving_up_is_logged_and_raised(make_link):
    gui, link, log = make_link({}, ack_timeout=0.2, max_retries=3)
    with pytest.raises(link_mod.OpenExoLinkError) as err:
        link.set_param_confirmed(ADDR, 50.0)
    assert err.value.reason is None               # silence, not a firmware rejection
    assert len(gui.set_params) == 3
    assert log_rows(log) == [("no_ack", "1"), ("no_ack", "2"), ("no_ack", "3"), ("gave_up", "3")]


def test_gui_refusal_is_logged_and_raised(make_link):
    _, link, log = make_link(refuse=True)
    with pytest.raises(link_mod.OpenExoLinkError):
        link.set_param_confirmed(ADDR, 50.0)
    assert log_rows(log) == [("gui_refused", "1")]
