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
