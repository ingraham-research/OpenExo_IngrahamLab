import pytest


@pytest.fixture
def trial_page(qapp):
    from pages.ActiveTrialPage import ActiveTrialPage
    page = ActiveTrialPage()
    yield page
    page.close()


def _sample(exo_time_s):
    """A 16-wide RT values list with the exo-time channel at index 9."""
    v = [0.0] * 16
    v[9] = exo_time_s
    return v


def test_exo_time_int16_wrap_keeps_plot_time_monotonic(trial_page):
    # The exo clock reaches the GUI as a signed-16-bit fixed-point value (RealTimeI2C
    # packs every RT channel as short(value*100)). seconds*100 overflows int16 at
    # 327.68s, so the raw value jumps +327.67 -> -327.68 (a ~655.36s drop). The plotted
    # x must stay monotonic across that wrap instead of snapping back to 0.
    trial_page._exo_time_idx = 9
    raw_seq = [327.40, 327.50, 327.60, 327.67, -327.68, -327.60, -327.50]
    ts = [trial_page._x_for_sample(_sample(r)) for r in raw_seq]
    assert all(b > a for a, b in zip(ts, ts[1:])), f"plot time not monotonic across wrap: {ts}"


def test_two_consecutive_wraps_stay_monotonic(trial_page):
    # Cover a full int16 period boundary twice (wrap recurs every 655.36s).
    trial_page._exo_time_idx = 9
    raw_seq = [327.67, -327.68, 0.0, 327.67, -327.68, 0.0]
    ts = [trial_page._x_for_sample(_sample(r)) for r in raw_seq]
    assert all(b > a for a, b in zip(ts, ts[1:])), f"plot time not monotonic across two wraps: {ts}"


def test_genuine_reboot_still_reanchors_to_zero(trial_page):
    # A real exo reboot (millis()->0) drops the clock by less than half the int16 range,
    # so it must NOT be mistaken for a wrap; the axis re-anchors to 0 as before.
    trial_page._exo_time_idx = 9
    trial_page._x_for_sample(_sample(120.0))
    t_after = trial_page._x_for_sample(_sample(120.10))
    assert t_after == pytest.approx(0.10, abs=1e-6)
    t_reboot = trial_page._x_for_sample(_sample(0.0))  # uptime reset, drop of ~120s (< 327.68)
    assert t_reboot == pytest.approx(0.0, abs=1e-6)
