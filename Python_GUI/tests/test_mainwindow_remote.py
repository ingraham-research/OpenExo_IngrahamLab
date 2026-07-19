import pytest


@pytest.fixture
def window(qapp, monkeypatch):
    # Bind an ephemeral port so repeated window construction can't collide on 9750,
    # and so a lingering socket from a prior test never fails the next bind.
    from utils import RemoteConfig
    monkeypatch.setattr(RemoteConfig, "PORT", 0)
    from MainWindow import MainWindow
    w = MainWindow()
    yield w
    # Release the UDP socket promptly rather than waiting on GC.
    if getattr(w, "remote", None) is not None:
        try:
            w.remote._socket.close()
        except Exception:
            pass
    w.close()


def test_remote_service_is_bound(window):
    assert window.remote is not None
    assert window.remote.is_bound()


def test_apply_param_update_does_not_navigate(window):
    # Land on the settings page, then apply a remote-style update.
    window.stack.setCurrentWidget(window.settings_page)
    before = window.stack.currentWidget()
    window._apply_param_update([False, 68, 0, 1, 3.0])  # not connected -> no BLE, but must not navigate
    assert window.stack.currentWidget() is before


def test_on_apply_settings_navigates_to_trial(window):
    window.stack.setCurrentWidget(window.settings_page)
    window._on_apply_settings([False, 68, 0, 1, 3.0])
    assert window.stack.currentWidget() is window.trial_page


def test_setparam_signal_is_wired_to_apply(window):
    # Qt binds a slot at connect() time, so monkeypatching the attribute afterward
    # would not redirect the signal. Assert the connection directly: disconnect()
    # succeeds if wired, and raises if not.
    try:
        window.remote.setParamRequested.disconnect(window._apply_param_update)
    except (RuntimeError, TypeError):
        pytest.fail("setParamRequested is not connected to _apply_param_update")
    finally:
        # Best-effort reconnect so the window is left functional.
        try:
            window.remote.setParamRequested.connect(window._apply_param_update)
        except Exception:
            pass
