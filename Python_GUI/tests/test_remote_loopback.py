import threading
import time

import pytest

from remote.service import RemoteControlService


def _pump_until_done(qapp, thread, timeout=8.0):
    """Run the Qt event loop cooperatively until the worker thread finishes.

    The client runs in a worker thread doing blocking UDP; the main thread must
    pump events so the service's readyRead fires and it can reply. Pump until the
    thread is done (not just until the first reply) so the client's __exit__
    unsubscribe is serviced too, and the thread exits cleanly.
    """
    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        qapp.processEvents()
        time.sleep(0.005)
    thread.join(timeout=2.0)
    qapp.processEvents()


def test_loopback_set_param_and_matrix(qapp):
    svc = RemoteControlService(host="127.0.0.1", port=0)
    assert svc.is_bound()
    host, port = svc.address()
    svc.set_controller_matrix([
        ["Ankle(L) (68)", "68", "spline", "1", "node1_y", "use_pid"],
    ])
    svc.set_param_names(["percent_gait", "torque_l"])

    captured = []
    svc.setParamRequested.connect(lambda p: captured.append(p))

    result = {}

    def worker():
        from remote.client import ExoRemote
        try:
            with ExoRemote(host, port, timeout=3.0) as exo:
                result["matrix"] = exo.wait_for_matrix(timeout=3.0)
                result["reply"] = exo.set_param("Ankle(L)", "spline", "node1_y", 3.0)
        except Exception as e:  # surface worker failures to the assertions below
            result["error"] = repr(e)

    t = threading.Thread(target=worker)
    t.start()
    _pump_until_done(qapp, t)

    assert "error" not in result, result.get("error")
    assert result["reply"]["ok"] is True
    assert captured == [[False, 68, 1, 0, 3.0]]
    assert result["matrix"][0][2] == "spline"


def test_loopback_rejection_raises(qapp):
    from remote.client import ExoRemote, RemoteError

    svc = RemoteControlService(host="127.0.0.1", port=0)
    host, port = svc.address()
    svc.set_controller_matrix([["Ankle(L) (68)", "68", "spline", "1", "node1_y"]])

    result = {}

    def worker():
        try:
            with ExoRemote(host, port, timeout=3.0) as exo:
                exo.wait_for_matrix(timeout=3.0)
                exo.set_param("Ankle(L)", "spline", "does_not_exist", 1.0)
            result["outcome"] = "no-raise"
        except RemoteError as e:
            result["outcome"] = "raised"
            result["code"] = e.code

    t = threading.Thread(target=worker)
    t.start()
    _pump_until_done(qapp, t)

    assert result["outcome"] == "raised"
    assert result["code"] == "bad_name"


def test_end_to_end_fake_rt_reaches_client(qapp, monkeypatch):
    """Full receive path with NO BLE: emit RtBridge signals as if data were
    received, and confirm a UDP client subscribed through the real MainWindow
    wiring gets the labeled RT frame."""
    from utils import RemoteConfig
    monkeypatch.setattr(RemoteConfig, "PORT", 0)
    from MainWindow import MainWindow

    w = MainWindow()
    try:
        assert w.remote is not None and w.remote.is_bound()
        host, port = w.remote.address()
        # Seed param names so RT frames are labeled, as they would be post-handshake.
        w.rt_bridge.parameterNamesReceived.emit(["percent_gait", "torque_l"])

        got = {}

        def worker():
            from remote.client import ExoRemote
            with ExoRemote(host, port, timeout=3.0) as exo:
                for frame in exo.stream("rt", timeout=3.0):
                    got["frame"] = frame
                    break

        t = threading.Thread(target=worker)
        t.start()

        # Wait until the client's subscribe has registered (UDP has no pre-subscribe
        # buffering, so we must not emit RT before the subscriber exists).
        deadline = time.time() + 3.0
        while not w.remote._subscribers and time.time() < deadline:
            qapp.processEvents()
            time.sleep(0.005)
        assert w.remote._subscribers, "client did not subscribe in time"

        # Emit fake RT (full 16-wide frame, as RtBridge normalizes) until captured.
        deadline = time.time() + 3.0
        while t.is_alive() and time.time() < deadline:
            w.rt_bridge.rtDataUpdated.emit([12.5, 1.1] + [0.0] * 14)
            qapp.processEvents()
            time.sleep(0.01)
        t.join(timeout=2.0)
        qapp.processEvents()

        assert got.get("frame") == {"percent_gait": 12.5, "torque_l": 1.1}
    finally:
        try:
            w.remote._socket.close()
        except Exception:
            pass
        w.close()
