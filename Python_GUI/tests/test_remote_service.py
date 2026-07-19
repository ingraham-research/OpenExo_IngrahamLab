import pytest

from remote.service import RemoteControlService, encode


@pytest.fixture
def svc(qapp):
    s = RemoteControlService(host="127.0.0.1", port=0)
    assert s.is_bound()
    return s


@pytest.fixture
def sent(svc, monkeypatch):
    """Capture outbound datagrams instead of putting them on the wire."""
    box = []
    monkeypatch.setattr(svc, "_send",
                        lambda obj, host, port: box.append((obj, host, port)) or True)
    return box


def test_bind_reports_address(svc):
    host, port = svc.address()
    assert host == "127.0.0.1"
    assert port != 0


def test_set_param_emits_payload_and_acks(svc, sent):
    # Set matrix state directly; the public setter arrives in Task 4.
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y", "use_pid"]]
    emitted = []
    svc.setParamRequested.connect(lambda p: emitted.append(p))

    svc.handle_datagram(
        encode({"cmd": "set_param", "joint": "Ankle(L)", "controller": "spline",
                "param": "node1_y", "value": 3.0, "id": 7}),
        "127.0.0.1", 55555,
    )

    assert emitted == [[False, 68, 1, 0, 3.0]]
    assert ({"ok": True, "id": 7}, "127.0.0.1", 55555) in sent


def test_bad_json_gets_error_reply(svc, sent):
    svc.handle_datagram(b"garbage{{{", "127.0.0.1", 55555)
    obj, host, port = sent[-1]
    assert obj["ok"] is False
    assert obj["code"] == "bad_json"


def test_unknown_name_error_reply_has_code(svc, sent):
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    svc.handle_datagram(
        encode({"cmd": "set_param", "joint": "Ankle(L)", "controller": "nope",
                "param": "node1_y", "value": 1.0, "id": 9}),
        "127.0.0.1", 55555,
    )
    obj, _, _ = sent[-1]
    assert obj == {"ok": False, "id": 9, "error": obj["error"], "code": "bad_name"}


def test_subscribe_registers_sender(svc, sent):
    svc.handle_datagram(
        encode({"cmd": "subscribe", "streams": ["rt", "ack"], "id": 1}),
        "127.0.0.1", 44444,
    )
    assert ("127.0.0.1", 44444) in svc._subscribers
    assert svc._subscribers[("127.0.0.1", 44444)]["streams"] == {"rt", "ack"}
    obj, _, _ = sent[-1]
    assert obj["ok"] is True


def test_subscribe_unknown_stream_rejected(svc, sent):
    svc.handle_datagram(
        encode({"cmd": "subscribe", "streams": ["bogus"], "id": 2}),
        "127.0.0.1", 44444,
    )
    obj, _, _ = sent[-1]
    assert obj["ok"] is False and obj["code"] == "bad_stream"
    assert ("127.0.0.1", 44444) not in svc._subscribers


def test_unsubscribe_removes_sender(svc, sent):
    svc.handle_datagram(encode({"cmd": "subscribe"}), "127.0.0.1", 44444)
    svc.handle_datagram(encode({"cmd": "unsubscribe"}), "127.0.0.1", 44444)
    assert ("127.0.0.1", 44444) not in svc._subscribers


def test_get_matrix_replies_with_cached_matrix(svc, sent):
    svc._matrix = [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    svc._param_names = ["percent_gait", "torque_l"]
    svc.handle_datagram(encode({"cmd": "get_matrix", "id": 3}), "127.0.0.1", 44444)
    # ok reply, then a matrix stream frame.
    assert sent[0][0] == {"ok": True, "id": 3}
    frame = sent[1][0]
    assert frame["stream"] == "matrix"
    assert frame["matrix"] == [["Ankle(L) (68)", "68", "spline", "1", "node1_y"]]
    assert frame["names"] == ["percent_gait", "torque_l"]


def test_unknown_cmd_rejected(svc, sent):
    svc.handle_datagram(encode({"cmd": "explode", "id": 5}), "127.0.0.1", 44444)
    obj, _, _ = sent[-1]
    assert obj["ok"] is False and obj["code"] == "bad_cmd"


def _subscribe(svc, host, port, streams):
    svc._subscribers[(host, port)] = {"streams": set(streams), "failures": 0}


def test_publish_rt_reaches_only_rt_subscribers(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    _subscribe(svc, "127.0.0.1", 2222, ["ack"])
    svc.set_param_names(["percent_gait", "torque_l", "torque_r"])
    sent.clear()

    svc.publish_rt([10.0, 1.5, -1.5])

    targets = {(port, obj["stream"]) for obj, host, port in sent}
    assert (1111, "rt") in targets
    assert all(port != 2222 for _, _, port in sent)  # ack-only sub gets nothing
    rt_frame = next(obj for obj, _, port in sent if port == 1111)
    assert rt_frame["values"] == [10.0, 1.5, -1.5]
    assert rt_frame["names"] == ["percent_gait", "torque_l", "torque_r"]


def test_publish_ack_maps_reason_text(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["ack"])
    sent.clear()
    svc.publish_ack({"joint_id": 68, "controller_id": 0, "param_index": 1,
                     "accepted": False, "reason": 5})
    frame = sent[-1][0]
    assert frame["stream"] == "ack"
    assert frame["accepted"] is False
    assert frame["reason_code"] == 5
    assert frame["reason"] == "value out of bounds"


def test_set_controller_matrix_broadcasts_to_matrix_subscribers(svc, sent):
    _subscribe(svc, "127.0.0.1", 1111, ["matrix"])
    sent.clear()
    svc.set_controller_matrix([["Ankle(L) (68)", "68", "spline", "1", "node1_y"]])
    frame = sent[-1][0]
    assert frame["stream"] == "matrix"
    assert frame["matrix"][0][2] == "spline"


def test_subscriber_evicted_after_max_failures(svc):
    # _send returns False (failure) every time.
    svc._send = lambda obj, host, port: False
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    for _ in range(svc._max_failures):
        svc.publish_rt([1.0])
    assert ("127.0.0.1", 1111) not in svc._subscribers


def test_successful_send_resets_failure_count(svc):
    _subscribe(svc, "127.0.0.1", 1111, ["rt"])
    sub = svc._subscribers[("127.0.0.1", 1111)]
    # Fail a few, but not enough to evict.
    svc._send = lambda obj, host, port: False
    for _ in range(svc._max_failures - 1):
        svc.publish_rt([1.0])
    assert sub["failures"] == svc._max_failures - 1
    # One success resets.
    svc._send = lambda obj, host, port: True
    svc.publish_rt([1.0])
    assert sub["failures"] == 0
