"""RtBridge must keep accepting param-update acks with MORE than five fields.

The Nano's PARAM_ACK_DIAG build (ExoCode/src/ComsMCU.cpp) appends three counters to every ack. The GUI relies
on reading only the first five and logging the whole frame; this pins that down.
"""
from services.RtBridge import RtBridge


def _acks_from(qapp, frame):
    bridge = RtBridge()
    got = []
    bridge.paramUpdateAckReceived.connect(got.append)
    bridge.feed_bytes(frame)
    return got


def test_standard_five_field_ack(qapp):
    got = _acks_from(qapp, b"Sa5c6800n1300n1000n100n0n")
    assert got == [{"joint_id": 68, "controller_id": 13, "param_index": 10, "accepted": True, "reason": 0}]


def test_diagnostic_eight_field_ack_reads_the_first_five(qapp):
    got = _acks_from(qapp, b"Sa8c6800n1300n1000n100n0n1234500n1300n1200n")
    assert got == [{"joint_id": 68, "controller_id": 13, "param_index": 10, "accepted": True, "reason": 0}]
