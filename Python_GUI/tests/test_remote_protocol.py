import pytest

from remote.service import (
    encode, decode, ProtocolError, ACK_REASONS, VALID_STREAMS,
)


def test_encode_decode_round_trip():
    obj = {"cmd": "set_param", "joint": "Ankle(L)", "value": 3.0, "id": 42}
    assert decode(encode(obj)) == obj


def test_decode_rejects_non_json():
    with pytest.raises(ProtocolError):
        decode(b"not json {{{")


def test_decode_rejects_json_array():
    with pytest.raises(ProtocolError):
        decode(b"[1, 2, 3]")


def test_ack_reasons_cover_firmware_codes():
    # Mirror of MainWindow._PARAM_UPDATE_REASONS (1..6) plus accepted (0).
    for code in range(0, 7):
        assert code in ACK_REASONS


def test_valid_streams():
    assert VALID_STREAMS == ("rt", "ack", "matrix", "status")


from remote.service import resolve_set_param, CommandError

MATRIX = [
    ["Ankle(L) (68)", "68", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"],
    ["Ankle(L) (68)", "68", "spline", "1", "node1_x", "node1_y", "use_pid"],
    ["Ankle(R) (36)", "36", "zeroTorque", "0", "use_pid", "p_gain", "i_gain", "d_gain"],
]


def test_resolve_by_name():
    assert resolve_set_param(
        {"joint": "Ankle(L)", "controller": "spline", "param": "node1_y", "value": 3.0},
        MATRIX,
    ) == [False, 68, 1, 1, 3.0]


def test_resolve_is_case_insensitive():
    assert resolve_set_param(
        {"joint": "ankle(l)", "controller": "SPLINE", "param": "USE_PID", "value": 1},
        MATRIX,
    ) == [False, 68, 1, 2, 1.0]


def test_resolve_bilateral_flag_passes_through():
    out = resolve_set_param(
        {"joint": "Ankle(L)", "controller": "zeroTorque", "param": "p_gain",
         "value": 2.5, "bilateral": True},
        MATRIX,
    )
    assert out == [True, 68, 0, 1, 2.5]


def test_resolve_raw_ids_pass_through_without_matrix():
    assert resolve_set_param(
        {"joint": 68, "controller": 0, "param": 1, "value": 4.0}, []
    ) == [False, 68, 0, 1, 4.0]


def test_resolve_names_without_matrix_raises_no_matrix():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline",
                           "param": "node1_y", "value": 3.0}, [])
    assert ei.value.code == "no_matrix"


def test_resolve_unknown_controller_raises_bad_name():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "nope",
                           "param": "p_gain", "value": 1.0}, MATRIX)
    assert ei.value.code == "bad_name"


def test_resolve_param_index_out_of_range_raises_invalid_index():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline",
                           "param": 99, "value": 1.0}, MATRIX)
    assert ei.value.code == "invalid_index"


def test_resolve_missing_field_raises_malformed():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": "Ankle(L)", "controller": "spline"}, MATRIX)
    assert ei.value.code == "malformed"


def test_resolve_non_numeric_value_raises_malformed():
    with pytest.raises(CommandError) as ei:
        resolve_set_param({"joint": 68, "controller": 0, "param": 1,
                           "value": "high"}, [])
    assert ei.value.code == "malformed"
