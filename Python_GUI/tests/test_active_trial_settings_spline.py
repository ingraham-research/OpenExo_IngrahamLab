import pytest


def _node_param_names(node_count):
    names = []
    for n in range(1, node_count + 1):
        names.append(f"node{n}_x")
        names.append(f"node{n}_y")
    names += ["sim_gait", "use_percent_gait", "use_pid", "p_gain", "i_gain", "d_gain"]
    return names


@pytest.fixture
def page(qapp):
    from pages.ActiveTrialSettingsPage import ActiveTrialSettingsPage
    p = ActiveTrialSettingsPage()
    yield p
    p.close()


# --- _split_spline_params -------------------------------------------------

def test_split_spline_params_detects_all_nodes(page):
    params = _node_param_names(12)
    node_map, names, indices = page._split_spline_params("spline", params)
    assert len(node_map) == 12
    assert node_map[1] == {"x": 0, "y": 1}
    assert node_map[12] == {"x": 22, "y": 23}
    assert names == ["sim_gait", "use_percent_gait", "use_pid", "p_gain", "i_gain", "d_gain"]
    assert indices == [24, 25, 26, 27, 28, 29]


def test_split_spline_params_handles_5_node_hip_config(page):
    params = _node_param_names(5)
    node_map, names, indices = page._split_spline_params("spline", params)
    assert len(node_map) == 5
    assert node_map[5] == {"x": 8, "y": 9}
    assert names == ["sim_gait", "use_percent_gait", "use_pid", "p_gain", "i_gain", "d_gain"]
    assert indices == [10, 11, 12, 13, 14, 15]


def test_split_spline_params_non_spline_controller_is_unfiltered(page):
    params = ["p_gain", "i_gain", "d_gain"]
    node_map, names, indices = page._split_spline_params("pjmc_plus", params)
    assert node_map == {}
    assert names == params
    assert indices == [0, 1, 2]


def test_split_spline_params_controller_named_spline_but_no_nodes_falls_back(page):
    params = ["use_pid", "p_gain"]
    node_map, names, indices = page._split_spline_params("spline", params)
    assert node_map == {}
    assert names == params
    assert indices == [0, 1]


# --- page-level wiring -----------------------------------------------------

def _matrix_row_for_spline(node_count):
    return ["Ankle(L) (68)", "68", "spline", "1", *_node_param_names(node_count)]


def _values_for(node_count, x_step=8.0, y_step=1.0):
    values = []
    for n in range(1, node_count + 1):
        values.append(str(n * x_step))
        values.append(str(n * y_step))
    values += ["1", "1", "0", "5", "0", "0.01"]  # sim_gait, use_percent_gait, use_pid, p/i/d gain
    return values


def test_selecting_spline_controller_shows_node_editor_with_device_values(page):
    from Widgets.SplineNodeEditor import SplineNodeEditor

    row = _matrix_row_for_spline(12)
    page.set_controller_matrix([row])
    page.set_controller_values({("68", "1"): _values_for(12)})

    assert page.top_stack.currentWidget() is page.spline_editor
    assert page.spline_editor.node_count() == 12

    x_nodes, y_nodes = page.spline_editor.get_nodes()
    assert x_nodes == [n * 8.0 for n in range(1, 13)]
    assert y_nodes == [n * 1.0 for n in range(1, 13)]

    # Node params must not appear in the generic Parameter dropdown.
    combo_items = [page.combo_param.itemText(i) for i in range(page.combo_param.count())]
    assert combo_items == ["sim_gait", "use_percent_gait", "use_pid", "p_gain", "i_gain", "d_gain"]


def test_selecting_non_spline_controller_shows_table(page):
    row = ["Ankle(L) (68)", "68", "pjmc_plus", "0", "use_pid", "p_gain", "i_gain", "d_gain"]
    page.set_controller_matrix([row])
    assert page.top_stack.currentWidget() is page.table


def test_apply_sends_all_node_values_plus_selected_generic_param(page):
    row = _matrix_row_for_spline(12)
    page.set_controller_matrix([row])
    page.set_controller_values({("68", "1"): _values_for(12)})

    page.combo_param.setCurrentIndex(3)  # "p_gain"
    page.spin_value.setValue(42.0)

    emitted = []
    page.applyRequested.connect(lambda payload: emitted.append(payload))
    page._on_apply()

    # 12 nodes * 2 (x, y) + 1 generic param = 25 payloads.
    assert len(emitted) == 25

    node_payloads = {(p[3], p[4]) for p in emitted[:24]}
    expected_node_payloads = set()
    for n in range(1, 13):
        expected_node_payloads.add((2 * (n - 1), n * 8.0))       # node n x -> global idx
        expected_node_payloads.add((2 * (n - 1) + 1, n * 1.0))   # node n y -> global idx
    assert node_payloads == expected_node_payloads

    generic_payload = emitted[-1]
    is_bilateral, joint_id, controller_id, param_idx, value = generic_payload
    assert joint_id == 68
    assert controller_id == 1
    assert param_idx == 27  # global index of "p_gain"
    assert value == 42.0
