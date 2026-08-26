import pytest


@pytest.fixture
def editor(qapp):
    from Widgets.SplineNodeEditor import SplineNodeEditor
    w = SplineNodeEditor()
    yield w
    w.close()


def test_configure_builds_requested_node_count(editor):
    editor.configure(5, x_nodes=[0, 25, 50, 75, 100], y_nodes=[0, 3, 15, 4, 0])
    assert editor.node_count() == 5
    assert len(editor._x_spins) == 5
    assert len(editor._y_spins) == 5
    assert editor.get_nodes() == ([0, 25, 50, 75, 100], [0, 3, 15, 4, 0])


def test_configure_reduces_node_count_from_previous(editor):
    editor.configure(12, x_nodes=list(range(12)), y_nodes=[0] * 12)
    editor.configure(5, x_nodes=[0, 25, 50, 75, 100], y_nodes=[0, 3, 15, 4, 0])
    assert editor.node_count() == 5
    assert len(editor._x_spins) == 5


def test_editing_a_spinbox_updates_get_nodes_and_emits_signal(editor):
    editor.configure(5, x_nodes=[0, 25, 50, 75, 100], y_nodes=[0, 3, 15, 4, 0])
    received = []
    editor.nodesChanged.connect(lambda: received.append(True))

    editor._y_spins[2].setValue(18.0)

    x_nodes, y_nodes = editor.get_nodes()
    assert y_nodes[2] == 18.0
    assert received


def test_invalid_node_order_shows_warning(editor):
    editor.configure(5, x_nodes=[0, 25, 50, 75, 100], y_nodes=[0, 3, 15, 4, 0])
    editor._x_spins[3].setValue(10.0)  # node 4's x now below node 3's x (50)
    assert "Invalid node order" in editor.lbl_warning.text()


def test_valid_order_clears_warning(editor):
    editor.configure(5, x_nodes=[0, 25, 50, 75, 100], y_nodes=[0, 3, 15, 4, 0])
    editor._x_spins[3].setValue(10.0)
    assert editor.lbl_warning.text() != ""
    editor._x_spins[3].setValue(60.0)
    assert editor.lbl_warning.text() == ""
