import os
import sys

# Headless Qt for all tests in this suite.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Make Python_GUI importable (remote/, utils/, MainWindow, ...) regardless of cwd.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402
from PySide6 import QtWidgets  # noqa: E402


@pytest.fixture(scope="session")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app
