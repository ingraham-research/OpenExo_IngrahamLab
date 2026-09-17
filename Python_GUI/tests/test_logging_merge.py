"""One log file per session, with verbosity behind a flag.

The BLE layer used to log into its own device_manager_*.log, so reading a session meant correlating two files
by timestamp (and every crash was written to both). It now logs into the application-wide "OpenExo" logger.
Nothing is trimmed: GUI.py's --verbose-log decides whether DEBUG lines reach the file.
"""
import glob
import logging
import os

import GUI
from services.QtExoDeviceManager import QtExoDeviceManager

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Saved_Data", "logs")


class _Capture(logging.Handler):
    def __init__(self):
        super().__init__(level=logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)


def test_device_manager_logs_reach_the_shared_logger(qapp):
    shared = logging.getLogger("OpenExo")
    cap = _Capture()
    shared.addHandler(cap)
    shared.setLevel(logging.DEBUG)
    try:
        QtExoDeviceManager()
    finally:
        shared.removeHandler(cap)
    names = {r.name for r in cap.records}
    assert any(n.startswith("OpenExo.DeviceManager") for n in names), names


def test_device_manager_writes_no_log_file_of_its_own(qapp):
    before = set(glob.glob(os.path.join(LOG_DIR, "device_manager_*.log")))
    QtExoDeviceManager()
    assert set(glob.glob(os.path.join(LOG_DIR, "device_manager_*.log"))) == before


def test_log_file_path_comes_from_the_shared_logger(qapp, tmp_path):
    logger, log_file = GUI.setup_crash_logger(log_dir=str(tmp_path))
    try:
        assert QtExoDeviceManager().get_log_file_path() == log_file
    finally:
        logger.handlers.clear()


def test_verbose_flag_opens_the_file_to_debug(tmp_path):
    quiet, _ = GUI.setup_crash_logger(log_dir=str(tmp_path))
    quiet_levels = [h.level for h in quiet.handlers]
    quiet.handlers.clear()
    loud, _ = GUI.setup_crash_logger(log_dir=str(tmp_path), verbose=True)
    loud_levels = [h.level for h in loud.handlers]
    loud.handlers.clear()
    assert quiet_levels == [logging.INFO, logging.WARNING]     # file, console
    assert loud_levels == [logging.DEBUG, logging.INFO]


def test_the_flag_decides_whether_ble_debug_lines_are_kept(tmp_path):
    # The per-command BLE lines are DEBUG: gated in production, kept for a debugging session.
    for verbose, expected in ((False, False), (True, True)):
        logger, log_file = GUI.setup_crash_logger(log_dir=str(tmp_path / str(verbose)), verbose=verbose)
        try:
            logging.getLogger("OpenExo.DeviceManager").debug("Submitting coroutine: _marked")
            logging.getLogger("OpenExo.DeviceManager").info("Sending parameter update: joint=68")
        finally:
            for h in logger.handlers:
                h.flush()
            logger.handlers.clear()
        text = open(log_file, encoding="utf-8").read()
        assert ("Submitting coroutine" in text) is expected
        assert "Sending parameter update" in text        # INFO is kept either way
