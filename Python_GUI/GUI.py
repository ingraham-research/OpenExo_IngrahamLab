import sys
import os
import logging
import traceback
from datetime import datetime

try:
    from PySide6 import QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

from MainWindow import MainWindow
from styles import DARK_STYLESHEET


class FlushingFileHandler(logging.FileHandler):
    """File handler that flushes after ERROR and CRITICAL messages."""
    def emit(self, record):
        super().emit(record)
        if record.levelno >= logging.ERROR:
            self.flush()


def setup_crash_logger(verbose: bool = False, log_dir: str = None):
    """Setup the one application logger: the crash hook, the session file, and the console.

    EVERYTHING under the "OpenExo" name lands here, including the BLE layer
    (QtExoDeviceManager logs as "OpenExo.DeviceManager"). It used to keep a second logger and a second
    file, so reading a session meant correlating device_manager_*.log and app_crash_*.log by timestamp,
    and every crash was written to both.

    `verbose` (--verbose-log, or EXO_LOG_VERBOSE=1) decides how much is kept, not what exists: DEBUG to the
    file and INFO to the console, instead of INFO and WARNING. The chatty per-command BLE lines are DEBUG, so
    a production session stays readable while a debugging session keeps everything.

    `log_dir` is for tests; it defaults to Saved_Data/logs.
    """
    if log_dir is None:
        base_dir = os.path.dirname(__file__)
        log_dir = os.path.join(base_dir, "Saved_Data", "logs")
    os.makedirs(log_dir, exist_ok=True)
    
    # Create crash log file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"app_crash_{timestamp}.log")
    
    # Configure root logger
    logger = logging.getLogger("OpenExo")
    logger.setLevel(logging.DEBUG)
    
    # Remove any existing handlers
    logger.handlers.clear()
    
    # File handler with detailed formatting
    # Use FlushingFileHandler to auto-flush ERROR and CRITICAL messages
    file_handler = FlushingFileHandler(log_file, encoding='utf-8', delay=False)
    file_handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    formatter = logging.Formatter(
        '%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)-20s | %(funcName)-25s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    # Console handler for immediate visibility
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO if verbose else logging.WARNING)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Where anything that wants to tell the user about the log can find it (QtExoDeviceManager does)
    logger.log_file_path = log_file

    logger.info("=" * 100)
    logger.info("OpenExo Application Started")
    logger.info(f"Log file: {log_file} (verbose={verbose})")
    logger.info(f"Python version: {sys.version}")
    logger.info("=" * 100)
    
    # Install global exception hook
    def global_exception_handler(exc_type, exc_value, exc_traceback):
        """Catch ALL unhandled exceptions and log them."""
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return
        
        logger.critical("=" * 100)
        logger.critical("!!! APPLICATION CRASH DETECTED !!!")
        logger.critical("=" * 100)
        logger.critical(f"Exception Type: {exc_type.__name__}")
        logger.critical(f"Exception Value: {exc_value}")
        logger.critical(f"Exception Location: {exc_traceback.tb_frame.f_code.co_filename}:{exc_traceback.tb_lineno}")
        logger.critical("")
        logger.critical("Full Traceback:")
        for line in traceback.format_tb(exc_traceback):
            logger.critical(line.rstrip())
        logger.critical("=" * 100)
        
        # CRITICAL: Force flush all handlers to ensure crash log is written to disk
        for handler in logger.handlers:
            try:
                handler.flush()
            except:
                pass
        
        # Show error dialog to user
        try:
            app = QtWidgets.QApplication.instance()
            if app:
                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Critical)
                msg.setWindowTitle("Application Crash")
                msg.setText(f"The application encountered a critical error:\n\n{exc_type.__name__}: {exc_value}")
                msg.setInformativeText(f"Details have been logged to:\n{log_file}")
                msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
                msg.exec()
        except:
            pass
        
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
    
    sys.excepthook = global_exception_handler
    logger.info("Global exception handler installed - all crashes will be logged")
    
    return logger, log_file


def main():
    # Kept out of argparse on purpose: sys.argv goes to QApplication as-is, which is where Qt's own flags live
    verbose = ("--verbose-log" in sys.argv) or (os.environ.get("EXO_LOG_VERBOSE", "0") not in ("", "0"))
    logger, _ = setup_crash_logger(verbose=verbose)

    try:
        logger.info("Creating QApplication...")
        app = QtWidgets.QApplication(sys.argv)
        
        logger.info("Applying dark stylesheet...")
        app.setStyleSheet(DARK_STYLESHEET)
        
        logger.info("Creating MainWindow...")
        w = MainWindow()
        
        logger.info("Showing MainWindow...")
        w.show()
        
        logger.info("Application ready - entering event loop")
        exit_code = app.exec()
        
        logger.info(f"Application exited normally with code {exit_code}")
        sys.exit(exit_code)
        
    except Exception as e:
        logger.critical("=" * 100)
        logger.critical("!!! EXCEPTION IN MAIN FUNCTION !!!")
        logger.critical("=" * 100)
        logger.critical(f"Exception: {type(e).__name__}: {e}")
        logger.critical("Traceback:")
        logger.critical(traceback.format_exc())
        logger.critical("=" * 100)
        
        # Force flush to ensure crash is logged
        for handler in logger.handlers:
            try:
                handler.flush()
            except:
                pass
        
        raise


if __name__ == "__main__":
    main()


