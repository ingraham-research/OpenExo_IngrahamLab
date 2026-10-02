-To run the GUI, first ensure that Python is installed on your computer. You can find more information here: https://www.python.org/downloads/

-Once Python is installed, open a terminal in the Python_GUI folder and run python GUI_install_dependencies.py. Wait until all dependencies are installed.

-Finally, to start the GUI, run python GUI.py. Saved exoskeleton datastreams will be located in the Saved_Data folder.

Launch options
--------------
Every session writes one log file to Saved_Data/logs/app_crash_<date>_<time>.log (the name is historical: it is
the full session log, not only crashes). At startup the terminal prints one line saying which log mode is on and
where the file is, e.g.
    OpenExo log: verbose (launch with --quiet to trim) -> ...\Saved_Data\logs\app_crash_20260929_101500.log

-python GUI.py
    Default = VERBOSE logging. The log file gets everything (DEBUG and up, including the per-command BLE lines),
    and the terminal shows INFO and up, e.g. the "CONN_PARAMS [heartbeat]" line with the BLE connection interval
    every 10 s.

-python GUI.py --quiet
    Trimmed logging for routine use. The log file gets INFO and up (CONN_PARAMS, parameter updates, etc. are
    still recorded), and the terminal only shows warnings and errors. Nothing else about the GUI changes.

-EXO_QUIET=1 (environment variable)
    Same as --quiet, for launches that cannot pass arguments (e.g. an IDE "Run" button).
    PowerShell:  $env:EXO_QUIET=1; python GUI.py        cmd:  set EXO_QUIET=1  (then)  python GUI.py
    It stays set for that terminal window; set it to 0 (or open a new terminal) to go back to verbose.

Notes:
    - The flag is exactly --quiet. Anything unrecognised is ignored, so a typo leaves you in verbose mode.
    - --verbose-log (the old opt-in flag) is still accepted but does nothing: verbose is already the default.
    - The first lines of each log file record the mode as "(verbose=True)" or "(verbose=False)".
