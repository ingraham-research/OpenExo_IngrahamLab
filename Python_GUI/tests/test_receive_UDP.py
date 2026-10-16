import importlib.util, os, time

hip = r"C:\Users\Ingraham Lab\Codes\Hippo\Hip-exo-control-codes"
spec = importlib.util.spec_from_file_location(
    "UDP_utilities", os.path.join(hip, "Utilities", "UDP_utilities.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

r = m.UDPReceiver(port=5004, fmt='f')
r.start_listening()
print("Listening on 5004, Ctrl-C to stop")
try:
    while True:
        v = r.get_latest_timing_value()
        if v is not None:
            print("Got:", v)
        time.sleep(0.05)
except KeyboardInterrupt:
    r.stop()