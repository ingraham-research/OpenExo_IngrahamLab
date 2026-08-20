"""Interactive console for the OpenExo GUI's UDP remote control.

A tiny REPL that stands in for your own external control program: type short
commands and it sends the corresponding JSON to the GUI, using the same
remote/client.py library your real program would use.

Run the GUI first (python GUI.py). Then, from the Python_GUI folder:
    python examples/remote_console.py [host] [port]
(defaults to 127.0.0.1 9750)

It is safe to run this while the GUI is open — they are separate processes with
separate sockets. Only the GUI binds the port; this console just sends to it.

Commands (type `help` to reprint):
    help                                  show this list
    ping                                  check the GUI is listening
    matrix                                print all joints / controllers / params
    joints                                list joint names
    controllers <joint>                   list controllers on a joint
    params <joint> <controller>           list a controller's parameter names
    set <joint> <ctrl> <param> <val> [bi] set a parameter (add 'bi' for bilateral)
    watch <rt|ack|status|matrix> [secs]   stream a channel for N seconds (default 5)
    ack                                   show the last firmware ack received
    raw <json>                            send a raw JSON object (escape hatch)
    quit                                  exit

Notes:
  - <joint>/<ctrl>/<param> may be NAMES (e.g. Ankle(L) spline node1_y) or
    integer IDs (e.g. 68 1 1). Names need the GUI to be connected to an exo
    (so the controller matrix exists); integer IDs work even with no exo.
  - A `set` that returns 'ok' means the GUI accepted and transmitted it. Whether
    the exo accepted the value shows up on the `ack` stream (watch ack), only
    when a device is actually connected.

Examples (ankle; Ankle(L) = joint id 68, Ankle(R) = 36; trailing `bi` mirrors L<->R):
  Switching controller "mode" IS a set_param: sending a parameter to a controller
  that isn't the active one switches the joint to it and loads that controller's
  defaults, then applies the one parameter you named. That's why each example sets
  a parameter. Run `matrix` first to confirm the exact controller/param names your
  firmware advertises (names are case-insensitive; long names may be abbreviated in
  the handshake). If a name won't resolve, use the numeric controller-id / param-
  index form shown under each example - those always work, even with no exo yet.

  1) Set BOTH ankles to spline mode (uses spline's default first node, Node1_x = 0):
        set Ankle(L) spline Node1_x 0 bi
        # numeric equivalent  (spline = controller id 12, Node1_x = param index 0):
        set 68 12 0 0 bi

  2) Set BOTH ankles to PJMC with max stance torque = 12 Nm (rest load from PJMC
     defaults). "max stance torque (Nm)" is param index 0:
        set 68 3 0 12 bi
        # by name (if your handshake advertises it un-abbreviated; note the quotes):
        set Ankle(L) PJMC "max stance torque (Nm)" 12 bi

  3) Set BOTH ankles to zeroTorque mode (transparent; use_pid = 1 is its default):
        set Ankle(L) zeroTorque use_pid 1 bi
        # numeric equivalent  (zeroTorque = controller id 2, use_pid = param index 0):
        set 68 2 0 1 bi

  To target only the LEFT ankle, drop the trailing `bi`. For only the RIGHT ankle,
  use joint 36 / Ankle(R) instead (e.g. `set 36 12 0 0`).
"""
import json
import os
import shlex
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "remote"))
from client import ExoRemote, RemoteError  # noqa: E402  # type: ignore[import-not-found]  (resolved at runtime via sys.path above)


def _coerce(token):
    """Integer-looking tokens become ints (raw IDs); everything else stays a string (names)."""
    try:
        return int(token)
    except ValueError:
        return token


def _joint_name(row):
    return str(row[0]).rsplit(" (", 1)[0].strip()


def cmd_matrix(exo):
    matrix = exo.get_matrix()
    if not matrix:
        print("  (matrix empty - the GUI is not connected to an exo yet)")
        return
    last_joint = None
    for row in matrix:
        if len(row) < 4:
            continue
        jname = _joint_name(row)
        if jname != last_joint:
            print(f"\n  {jname}  (id {row[1]})")
            last_joint = jname
        params = ", ".join(str(p) for p in row[4:]) or "(no params)"
        print(f"      {row[2]:<14} (id {row[3]}): {params}")
    print()


def cmd_joints(exo):
    matrix = exo.get_matrix()
    if not matrix:
        print("  (matrix empty - the GUI is not connected to an exo yet)")
        return
    seen = []
    for row in matrix:
        j = _joint_name(row)
        if j not in seen:
            seen.append(j)
    print("  " + ", ".join(seen))


def cmd_controllers(exo, args):
    if not args:
        print("  usage: controllers <joint>")
        return
    names = exo.controllers(args[0])
    print("  " + (", ".join(names) if names else f"(no controllers for {args[0]!r})"))


def cmd_params(exo, args):
    if len(args) < 2:
        print("  usage: params <joint> <controller>")
        return
    joint, ctrl = args[0].strip().lower(), args[1].strip().lower()
    for row in exo.get_matrix():
        if len(row) < 4:
            continue
        if _joint_name(row).lower() == joint or str(row[1]) == args[0]:
            if str(row[2]).lower() == ctrl or str(row[3]) == args[1]:
                params = row[4:]
                for i, p in enumerate(params):
                    print(f"      [{i}] {p}")
                return
    print(f"  (no controller {args[1]!r} on joint {args[0]!r})")


def cmd_set(exo, args):
    if len(args) < 4:
        print("  usage: set <joint> <controller> <param> <value> [bi]")
        return
    joint = _coerce(args[0])
    controller = _coerce(args[1])
    param = _coerce(args[2])
    try:
        value = float(args[3])
    except ValueError:
        print(f"  value must be a number, got {args[3]!r}")
        return
    bilateral = len(args) >= 5 and args[4].lower() in ("bi", "bilat", "bilateral", "true", "1")
    try:
        exo.set_param(joint, controller, param, value, bilateral=bilateral)
        tag = " (bilateral)" if bilateral else ""
        print(f"  ok - GUI accepted set {joint}/{controller}/{param} = {value}{tag}")
        print("       (watch ack to see whether the exo accepted it)")
    except RemoteError as e:
        print(f"  REJECTED by GUI: {e}  (code={e.code})")


def _fmt_frame(which, msg):
    if which == "rt":
        if "names" in msg:
            pairs = ", ".join(f"{n}={v:.3f}" for n, v in zip(msg["names"], msg["values"]))
            return "  rt   " + pairs
        return "  rt   " + ", ".join(f"{v:.3f}" for v in msg.get("values", []))
    if which == "ack":
        verdict = "ACCEPTED" if msg.get("accepted") else f"REJECTED ({msg.get('reason')})"
        return (f"  ack  joint={msg.get('joint_id')} controller={msg.get('controller_id')} "
                f"param={msg.get('param_index')} -> {verdict}")
    if which == "status":
        extra = {k: v for k, v in msg.items() if k not in ("stream", "event")}
        return f"  stat {msg.get('event')} {extra if extra else ''}".rstrip()
    return "  " + json.dumps({k: v for k, v in msg.items() if k != "stream"})


def cmd_watch(exo, args):
    if not args:
        print("  usage: watch <rt|ack|status|matrix> [seconds]")
        return
    which = args[0].lower()
    if which not in ("rt", "ack", "status", "matrix"):
        print("  stream must be one of: rt, ack, status, matrix")
        return
    seconds = 5.0
    if len(args) >= 2:
        try:
            seconds = float(args[1])
        except ValueError:
            pass
    try:
        exo.subscribe([which])
    except RemoteError as e:
        print(f"  could not subscribe: {e}")
        return
    print(f"  watching {which} for {seconds:g}s  (Ctrl-C to stop early)...")
    exo._sock.settimeout(0.3)
    deadline = time.time() + seconds
    count = 0
    try:
        while time.time() < deadline:
            try:
                msg = exo._recv()
            except socket.timeout:
                continue
            except (ValueError, OSError):
                continue
            exo._absorb(msg)
            if msg.get("stream") == which:
                count += 1
                print(_fmt_frame(which, msg))
    except KeyboardInterrupt:
        print("  (stopped)")
    finally:
        exo._sock.settimeout(2.0)
        try:
            exo.unsubscribe()
        except RemoteError:
            pass
    print(f"  [{count} {which} frame(s)]")


def cmd_ack(exo):
    ack = exo.last_ack()
    if not ack:
        print("  (no ack seen yet - do `watch ack` around a `set`)")
        return
    print("  " + _fmt_frame("ack", ack))


def cmd_raw(exo, raw_text):
    if not raw_text.strip():
        print('  usage: raw {"cmd":"ping"}')
        return
    try:
        obj = json.loads(raw_text)
    except json.JSONDecodeError as e:
        print(f"  not valid JSON: {e}")
        return
    try:
        reply = exo._command(obj)
        print("  reply: " + json.dumps(reply))
    except RemoteError as e:
        print(f"  error: {e}  (code={e.code})")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9750

    exo = ExoRemote(host, port, timeout=2.0)
    print(f"OpenExo remote console -> udp://{host}:{port}")
    try:
        exo.ping()
        print("Connected: the GUI is listening.")
    except RemoteError:
        print("WARNING: no reply from the GUI. Is it running? "
              "(You can still type commands; start the GUI and retry.)")
    print("Type `help` for commands, `quit` to exit.\n")

    try:
        while True:
            try:
                line = input("exo> ").strip()
            except EOFError:
                break
            if not line:
                continue
            try:
                parts = shlex.split(line)
            except ValueError:
                parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd in ("help", "?", "h"):
                    print(__doc__)
                elif cmd == "ping":
                    t0 = time.time()
                    exo.ping()
                    print(f"  pong ({(time.time() - t0) * 1000:.1f} ms)")
                elif cmd == "matrix":
                    cmd_matrix(exo)
                elif cmd == "joints":
                    cmd_joints(exo)
                elif cmd == "controllers":
                    cmd_controllers(exo, args)
                elif cmd == "params":
                    cmd_params(exo, args)
                elif cmd == "set":
                    cmd_set(exo, args)
                elif cmd == "watch":
                    cmd_watch(exo, args)
                elif cmd == "ack":
                    cmd_ack(exo)
                elif cmd == "raw":
                    cmd_raw(exo, line[len("raw"):].strip())
                else:
                    print(f"  unknown command {cmd!r} - type `help`")
            except RemoteError as e:
                print(f"  error: {e}  (is the GUI running?)")
    finally:
        try:
            exo._sock.close()
        except Exception:
            pass
    print("bye")


if __name__ == "__main__":
    main()
