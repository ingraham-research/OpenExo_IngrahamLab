"""Standalone BLE environment check - run this on a machine where the GUI cannot find the Nano.

    python ble_env_check.py

It needs nothing from the GUI (no PySide6, no repo imports): just `pip install bleak`.
It answers, in one run, the three questions the GUI's own "No OpenExo devices found" cannot:

  1. Can this adapter do an LE scan AT ALL?            -> total device count
  2. Does it see the Nano?                             -> EXOBLE_ name match
  3. Does the advertisement carry the UART service     -> the GUI filters ONLY on this UUID
     UUID that the GUI filters on?                        (QtExoDeviceManager._filter_exo)

A machine that sees plenty of devices but zero service_uuids is a different bug from one
that sees nothing at all, and the GUI reports both as the same empty list.
"""

import asyncio
import platform
import sys

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NAME_PREAMBLE = "EXOBLE_"
SCAN_SECONDS = 15.0  # longer than the GUI's 3.0 s: a marginal adapter needs more advertising intervals


def _header():
    print("=" * 78)
    print("OpenExo BLE environment check")
    print("=" * 78)
    print(f"Host          : {platform.node()}")
    print(f"OS            : {platform.platform()}")
    print(f"Python        : {sys.version.split()[0]} ({sys.executable})")
    try:
        import bleak
        # bleak 2.x dropped the module-level __version__ that 0.x/1.x exposed.
        try:
            from importlib.metadata import version as _pkg_version
            ver = _pkg_version("bleak")
        except Exception:
            ver = getattr(bleak, "__version__", "unknown")
        print(f"bleak         : {ver}")
    except Exception as ex:  # pragma: no cover - reported, not raised
        print(f"bleak         : NOT IMPORTABLE -> {type(ex).__name__}: {ex}")
        print("")
        print("FAIL: this is already the answer. The GUI catches this same import error at")
        print("      module load and silently sets BLE_AVAILABLE=False, so Scan can never")
        print("      return anything. Install into the SAME interpreter that runs GUI.py.")
        raise SystemExit(1)
    try:
        import bleak_winrt  # noqa: F401
        print("bleak_winrt   : present")
    except Exception:
        try:
            import winrt  # noqa: F401
            print("winrt         : present")
        except Exception:
            print("winrt         : not found (expected on non-Windows)")
    print("-" * 78)


async def _radio_state():
    """Windows only: ask WinRT whether a BLE-capable radio exists and is ON."""
    if sys.platform != "win32":
        return
    try:
        from winrt.windows.devices.radios import Radio, RadioKind
    except Exception:
        try:
            from bleak_winrt.windows.devices.radios import Radio, RadioKind
        except Exception:
            print("Radio state   : winrt Radio API unavailable, skipping")
            print("-" * 78)
            return
    try:
        radios = await Radio.get_radios_async()
        found = False
        for r in radios:
            if r.kind == RadioKind.BLUETOOTH:
                found = True
                print(f"Radio state   : '{r.name}' -> {r.state.name}")
        if not found:
            print("Radio state   : NO BLUETOOTH RADIO REPORTED BY WINDOWS")
    except Exception as ex:
        print(f"Radio state   : query failed -> {ex}")
    print("-" * 78)


async def _scan():
    from bleak import BleakScanner

    print(f"Scanning {SCAN_SECONDS:.0f}s (active)...\n")
    try:
        found = await BleakScanner.discover(timeout=SCAN_SECONDS, return_adv=True)
    except Exception as ex:
        print(f"SCAN RAISED: {type(ex).__name__}: {ex}")
        print("\nThis is the adapter/driver refusing to scan - not a filtering problem.")
        return None

    if not found:
        print("RESULT: zero BLE devices of ANY kind.")
        print("        A normal room shows several. The adapter is not doing LE scanning.")
        return found

    print(f"{'NAME':<28} {'ADDRESS':<20} {'RSSI':>5}  SERVICE UUIDs IN ADV")
    print("-" * 78)
    exo_by_name = []
    exo_by_uuid = []
    for dev, adv in sorted(found.values(), key=lambda t: -(t[1].rssi or -999)):
        name = (adv.local_name or dev.name or "") or "<no name>"
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        has_uart = UART_SERVICE_UUID in uuids
        if name.upper().startswith(NAME_PREAMBLE):
            exo_by_name.append((name, dev.address, has_uart))
        if has_uart:
            exo_by_uuid.append((name, dev.address))
        shown = ", ".join(uuids) if uuids else "(none)"
        mark = " <-- UART" if has_uart else ""
        print(f"{name[:28]:<28} {dev.address:<20} {adv.rssi if adv.rssi is not None else 0:>5}  {shown}{mark}")

    print("-" * 78)
    print(f"Total devices seen              : {len(found)}")
    print(f"Devices advertising ANY uuid    : {sum(1 for _, a in found.values() if a.service_uuids)}")
    print(f"Named {NAME_PREAMBLE}*                  : {len(exo_by_name)}")
    print(f"Advertising the UART service    : {len(exo_by_uuid)}   <- this is what the GUI requires")
    print("-" * 78)

    if exo_by_uuid:
        print("VERDICT: the Nano is visible AND passes the GUI's filter.")
        print("         The scan is not the failure - look at connect, not discovery.")
    elif exo_by_name:
        print("VERDICT: the Nano IS being seen by name, but its advertisement carries no")
        print("         UART service UUID on this machine. The GUI filters on that UUID")
        print("         only (QtExoDeviceManager._filter_exo), so it drops the device.")
    elif len(found) == 0:
        print("VERDICT: no LE scanning on this machine at all.")
    else:
        print("VERDICT: LE scanning works here (other devices seen) but the Nano never")
        print("         advertised during the window. Either it is powered off, out of")
        print("         range, or still connected to another host (it stops advertising")
        print("         while connected).")
    return found


async def main():
    _header()
    await _radio_state()
    await _scan()


if __name__ == "__main__":
    asyncio.run(main())
