#!/usr/bin/env python3
"""OPERATOR DIAGNOSTIC -- hold the RH solenoid valve (Y001) ENERGIZED (PWM=1).

The operator asked to pin the 3-way solenoid pinch valve in its energized state
so they can see which tube is pinched shut. This drives coil ``Y001`` (Modbus
protocol address 8192, vendor-confirmed: CLICK Ver3.60 help 299.htm, Y001-Y816
= 984 addresses 8193-8464, FC5/FC15 writable) True and holds it, re-asserting
every second with a read-back, until the process is stopped.

WHY THIS IS NOT tools/ AND NOT THE DASHBOARD VALVE BUTTON
  There is deliberately NO valve-control API: it was left unwired because
  whether the RH PWM ladder rung is gated by C1 (PID Auto) is UNKNOWN, so a
  host write to Y001 might be overwritten by the ladder on the next scan. This
  script therefore MEASURES whether the write holds (samples the coil back) and
  says so, rather than assuming. It is a one-off operator diagnostic, gated and
  claimed like any actuation.

SAFETY
  * Refuses unless ``environment.allow_actuation`` is true.
  * Refuses if C1 (PID Auto) reads True -- it will not fight a running loop.
  * Takes the ``environment`` occupancy claim for its whole life, so the
    dashboard will not open a competing Modbus session (the CLICK allows only 3).
  * Energizing admits (per the 2026-09-09 live run, INFERENCE) the DRY air
    stream -- benign: dry air into the chamber, no heat, no pressure.
  * On stop (SIGINT/SIGTERM) it releases Y001 to False and drives C1 False, then
    closes. Stop it with:  kill <pid>
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

sys.path.insert(0, "/home/lamp/SDLsetup")

from tools import occupancy                                  # noqa: E402
from tools.environment.plc import PlcClient, PlcSettings     # noqa: E402

#: Y001, the RH solenoid discrete output. Vendor-confirmed 0-based protocol addr.
Y001_COIL = 8192


class _Stop(Exception):
    pass


def _install_stop_handlers() -> None:
    # Under `nohup ... &` from a non-interactive shell SIGINT is inherited as
    # SIG_IGN; set it back so `kill -INT` works, and make SIGTERM raise too.
    signal.signal(signal.SIGINT, signal.default_int_handler)

    def _term(_signum, _frame):
        raise _Stop()

    signal.signal(signal.SIGTERM, _term)


def _read_coil(client: PlcClient, addr: int, device_id: int) -> bool | None:
    try:
        result = client._ensure_transport().read_coils(
            addr, count=1, device_id=device_id)
    except Exception as exc:                                        # noqa: BLE001
        client.last_error = "read_coils(%d): %s: %s" % (addr, type(exc).__name__, exc)
        return None
    if result is None or client._is_error(result):
        return None
    bits = getattr(result, "bits", None)
    if not bits:
        return None
    bit = bits[0]
    if bit is True or bit is False:
        return bit
    if isinstance(bit, int) and bit in (0, 1):
        return bool(bit)
    return None


def _write_coil(client: PlcClient, addr: int, value: bool, device_id: int) -> bool:
    try:
        result = client._ensure_transport().write_coil(
            addr, value, device_id=device_id)
    except Exception as exc:                                        # noqa: BLE001
        client.last_error = "write_coil(%d, %r): %s: %s" % (
            addr, value, type(exc).__name__, exc)
        return False
    return not (result is None or client._is_error(result))


def run(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="/home/lamp/SDLsetup/configs/config.yaml")
    p.add_argument("--max-seconds", type=float, default=1800.0,
                   help="auto-release after this long even if not killed")
    args = p.parse_args(argv)

    settings = PlcSettings.from_config(args.config)
    print("PLC %s:%d  device_id=%d  allow_actuation=%s"
          % (settings.host, settings.port, settings.device_id,
             settings.allow_actuation), flush=True)
    if not settings.allow_actuation:
        print("REFUSED: environment.allow_actuation is false. Open the gate "
              "first.", file=sys.stderr)
        return 2

    _install_stop_handlers()
    released = False
    client = PlcClient(settings)
    try:
        with occupancy.claim("environment", doing="hold valve Y001 energized (diagnostic)"):
            if not client.connect():
                print("REFUSED: could not open a Modbus session: %s"
                      % client.last_error, file=sys.stderr)
                return 2

            pid = client.read_pid_enabled()
            if pid is True:
                print("REFUSED: C1 (PID Auto) is True -- a loop is running. Not "
                      "fighting it. Stop the run first.", file=sys.stderr)
                return 2
            print("C1 (PID Auto) = %r (None = could not read)" % pid, flush=True)

            before = _read_coil(client, Y001_COIL, settings.device_id)
            print("Y001 before = %r" % before, flush=True)

            # Energize, then sample whether it HOLDS over ~4 s (detects a ladder
            # rung that overwrites it -- the UNKNOWN this script is here to test).
            _write_coil(client, Y001_COIL, True, settings.device_id)
            samples = []
            for _ in range(8):
                time.sleep(0.5)
                samples.append(_read_coil(client, Y001_COIL, settings.device_id))
            held = sum(1 for s in samples if s is True)
            print("HOLD TEST: Y001 read True on %d/8 samples over 4s: %r"
                  % (held, samples), flush=True)
            if held == 0:
                print("VERDICT: the write did NOT hold -- either the ladder is "
                      "driving Y001 (C1-ungated PWM) or 8192 is not the coil. "
                      "The valve is NOT reliably energized.", file=sys.stderr,
                      flush=True)
            elif held < 8:
                print("VERDICT: Y001 is TOGGLING (ladder contention or PWM). The "
                      "valve is chattering, not steadily energized.", flush=True)
            else:
                print("VERDICT: Y001 HELD True. The valve is energized (PWM=1). "
                      "Look now -- one tube is pinched shut.", flush=True)

            # Hold: re-assert + read back once a second until killed / timeout.
            deadline = time.monotonic() + args.max_seconds
            n = 0
            while time.monotonic() < deadline:
                _write_coil(client, Y001_COIL, True, settings.device_id)
                got = _read_coil(client, Y001_COIL, settings.device_id)
                n += 1
                if n % 15 == 0 or got is not True:
                    print("[%4ds] Y001=%r (holding energized; kill to release)"
                          % (n, got), flush=True)
                time.sleep(1.0)
            print("max-seconds reached; releasing.", flush=True)
    except (_Stop, KeyboardInterrupt):
        print("\nstop requested -- releasing Y001 and disabling PID.", flush=True)
    finally:
        # Fail-safe: de-energize the valve, drive C1 False, close. Best-effort;
        # each step guarded so one failure does not skip the next.
        try:
            _write_coil(client, Y001_COIL, False, settings.device_id)
            released = True
        except Exception as exc:                                    # noqa: BLE001
            print("release Y001 FAILED: %s" % exc, file=sys.stderr, flush=True)
        try:
            client.set_pid(False)
        except Exception as exc:                                    # noqa: BLE001
            print("set_pid(False) FAILED: %s" % exc, file=sys.stderr, flush=True)
        try:
            client.close()
        except Exception:                                           # noqa: BLE001
            pass
        print("released=%s -- Y001 driven False, C1 driven False." % released,
              flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
