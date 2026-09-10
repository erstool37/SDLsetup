#!/usr/bin/env python3
"""OPERATOR DIAGNOSTIC -- alternate DF10 between 0 and 100 slowly so the operator
can SEE which tube pinches in each state.

3-way pinch valve: in every state one tube is pinched and one open -- the two
states look alike in a single glance, so this alternates them on a fixed cadence
and the operator watches ONE tube blink open/closed in sync. DF10=0 -> Y001
energized (measured); DF10=100 -> Y001 de-energized (measured). PID (C1) must be
False or the ladder recomputes DF10. Benign: only switches which air path flows.
Leaves DF10 at --end on exit (default 100 = humidifier feed closed) and can be
stopped early with `kill <pid>` (releases cleanly).
"""
from __future__ import annotations

import argparse
import datetime
import signal
import sys
import time

sys.path.insert(0, "/home/lamp/SDLsetup")

from tools import occupancy                                        # noqa: E402,I001
from tools.environment import registers                           # noqa: E402
from tools.environment.plc import PlcClient, PlcSettings           # noqa: E402

DF10_ADDR = registers.df_address(10)
Y001 = 8192


class _Stop(Exception):
    pass


def _stamp() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S")


def _write_df10(client, s, value):
    low, high = registers.f32_to_regs(value)
    client._ensure_transport().write_registers(DF10_ADDR, [low, high],
                                                device_id=s.device_id)
    back = client._read_registers(DF10_ADDR, 2)
    return back is not None and tuple(back) == (low, high)


def _y001(client, s):
    r = client._ensure_transport().read_coils(Y001, count=1, device_id=s.device_id)
    b = getattr(r, "bits", None)
    return b[0] if b else None


def run(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cycles", type=int, default=6)
    p.add_argument("--dwell", type=float, default=12.0)
    p.add_argument("--end", type=float, default=100.0)
    p.add_argument("--config", default="/home/lamp/SDLsetup/configs/config.yaml")
    args = p.parse_args(argv)

    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(_Stop()))

    s = PlcSettings.from_config(args.config)
    if not s.allow_actuation:
        print("REFUSED: environment.allow_actuation is false.", file=sys.stderr)
        return 2
    client = PlcClient(s)
    try:
        with occupancy.claim("environment", doing="blink DF10 0<->100 (valve id)"):
            if not client.connect():
                print("REFUSED: connect: %s" % client.last_error, file=sys.stderr)
                return 2
            if client.read_pid_enabled() is not False:
                print("REFUSED: C1 (PID Auto) is not False.", file=sys.stderr)
                return 2
            print("%s BLINK START: %d cycles, %.0fs each. Watch the bottom "
                  "(humidifier-feed) tube." % (_stamp(), args.cycles, args.dwell),
                  flush=True)
            for i in range(1, args.cycles + 1):
                _write_df10(client, s, 0.0)
                print("%s cycle %d/%d  DF10=0  -> Y001=%s  ENERGIZED  "
                      "(bottom tube should be OPEN: gas into humidifier)"
                      % (_stamp(), i, args.cycles, _y001(client, s)), flush=True)
                time.sleep(args.dwell)
                _write_df10(client, s, 100.0)
                print("%s cycle %d/%d  DF10=100 -> Y001=%s  DE-ENERGIZED "
                      "(bottom tube should be PINCHED shut)"
                      % (_stamp(), i, args.cycles, _y001(client, s)), flush=True)
                time.sleep(args.dwell)
            _write_df10(client, s, args.end)
            print("%s DONE. Left DF10=%g -> Y001=%s"
                  % (_stamp(), args.end, _y001(client, s)), flush=True)
    except (_Stop, KeyboardInterrupt):
        print("%s stopped; leaving DF10=%g." % (_stamp(), args.end), flush=True)
        try:
            _write_df10(client, s, args.end)
        except Exception:                                          # noqa: BLE001
            pass
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
