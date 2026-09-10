#!/usr/bin/env python3
"""OPERATOR DIAGNOSTIC -- set the RH PWM duty register DF10 to a fixed value.

The RH PWM ladder rung runs UNCONDITIONALLY (observed live 2026-09-09: with C1
/ PID Auto False, DF10 was frozen at 18.96 and Y001 still toggled at that duty).
So the only way to pin the solenoid valve steady for a visual hose check is to
write the duty register itself: DF10 = 0 -> 0% duty -> valve steady one way and
QUIET; DF10 = 100 -> 100% duty -> valve steady the other way and QUIET.

DF10 (rh_pid_output_pct, protocol addr 28690) is marked read-only BY POLICY in
registers.py because it is a PID internal -- writing it is meaningful ONLY while
the PID is not recomputing it, i.e. C1 (PID Auto) is False. This script refuses
otherwise. Encoding and read-back are copied from PlcClient.write_float32
(float32, low word first). Benign: it only changes which air stream flows.
"""
from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "/home/lamp/SDLsetup")

from tools import occupancy                                        # noqa: E402
from tools.environment import registers                           # noqa: E402
from tools.environment.plc import PlcClient, PlcSettings           # noqa: E402

DF10_ADDR = registers.df_address(10)   # 28690
Y001_COIL = 8192


def _read_coil(client, addr, device_id):
    try:
        r = client._ensure_transport().read_coils(addr, count=1, device_id=device_id)
    except Exception as exc:                                       # noqa: BLE001
        return "ERR:%s" % exc
    if r is None or client._is_error(r):
        return None
    bits = getattr(r, "bits", None)
    return bits[0] if bits else None


def run(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("value", type=float, help="DF10 duty %% to write (0..100)")
    p.add_argument("--config", default="/home/lamp/SDLsetup/configs/config.yaml")
    args = p.parse_args(argv)

    if not (0.0 <= args.value <= 100.0):
        print("REFUSED: DF10 duty must be 0..100, got %g" % args.value,
              file=sys.stderr)
        return 2

    s = PlcSettings.from_config(args.config)
    print("PLC %s:%d device_id=%d allow_actuation=%s  DF10@%d"
          % (s.host, s.port, s.device_id, s.allow_actuation, DF10_ADDR), flush=True)
    if not s.allow_actuation:
        print("REFUSED: environment.allow_actuation is false.", file=sys.stderr)
        return 2

    low, high = registers.f32_to_regs(args.value)
    words = [low, high]
    client = PlcClient(s)
    try:
        with occupancy.claim("environment", doing="set DF10=%g (valve diagnostic)" % args.value):
            if not client.connect():
                print("REFUSED: connect: %s" % client.last_error, file=sys.stderr)
                return 2
            pid = client.read_pid_enabled()
            if pid is not False:
                print("REFUSED: C1 (PID Auto) = %r, must be False. Writing DF10 "
                      "while the loop recomputes it is pointless." % pid,
                      file=sys.stderr)
                return 2

            before = client._read_registers(DF10_ADDR, 2)
            print("DF10 before = %s (decodes %s)"
                  % (before, registers.regs_to_f32(*before) if before else None),
                  flush=True)

            t = client._ensure_transport()
            res = t.write_registers(DF10_ADDR, words, device_id=s.device_id)
            if res is None or client._is_error(res):
                print("WRITE FAILED: %r" % res, file=sys.stderr)
                return 1
            back = client._read_registers(DF10_ADDR, 2)
            if back is None or tuple(back) != (low, high):
                print("READ-BACK MISMATCH: wrote [0x%04X 0x%04X], read %s"
                      % (low, high, back), file=sys.stderr)
                return 1
            print("DF10 CONFIRMED = %g  (words [0x%04X 0x%04X])"
                  % (registers.regs_to_f32(*back), low, high), flush=True)

            # Show the resulting steady Y001 state.
            samples = []
            for _ in range(8):
                samples.append(_read_coil(client, Y001_COIL, s.device_id))
                time.sleep(0.4)
            trues = sum(1 for x in samples if x is True)
            print("Y001 over 3.2s: %d/8 True  %r" % (trues, samples), flush=True)
            steady = "STEADY True (energized)" if trues == 8 else (
                "STEADY False (de-energized)" if trues == 0 else "STILL TOGGLING")
            print("VALVE: %s" % steady, flush=True)
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
