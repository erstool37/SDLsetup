#!/usr/bin/env python3
"""End-to-end over REAL Modbus TCP framing, against a CLICK emulator.

Every other test in this layer drives ``FakePlc``, which proves the call shape
but not the wire. This one starts a dependency-free CLICK emulator on a
localhost port, points a real ``pymodbus.client.ModbusTcpClient`` at it through
:class:`~tools.environment.plc.PlcClient`, and checks the numbers that come
back through the whole stack: socket, MBAP framing, function codes, the
two-register float32 codec, and the decoder.

The emulator is seeded with **one real logged row from 2025-07-26 15:52:32**,
so the expected values are measurements rather than round numbers chosen to
make the arithmetic easy::

    Temp SP 25.000   RH SP 95.000
    Temp Filtered 24.866   RH Filtered 95.180
    Temp PID output 24.774   RH PID output 25.340
    PID auto (C1) on

It also performs the write the prior code got wrong -- ``25.3``, the value
observed live to come back as ``25.3587`` when only the high word was sent --
and asserts the read-back over real framing is ``25.3``.

No PLC is involved. The real one at 169.254.33.33 was unreachable when this was
written and is never contacted here.

    python dev/tests/test_environment_integration.py
"""
from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

HELPER = Path(__file__).resolve().parent / "click_emu_helper.py"

#: The logged row the emulator is seeded with, by our field name.
EXPECTED = {
    "temp_sp_c": 25.000,
    "rh_sp_pct": 95.000,
    "temp_filtered_c": 24.866,
    "rh_filtered_pct": 95.180,
    "temp_pid_output_c": 24.774,
    "rh_pid_output_pct": 25.340,
}
#: float32 cannot hold 24.866 exactly; this is well inside that quantisation.
TOL = 1e-4

if not HELPER.exists():
    # Skip LOUDLY. A fixture-dependent test that quietly passes without its
    # fixture is worse than no test: it reports coverage it does not have.
    print("[test] SKIP: the CLICK emulator helper is missing")
    print("[test] SKIP: expected it at %s" % HELPER)
    print("[test] SKIP: nothing on the wire was verified by this run")
    print("SKIPPED (no emulator helper)")
    sys.exit(0)

from tools.environment import registers  # noqa: E402
from tools.environment.plc import CONFIRMED, PlcClient, PlcSettings  # noqa: E402
from tools.environment.safety import SetpointLimits  # noqa: E402

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


def close_to(got, want: float, message: str) -> None:
    ok(got is not None and abs(got - want) < TOL, message,
       "want %.3f got %r" % (want, got))


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def wait_until_listening(port: int, timeout_s: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.05)
    return False


port = free_port()
emulator = subprocess.Popen(
    [sys.executable, str(HELPER), str(port)],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
client: PlcClient | None = None
try:
    ok(wait_until_listening(port), "the emulator is listening on 127.0.0.1:%d" % port)

    settings = PlcSettings.from_config(
        {}, host="127.0.0.1", port=port, allow_actuation=True, timeout_s=3.0)
    client = PlcClient(settings)
    ok(client.connect() is True, "a real ModbusTcpClient connected",
       str(client.last_error or ""))
    ok(type(client._transport).__name__ == "ModbusTcpClient",
       "and it really is pymodbus, not the fake",
       type(client._transport).__name__)

    print("\n--- the logged row of 2025-07-26 15:52:32, over the wire ---")
    record = client.read_block()
    ok(record.read_ok is True, "the block read succeeded", str(client.last_error or ""))
    ok(record.partial is False, "and all 23 DF pairs arrived")
    ok(len(record.raw_registers) == registers.BLOCK_COUNT,
       "%d registers came back" % registers.BLOCK_COUNT,
       "%d" % len(record.raw_registers))
    for field, want in EXPECTED.items():
        close_to(record.channels.get(field), want, field)
    ok(record.pid_enabled is True, "coil C1 reads PID auto = on")
    ok(set(record.unknown_channels) == {"DF3", "DF4"},
       "DF3/DF4 arrive as unknown-meaning channels, not as named ones",
       "%s" % sorted(record.unknown_channels))
    ok(record.quality["rh_raw_pct"].in_valid_range is True,
       "the RH channel is inside the range the PLC's own scaling defines")

    print("\n--- the write the prior code got wrong: 25.3 ---")
    limits = SetpointLimits()
    result = client.write_float32(limits.validate("temp_sp_c", 25.3))
    ok(result.outcome == CONFIRMED, "the FC16 write came back confirmed",
       "%s %s" % (result.outcome, result.error or ""))
    ok(len(result.values) == 2, "two registers went out")
    ok(result.readback == result.values,
       "and the read-back matched word-for-word",
       "%r vs %r" % (result.readback, result.values))
    after = client.read_block()
    close_to(after.channels.get("temp_sp_c"), 25.3,
             "a fresh block read now shows 25.3")
    got = after.channels.get("temp_sp_c")
    ok(got is not None and abs(got - 25.3587) > 1e-3,
       "and NOT 25.3587, which is what a high-word-only write produced live",
       "%r" % got)

    print("\n--- the other setpoint, to prove the address arithmetic ---")
    rh = client.write_float32(limits.validate("rh_sp_pct", 87.6))
    ok(rh.outcome == CONFIRMED, "RH SP write confirmed", str(rh.error or ""))
    both = client.read_block()
    close_to(both.channels.get("rh_sp_pct"), 87.6, "DF6 now reads 87.6")
    close_to(both.channels.get("temp_sp_c"), 25.3,
             "and DF5 was not disturbed by it")

    print("\n--- the PID coil, over real framing ---")
    off = client.set_pid(False)
    ok(off.outcome == CONFIRMED, "set_pid(False) confirmed", str(off.error or ""))
    ok(client.read_pid_enabled() is False, "and C1 reads off")
    on = client.set_pid(True)
    ok(on.outcome == CONFIRMED, "set_pid(True) confirmed", str(on.error or ""))
    ok(client.read_pid_enabled() is True, "and C1 reads on again")
finally:
    if client is not None:
        client.close()
    emulator.terminate()
    try:
        emulator.wait(timeout=5)
    except subprocess.TimeoutExpired:
        emulator.kill()
        emulator.wait(timeout=5)

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
