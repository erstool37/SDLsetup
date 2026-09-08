#!/usr/bin/env python3
"""Known-answer tests for the circulator setpoint encoding. No hardware, no port.

Nothing here opens a serial port. It cannot: this file imports the codec only,
and the codec is pure arithmetic over a float.

The seven register triples below are not invented -- they were produced by the
prior working system's own expression and verified bit-identical against the
clean equivalent this module implements. They are the only thing about address
980 that IS established, so they are the regression fence around it.

The last block is the defect being fixed. The prior code encoded whatever
arrived, including NaN, which packs without complaint into
[0x0000, 0x0000, 0x0000, 0xF87F] and was sent to a temperature controller with
no documented clamp of its own. encode_setpoint must refuse it instead.

    python dev/tests/test_circulator_codec.py
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.circulator.codec import (  # noqa: E402
    SETPOINT_ADDRESS,
    SETPOINT_COUNT,
    decode_setpoint,
    encode_setpoint,
)

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        fails += 1


def raises(fn, exc: type, message: str) -> None:
    global fails
    try:
        value = fn()
    except exc as caught:
        print("[test] PASS: %s  -> %s" % (message, str(caught)[:70]))
        return
    except Exception as other:  # noqa: BLE001 - wrong exception type is a failure
        print("[test] FAIL: %s -- raised %s, not %s"
              % (message, type(other).__name__, exc.__name__))
        fails += 1
        return
    print("[test] FAIL: %s -- nothing raised, returned %r" % (message, value))
    fails += 1


def prior_regs(value: float) -> list[int]:
    """The prior system's expression, kept verbatim as the reference oracle.

    MATLAB: ``regs = flip(typecast(flip(typecast(x,'uint8')),'uint16'))``.
    encode_setpoint must agree with this on every FINITE value and must refuse
    the non-finite ones this happily encoded.
    """
    packed = struct.pack("<d", value)[::-1]
    w0, w1, w2, w3 = struct.unpack("<4H", packed)
    return [w3, w2, w1, w0]


#: value -> registers written at 980..983. Verified against the prior code.
KNOWN = [
    (25.0, [0x0000, 0x0000, 0x0000, 0x3940]),
    (30.0, [0x0000, 0x0000, 0x0000, 0x3E40]),
    (0.0, [0x0000, 0x0000, 0x0000, 0x0000]),
    (-5.25, [0x0000, 0x0000, 0x0000, 0x15C0]),
    (24.774, [0xA01A, 0x2FDD, 0x24C6, 0x3840]),
    (11.4, [0xCDCC, 0xCCCC, 0xCCCC, 0x2640]),
    (123.456, [0x77BE, 0x9F1A, 0x2FDD, 0x5E40]),
]

print("--- the register block, which is all that is known about this device ---")
ok(SETPOINT_ADDRESS == 980, "setpoint block starts at 0-based address 980")
ok(SETPOINT_COUNT == 4, "and is 4 holding registers wide (one float64)")

print("\n--- known answers, forward ---")
for value, expected in KNOWN:
    got = encode_setpoint(value)
    ok(got == expected, "encode_setpoint(%r)" % value,
       "[%s]" % ", ".join("0x%04X" % w for w in got))

print("\n--- known answers, backward (decode is the inverse) ---")
for value, expected in KNOWN:
    back = decode_setpoint(expected)
    ok(back == value, "decode_setpoint of the %r block returns it exactly" % value,
       repr(back))

print("\n--- and it agrees with the prior system's own expression ---")
for value, _ in KNOWN:
    ok(encode_setpoint(value) == prior_regs(value),
       "bit-identical to the prior encoding for %r" % value)
for value in (1.0, -0.0, 1e-300, 1e300, 273.15, 0.1):
    ok(encode_setpoint(value) == prior_regs(value),
       "bit-identical to the prior encoding for %r" % value)

print("\n--- round trip over values the bound would actually allow ---")
# The bound is the OPEN interval (0, 30) -- see tools/circulator/safety.py.
# 0.0 and 30.0 encode fine (that is this module's job) but are not commandable.
for value in (0.001, 4.0, 11.4, 24.774, 29.999):
    ok(decode_setpoint(encode_setpoint(value)) == value,
       "round trip is exact for %r" % value)

print("\n--- THE DEFECT: the prior code encoded NaN and sent it ---")
nan_block = prior_regs(float("nan"))
ok(nan_block == [0x0000, 0x0000, 0x0000, 0xF87F],
   "the prior encoding turned NaN into a frame with no complaint",
   "[%s]" % ", ".join("0x%04X" % w for w in nan_block))
raises(lambda: encode_setpoint(float("nan")), ValueError,
       "encode_setpoint refuses NaN instead of encoding it")
raises(lambda: encode_setpoint(float("inf")), ValueError,
       "encode_setpoint refuses +inf")
raises(lambda: encode_setpoint(float("-inf")), ValueError,
       "encode_setpoint refuses -inf")
raises(lambda: encode_setpoint("25.0"), ValueError,
       "encode_setpoint refuses a string rather than coercing it")
raises(lambda: encode_setpoint(None), ValueError,
       "encode_setpoint refuses None")

print("\n--- decode rejects a malformed block rather than guessing ---")
raises(lambda: decode_setpoint([0, 0, 0]), ValueError,
       "decode_setpoint refuses 3 registers")
raises(lambda: decode_setpoint([0, 0, 0, 0, 0]), ValueError,
       "decode_setpoint refuses 5 registers")
raises(lambda: decode_setpoint([0, 0, 0, 0x10000]), ValueError,
       "decode_setpoint refuses a register above 0xFFFF")
raises(lambda: decode_setpoint([0, 0, 0, -1]), ValueError,
       "decode_setpoint refuses a negative register")

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
