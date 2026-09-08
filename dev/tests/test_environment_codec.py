#!/usr/bin/env python3
"""The float32 codec for the CLICK PLC's DF registers, against live-verified answers.

A CLICK float32 spans two 16-bit registers. Inside each register the bytes are
big-endian; across the pair the LOW-order 16 bits sit in the FIRST
(lower-addressed) register. Get that word order backwards and the decode does
not fail -- it returns a plausible-looking number, which is why the five
known answers below were captured from a live round-trip and are asserted here
rather than reasoned about.

No hardware, no network: every value in this file is a literal.

    python dev/tests/test_environment_codec.py
"""
from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.environment import reading, registers  # noqa: E402

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        failures += 1


def raises(fn, exc, message: str) -> None:
    global failures
    try:
        result = fn()
    except exc as err:
        print("[test] PASS: %s  -> %s" % (message, str(err)[:70]))
        return
    except Exception as err:
        print("[test] FAIL: %s -- raised %s, not %s"
              % (message, type(err).__name__, exc.__name__))
        failures += 1
        return
    print("[test] FAIL: %s -- returned %r instead of raising" % (message, result))
    failures += 1


# Captured from a live PLC round-trip. (value, lo_register, hi_register)
KNOWN = (
    (25.0, 0x0000, 0x41C8),
    (24.866, 0xED91, 0x41C6),
    (95.18, 0x5C29, 0x42BE),
    (-12.5, 0x0000, 0xC148),
    (0.0, 0x0000, 0x0000),
)


def f32(value: float) -> float:
    """value rounded to float32, so a comparison is not asserting float64 precision."""
    return struct.unpack(">f", struct.pack(">f", value))[0]


print("--- the five live-verified known answers, both directions ---")
for value, lo, hi in KNOWN:
    got = registers.regs_to_f32(lo, hi)
    ok(abs(got - value) < 1e-4,
       "decode [0x%04X, 0x%04X] -> %r" % (lo, hi, value), repr(got))
    ok(registers.f32_to_regs(value) == (lo, hi),
       "encode %r -> [0x%04X, 0x%04X]" % (value, lo, hi),
       "got %r" % (registers.f32_to_regs(value),))

print("\n--- encode and decode are inverse ---")
for value in (0.0, 1.0, -1.0, 0.5, 25.0, -40.0, 180.0, 100.0, 1e-8, 1e8,
              3.14159, -273.15, 65535.5):
    lo, hi = registers.f32_to_regs(value)
    ok(registers.regs_to_f32(lo, hi) == f32(value),
       "round-trip %r" % value, "lo=0x%04X hi=0x%04X" % (lo, hi))

print("\n--- the codec is NOT word-order invariant (the trap this guards) ---")
swapped = registers.regs_to_f32(0x41C8, 0x0000)   # lo and hi handed over backwards
ok(abs(swapped - 25.0) > 1.0,
   "swapping lo/hi for 25.0 does NOT yield ~25.0", repr(swapped))
ok(abs(swapped) < 1e-30,
   "it yields a visibly absurd magnitude, not a plausible temperature", repr(swapped))
for value, lo, hi in KNOWN:
    if lo == hi:
        continue                                   # symmetric pair, nothing to swap
    ok(abs(registers.regs_to_f32(hi, lo) - value) > 1e-3,
       "swapped decode of %r is wrong" % value, repr(registers.regs_to_f32(hi, lo)))

print("\n--- a non-finite value is refused, never encoded as a plausible number ---")
raises(lambda: registers.f32_to_regs(float("nan")), ValueError, "f32_to_regs(nan) raises")
raises(lambda: registers.f32_to_regs(float("inf")), ValueError, "f32_to_regs(inf) raises")
raises(lambda: registers.f32_to_regs(float("-inf")), ValueError, "f32_to_regs(-inf) raises")

print("\n--- a decoded NaN stays NaN and is flagged, not substituted ---")
nan_regs = struct.unpack(">HH", struct.pack(">f", float("nan")))
nan_block = [0] * registers.BLOCK_COUNT
nan_block[2 * (18 - 1)] = nan_regs[1]              # DF18 temp_filtered_c, LSW first
nan_block[2 * (18 - 1) + 1] = nan_regs[0]
r = reading.decode_block(nan_block, t_utc="2026-09-07T00:00:00Z", monotonic_s=1.0)
ok(math.isnan(r.channels["temp_filtered_c"]), "a NaN channel stays NaN")
ok(r.quality["temp_filtered_c"].finite is False, "and is flagged finite=False")

print("\n--- a full 46-register block decodes every named channel ---")
VALUES = {
    1: 44.5, 2: 24.866, 3: 12.25, 4: 7.5, 5: 25.0, 6: 45.0, 7: 0.134, 8: 0.5,
    9: 3.25, 10: 1.5, 11: 2.0, 12: 60.0, 13: 1.25, 14: 90.0, 15: 0.2, 16: 0.4,
    17: 0.1, 18: 24.9, 19: 44.6, 20: 100.0, 21: 0.0, 22: 100.0, 23: 0.0,
}
block: list[int] = []
for df in range(1, 24):
    lo, hi = registers.f32_to_regs(VALUES[df])
    block.extend((lo, hi))
ok(len(block) == registers.BLOCK_COUNT,
   "the block is BLOCK_COUNT registers long", str(len(block)))

full = reading.decode_block(block, t_utc="2026-09-07T00:00:00Z", monotonic_s=12.5,
                            pid_enabled=True)
ok(full.partial is False, "a complete block is not partial")
ok(full.read_ok is True, "and reads ok")
ok(full.raw_registers == tuple(block), "every raw uint16 is kept for later re-decoding")
ok(len(full.channels) == 21, "21 named channels decoded (23 DFs minus the 2 unknown)",
   str(len(full.channels)))
for df, spec in ((1, "rh_raw_pct"), (2, "temp_raw_c"), (18, "temp_filtered_c"),
                 (19, "rh_filtered_pct"), (23, "rh_output_lb_pct")):
    ok(abs(full.channels[spec] - VALUES[df]) < 1e-3,
       "DF%d -> %s == %r" % (df, spec, VALUES[df]), repr(full.channels[spec]))
ok(full.pid_enabled is True, "pid_enabled is carried through")
ok(full.temp_filtered_c is not None and abs(full.temp_filtered_c - 24.9) < 1e-3,
   "the temp_filtered_c convenience property agrees with channels")
ok(full.temp_sp_c is not None and abs(full.temp_sp_c - 25.0) < 1e-3,
   "the temp_sp_c convenience property agrees with channels")

print("\n--- the two unknown channels are never given a physical name ---")
ok(set(full.unknown_channels) == {"DF3", "DF4"},
   "unknown_channels is keyed DF3/DF4", repr(sorted(full.unknown_channels)))
ok(abs(full.unknown_channels["DF3"]["value"] - 12.25) < 1e-3, "DF3 carries its value")
ok(full.unknown_channels["DF3"]["meaning"] == "UNKNOWN", "DF3 meaning is UNKNOWN")
ok(full.unknown_channels["DF3"]["units"] == "UNKNOWN", "DF3 units are UNKNOWN")
ok("AD3" in full.unknown_channels["DF3"]["scale"],
   "DF3 carries the PLC's own scale string", full.unknown_channels["DF3"]["scale"])
ok("AD4" in full.unknown_channels["DF4"]["scale"], "DF4 likewise")

print("\n--- a short block is partial, not an exception and not a fabrication ---")
short = reading.decode_block(block[:36], t_utc="2026-09-07T00:00:00Z", monotonic_s=13.0)
ok(short.partial is True, "36 registers (DF1..DF18) sets partial=True")
ok(short.read_ok is True, "what did arrive is still usable")
ok(len(short.raw_registers) == 36, "and the raw words are kept as received")
ok("temp_filtered_c" in short.channels, "DF18 is present -- its pair is complete")
for missing in ("rh_filtered_pct", "temp_output_ub_c", "temp_output_lb_c",
                "rh_output_ub_pct", "rh_output_lb_pct"):
    ok(missing not in short.channels, "%s is absent, not invented" % missing)
    ok(missing not in short.quality, "%s has no quality entry either" % missing)

odd = reading.decode_block(block[:37], t_utc="2026-09-07T00:00:00Z", monotonic_s=13.5)
ok(odd.partial is True, "an odd-length block is partial")
ok("rh_filtered_pct" not in odd.channels, "a half-arrived pair is not decoded")
empty = reading.decode_block([], t_utc="2026-09-07T00:00:00Z", monotonic_s=14.0)
ok(empty.read_ok is False and empty.partial is True,
   "an empty block reads not-ok and partial")
ok(empty.channels == {}, "and invents nothing")

print("\n--- range flags are facts about the reading, not verdicts ---")
bad = dict(VALUES)
bad[1] = 137.0          # RH above the PLC's 0..100 scale
bad[2] = -55.0          # Temp below the PLC's -40..180 scale
bad_block: list[int] = []
for df in range(1, 24):
    lo, hi = registers.f32_to_regs(bad[df])
    bad_block.extend((lo, hi))
r2 = reading.decode_block(bad_block, t_utc="2026-09-07T00:00:00Z", monotonic_s=15.0)
ok(abs(r2.channels["rh_raw_pct"] - 137.0) < 1e-3,
   "an out-of-range RH is reported unchanged, not clamped", repr(r2.channels["rh_raw_pct"]))
ok(r2.quality["rh_raw_pct"].in_valid_range is False, "and flagged out of range")
ok(r2.quality["temp_raw_c"].in_valid_range is False, "an out-of-range Temp likewise")
ok(r2.quality["temp_filtered_c"].in_valid_range is None,
   "a register with no PLC-defined range flags None, not False")
ok(r2.read_ok is True, "the reading is still read_ok -- flagging is not a verdict")

print("\n--- as_dict() is JSON-serialisable ---")
payload = full.as_dict()
text = json.dumps(payload)
ok(isinstance(text, str) and len(text) > 100, "as_dict() survives json.dumps")
back = json.loads(text)
ok(back["channels"]["temp_raw_c"] == payload["channels"]["temp_raw_c"],
   "a channel round-trips through JSON")
ok(isinstance(back["raw_registers"], list) and len(back["raw_registers"]) == 46,
   "raw_registers is a JSON list of all 46 words")
ok(back["quality"]["rh_raw_pct"]["finite"] is True,
   "quality is plain dicts, not dataclass objects")
ok(back["unknown_channels"]["DF3"]["meaning"] == "UNKNOWN",
   "unknown_channels survives intact")

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
