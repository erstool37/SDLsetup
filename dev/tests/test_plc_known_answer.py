#!/usr/bin/env python3
"""The first live read of the CLICK PLC, locked in as a known answer.

This is the only ground truth in the environment layer that did not come from
the code being tested: on **2026-09-07 ~17:49 KST** the live PLC at
``169.254.33.33:502`` was read read-only, five times, with
``read_holding_registers(28672, count=46)`` + ``read_coils(16384, count=1)``,
``device_id=1``, ``C1 PID Auto`` **OFF**. Nothing was written.

What it locks in, and why each one matters
------------------------------------------

``DF21 Temp Output LB = 10.0``
    The codebase carried ``0.0``, inferred from the saturation envelope of the
    prior system's logs. **That inference was wrong.** The PLC clamps its own
    temperature-PID output to ``[10.0, 30.0]`` C, so the circulator can never
    legitimately receive a value below 10 C. Every other bound in the tree that
    quoted ``0.0`` was quoting a guess.

``DF3 == DF4 == 0.000``
    Their scaling is 0-20 mA -> 0..100, so exactly 0.000 means **0 mA: nothing
    is wired to those analogue inputs.** They stay ``MEANING UNKNOWN`` -- this
    read says they are currently unwired, not what they would measure.

raw jitters, filtered does not
    DF1/DF2 move across the five samples while DF18/DF19 barely do. That is the
    PLC's own first-order filter (``Alpha = 0.01``) genuinely running, and it is
    also the evidence that the two sensors are genuinely connected rather than
    reading a frozen register.

``DF1``/``DF2`` are not swapped
    The prior code had them the other way round. The alternative reading of this
    block is **21.8 %RH and 70.8 C**, and 70.8 C is impossible for this room, so
    the PLC's own nickname table is corroborated. This is the *only* channel
    identity today's read confirms -- see the note at the end of this file.

No hardware, no network. Reads a saved capture and **SKIPS LOUDLY** if it is
absent rather than passing quietly.

    python dev/tests/test_plc_known_answer.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

CAPTURE_NAME = "first_live_read_20260907_174926.json"

#: Copy beside the test (~4 KB). NOT committed -- `.gitignore` carries
#: `dev/tests/fixtures/*` and negates only README.md. See the TODO below.
FIXTURE = Path(__file__).resolve().parent / "fixtures" / CAPTURE_NAME
#: Where the run that produced it wrote it. ``dataset/`` is gitignored.
DATASET_COPY = REPO / "dataset" / "plc_first_live_read" / CAPTURE_NAME

CONFIG_PATH = REPO / "configs" / "config.yaml"

fails = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        fails += 1


source = FIXTURE if FIXTURE.exists() else DATASET_COPY
if not source.exists():
    print("[test] SKIP: no capture at either")
    print("         %s" % FIXTURE)
    print("         %s" % DATASET_COPY)
    print("[test] SKIP: the live PLC decode is UNVERIFIED in this checkout.")
    print("       The capture is five reads of the live CLICK PLC at "
          "169.254.33.33:502 on")
    print("       2026-09-07 ~17:49 KST -- read_holding_registers(28672, count=46)")
    print("       + read_coils(16384, count=1), device_id=1, PID Auto OFF, nothing")
    print("       written. Regenerate it by re-reading the PLC read-only and "
          "writing the")
    print("       same {artifact, host, read, decode, samples[{t_utc, "
          "raw_registers, pid_auto}]}")
    print("       shape back to %s." % DATASET_COPY)
    print("SKIPPED (no live PLC capture)")
    sys.exit(0)

from tools import config as _config  # noqa: E402
from tools.circulator.circulator import CirculatorSettings  # noqa: E402
from tools.environment import reading as _reading  # noqa: E402
from tools.environment import registers  # noqa: E402

capture = json.loads(source.read_text(encoding="utf-8"))
samples = capture["samples"]

print("--- the capture itself, and where it came from ---")
ok(source == FIXTURE,
   "the test reads the dev/tests/fixtures/ copy in preference to dataset/",
   str(source))
if FIXTURE.exists() and DATASET_COPY.exists():
    ok(FIXTURE.read_bytes() == DATASET_COPY.read_bytes(),
       "and the fixture copy is byte-identical to the run's own output")
# NOT committed yet, and saying so beats implying it. `.gitignore` carries
# `dev/tests/fixtures/*` with only README.md negated, so this 4 KB capture is
# ignored like the image fixtures beside it and a fresh clone falls through to
# the SKIP branch above.
# TODO(operator): add `!dev/tests/fixtures/*.json` to .gitignore (one line,
# right after the existing `!dev/tests/fixtures/README.md`) if this capture
# should survive a clone. .gitignore was outside this change's write scope.
print("[test] NOTE: dev/tests/fixtures/* is gitignored -- this capture is NOT")
print("       committed. See the TODO in this file's source.")
ok(len(samples) == 5, "five samples", str(len(samples)))
ok(capture.get("device_id") == 1, "read with device_id=1", repr(capture.get("device_id")))
ok(capture.get("read") == "read_holding_registers(28672, count=46) + "
                          "read_coils(16384, count=1)",
   "with the block read this layer performs", repr(capture.get("read")))
ok("PID Auto was OFF" in str(capture.get("note", "")),
   "and the capture records that the PID was OFF")

decoded = [
    _reading.decode_block(s["raw_registers"], t_utc=s["t_utc"], monotonic_s=0.0,
                          pid_enabled=s["pid_auto"])
    for s in samples
]

print("\n--- every sample decodes as a whole block, with the PID off ---")
for index, (raw, record) in enumerate(zip(samples, decoded, strict=True)):
    ok(len(raw["raw_registers"]) == registers.BLOCK_COUNT,
       "sample %d carries all %d words" % (index, registers.BLOCK_COUNT),
       str(len(raw["raw_registers"])))
    ok(record.read_ok and not record.partial,
       "sample %d decodes complete, not partial" % index)
    ok(record.pid_enabled is False,
       "sample %d reports pid_auto False -- False, not None" % index,
       repr(record.pid_enabled))

# The first sample's first twelve words, recorded independently of this file.
ok(list(samples[0]["raw_registers"][:12])
   == [29675, 17037, 47480, 16813, 0, 0, 0, 0, 0, 16840, 0, 17076],
   "the first sample's first twelve raw words are the recorded ones")

print("\n--- the STEADY channels: one value, every sample, tolerance 1e-3 ---")
#: field -> value, straight off the live PLC. Held here rather than imported so
#: the test does not check the code against itself.
STEADY = {
    "temp_sp_c": 25.0,
    "rh_sp_pct": 90.0,
    "temp_error_c": 0.0,
    "rh_error_pct": 0.0,
    "temp_pid_output_c": 25.0,
    "rh_pid_output_pct": 100.0,
    "temp_kp": 0.5,
    "temp_ti": 180.0,
    "rh_kp": -3.0,
    "rh_ti": 60.0,
    "temp_old_error_c": 0.0,
    "rh_old_error_pct": 0.0,
    "alpha": 0.01,
    "temp_output_ub_c": 30.0,
    "temp_output_lb_c": 10.0,
    "rh_output_ub_pct": 100.0,
    "rh_output_lb_pct": 0.0,
}
for field, expected in STEADY.items():
    seen = [record.channels.get(field) for record in decoded]
    agree = all(v is not None and abs(v - expected) <= 1e-3 for v in seen)
    ok(agree, "%s == %g in all five samples" % (field, expected),
       "saw %s" % [None if v is None else round(v, 6) for v in seen])

print("\n--- the correction this read forced: DF21 is 10.0, not 0.0 ---")
lb = registers.BY_FIELD["temp_output_lb_c"]
ok(lb.df == 21, "temp_output_lb_c is DF21", str(lb.df))
ok(all(abs(record.channels["temp_output_lb_c"] - 10.0) <= 1e-3 for record in decoded),
   "and the live PLC reports it as 10.0 C -- the tree carried 0.0, inferred "
   "from log saturation, and that inference was WRONG")
ok(all(abs(record.channels["temp_output_ub_c"] - 30.0) <= 1e-3 for record in decoded),
   "the upper clamp really is 30.0 C, so the PLC's temperature-PID output "
   "lives in [10.0, 30.0]")
ok(getattr(registers, "MEASURED_OUTPUT_CLAMPS", None)
   == {20: 30.0, 21: 10.0, 22: 100.0, 23: 0.0},
   "registers.MEASURED_OUTPUT_CLAMPS carries the four measured clamps",
   repr(getattr(registers, "MEASURED_OUTPUT_CLAMPS", None)))
ok(getattr(registers, "MEASURED_DATE", None) == "2026-09-07",
   "and the date they were measured on",
   repr(getattr(registers, "MEASURED_DATE", None)))
for df in range(20, 24):
    spec = registers.REGISTERS[df - 1]
    ok(spec.provenance == getattr(registers.Provenance, "MEASURED", "no-MEASURED-member"),
       "DF%d (%s) is now MEASURED, not INFERRED" % (df, spec.nickname),
       str(spec.provenance))

print("\n--- DF3/DF4: exactly 0.000, i.e. 0 mA, i.e. nothing is wired there ---")
for record in decoded:
    unknown = record.unknown_channels
    ok(set(unknown) == {"DF3", "DF4"},
       "the unknown channels are exactly DF3 and DF4", str(sorted(unknown)))
    ok(abs(unknown["DF3"]["value"]) <= 1e-3 and abs(unknown["DF4"]["value"]) <= 1e-3,
       "and both read 0.000 -- 0 mA on a 0-20 mA input",
       "%r / %r" % (unknown["DF3"]["value"], unknown["DF4"]["value"]))
    ok(unknown["DF3"]["meaning"] == "UNKNOWN" and unknown["DF4"]["meaning"] == "UNKNOWN",
       "their MEANING stays UNKNOWN -- 'currently unwired' is not 'now understood'")
    break

print("\n--- the PLC's own filter is running: raw jitters, filtered does not ---")


def spread(field: str) -> float:
    values = [record.channels[field] for record in decoded]
    return max(values) - min(values)


for raw_field, filt_field in (("rh_raw_pct", "rh_filtered_pct"),
                              ("temp_raw_c", "temp_filtered_c")):
    raw_spread, filt_spread = spread(raw_field), spread(filt_field)
    ok(raw_spread > 0.0,
       "%s actually varies across the five samples" % raw_field,
       "spread %.6f" % raw_spread)
    ok(filt_spread < raw_spread,
       "and %s varies LESS -- Alpha=0.01 is a live first-order filter, so the "
       "sensor is genuinely connected" % filt_field,
       "filtered %.6f < raw %.6f" % (filt_spread, raw_spread))

print("\n--- the jittering channels sit in the window the operator observed ---")
# Ranges recorded from the live PLC while it was being watched. The five saved
# samples are a subset of that window, so containment is the assertion; the
# saved spread is NOT the whole observed spread.
OBSERVED = {
    "rh_raw_pct": (70.726, 70.818),
    "temp_raw_c": (21.648, 21.917),
}
for field, (low, high) in OBSERVED.items():
    values = [record.channels[field] for record in decoded]
    ok(all(low - 1e-3 <= v <= high + 1e-3 for v in values),
       "%s stays inside the observed %.3f..%.3f window" % (field, low, high),
       "saw %.6f..%.6f" % (min(values), max(values)))

# The filtered channels are asserted against THIS capture, not against the
# operator's wider live window: the reported filtered window (21.780-21.824 C,
# 70.770-70.790 %) does NOT contain these five samples -- see the note at the
# end of this file. Locking the capture is the honest known answer.
FILTERED_IN_CAPTURE = {
    "temp_filtered_c": (21.7638, 21.7789),
    "rh_filtered_pct": (70.7434, 70.7523),
}
for field, (low, high) in FILTERED_IN_CAPTURE.items():
    values = [record.channels[field] for record in decoded]
    ok(all(low - 1e-3 <= v <= high + 1e-3 for v in values),
       "%s matches this capture's recorded %.4f..%.4f" % (field, low, high),
       "saw %.6f..%.6f" % (min(values), max(values)))

print("\n--- what this read does, and does NOT, corroborate ---")
rh, temp = registers.BY_FIELD["rh_raw_pct"], registers.BY_FIELD["temp_raw_c"]
ok(rh.verified_live and temp.verified_live,
   "DF1/DF2 are verified_live: the swap would read 21.8 %RH and 70.8 C, and "
   "70.8 C is impossible for this room")
still_unverified = [spec.df for spec in registers.REGISTERS
                    if spec.df not in (1, 2) and spec.verified_live]
ok(still_unverified == [],
   "and NOTHING else claims verified_live -- channel identity beyond DF1/DF2 "
   "rests on the nickname table, which this read did not test",
   str(still_unverified))
ok(registers.COIL_SPEC.verified_live is False,
   "the coil is not verified_live either: reading C1 as False does not prove "
   "C1 is the PID flag")

print("\n--- the corrected config actually resolves, and is actually consumed ---")
if not CONFIG_PATH.exists():
    print("[test] SKIP: configs/config.yaml is missing at %s" % CONFIG_PATH)
    print("[test] SKIP: the command_min_c and unit_id corrections were NOT verified")
else:
    live = CirculatorSettings.from_config(CONFIG_PATH)
    ok(abs(live.command_min_c - 10.0) <= 1e-9,
       "circulator.command_min_c resolves to 10.0 -- DF21, measured",
       repr(live.command_min_c))
    ok(live.sources.get("command_min_c") == "config.yaml",
       "and it comes from config.yaml, not the 0.0 code default",
       repr(live.sources.get("command_min_c")))
    ok(abs(live.limits.min_c - 10.0) <= 1e-9 and abs(live.limits.max_c - 30.0) <= 1e-9,
       "so the command bound this settings object authorises is (10.0, 30.0) "
       "EXCLUSIVE", live.limits.describe())
    # The dead key: config.yaml said `device_id`, the code resolves `unit_id`.
    # 7 is not the code default (1) and not the inventory value (1), so a pass
    # here can only come from the config layer.
    from_config = CirculatorSettings.from_config({"circulator": {"unit_id": 7}})
    ok(from_config.unit_id == 7,
       "circulator.unit_id is read from config.yaml (7 is neither the code "
       "default nor circulator.json, both 1)", repr(from_config.unit_id))
    ok(from_config.sources.get("unit_id") == "config.yaml",
       "and its provenance says config.yaml", repr(from_config.sources.get("unit_id")))
    raw_section = _config.load_yaml(CONFIG_PATH).get("circulator", {})
    ok("unit_id" in raw_section,
       "the live configs/config.yaml spells the key unit_id")
    ok("device_id" not in raw_section,
       "and no longer carries the dead device_id key, which the code never read",
       str(sorted(raw_section)))

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
