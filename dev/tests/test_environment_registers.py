#!/usr/bin/env python3
"""The CLICK PLC register table, and the swapped-label defect it exists to prevent.

The prior code labelled the two analogue inputs `T, H` -- the PLC's own
nickname table says DF1 is `RH` and DF2 is `Temp`. Every temperature it read
was a humidity. The PLC's table is authoritative, and the labels are asserted
here so the swap cannot come back; single-letter field names are rejected
outright, because `T`/`H` is what made the mistake invisible.

No hardware, no network: this file reads a table of constants.

    python dev/tests/test_environment_registers.py
"""
from __future__ import annotations

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


print("--- the swapped-label regression: the PLC's nicknames are authoritative ---")
rh = registers.BY_FIELD.get("rh_raw_pct")
temp = registers.BY_FIELD.get("temp_raw_c")
ok(rh is not None and rh.df == 1, "rh_raw_pct is DF1", str(rh.df) if rh else "missing")
ok(rh is not None and rh.nickname == "RH",
   'DF1 carries the PLC nickname "RH"', rh.nickname if rh else "missing")
ok(temp is not None and temp.df == 2, "temp_raw_c is DF2",
   str(temp.df) if temp else "missing")
ok(temp is not None and temp.nickname == "Temp",
   'DF2 carries the PLC nickname "Temp"', temp.nickname if temp else "missing")
ok(rh is not None and temp is not None and rh.df < temp.df,
   "humidity comes FIRST -- the order the prior code had backwards")

print("\n--- a single-letter field name is refused outright ---")
all_specs = list(registers.REGISTERS) + [registers.COIL_SPEC]
bad_names = [s.field for s in all_specs
             if s.field is not None and s.field.upper() in {"T", "H"}]
ok(not bad_names, "no field is named T or H", repr(bad_names))
short_names = [s.field for s in all_specs
               if s.field is not None and len(s.field) < 3]
ok(not short_names, "no field name is shorter than three characters", repr(short_names))
ok(all(s.field is None or s.field == s.field.lower() for s in all_specs),
   "every field name is lower_snake_case")

print("\n--- the address formula ---")
ok(registers.DF_BASE == 28672, "DF_BASE is 28672", str(registers.DF_BASE))
ok(registers.C1_COIL == 16384, "C1_COIL is 16384", str(registers.C1_COIL))
ok(len(registers.REGISTERS) == 23, "23 DF registers are declared",
   str(len(registers.REGISTERS)))
ok([s.df for s in registers.REGISTERS] == list(range(1, 24)),
   "declared in DF order, 1..23 with no gap")
for spec in registers.REGISTERS:
    want = 28672 + 2 * (spec.df - 1)
    ok(spec.address == want, "DF%d address == %d" % (spec.df, want), str(spec.address))
    ok(registers.df_address(spec.df) == want, "df_address(%d) == %d" % (spec.df, want))
ok(registers.BLOCK_COUNT == 46, "BLOCK_COUNT covers DF1..DF23 as 46 registers",
   str(registers.BLOCK_COUNT))
ok(registers.BLOCK_COUNT == 2 * len(registers.REGISTERS),
   "and equals two registers per DF")

print("\n--- the coil ---")
ok(registers.COIL_SPEC.field == "pid_auto", "the coil is pid_auto",
   str(registers.COIL_SPEC.field))
ok(registers.COIL_SPEC.address == 16384, "at address 16384",
   str(registers.COIL_SPEC.address))
ok(registers.COIL_SPEC.nickname == "PID Auto", 'nickname "PID Auto"',
   registers.COIL_SPEC.nickname)
ok(registers.COIL_SPEC.rw == "RW", "and is writable", registers.COIL_SPEC.rw)

print("\n--- DF3 and DF4 have no meaning, and are never given a name ---")
ok(registers.UNKNOWN_DF == (3, 4), "UNKNOWN_DF is (3, 4)", repr(registers.UNKNOWN_DF))
for df in registers.UNKNOWN_DF:
    spec = registers.REGISTERS[df - 1]
    ok(spec.df == df and spec.field is None,
       "DF%d has field is None" % df, repr(spec.field))
    ok(spec.units == "UNKNOWN", "DF%d units are UNKNOWN" % df, spec.units)
    ok(spec.provenance == registers.Provenance.UNKNOWN,
       "DF%d provenance is UNKNOWN" % df, str(spec.provenance))
ok(all(s.field is not None for s in registers.REGISTERS
       if s.df not in registers.UNKNOWN_DF),
   "every other DF does have a field name")
named_dfs = {s.df for s in registers.REGISTERS if s.field is not None}
ok(named_dfs.isdisjoint(set(registers.UNKNOWN_DF)),
   "no unknown DF leaked into BY_FIELD")
ok(len(registers.BY_FIELD) == 21, "BY_FIELD holds the 21 named registers",
   str(len(registers.BY_FIELD)))

print("\n--- and in a decoded reading they stay out of channels ---")
block: list[int] = []
for df in range(1, 24):
    lo, hi = registers.f32_to_regs(float(df))
    block.extend((lo, hi))
r = reading.decode_block(block, t_utc="2026-09-07T00:00:00Z", monotonic_s=0.0)
ok(set(r.unknown_channels) == {"DF3", "DF4"},
   "DF3/DF4 appear in unknown_channels", repr(sorted(r.unknown_channels)))
ok(all(not k.startswith("DF") for k in r.channels),
   "and nothing DF-keyed leaked into channels", repr(sorted(r.channels)[:3]))
ok(all(k in registers.BY_FIELD for k in r.channels),
   "every channel key is a declared field name")
ok(abs(r.unknown_channels["DF3"]["value"] - 3.0) < 1e-6,
   "DF3's value is decoded, just not interpreted")

print("\n--- what our layer may write, and what it must not ---")
ok(set(registers.WRITABLE) == {"temp_sp_c", "rh_sp_pct"},
   "WRITABLE is exactly the two setpoints", repr(sorted(registers.WRITABLE)))
ok(all(registers.WRITABLE[f].rw == "RW" for f in registers.WRITABLE),
   "and both are marked RW")
ok(registers.WRITABLE["temp_sp_c"].df == 5, "temp_sp_c is DF5")
ok(registers.WRITABLE["rh_sp_pct"].df == 6, "rh_sp_pct is DF6")

# The PID is hand-written in ladder logic, so its gains tune a loop whose
# structure we cannot read back. Read-only by POLICY, not by device capability.
for df in list(range(11, 18)) + list(range(20, 24)):
    spec = registers.REGISTERS[df - 1]
    ok(spec.rw == "R", "DF%d (%s) is read-only by policy" % (df, spec.nickname), spec.rw)
    ok(spec.field not in registers.WRITABLE,
       "DF%d is not writable through our layer" % df)

print("\n--- provenance is on every register, and never overstated ---")
ok(all(isinstance(s.provenance, registers.Provenance) for s in registers.REGISTERS),
   "every spec carries a Provenance")
ok(not any(s.provenance == registers.Provenance.VENDOR for s in registers.REGISTERS),
   "nothing claims VENDOR -- AutomationDirect publishes no static table")
# DF20-23 were INFERRED until 2026-09-07, when the live PLC was read read-only
# and reported DF21 Temp Output LB = 10.0 -- not the 0.0 inferred from the prior
# system's log saturation. The inference was wrong, so the provenance changed.
for df in range(20, 24):
    spec = registers.REGISTERS[df - 1]
    ok(spec.provenance == registers.Provenance.MEASURED,
       "DF%d (%s) is MEASURED, read off the live PLC on %s"
       % (df, spec.nickname, registers.MEASURED_DATE), str(spec.provenance))
    ok(spec.df in registers.MEASURED_OUTPUT_CLAMPS,
       "  and its measured value is recorded",
       repr(registers.MEASURED_OUTPUT_CLAMPS.get(spec.df)))
ok(registers.MEASURED_OUTPUT_CLAMPS[21] == 10.0,
   "DF21 Temp Output LB is 10.0, superseding the 0.0 inferred from log saturation",
   repr(registers.MEASURED_OUTPUT_CLAMPS[21]))
ok(not any(s.provenance == registers.Provenance.INFERRED for s in registers.REGISTERS),
   "and no register claims INFERRED any more -- the four that did were measured")
for spec in registers.REGISTERS:
    if spec.df >= 20 or spec.field is None:
        continue
    ok(spec.provenance == registers.Provenance.THIRD_PARTY,
       "DF%d (%s) is THIRD_PARTY" % (spec.df, spec.nickname), str(spec.provenance))
ok(registers.COIL_SPEC.provenance == registers.Provenance.THIRD_PARTY,
   "the coil is THIRD_PARTY")

table = registers.provenance_table()
ok(isinstance(table, str) and table.strip() != "", "provenance_table() is non-empty")
ok("MEASURED" in table,
   "and names MEASURED so a measurement is distinguishable from an inference")
ok("INFERRED" in table,
   "and still names INFERRED, so the reader knows the distinction is being drawn")
ok("verified_live is the separate IDENTITY flag" in table,
   "and states that MEASURED is about a VALUE, not about a channel's identity")
ok("THIRD_PARTY" in table, "and names THIRD_PARTY")
ok("UNKNOWN" in table, "and names UNKNOWN")
ok("RH" in table and "Temp" in table, "and carries the PLC's own nicknames")
ok(all(str(s.df) in table for s in registers.REGISTERS), "every DF appears in it")
ok("verified_live" in table or "verified" in table.lower(),
   "and states whether each row was verified live")

print("\n--- the PLC's own scaling defines the valid ranges, and only for DF1/DF2 ---")
ok(rh is not None and (rh.valid_min, rh.valid_max) == (0.0, 100.0),
   "RH valid range is 0..100 %", repr((rh.valid_min, rh.valid_max)) if rh else "-")
ok(temp is not None and (temp.valid_min, temp.valid_max) == (-40.0, 180.0),
   "Temp valid range is -40..180 C", repr((temp.valid_min, temp.valid_max)) if temp else "-")
ranged = [s.df for s in registers.REGISTERS if s.valid_min is not None]
ok(ranged == [1, 2], "only DF1 and DF2 declare a range -- the rest are not invented",
   repr(ranged))

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
