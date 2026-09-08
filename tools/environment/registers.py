"""The AutomationDirect CLICK PLC register map, its float32 codec, and its provenance.

The humidity/temperature controller is a CLICK PLC spoken to over Modbus TCP.
Every address here is a **0-based protocol address** -- what pymodbus takes
directly, with no off-by-one adjustment.

Address formula
---------------

::

    DF n  ->  28672 + 2 * (n - 1)          # a float32 spans TWO registers
    C 1   ->  16384

**Its provenance is THIRD_PARTY, not VENDOR.** AutomationDirect publishes no
static address table for the CLICK. The formula comes from the open-source
``numat/clickplc`` driver (``C: 16384 + start - 1``; ``DF: 28672 + 2*(start-1)``)
corroborated by an Ignition forum thread, and was cross-validated at six points
against the recovered working code for this rig. It is well supported. It is
not a vendor guarantee, and :func:`provenance_table` exists so an operator can
never mistake one for the other.

Word order -- the part that fails silently
------------------------------------------

Within each 16-bit register the bytes are big-endian. Across the register pair
the **low-order 16 bits occupy the FIRST (lower-addressed) register**. Worked
example, verified by live round-trip:

::

    25.0  ==  0x41C80000  ->  register[0] = 0x0000  (LSW, low address)
                              register[1] = 0x41C8  (MSW, high address)

Hand the two words over backwards and nothing raises -- ``regs_to_f32(0x41C8,
0x0000)`` returns 2.36e-41, and a subtler pair returns a number that looks like
a plausible reading. That is why ``dev/tests/test_environment_codec.py`` asserts
five live-captured known answers and asserts that the codec is *not* word-order
invariant, rather than reasoning about endianness.

DF3 and DF4 mean nothing to us
------------------------------

Both are wired analogue inputs the PLC scales 0-20 mA to 0..100, and **neither
has a nickname in the PLC project's own table**. What they measure is
MEANING UNKNOWN. They are deliberately given ``field=None`` so no physical name
can be attached to them by accident; a decoded reading carries their values in
a separate ``unknown_channels`` dict, never in ``channels``.

DF20-DF23: measurement replaced inference on 2026-09-07
-------------------------------------------------------

The four output clamps used to be ``INFERRED``. The inference was that the
temperature-PID output was clamped to ``[0.0, 30.0]`` C, read off the saturation
envelope of the prior system's logs. **It was wrong.** A read-only read of the
live PLC on 2026-09-07 reports :data:`MEASURED_OUTPUT_CLAMPS` --
``DF21 Temp Output LB = 10.0``, not ``0.0`` -- so the PLC clamps its own
temperature-PID output to ``[10.0, 30.0]`` C and the circulator can never
legitimately receive a value below 10 C. Their provenance is now
:attr:`Provenance.MEASURED`.

``MEASURED`` is about a register's **value**, not its identity. That read was
our own Modbus read through this very table, so it cannot confirm what a
register *is* -- only what it held. ``verified_live`` still answers the identity
question and is therefore set on **DF1 and DF2 only**: the alternative reading
of that block is 21.8 %RH and 70.8 C, and 70.8 C is impossible for this room, so
the PLC's nickname table is corroborated and the prior code's swap is refuted.
Everything else rests on the nickname table, unchanged.

The same read also observed ``DF3``/``DF4`` at exactly ``0.000``. Their scaling
is 0-20 mA -> ``0..100``, so ``0.000`` is **0 mA: nothing is wired to those
inputs**. That is a fact about the wiring, not about the meaning -- they stay
MEANING UNKNOWN. And raw channels jittered while the filtered ones did not,
which is the PLC's own first-order filter (``Alpha = 0.01``) genuinely running.

DF11-DF17 and DF20-DF23 are read-only BY POLICY
-----------------------------------------------

The PLC would accept writes to these -- the PID gains, the filter alpha, the
stored previous error, and the output bounds. Our layer must not write them.
The PID on this rig is **hand-written in ladder logic** (the CLICK's built-in
PID instruction block is entirely factory-default and unused), so these tune a
bespoke loop whose structure cannot be read back from the controller. Changing
a gain of a loop you cannot read is not tuning, it is guessing, and the only
writable registers here are therefore the two setpoints (:data:`WRITABLE`).

Nothing in this module performs I/O, and nothing in it decides anything.
"""
from __future__ import annotations

import enum
import math
import struct
from dataclasses import dataclass

# 0-based protocol addresses. THIRD_PARTY provenance -- see the module docstring.
DF_BASE = 28672
C1_COIL = 16384

#: Registers spanned by a single block read of DF1..DF23 (two per float32).
BLOCK_COUNT = 46

#: The PLC's own ``[CPUBuild]`` scaling lines for the two un-nicknamed inputs,
#: verbatim. Carried so a reading can report *what* it does not know the
#: meaning of, rather than dropping the channel.
UNKNOWN_SCALE = {
    3: "AD3=DF3,20.0,0.0,100.0,0.0,1,0.02442,0",
    4: "AD4=DF4,20.0,0.0,100.0,0.0,1,0.02442,0",
}


#: The date the live read that produced :data:`MEASURED_OUTPUT_CLAMPS` was taken.
MEASURED_DATE = "2026-09-07"

#: DF -> value, read from the LIVE PLC on :data:`MEASURED_DATE` (read-only,
#: five samples, ``read_holding_registers(28672, count=46)``, ``device_id=1``,
#: ``C1 PID Auto`` OFF, nothing written).
#:
#: **DF21 is 10.0, not 0.0.** The tree previously carried ``0.0``, inferred from
#: the saturation envelope of the prior system's logs, and that inference was
#: wrong: the PLC clamps its temperature-PID output to ``[10.0, 30.0]`` C.
#: ``tools/circulator`` tightens its command floor to match, in
#: ``configs/config.yaml``.
#:
#: Nothing here is a control limit this layer enforces -- these are the PLC's
#: own clamps on its own outputs, recorded so an operator can see what the
#: controller will actually emit.
MEASURED_OUTPUT_CLAMPS: dict[int, float] = {
    20: 30.0,     # Temp Output UB, degC
    21: 10.0,     # Temp Output LB, degC  <- the correction
    22: 100.0,    # RH Output UB, %
    23: 0.0,      # RH Output LB, %
}


class Provenance(enum.StrEnum):
    """Where a table entry came from. Never widen an entry without evidence."""

    #: From AutomationDirect's own published documentation. Nothing qualifies.
    VENDOR = "VENDOR"
    #: Third-party driver, forum, or the PLC project's own nickname table.
    THIRD_PARTY = "THIRD_PARTY"
    #: The register's VALUE was read off the live PLC. Says nothing about the
    #: register's identity: the read went through this table, so it cannot
    #: confirm the table. ``verified_live`` is the identity flag, and the two
    #: are deliberately separate -- see the module docstring.
    MEASURED = "MEASURED"
    #: Read off the ladder logic's structure; the nickname implies the role.
    INFERRED = "INFERRED"
    #: We do not know what this is.
    UNKNOWN = "UNKNOWN"


def df_address(n: int) -> int:
    """Protocol address of the first register of ``DF n``.

    Raises ``ValueError`` for a non-positive DF number rather than returning an
    address below the DF block.
    """
    if n < 1:
        raise ValueError("DF numbers start at 1, got %r" % (n,))
    return DF_BASE + 2 * (n - 1)


@dataclass(frozen=True)
class RegisterSpec:
    """One DF float32 register, with the PLC's own nickname and our provenance.

    ``nickname`` is the string in the PLC project's nickname table, kept beside
    our ``field`` name so a mislabelling is visible in the table itself rather
    than only in a comment. ``field`` is ``None`` when the register has no
    nickname and therefore no known meaning.

    ``verified_live`` answers **identity**: was this register confirmed to be
    what the nickname says, against the live machine? It is deliberately not the
    same question as :attr:`Provenance.MEASURED`, which answers whether the
    register's *value* was read live. A read taken through this table can
    establish the second and never the first.

    ``valid_min``/``valid_max`` are populated only where the PLC's own
    ``[CPUBuild]`` scaling defines a range. They are a measurement-quality
    bound, not a control limit: a value outside them is a sensor or wiring
    fault to *flag*, and nothing here decides what to do about one.
    """

    df: int
    field: str | None
    nickname: str
    units: str
    rw: str
    provenance: Provenance
    verified_live: bool = False
    valid_min: float | None = None
    valid_max: float | None = None

    @property
    def address(self) -> int:
        """Protocol address of this register's low-order (first) word."""
        return df_address(self.df)


@dataclass(frozen=True)
class CoilSpec:
    """A single Modbus coil. Separate from :class:`RegisterSpec` because a coil
    has no DF number and no float32 pair -- forcing it into ``df`` would invent
    an address formula that does not apply to it."""

    coil: int
    field: str
    nickname: str
    units: str
    rw: str
    provenance: Provenance
    verified_live: bool = False

    @property
    def address(self) -> int:
        return self.coil


_R = "R"
_RW = "RW"
_TP = Provenance.THIRD_PARTY
_MEAS = Provenance.MEASURED
_UNK = Provenance.UNKNOWN

# Verbatim from the PLC project's nickname table, in DF order. The PLC's
# nicknames are AUTHORITATIVE: DF1 is RH and DF2 is Temp. The prior code had
# these two swapped, so every temperature it reported was a humidity.
#
# `verified_live` means the register's IDENTITY was confirmed against the live
# machine, and it is True on DF1/DF2 ONLY. The 2026-09-07 read decoded that
# block to 70.8 %RH and 21.8 C; read the other way round it would be 21.8 %RH
# and 70.8 C, and 70.8 C is impossible for this room -- so the two are
# corroborated as NOT swapped, which is exactly the prior code's bug.
#
# Nothing else is set, and that is not an oversight. The read went through this
# table, so it can confirm a register's VALUE (see Provenance.MEASURED) but not
# what the register IS. TODO(operator): read each remaining DF against the
# PLC's own data view -- an independent source -- and set verified_live=True per
# row as each identity is confirmed.
REGISTERS: tuple[RegisterSpec, ...] = (
    RegisterSpec(1, "rh_raw_pct", "RH", "%", _R, _TP, verified_live=True,
                 valid_min=0.0, valid_max=100.0),
    RegisterSpec(2, "temp_raw_c", "Temp", "C", _R, _TP, verified_live=True,
                 valid_min=-40.0, valid_max=180.0),
    RegisterSpec(3, None, "(AD3, no nickname)", "UNKNOWN", _R, _UNK),
    RegisterSpec(4, None, "(AD4, no nickname)", "UNKNOWN", _R, _UNK),
    RegisterSpec(5, "temp_sp_c", "Temp SP", "C", _RW, _TP),
    RegisterSpec(6, "rh_sp_pct", "RH SP", "%", _RW, _TP),
    RegisterSpec(7, "temp_error_c", "Temp Error", "C", _R, _TP),
    RegisterSpec(8, "rh_error_pct", "RH Error", "%", _R, _TP),
    RegisterSpec(9, "temp_pid_output_c", "Tem PID Output", "C", _R, _TP),
    RegisterSpec(10, "rh_pid_output_pct", "RH PID Output", "%", _R, _TP),
    RegisterSpec(11, "temp_kp", "Temp Kp", "", _R, _TP),
    RegisterSpec(12, "temp_ti", "Temp Ti", "", _R, _TP),
    RegisterSpec(13, "rh_kp", "RH Kp", "", _R, _TP),
    RegisterSpec(14, "rh_ti", "RH Ti", "", _R, _TP),
    RegisterSpec(15, "temp_old_error_c", "Temp Old Error", "C", _R, _TP),
    RegisterSpec(16, "rh_old_error_pct", "RH Old Error", "%", _R, _TP),
    RegisterSpec(17, "alpha", "Alpha", "", _R, _TP),
    RegisterSpec(18, "temp_filtered_c", "Temp Filtered", "C", _R, _TP),
    RegisterSpec(19, "rh_filtered_pct", "RH Filtered", "%", _R, _TP),
    # MEASURED 2026-09-07, superseding the [0.0, 30.0] inferred from log
    # saturation. See MEASURED_OUTPUT_CLAMPS. verified_live stays False: the
    # values are measured, the channel identities are not.
    RegisterSpec(20, "temp_output_ub_c", "Temp Output UB", "C", _R, _MEAS),
    RegisterSpec(21, "temp_output_lb_c", "Temp Output LB", "C", _R, _MEAS),
    RegisterSpec(22, "rh_output_ub_pct", "RH Output UB", "%", _R, _MEAS),
    RegisterSpec(23, "rh_output_lb_pct", "RH Output LB", "%", _R, _MEAS),
)

#: The PID auto/manual flag, coil ``C1``.
COIL_SPEC = CoilSpec(C1_COIL, "pid_auto", "PID Auto", "", _RW, _TP)

#: Named registers only. DF3/DF4 are absent by construction.
BY_FIELD: dict[str, RegisterSpec] = {
    spec.field: spec for spec in REGISTERS if spec.field is not None
}

#: DF numbers whose meaning is unknown. Never given a physical name.
UNKNOWN_DF: tuple[int, ...] = tuple(
    spec.df for spec in REGISTERS if spec.field is None
)

# The only registers our layer may write. Derived from `rw` so the table is the
# single source of truth -- see the docstring on read-only-by-policy.
#
# WRITABLE says *which* registers may be written. It deliberately does NOT say
# within what limits, and `valid_min`/`valid_max` are not that limit either:
# those are the PLC's own SENSOR SCALING bounds, and they are checked
# INCLUSIVELY, because a scaled input reading of exactly 100.0 %RH or exactly
# 180.0 C is a legitimate reading at the top of its range.
#
# A setpoint COMMAND bound is a different thing and is enforced EXCLUSIVELY by
# the layer that performs the write: the MATLAB original this rig replaces
# enforced `Temp SP` as the OPEN interval (0, 30) -- exactly 0.0 and exactly
# 30.0 were refused. Do not read an inclusive measurement-range flag from
# `reading.ChannelQuality` as permission to command that value.
WRITABLE: dict[str, RegisterSpec] = {
    spec.field: spec
    for spec in REGISTERS
    if spec.field is not None and spec.rw == _RW
}


def regs_to_f32(lo: int, hi: int) -> float:
    """Decode a CLICK float32 from its two registers, LSW first.

    ``lo`` is the word at the LOWER address. Handing the two over backwards
    does not raise -- see the word-order section of the module docstring.
    """
    return struct.unpack(">f", struct.pack(">HH", hi, lo))[0]


def f32_to_regs(v: float) -> tuple[int, int]:
    """Encode a float32 as ``(lo, hi)`` -- LSW first, matching the register order.

    Raises ``ValueError`` on a non-finite value. NaN and infinity have valid
    float32 bit patterns, so encoding one would put a word pair on the wire that
    the PLC reads as an arbitrary number; refusing is the only honest option.
    """
    if not math.isfinite(v):
        raise ValueError("refusing to encode a non-finite value: %r" % (v,))
    try:
        hi, lo = struct.unpack(">HH", struct.pack(">f", v))
    except OverflowError as exc:
        # Out of float32 range. Raised, never clamped to the nearest finite
        # value -- a clamp would put a wrong number on the wire silently.
        raise ValueError("value out of float32 range: %r" % (v,)) from exc
    return lo, hi


_HEADER = ("DF", "addr", "field", "PLC nickname", "units", "rw",
           "provenance", "verified_live", "valid range")


def provenance_table() -> str:
    """A printable table of every register, its provenance, and whether it was
    verified against live hardware.

    This exists so an operator reading a number off this layer can never
    mistake an inference for a measurement. Print it in any run that records
    environment data.
    """
    rows: list[tuple[str, ...]] = [_HEADER]
    for spec in REGISTERS:
        if spec.valid_min is None or spec.valid_max is None:
            rng = "-"
        else:
            rng = "%g..%g" % (spec.valid_min, spec.valid_max)
        rows.append((
            str(spec.df),
            str(spec.address),
            spec.field or "(none)",
            spec.nickname,
            spec.units or "-",
            spec.rw,
            str(spec.provenance),
            "yes" if spec.verified_live else "no",
            rng,
        ))
    rows.append((
        "C1",
        str(COIL_SPEC.address),
        COIL_SPEC.field,
        COIL_SPEC.nickname,
        COIL_SPEC.units or "-",
        COIL_SPEC.rw,
        str(COIL_SPEC.provenance),
        "yes" if COIL_SPEC.verified_live else "no",
        "-",
    ))

    widths = [max(len(row[i]) for row in rows) for i in range(len(_HEADER))]
    lines = [
        "CLICK PLC register map -- address formula provenance: THIRD_PARTY,",
        "not VENDOR (AutomationDirect publishes no static table).",
        "DF n -> %d + 2*(n-1); C1 -> %d. float32 word order: LSW first." % (
            DF_BASE, C1_COIL),
        "rw=R on DF11-17 and DF20-23 is POLICY, not device capability: the PID",
        "is hand-written in ladder logic and its structure cannot be read back.",
        "MEASURED (DF20-23) means the VALUE was read off the live PLC on %s;"
        % MEASURED_DATE,
        "the clamps are %s. INFERRED appears nowhere any more."
        % ", ".join("DF%d=%g" % item for item in
                    sorted(MEASURED_OUTPUT_CLAMPS.items())),
        "verified_live is the separate IDENTITY flag, and is yes on DF1/DF2 only:",
        "the swapped reading would be 21.8 %RH and 70.8 C, impossible for a room.",
        "",
    ]
    for index, row in enumerate(rows):
        lines.append("  " + "  ".join(
            cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip())
        if index == 0:
            lines.append("  " + "  ".join("-" * w for w in widths))
    return "\n".join(lines) + "\n"
