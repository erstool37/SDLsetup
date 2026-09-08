"""The circulator's setpoint register block, and the one encoding known to work.

WHAT IS ESTABLISHED, AND HOW LITTLE THAT IS
===========================================

This device has **no vendor, no model, no manual, no firmware source, no
schematic and no register map**. The string ``RW-3`` in the code this replaces
was a MATLAB cell-divider comment -- a section name the author wrote beside
``%% PLC`` and ``%% video`` -- and **not** a model number, so no commercial
documentation applies to it.

Exactly **one transaction** is established, by observation of the prior working
system: a write of **4 holding registers at 0-based address 980**, carrying an
**IEEE-754 float64** temperature setpoint in degrees Celsius.

Everything else about the device is ``MEANING UNKNOWN``: there are no known
mixing, flow or RPM registers, no known status or alarm word, and **no
read-back of any kind** -- the prior code's only read was commented out. Do not
add one on the assumption that a controller must have one.

Provenance of address 980 is the prior working code and nothing else. Its
*official* meaning is **unverified**; what is verified is that writing this
block with this encoding drove the setpoint.

THE ENCODING, IN STANDARD TERMS
===============================

**float64, byteorder = LITTLE, wordorder = LITTLE.** The 8-byte IEEE-754
double is laid into 4 registers with the bytes inside each register in
little-endian order *and* the register sequence reversed (least-significant
word first).

Worked example, ``25.0``::

    IEEE-754 float64, MSB first   40 39 00 00 00 00 00 00
    split into 4 16-bit groups    4039  0000  0000  0000
    bytes swapped in each group   3940  0000  0000  0000
    register order reversed       0000  0000  0000  3940
                                  ^980  ^981  ^982  ^983

so registers 980..983 receive ``[0x0000, 0x0000, 0x0000, 0x3940]``.

:func:`encode_setpoint` is bit-identical to the prior system's own expression
(MATLAB ``flip(typecast(flip(typecast(x,'uint8')),'uint16'))``) on every finite
value -- verified over the seven known answers plus a spread of magnitudes in
``dev/tests/test_circulator_codec.py``.

.. warning::

   **THIS IS NOT THE PLC'S ENCODING.** The PLC on this rig uses **float32,
   byteorder BIG, wordorder LITTLE**. Two devices, two different encodings, on
   the same bench, both carrying temperatures. Reusing one device's helper for
   the other produces a well-formed frame with a meaningless value in it, and
   nothing in either protocol will complain -- which is why they live in
   separate modules and why this paragraph is here.

WHAT THIS MODULE REFUSES
========================

Non-finite values. The prior code encoded whatever arrived: a failed upstream
read still produced a forwarded value, and ``NaN`` packs silently into
``[0x0000, 0x0000, 0x0000, 0xF87F]`` and was sent to a temperature controller
whose own clamping behaviour is unknown. :func:`encode_setpoint` raises instead.
Bounding the *magnitude* is a separate job and belongs to
:mod:`tools.circulator.safety`; this module only guarantees the number is a
number.
"""
from __future__ import annotations

import math
import struct
from collections.abc import Sequence

#: 0-based Modbus holding-register address of the setpoint block.
#:
#: Provenance: the prior working code only. Official meaning UNVERIFIED -- no
#: register map for this board exists. Do not derive neighbouring addresses
#: from it; adjacency in a register space is not evidence of anything.
SETPOINT_ADDRESS = 980

#: Registers in the block. 4 x 16 bits = the 64 bits of one float64.
SETPOINT_COUNT = 4

#: Widest value a single Modbus holding register can carry.
REGISTER_MAX = 0xFFFF


def encode_setpoint(value: float) -> list[int]:
    """Encode a Celsius setpoint into the 4 registers written at :data:`SETPOINT_ADDRESS`.

    See the module docstring for the encoding and the worked ``25.0`` example.

    Raises :class:`ValueError` on anything that is not a finite real number --
    including ``NaN`` and the infinities, which the prior code encoded and sent.
    This is a *representability* check, not a safety bound: use
    :meth:`tools.circulator.safety.CommandLimits.validate` for the range.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f"setpoint must be a real number, got {type(value).__name__} {value!r}; "
            f"refusing to coerce -- a coerced setpoint is an unchecked setpoint"
        )
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(
            f"setpoint {number!r} is not finite. The prior code encoded this without "
            f"complaint (NaN -> [0x0000, 0x0000, 0x0000, 0xF87F]) and sent it to a "
            f"temperature controller with no documented clamp of its own."
        )
    # pack(">d") gives the canonical MSB-first bytes; unpack("<4H") reads each
    # 2-byte group back byte-swapped (byteorder LITTLE); reversing the tuple
    # puts the least-significant word first (wordorder LITTLE).
    words = struct.unpack("<4H", struct.pack(">d", number))
    return list(reversed(words))


def decode_setpoint(regs: Sequence[int]) -> float:
    """Inverse of :func:`encode_setpoint`. **For tests and diagnostics only.**

    The device has no read-back, so no register block ever arrives *from* it.
    This exists so a known-answer test can check the encoding in both
    directions, and so a captured frame can be read by a human.
    """
    values = list(regs)
    if len(values) != SETPOINT_COUNT:
        raise ValueError(
            f"expected {SETPOINT_COUNT} registers, got {len(values)}"
        )
    out: list[int] = []
    for index, raw in enumerate(values):
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ValueError(f"register {index}: not an integer: {raw!r}")
        if not 0 <= raw <= REGISTER_MAX:
            raise ValueError(
                f"register {index}: {raw!r} is outside 0..0x{REGISTER_MAX:04X}"
            )
        out.append(raw)
    packed = struct.pack("<4H", *reversed(out))
    return float(struct.unpack(">d", packed)[0])


def describe_frame(value: float) -> str:
    """One operator-readable line: the value, the address, and the registers."""
    words = encode_setpoint(value)
    return "%.6g C -> %d..%d = [%s]" % (
        value, SETPOINT_ADDRESS, SETPOINT_ADDRESS + SETPOINT_COUNT - 1,
        ", ".join("0x%04X" % word for word in words),
    )


__all__ = [
    "REGISTER_MAX",
    "SETPOINT_ADDRESS",
    "SETPOINT_COUNT",
    "decode_setpoint",
    "describe_frame",
    "encode_setpoint",
]
