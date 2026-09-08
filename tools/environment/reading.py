"""One environment reading: what the PLC reported, and how good it was.

Pure decoding. :func:`decode_block` performs no I/O, opens no socket, and
imports no vendor SDK -- hand it the uint16 words a block read returned and it
hands back an :class:`EnvironmentReading`.

**It decides nothing.** No retry, no substitution, no clamping, no dropping of
a reading it dislikes. A NaN stays NaN with ``finite=False``; a value outside
the range the PLC's own scaling defines is reported unchanged with
``in_valid_range=False``. That flag is a measurement-quality *fact* -- "this
reading is outside the valid range and is therefore unreliable" -- and what to
do about it belongs to the script that asked for the reading.

Two things are preserved deliberately:

* ``raw_registers`` keeps **every** uint16 as received, so any channel can be
  re-decoded later from the record without going back to the PLC. If the word
  order or a register's meaning is ever corrected, old readings can be replayed
  through the new codec.
* ``unknown_channels`` holds DF3 and DF4 -- wired analogue inputs with no
  nickname in the PLC project and therefore no known meaning. They are kept in
  a separate dict, keyed ``"DF3"``/``"DF4"``, and are never given a physical
  name or merged into ``channels``. Guessing that one of them is a second
  thermocouple is exactly how a wrong label becomes permanent.

A short block is not an error. Some of the pairs arrived, so those are decoded,
``partial`` is set, and the channels whose registers did not arrive are simply
absent -- never filled in with a default.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from . import registers


@dataclass(frozen=True)
class ChannelQuality:
    """How much this channel can be trusted, as facts and not as a verdict.

    ``in_valid_range`` is ``None`` when the register declares no range -- most
    of them do not, and ``False`` would be a claim we cannot make. Only DF1
    (0..100 %RH) and DF2 (-40..180 C) carry ranges, because only those two are
    defined by the PLC's ``[CPUBuild]`` scaling.
    """

    in_valid_range: bool | None
    finite: bool


@dataclass(frozen=True)
class EnvironmentReading:
    """One block read, decoded. Immutable; carries its own provenance."""

    t_utc: str
    monotonic_s: float
    read_ok: bool
    partial: bool
    #: Every uint16 word exactly as received, so any value can be re-decoded.
    raw_registers: tuple[int, ...]
    #: Named channels only, keyed by our field name (never a PLC nickname).
    channels: dict[str, float]
    quality: dict[str, ChannelQuality]
    #: DF3/DF4 -- decoded but MEANING UNKNOWN. Never merged into ``channels``.
    unknown_channels: dict[str, dict[str, Any]]
    pid_enabled: bool | None = None

    # -- the channels a caller most often wants, by name rather than by key --

    @property
    def temp_filtered_c(self) -> float | None:
        return self.channels.get("temp_filtered_c")

    @property
    def rh_filtered_pct(self) -> float | None:
        return self.channels.get("rh_filtered_pct")

    @property
    def temp_pid_output_c(self) -> float | None:
        return self.channels.get("temp_pid_output_c")

    @property
    def temp_sp_c(self) -> float | None:
        return self.channels.get("temp_sp_c")

    @property
    def rh_sp_pct(self) -> float | None:
        return self.channels.get("rh_sp_pct")

    def as_dict(self) -> dict[str, Any]:
        """A JSON-serialisable record of this reading, dataclasses flattened.

        A NaN channel serialises as ``NaN`` under ``json.dumps`` defaults; it is
        left as it is rather than substituted, per this module's contract.
        """
        return {
            "t_utc": self.t_utc,
            "monotonic_s": self.monotonic_s,
            "read_ok": self.read_ok,
            "partial": self.partial,
            "raw_registers": list(self.raw_registers),
            "channels": dict(self.channels),
            "quality": {
                name: {"in_valid_range": q.in_valid_range, "finite": q.finite}
                for name, q in self.quality.items()
            },
            "unknown_channels": {
                key: dict(value) for key, value in self.unknown_channels.items()
            },
            "pid_enabled": self.pid_enabled,
        }


def decode_block(
    regs: Sequence[int],
    *,
    t_utc: str,
    monotonic_s: float,
    pid_enabled: bool | None = None,
) -> EnvironmentReading:
    """Decode a DF1..DF23 block read into an :class:`EnvironmentReading`.

    ``regs`` is the raw uint16 sequence, ``regs[0]`` being the low-order word of
    DF1. A sequence shorter than :data:`registers.BLOCK_COUNT` sets
    ``partial=True`` and decodes only the pairs that fully arrived -- it does
    **not** raise, and does not fabricate the missing channels. An odd trailing
    word is kept in ``raw_registers`` but its half-arrived pair is not decoded.

    ``t_utc`` and ``monotonic_s`` are supplied by the caller rather than read
    here, so this function stays pure and a recorded block can be re-decoded
    later with its original timestamps.
    """
    raw = tuple(int(word) for word in regs)
    complete_pairs = len(raw) // 2

    channels: dict[str, float] = {}
    quality: dict[str, ChannelQuality] = {}
    unknown: dict[str, dict[str, Any]] = {}

    for spec in registers.REGISTERS:
        index = 2 * (spec.df - 1)
        if index + 1 >= len(raw):
            continue                     # this pair did not arrive; say nothing
        value = registers.regs_to_f32(raw[index], raw[index + 1])

        if spec.field is None:
            unknown["DF%d" % spec.df] = {
                "value": value,
                "meaning": "UNKNOWN",
                "units": "UNKNOWN",
                "scale": registers.UNKNOWN_SCALE.get(spec.df, "UNKNOWN"),
            }
            continue

        finite = math.isfinite(value)
        if spec.valid_min is None or spec.valid_max is None:
            in_valid_range: bool | None = None
        else:
            in_valid_range = finite and spec.valid_min <= value <= spec.valid_max

        channels[spec.field] = value
        quality[spec.field] = ChannelQuality(
            in_valid_range=in_valid_range, finite=finite)

    return EnvironmentReading(
        t_utc=t_utc,
        monotonic_s=monotonic_s,
        # Pure decode: "ok" here means words arrived and decoded. A transport
        # failure is the I/O layer's to report, and it constructs its own
        # reading with read_ok=False.
        read_ok=complete_pairs > 0,
        partial=complete_pairs < len(registers.REGISTERS),
        raw_registers=raw,
        channels=channels,
        quality=quality,
        unknown_channels=unknown,
        pid_enabled=pid_enabled,
    )
