"""The circulator's command bound, enforced by a type rather than by discipline.

WHY EVERY BOUND LIVES HERE
==========================

The circulator is a **custom MCU board** with no vendor, no model, no manual,
no firmware source and no schematic. Whether it has an over-temperature cutoff
or a setpoint clamp of its own is not merely undocumented -- it is
**unknowable** without reverse-engineering the board. So the device cannot be
assumed to protect itself, and every bound that exists is the one in this
module.

The prior code had none. It forwarded whatever arrived, including the ``NaN``
produced by a *failed* upstream read, and never looked at what the write
returned.

THE BOUND: 10.0 < setpoint < 30.0 degrees Celsius in this rig, STRICT
=====================================================================

**Both endpoints are refused.** Exactly the floor and exactly the ceiling raise.

*The floor is MEASURED. The ceiling is still INFERRED. They no longer share a
provenance, and the difference matters.*

**Floor -- MEASURED, 2026-09-07.** The live PLC was read read-only (five
samples, ``read_holding_registers(28672, count=46)``, ``device_id=1``,
``C1 PID Auto`` OFF, nothing written) and reports
``DF21 Temp Output LB = 10.0``. The PLC clamps its own temperature-PID output to
``[10.0, 30.0]`` C, so **10 C is the lowest value the circulator can
legitimately receive**; anything below it is a fault upstream, not a setpoint.
``configs/config.yaml`` therefore sets ``circulator.command_min_c: 10.0``, which
TIGHTENS the ceiling below.

This **superseded an inference that was wrong.** The floor used to be ``0.0``,
taken from the low end of the prior system's log envelope -- but ``0.000`` in
those logs was the prior system's own behaviour, not the PLC's clamp. Do not
restore ``0.0`` from any log-derived argument: the measurement outranks it.
See ``tools/environment/registers.py`` ``MEASURED_OUTPUT_CLAMPS``.

**Ceiling -- still INFERRED.** ``30.0`` is the top of that same observed
envelope of the prior system's output across all 49,147 rows of its logs
(``T output`` in ``[0.000, 30.000]``). It happens to agree with the measured
``DF20 Temp Output UB = 30.0``, which is corroboration -- but the log envelope
on its own was never independent evidence, and it still is not. No datasheet
states either number, because there is no datasheet.

**The code ceiling was deliberately left at 0.0.** :data:`COMMAND_MIN_C` below
is the widest *range* this layer will permit, and ``config.yaml`` tightens it to
the measured 10.0 for this rig. It was not raised, for a stated reason: DF21 is
a fact about what the *PLC* emits, not about what *this board* tolerates, and
raising a code ceiling is a hardware-clearance decision about a board whose own
protections are unknown. Two consequences a reader must know about:

* ``RelayPolicy`` validates ``safe_setpoint_c`` against ``CommandLimits()`` --
  the code ceiling ``(0, 30)`` -- not against the tightened config range. A
  declared safe setpoint in ``(0, 10]`` is accepted at policy construction and
  then **refused during an abort**, which is the worst moment to find out.
  ``scripts/environment/hold_environment.py`` re-checks it against the resolved
  range before its loop starts; nothing else does.
* TODO(operator): decide whether :data:`COMMAND_MIN_C` should also become 10.0.
  That is a one-line change here plus its assertion in
  ``dev/tests/test_circulator_safety.py``, and it would close the gap above at
  the type level rather than per-script.

**The envelope's maximum is partly a saturation artifact.** The 2026-01-11 log
ends with the temperature output pinned at exactly ``30.000`` while the
measured temperature read 11.4 C -- saturated against the clamp during an
aborted run. So the envelope and the clamp are not independent evidence, and
``30.0`` is a value the *broken* system produced. The number is kept because a
bound cannot honestly be looser than what the hardware has already been
driven to; it is **not** evidence that 30.0 is safe.

*Why strict rather than inclusive.* Two reasons:

* **The original enforced an open interval.** The MATLAB source enforced
  ``Temp SP`` in ``(0, 30)``, refusing exactly 0.0 and 30.0.
* **Consistency across the two devices.** The PLC-side setpoint guard is strict
  for the same reason, so no caller has to remember which device behaves which
  way at its limits.

And the saturation finding above is the substantive argument: ``30.000`` is
precisely the value a pinned, failing controller emits, so it is the *worst*
candidate for a legal command. The measurement does not soften this -- it
extends it. ``10.000`` is now known to be the PLC's other clamp, and a value
sitting exactly on a clamp is what a saturated controller emits at that end too,
so the strictness applies at both ends for one reason rather than two.

.. note::

   ``tools/environment/registers.py`` carries ``valid_min``/``valid_max``
   fields that are checked **inclusively**. Those are the PLC's *sensor
   scaling* bounds (0..100 %RH, -40..180 C), where an exact 100.0 is a
   legitimate reading. A **command** bound is a different quantity and is
   exclusive. Do not import or mirror those fields as command limits --
   :class:`CommandLimits` is the single source of truth for what may be sent
   to the circulator.

:data:`COMMAND_MIN_C` / :data:`COMMAND_MAX_C` are the **code ceiling**: the
widest *range* this layer will ever permit. ``config.yaml`` may TIGHTEN it for
a run; it may never widen it. Same rule, and same reason, as the arm's
clearance limits in ``tools/arm/safety.py``: the numbers describe what the rig
was observed to do, not what an operator prefers today.

Note the asymmetry, which is deliberate: a range of exactly
``[0.0, 30.0]`` is a legal *range* (it equals the ceiling), while the *values*
0.0 and 30.0 are not commandable within it.

GUARDS RAISE, THEY NEVER CLAMP
==============================

Clamping turns an out-of-range request into a plausible in-range command, which
is how a wrong number becomes a silent wrong result. An out-of-range setpoint
is a bug upstream, and the caller has to see it.

THE TOKEN
=========

:class:`BoundedSetpoint` cannot be constructed except by
:meth:`CommandLimits.validate` -- its ``__init__`` demands a module-private
token. :meth:`tools.circulator.circulator.Circulator.write_setpoint` accepts
**only** that type, so a bare ``float`` cannot reach the wire and adding a path
that lets one through means editing this file or ``circulator.py``.

This is the same idiom as ``ValidatedMove`` in :mod:`tools.arm.safety`, adopted
for the same reason: consolidating the checks into a *function* would fix the
duplication and leave the bypass. The type itself is the evidence a guard ran.
"""
from __future__ import annotations

import dataclasses
import math

#: Floating-point slop, used ONLY when checking that a configured *range* sits
#: within the code ceiling -- so that a range of exactly [0.0, 30.0] is accepted
#: rather than rejected by representation error. It is deliberately NOT applied
#: to a commanded value: the value check is strict, and adding slop there would
#: quietly re-admit the endpoints the strictness exists to refuse.
EPS = 1e-6

#: Code ceiling: the widest RANGE this layer will ever permit. Commandable
#: values are strictly inside it -- see :meth:`CommandLimits.validate`.
#:
#: INFERRED from the prior system's observed output envelope over 49,147 logged
#: rows (T output in [0.000, 30.000]). NOT from any datasheet -- none exists,
#: and NOT a documented safe ceiling: the envelope's maximum is partly a
#: SATURATION ARTIFACT (2026-01-11, output pinned at exactly 30.000 while the
#: measurement read 11.4 C during an aborted run).
#:
#: **This is not the bound this rig runs to.** The live PLC's measured
#: temperature-PID clamp is [10.0, 30.0] C (DF21 = 10.0, read 2026-09-07), and
#: ``configs/config.yaml`` sets ``circulator.command_min_c: 10.0`` to tighten
#: the floor accordingly. The measured floor supersedes the 0.0 kept here as
#: the outer range -- see the module docstring for why the constant was left
#: alone, and for the RelayPolicy consequence of leaving it.
#:
#: Raising either constant is a hardware-clearance decision about a board whose
#: own protections are unknown; if the rig really changed, change the constant
#: and say why in a decision entry, never in a config file.
COMMAND_MIN_C = 0.0
COMMAND_MAX_C = 30.0


class CirculatorError(Exception):
    """Base for every refusal from this package. One except clause catches all."""


class SafetyError(CirculatorError):
    """A commanded value violates a bound, or a frame was attempted too early.

    Always raised, never clamped -- see the module docstring.
    """


class ActuationNotAllowed(CirculatorError):
    """An actuating operation was attempted while ``allow_actuation`` is off.

    On this device that includes **opening the serial port**: the FTDI FT232R
    bridge asserts DTR on open and DTR is capacitively coupled to /RESET, so the
    open itself hardware-resets the microcontroller. There is no read-only
    identify query to fall back on.
    """


_VALIDATION_TOKEN = object()


class BoundedSetpoint:
    """A Celsius setpoint that has passed :meth:`CommandLimits.validate`.

    Constructing one directly raises. The whole point is that possession of
    this object is proof the bound was applied, so the transport layer can
    accept it without re-deriving anything.
    """

    __slots__ = ("value_c", "limits")

    # value_c/limits carry defaults ONLY so that a wrong-arity direct call --
    # BoundedSetpoint(25.0, limits), the shape someone reaching for a plain
    # constructor would write -- still binds and hits the token check below,
    # and so raises SafetyError rather than a bare TypeError. Every arity ends
    # at the same refusal.
    def __init__(self, token: object, value_c: float = 0.0,
                 limits: CommandLimits | None = None) -> None:
        if token is not _VALIDATION_TOKEN:
            raise SafetyError(
                "BoundedSetpoint() may only be created by CommandLimits.validate(); "
                "constructing one directly would bypass the only temperature bound "
                "that exists for this device"
            )
        if limits is None:
            raise SafetyError("BoundedSetpoint requires the limits that approved it")
        self.value_c = float(value_c)
        self.limits = limits

    def __repr__(self) -> str:
        return (f"BoundedSetpoint({self.value_c:g} C, strictly within "
                f"({self.limits.min_c:g}, {self.limits.max_c:g}))")


@dataclasses.dataclass(frozen=True)
class CommandLimits:
    """The Celsius range one run may command strictly within.

    Defaults are the code ceiling. A run may pass a narrower pair; a wider one
    is refused at construction, so there is no moment at which a widened limit
    exists and could be used.
    """

    min_c: float = COMMAND_MIN_C
    max_c: float = COMMAND_MAX_C

    def __post_init__(self) -> None:
        for name in ("min_c", "max_c"):
            raw = getattr(self, name)
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise SafetyError(f"{name} must be a real number, got {raw!r}")
            if not math.isfinite(float(raw)):
                raise SafetyError(f"{name} must be finite, got {raw!r}")
        if self.max_c <= self.min_c:
            raise SafetyError(
                f"max_c ({self.max_c}) is not above min_c ({self.min_c}); the bound is "
                f"an OPEN interval, so an empty or inverted range would refuse every "
                f"value, which reads as a broken device"
            )
        if self.min_c < COMMAND_MIN_C - EPS or self.max_c > COMMAND_MAX_C + EPS:
            raise SafetyError(
                f"[{self.min_c}, {self.max_c}] would WIDEN the command bound past the "
                f"code ceiling [{COMMAND_MIN_C}, {COMMAND_MAX_C}]. A run may tighten "
                f"this range, never loosen it: the ceiling is INFERRED from the prior "
                f"system's observed output envelope -- whose maximum is partly a "
                f"saturation artifact -- and this board's own over-temperature "
                f"protection, if it has any, is unknown."
            )

    def describe(self) -> str:
        return ("commandable setpoint range (%g, %g) C EXCLUSIVE of both endpoints "
                "(code ceiling [%g, %g], INFERRED from 49,147 logged rows whose "
                "maximum is partly a saturation artifact)"
                % (self.min_c, self.max_c, COMMAND_MIN_C, COMMAND_MAX_C))

    def validate(self, value: float) -> BoundedSetpoint:
        """The **only** way to produce a :class:`BoundedSetpoint`. Raises; never clamps.

        The interval is **OPEN**: ``min_c < value < max_c``. Exactly ``min_c``
        and exactly ``max_c`` are refused -- see the module docstring for the
        MATLAB original, the cross-device consistency argument, and the
        saturation artifact that makes 30.000 the worst candidate for a legal
        command.
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SafetyError(
                f"setpoint must be a real number, got {type(value).__name__} "
                f"{value!r}; refusing to coerce"
            )
        number = float(value)
        if not math.isfinite(number):
            raise SafetyError(
                f"setpoint {number!r} is not finite. This is the prior system's defect: "
                f"a FAILED upstream read still forwarded a value, and NaN encodes "
                f"without error into a frame this device would have accepted."
            )
        # STRICT. No EPS: slop here would re-admit the endpoints on purpose
        # refused, and 30.000 in particular is the value a pinned, failing
        # controller emits.
        if number <= self.min_c or number >= self.max_c:
            raise SafetyError(
                f"setpoint {number:g} C is outside the OPEN interval "
                f"({self.min_c:g}, {self.max_c:g}) C -- both endpoints are refused. "
                f"Refusing, not clamping: a clamped setpoint is a wrong number that "
                f"looks right, and this device has no known clamp of its own."
            )
        return BoundedSetpoint(_VALIDATION_TOKEN, number, self)


__all__ = [
    "COMMAND_MAX_C",
    "COMMAND_MIN_C",
    "EPS",
    "ActuationNotAllowed",
    "BoundedSetpoint",
    "CirculatorError",
    "CommandLimits",
    "SafetyError",
]
