"""Setpoint guards for the CLICK PLC. Nothing here performs I/O.

Why this module exists as a *type* and not a convention
=======================================================

The code this layer replaces wrote a setpoint straight to the PLC with no
bounds check of any kind, because the bounds lived in a MATLAB GUI that was
thrown away when the control loop was ported to Python. So the guard here is
enforced by construction, exactly as :mod:`tools.arm.safety` enforces the
motion envelope:

* :class:`BoundedSetpoint` cannot be built except by
  :meth:`SetpointLimits.validate` -- its constructor demands a module-private
  token.
* :meth:`tools.environment.plc.PlcClient.write_float32` refuses any argument
  that is not a :class:`BoundedSetpoint`.

A caller who wants an unchecked setpoint has to edit this file, which is
exactly the visibility the previous arrangement lacked.

Where the numbers come from
===========================

``0 < Temp SP < 30`` degrees C and ``0 < RH SP < 100`` percent are the limits
the **MATLAB GUI enforced** on this rig, with *strict* inequalities -- a
setpoint of exactly 0 or exactly 30 was refused there, and is refused here.
The Python port that replaced that GUI dropped the check entirely; restoring it
is the point of this module. They are code ceilings: ``config.yaml`` may
tighten them for a run and may never loosen them, because they describe the
chamber and its heater, not a preference.

**The config expresses these as a min/max pair, and this module reads that pair
as an OPEN interval.** ``configs/config.yaml`` carries ``temp_sp_min_c: 0.0`` /
``temp_sp_max_c: 30.0`` and ``rh_sp_min_pct: 0.0`` / ``rh_sp_max_pct: 100.0``,
and a min/max pair *reads* as inclusive -- but the MATLAB GUI it cites enforced
``0 < SP < 30`` strictly. :meth:`SetpointLimits.validate` therefore compares
``low < value < high``, never ``<=``, and 0.0 and 30.0 are both refused. Do not
"fix" this to inclusive: doing so quietly restores a setpoint the original
system rejected, and the config file cannot state the distinction on its own.

Guards raise, never clamp. A clamp turns an out-of-range request into a
plausible in-range one, which is how a wrong number becomes an accident -- and
a setpoint silently moved from 45 C to 30 C is a run whose record says one
thing and whose chamber did another.

Nothing here decides anything. It reports that a value is admissible, or
refuses; what to do about a refusal belongs to the script that asked.
"""
from __future__ import annotations

import dataclasses
import math

from . import registers

#: Code ceilings, from the MATLAB GUI this layer replaces. Strict intervals.
TEMP_MIN_C = 0.0
TEMP_MAX_C = 30.0
RH_MIN_PCT = 0.0
RH_MAX_PCT = 100.0

#: Per-write step and interval caps. Operator-facing defaults, not hardware
#: limits: the chamber's own thermal inertia is unmeasured, so these are a
#: conservative starting point.
#: TODO(operator): measure the chamber's step response and set these from it.
MAX_STEP_C = 5.0
MAX_STEP_PCT = 10.0
MIN_INTERVAL_S = 10.0

#: Which pair of bounds a writable field is held to. Keyed explicitly rather
#: than sniffed from ``spec.units`` so a new writable register raises here
#: instead of silently inheriting the wrong ceiling.
_BOUND_KIND: dict[str, str] = {
    "temp_sp_c": "temp",
    "rh_sp_pct": "rh",
}


class PlcError(Exception):
    """Base of every error this layer raises.

    Named ``PlcError`` rather than ``EnvironmentError`` on purpose:
    ``EnvironmentError`` is a builtin alias of ``OSError``, and shadowing it
    would make ``except EnvironmentError`` in unrelated code catch our errors
    or miss real OS ones.
    """


class CommsLost(PlcError):
    """The PLC could not be reached, or answered with a protocol error.

    A transport fault, not a safety violation -- which is why
    :meth:`PlcClient.read_block` reports it in the reading's ``read_ok`` flag
    instead of raising it. It is raised only where continuing would mean
    *asserting* something about the PLC that was never read.
    """


class FailSafeIncomplete(CommsLost):
    """The fail-safe ran but could NOT be confirmed (F5).

    Raised by the relay's watchdogs when the safe-setpoint write and/or the PID
    disable did not both come back ``outcome="confirmed"`` -- a circulator gate
    off (``planned``), a dead serial link (``failed``), or a dead PLC (failed C1
    write). It is a subclass of :class:`CommsLost`, so existing ``except
    CommsLost`` handlers still catch it, but its distinct type lets a caller tell
    "the fail-safe completed" from "the fail-safe was only ATTEMPTED".

    It carries the two results it could not confirm: ``record`` (the safe-mode
    :class:`~tools.environment.relay.RelayRecord`), ``write`` (the safe-setpoint
    write dict), and ``pid`` (the PID-disable dict).

    **True dead-link safety is not achieved by this exception.** The circulator
    has no read-back, so even a raised message can only report what was
    attempted; closing the gap needs an independent PLC/MCU watchdog (a ladder
    heartbeat timeout on the CLICK's ``SD41``), which does not exist yet.
    """


class SafetyError(PlcError):
    """A commanded value violates a guard. Always raised, never clamped."""


class RateLimited(SafetyError):
    """A write is too large a step, or too soon after the previous one."""


class ActuationNotAllowed(SafetyError):
    """A write was attempted without ``allow_actuation``."""


_VALIDATION_TOKEN = object()


class BoundedSetpoint:
    """A setpoint that has passed :meth:`SetpointLimits.validate`.

    :meth:`PlcClient.write_float32` takes only these. Constructing one directly
    raises: the whole point is that the type itself is the evidence a guard
    ran, so a plain ``float`` can never reach the wire.

    **What this immutability does and does not stop (F2a residual).** The
    token and the frozen attributes stop *accidental* misuse and ordinary
    assignment -- a caller cannot rebind the value after validation. They do
    NOT stop a determined caller using ``object.__setattr__`` or a subclass
    with a stateful ``__getattribute__``; that is true of every Python
    "immutable", and real enforcement would need a different language or a
    process boundary. The write sinks snapshot the value once (H4) so a
    stateful subclass cannot change it between validation and encoding, but
    this object itself is not a security boundary.
    """

    __slots__ = ("field", "value", "spec", "limits")

    def __init__(self, token: object, field: str, value: float,
                 spec: registers.RegisterSpec, limits: SetpointLimits) -> None:
        if token is not _VALIDATION_TOKEN:
            raise SafetyError(
                "BoundedSetpoint() may only be created by "
                "SetpointLimits.validate(); constructing one directly would "
                "bypass every setpoint bound")
        object.__setattr__(self, "field", str(field))
        object.__setattr__(self, "value", float(value))
        object.__setattr__(self, "spec", spec)
        object.__setattr__(self, "limits", limits)

    def __setattr__(self, name: str, value: object) -> None:
        raise SafetyError(
            "BoundedSetpoint is immutable: reassigning value/field/spec after "
            "validation would bypass the bound (or redirect the write to a "
            "read-only register). Call SetpointLimits.validate() again.")

    def __delattr__(self, name: str) -> None:
        raise SafetyError("BoundedSetpoint is immutable; attributes cannot be deleted")

    @property
    def address(self) -> int:
        """Protocol address of the setpoint's low-order (first) register."""
        return self.spec.address

    @property
    def units(self) -> str:
        return self.spec.units

    def __repr__(self) -> str:
        return "BoundedSetpoint(%s=%g %s -> DF%d @ %d)" % (
            self.field, self.value, self.spec.units or "-", self.spec.df,
            self.spec.address)


@dataclasses.dataclass(frozen=True)
class SetpointLimits:
    """The admissible setpoint window. Construction refuses to widen the code
    ceilings above, so a tightened copy is the only copy that can exist."""

    temp_min_c: float = TEMP_MIN_C
    temp_max_c: float = TEMP_MAX_C
    rh_min_pct: float = RH_MIN_PCT
    rh_max_pct: float = RH_MAX_PCT

    def __post_init__(self) -> None:
        for name, ceiling, direction in (
            ("temp_min_c", TEMP_MIN_C, "below"),
            ("rh_min_pct", RH_MIN_PCT, "below"),
            ("temp_max_c", TEMP_MAX_C, "above"),
            ("rh_max_pct", RH_MAX_PCT, "above"),
        ):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise SafetyError("%s must be a finite number, got %r" % (name, value))
            value = float(value)
            if direction == "below" and value < ceiling:
                raise SafetyError(
                    "%s=%g is BELOW the code floor %g. config may tighten a "
                    "setpoint bound, never loosen it -- these are the limits "
                    "the MATLAB GUI enforced on this chamber." % (name, value, ceiling))
            if direction == "above" and value > ceiling:
                raise SafetyError(
                    "%s=%g is ABOVE the code ceiling %g. config may tighten a "
                    "setpoint bound, never loosen it -- these are the limits "
                    "the MATLAB GUI enforced on this chamber." % (name, value, ceiling))
        if self.temp_min_c >= self.temp_max_c:
            raise SafetyError("temp_min_c=%g is not below temp_max_c=%g"
                              % (self.temp_min_c, self.temp_max_c))
        if self.rh_min_pct >= self.rh_max_pct:
            raise SafetyError("rh_min_pct=%g is not below rh_max_pct=%g"
                              % (self.rh_min_pct, self.rh_max_pct))

    def bounds_for(self, field: str) -> tuple[float, float]:
        """The ``(low, high)`` open interval this field is held to."""
        kind = _BOUND_KIND.get(field)
        if kind == "temp":
            return self.temp_min_c, self.temp_max_c
        if kind == "rh":
            return self.rh_min_pct, self.rh_max_pct
        raise SafetyError(
            "no setpoint bounds are defined for %r. It is writable per "
            "tools.environment.registers.WRITABLE but this module does not "
            "know what it means, and refusing is the only honest option. "
            "TODO(operator): state its admissible range, then add it to "
            "_BOUND_KIND." % (field,))

    def validate(self, field: str, value: float) -> BoundedSetpoint:
        """The **only** way to produce a :class:`BoundedSetpoint`.

        Raises :class:`SafetyError` for a field that is not writable, a
        non-numeric or non-finite value, or a value outside the **strict** open
        interval. The strictness is inherited from the MATLAB GUI: ``0 < SP <
        30`` there, so 0.0 and 30.0 were both refused, and both are refused
        here.
        """
        spec = registers.WRITABLE.get(field)
        if spec is None:
            known = registers.BY_FIELD.get(field)
            if known is not None:
                raise SafetyError(
                    "%r is read-only by policy (DF%d, rw=%s). The PID on this "
                    "rig is hand-written in ladder logic, so its gains cannot "
                    "be read back; the only writable registers are %s."
                    % (field, known.df, known.rw, sorted(registers.WRITABLE)))
            raise SafetyError(
                "%r is not a register on this PLC. Writable fields: %s"
                % (field, sorted(registers.WRITABLE)))

        # Refused by TYPE, not coerced. float("25") succeeds and float(True)
        # is 1.0 -- both plausible-looking setpoints from a caller that never
        # parsed its input, and this file's config parser turns a bare `N` into
        # False. A setpoint arrives as a number or it does not arrive.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SafetyError(
                "%s: setpoint must be an int or float, got %s %r. It is not "
                "coerced: a string or a boolean here means the caller did not "
                "parse its input." % (field, type(value).__name__, value))
        number = float(value)
        if not math.isfinite(number):
            raise SafetyError("%s: setpoint is not finite (%r)" % (field, number))

        low, high = self.bounds_for(field)
        # STRICT, both ends. Not a rounding choice: the MATLAB GUI refused the
        # endpoints, and 0 %RH / 0 C are the states the chamber cannot hold.
        if not (low < number < high):
            raise SafetyError(
                "%s=%g%s is outside the admissible open interval (%g, %g). "
                "Refused, never clamped: a setpoint quietly moved to the "
                "nearest legal value makes the run record disagree with the "
                "chamber." % (field, number, self.spec_units(spec), low, high))
        return BoundedSetpoint(_VALIDATION_TOKEN, field, number, spec, self)

    @staticmethod
    def spec_units(spec: registers.RegisterSpec) -> str:
        return " %s" % spec.units if spec.units else ""

    def describe(self) -> str:
        return ("setpoint limits: %g < temp < %g C, %g < RH < %g %% "
                "(strict; code ceilings %g/%g and %g/%g)"
                % (self.temp_min_c, self.temp_max_c, self.rh_min_pct,
                   self.rh_max_pct, TEMP_MIN_C, TEMP_MAX_C, RH_MIN_PCT, RH_MAX_PCT))


@dataclasses.dataclass(frozen=True)
class RateLimiter:
    """How far and how often a setpoint may move.

    The clock is **injected**: :meth:`check` takes ``now`` as a required
    keyword and never reads wall time itself, so a test asserts the interval
    rule deterministically instead of sleeping through it -- and a caller
    replaying a recorded run gets the same verdicts it got live.

    It decides nothing. It reports that a step is admissible, or refuses. It
    does not wait, retry, or shrink the step to fit.
    """

    max_step_c: float = MAX_STEP_C
    max_step_pct: float = MAX_STEP_PCT
    min_interval_s: float = MIN_INTERVAL_S

    def __post_init__(self) -> None:
        for name in ("max_step_c", "max_step_pct", "min_interval_s"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise SafetyError("%s must be a finite number, got %r" % (name, value))
            if float(value) < 0.0:
                raise SafetyError("%s must not be negative, got %r" % (name, value))

    def max_step_for(self, field: str) -> float:
        kind = _BOUND_KIND.get(field)
        if kind == "temp":
            return self.max_step_c
        if kind == "rh":
            return self.max_step_pct
        raise SafetyError(
            "no step cap is defined for %r; TODO(operator): state one before "
            "writing it." % (field,))

    def check(self, field: str, value: float, *, now: float,
              last_value: float | None, last_time: float | None,
              allow_large_step: bool = False) -> None:
        """Raise :class:`RateLimited` if this write is too big or too soon.

        ``last_value``/``last_time`` are ``None`` when there is no previous
        write to compare against -- the first write of a session is bounded by
        :class:`SetpointLimits` alone, which is the whole of what is known.

        ``allow_large_step`` is the operator's explicit override for a
        deliberate large move (a cold start, a chamber purge). It suppresses
        the step cap only; the interval rule still applies, because two writes
        in quick succession are a bug in the caller either way.
        """
        if not math.isfinite(float(now)):
            raise SafetyError("%s: `now` is not finite (%r)" % (field, now))
        if not math.isfinite(float(value)):
            raise SafetyError("%s: setpoint is not finite (%r)" % (field, value))

        if last_time is not None:
            if not math.isfinite(float(last_time)):
                raise SafetyError("%s: `last_time` is not finite (%r)" % (field, last_time))
            elapsed = float(now) - float(last_time)
            if elapsed < 0.0:
                # Refused rather than treated as a long wait: a clock that ran
                # backwards means the interval is unknown, and an unknown
                # interval is not a satisfied one.
                raise RateLimited(
                    "%s: the clock ran backwards (now=%r is before last_time=%r), "
                    "so the interval since the previous write is unknown"
                    % (field, now, last_time))
            if elapsed < self.min_interval_s:
                raise RateLimited(
                    "%s: %.3f s since the previous write, minimum is %g s"
                    % (field, elapsed, self.min_interval_s))

        if last_value is None or allow_large_step:
            return
        cap = self.max_step_for(field)
        step = abs(float(value) - float(last_value))
        if step > cap:
            raise RateLimited(
                "%s: a step of %g from %g to %g exceeds the cap %g. Refused, "
                "not shrunk -- pass allow_large_step=True to say the large "
                "move is deliberate."
                % (field, step, last_value, value, cap))

    def describe(self) -> str:
        return ("rate limits: <=%g C and <=%g %% per write, >=%g s apart"
                % (self.max_step_c, self.max_step_pct, self.min_interval_s))


__all__ = [
    "MAX_STEP_C",
    "MAX_STEP_PCT",
    "MIN_INTERVAL_S",
    "RH_MAX_PCT",
    "RH_MIN_PCT",
    "TEMP_MAX_C",
    "TEMP_MIN_C",
    "ActuationNotAllowed",
    "BoundedSetpoint",
    "CommsLost",
    "FailSafeIncomplete",
    "PlcError",
    "RateLimited",
    "RateLimiter",
    "SafetyError",
    "SetpointLimits",
]
