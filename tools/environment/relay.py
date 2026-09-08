"""Forward the PLC's temperature-PID output to the circulator. One step per call.

What this is
============

The CLICK PLC runs a hand-written ladder PID for enclosure temperature and
computes its own output into DF9 (``temp_pid_output_c``). The circulator is that
loop's **actuator**: it receives DF9 as its bath setpoint. Humidity is actuated
by the PLC itself as a PWM duty cycle and never transits this host. So the host
is a **wire** between a brain and a muscle, and this module is that wire.

The prior system did this in a 1 s loop with no bound, no watchdog, and no
read-back check. A real log from 2026-01-11 ends with the forwarded output
pinned at exactly ``30.000`` while the measured temperature read 11.4 C -- the
loop had saturated and the host kept forwarding it. That log is why "hold the
last value" is not an option anywhere in this file.

The tension, stated rather than defined away
============================================

This repo's law is that ``tools/`` **reports** and ``scripts/`` **decides**. A
``tools/`` module that reads one instrument and commands another on its own
judgement is a decision inside a tool, and this module is one call touching two
instruments. It is accepted on three specific grounds, and it holds itself to
them:

* It is a **transport**, not a controller. The PLC's value goes out unchanged or
  nothing goes out. There is no smoothing, no slew limit, no clamp, no
  substitute value, and no retry. Shaping the forwarded number would silently
  alter the control loop the PLC is running, and the PLC would have no way to
  know.
* It is a **guard**. Every forward passes the circulator's own strict bound,
  which raises rather than clamping.
* Its one contingency is **pre-declared**. What to do on comms loss is a
  decision, and it is made by the operator *before the run* as
  :class:`RelayPolicy` and passed in. This module executes a declared
  fail-safe; it never selects among options at runtime.

**The loop belongs to a phase script.** :meth:`Relay.step` runs exactly one
iteration and returns data. Cadence, how many iterations, and what to do with a
:class:`~.safety.CommsLost` are the script's.

The reading is not trustworthy just because it arrived
======================================================

``EnvironmentReading.read_ok`` is **not** a sufficient gate.
:func:`tools.environment.reading.decode_block` sets ``read_ok=True`` for a
*short* block -- some register pairs arrived, so those decoded -- which means a
reading that is **missing the very channel this module forwards** still reports
``read_ok=True``. The unit that found this asserted it rather than changing the
contract.

So :meth:`Relay.step` checks, in order: ``read_ok``, then ``partial``, then the
actual presence of ``temp_pid_output_c`` in ``channels``, then that its value is
finite. Reducing that to ``if reading.read_ok`` forwards a value that is not
there. ``dev/tests/test_environment_relay.py`` fails if anyone does.

The honest residual
===================

**If the serial link is dead, the safe setpoint cannot be delivered.** This
module writes it and reports the outcome; it cannot make the bath receive it,
and the MCU keeps whatever value it last got. There is no read-back on that
device at all, so "we wrote it" is the most that can ever be said. Only a person
at the rig, or a PLC-side ladder change, resolves that state.

That sentence is :data:`Relay.HONEST_RESIDUAL`, so a phase script can pass it
straight to ``run.note()``. Silence there would read as "handled".
"""
from __future__ import annotations

import dataclasses
import datetime
import math
import time
from typing import Any

from .. import config as _config
from ..circulator import api as _circ
from . import registers
from .reading import EnvironmentReading
from .safety import CommsLost

#: The one channel this module forwards. DF9, the PLC's temperature-PID output.
SOURCE_CHANNEL = "temp_pid_output_c"

#: Section of ``configs/config.yaml`` that declares the comms-loss setpoint.
#: It is the CIRCULATOR's section, because the setpoint is a bath command.
POLICY_SECTION = "circulator"

#: Largest forwarded jump that is not flagged. CHOSEN, UNMEASURED -- it mirrors
#: ``environment.max_setpoint_step_c`` in magnitude but is a different thing: a
#: quality FLAG on a value that is still forwarded, not a cap that refuses one.
#: TODO(operator): measure the bath's step response and set this from it.
DEFAULT_LARGE_STEP_C = 5.0

#: Watchdog defaults, matching ``environment.plc_stale_s`` and
#: ``circulator.circ_stale_s`` in the shipped config.
DEFAULT_PLC_STALE_S = 5.0
DEFAULT_CIRC_STALE_S = 5.0

_MISSING = object()


def _scalar(section: dict[str, Any], key: str) -> Any:
    """One config value, with this repo's empty-key parse handled.

    ``tools.config``'s parser turns a bare ``key:`` with nothing after it into
    an **empty mapping**, because it cannot tell "null" from "a nested block
    follows". ``{}`` is falsy, so ``if value:`` reads a deliberately-empty key
    as absent -- which is exactly the absent/unusable collapse this layer must
    not perform. So an empty mapping is returned as :data:`_MISSING`, and the
    caller decides whether absent is admissible. A NON-empty mapping where a
    scalar belongs is a malformed file and raises.
    """
    if key not in section:
        return _MISSING
    value = section[key]
    if isinstance(value, dict):
        if value:
            raise _config.ConfigError(
                "configs/config.yaml: %s.%s is a nested block, but a single "
                "value is expected here (got keys %s); refusing to guess which "
                "one is meant" % (POLICY_SECTION, key, sorted(value)))
        return _MISSING
    return value


def _unset_safe_setpoint(detail: str) -> _config.ConfigError:
    """The one error message an operator has to be able to act on unaided."""
    return _config.ConfigError(
        "circulator.safe_setpoint_c is not set (%s).\n"
        "\n"
        "This is the bath setpoint the relay commands when the PLC goes quiet, "
        "and there is NO code default on purpose: a safe setpoint nobody "
        "declared is not safe. The relay refuses to start rather than pick one.\n"
        "\n"
        "TODO(operator): edit configs/config.yaml, section `circulator`, key "
        "`safe_setpoint_c`, and write the comms-loss bath setpoint in degC on "
        "that line -- e.g. `safe_setpoint_c: 20.0`. It must lie strictly inside "
        "the circulator command bound (%g, %g) C; both endpoints are refused, "
        "and 30.000 in particular is the value a pinned, failing controller "
        "emits.\n"
        "\n"
        "A blank `safe_setpoint_c:` line does NOT mean null in this file's "
        "parser -- it parses to an empty mapping, which is why it reads as "
        "unset here." % (detail, _circ.COMMAND_MIN_C, _circ.COMMAND_MAX_C))


@dataclasses.dataclass(frozen=True)
class RelayPolicy:
    """The contingency, declared by the operator BEFORE the run.

    Every field is a decision, which is why they live here and not inside
    :class:`Relay`: the tool executes a policy it was handed, so it never
    chooses one at runtime.

    ``safe_setpoint_c`` is **required and has no default.** It is validated
    against the circulator's own command bound at construction, not at write
    time -- discovering that the declared safe setpoint is unwritable during an
    abort is the worst possible moment to find out.
    """

    safe_setpoint_c: float
    plc_stale_s: float = DEFAULT_PLC_STALE_S
    circ_stale_s: float = DEFAULT_CIRC_STALE_S
    large_step_c: float = DEFAULT_LARGE_STEP_C
    #: The circulator's **resolved** command bound, i.e. after config has
    #: tightened it. ``None`` falls back to the code ceiling, which is only
    #: correct when no config narrowed the range. ``from_config`` always
    #: populates this; a hand-built policy should too.
    command_limits: Any = None
    sources: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        raw = self.safe_setpoint_c
        if isinstance(raw, dict):
            raise _unset_safe_setpoint(
                "it parsed to an empty mapping, i.e. the config line is blank"
                if not raw else "it parsed to a nested block %s" % sorted(raw))
        if raw is None:
            raise _unset_safe_setpoint("it is None")
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise _unset_safe_setpoint(
                "it is a %s (%r), not a number" % (type(raw).__name__, raw))
        if not math.isfinite(float(raw)):
            raise _unset_safe_setpoint("it is not finite (%r)" % (raw,))
        # The circulator's bound is the authority on what this device may be
        # commanded to. Raising here, at construction, means a run cannot start
        # holding a safe setpoint it could never write.
        # Validate against the bound this run will ACTUALLY write within. Using
        # the default CommandLimits() here was a real defect: config tightens the
        # floor to the PLC's measured DF21 Temp Output LB (10.0 C), so a declared
        # safe setpoint in (0, 10] passed construction and was then refused by
        # `bound()` during an abort -- the one moment it must not fail.
        limits = self.effective_limits
        try:
            limits.validate(float(raw))
        except _circ.SafetyError as exc:
            raise _config.ConfigError(
                "circulator.safe_setpoint_c=%r is not a commandable setpoint: %s\n"
                "TODO(operator): choose a value strictly inside (%g, %g) C in "
                "configs/config.yaml."
                % (raw, exc, limits.min_c, limits.max_c)) from exc

        for name in ("plc_stale_s", "circ_stale_s", "large_step_c"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise _config.ConfigError(
                    "relay policy %s must be a number, got %s %r"
                    % (name, type(value).__name__, value))
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise _config.ConfigError(
                    "relay policy %s must be positive and finite, got %r"
                    % (name, value))

    @property
    def effective_limits(self):
        """The command bound the safe setpoint is held to.

        ``command_limits`` when set (``from_config`` resolves it through the
        circulator's own settings, so config tightening is respected), else the
        code ceiling. Never widen: ``CommandLimits`` refuses that at
        construction.
        """
        return self.command_limits or _circ.CommandLimits()

    # -- construction -----------------------------------------------------
    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> RelayPolicy:
        """Resolve the policy through the precedence contract in :mod:`tools.config`.

        ``safe_setpoint_c`` and ``circ_stale_s`` come from the **circulator**
        section, because they describe the bath. ``plc_stale_s`` comes from
        :class:`~.plc.PlcSettings`, not from a second reader of the same key --
        two resolution paths for one value drift, and then one of them is wrong.

        ``large_step_c`` has **no config key**. It is a quality-flag threshold,
        not a limit, and ``environment.max_setpoint_step_c`` next door means
        something different (a cap that refuses a write). Giving them one key
        would make a flag look like a guard; pass it as an override when a run
        wants a different threshold.
        """
        from .plc import PlcSettings

        lab = _config.LabConfig.load(config)
        section = lab.section(POLICY_SECTION)
        sources: dict[str, str] = {}
        resolved: dict[str, Any] = {}

        def take(key: str, default: Any) -> None:
            raw = _scalar(section, key)
            item = _config.resolve(
                key, argument=overrides.get(key),
                config={key: None if raw is _MISSING else raw},
                default=default, inventory_name="circulator.json",
                cast=None if default is None else float,
            )
            resolved[key] = item.value
            sources[key] = item.source

        # No default: absent must stay absent so __post_init__ can say so.
        take("safe_setpoint_c", None)
        if resolved["safe_setpoint_c"] is None:
            raise _unset_safe_setpoint(
                "the key is absent from configs/config.yaml"
                if "safe_setpoint_c" not in section
                else "the key is present but carries no value")

        take("circ_stale_s", DEFAULT_CIRC_STALE_S)
        take("large_step_c", DEFAULT_LARGE_STEP_C)

        plc = PlcSettings.from_config(config, **{
            key: overrides[key] for key in ("plc_stale_s",) if key in overrides})
        resolved["plc_stale_s"] = plc.plc_stale_s
        sources["plc_stale_s"] = plc.sources.get("plc_stale_s", "default")

        # The circulator owns what it may be commanded to, and config may have
        # tightened it below the code ceiling. Resolve it here so __post_init__
        # checks the safe setpoint against the range the abort path will use.
        from tools.circulator.api import CirculatorSettings

        circ = CirculatorSettings.from_config(config)
        resolved["command_limits"] = circ.limits
        sources["command_limits"] = "circulator settings (%s)" % circ.limits.describe()

        return cls(sources=sources, **resolved)

    # -- reporting --------------------------------------------------------
    def describe(self) -> str:
        """Every value with the layer it came from. A policy whose provenance is
        unknown is not a policy -- print this in any run that relays."""
        lines = ["relay policy (from configs/config.yaml, section %r and the "
                 "environment section)" % POLICY_SECTION]
        for field in dataclasses.fields(self):
            if field.name == "sources":
                continue
            lines.append("  %-16s %-10r [%s]"
                         % (field.name, getattr(self, field.name),
                            self.sources.get(field.name, "-")))
        lines.append("  safe setpoint is validated against %s"
                     % self.effective_limits.describe())
        return "\n".join(lines)


@dataclasses.dataclass(frozen=True)
class RelayRecord:
    """What one relay iteration observed and did. Data, not a verdict.

    Shaped so ``run.record("environment", "relay", record.as_dict())`` takes it
    directly: :meth:`as_dict` is JSON-serialisable all the way down, with the
    nested reading and write already flattened to dicts by their own owners.

    ``forwarded`` means **the value was bounded and handed to the circulator**.
    It does not mean the bath received it -- that is ``write["outcome"]``, which
    is ``"planned"`` for a dry run, ``"confirmed"`` for a verified write, and
    ``"failed"`` otherwise. ``write["ok"]`` is False for a successful dry run,
    and that is not a bug; see :class:`tools.circulator.api.WriteResult`.
    """

    t_utc: str
    monotonic_s: float
    forwarded: bool
    value_c: float | None
    source_channel: str
    reading: dict
    write: dict | None
    safe_mode: bool
    quality: dict
    reason: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "t_utc": self.t_utc,
            "monotonic_s": self.monotonic_s,
            "forwarded": self.forwarded,
            "value_c": self.value_c,
            "source_channel": self.source_channel,
            "reading": dict(self.reading),
            "write": None if self.write is None else dict(self.write),
            "safe_mode": self.safe_mode,
            "quality": dict(self.quality),
            "reason": self.reason,
        }

    def describe(self) -> str:
        if not self.forwarded:
            return "NOT FORWARDED: %s" % (self.reason or "unstated")
        outcome = (self.write or {}).get("outcome", "unknown")
        flag = "  LARGE STEP" if self.quality.get("large_step") else ""
        return "forwarded %s=%.6g -> circulator [%s]%s" % (
            self.source_channel, self.value_c, outcome, flag)


class Relay:
    """One iteration per :meth:`step`. The loop is the phase script's.

    ``env_client`` is a :class:`~.plc.PlcClient`, ``circulator`` a
    :class:`tools.circulator.api.Circulator`, and ``policy`` the operator's
    pre-declared :class:`RelayPolicy`. The **clock is injected** so the
    watchdogs are asserted deterministically instead of slept through.
    """

    #: Pass this to ``run.note()``. See the module docstring.
    HONEST_RESIDUAL = (
        "RESIDUAL RISK, unhandled by design: if the circulator's serial link is "
        "dead, the safe setpoint CANNOT be delivered. The relay writes it and "
        "reports the outcome; the MCU keeps whatever value it last received, and "
        "that device has no read-back of any kind, so 'we wrote it' is the most "
        "that can ever be said. Recovering from that state needs a person at the "
        "rig or a PLC-side ladder change. Nothing in software closes this gap.")

    SOURCE_CHANNEL = SOURCE_CHANNEL

    def __init__(self, env_client: Any, circulator: Any, policy: RelayPolicy,
                 *, clock: Any = None) -> None:
        if not isinstance(policy, RelayPolicy):
            raise _config.ConfigError(
                "Relay needs a RelayPolicy, got %s. The comms-loss setpoint is "
                "an operator decision declared before the run, not a default "
                "this layer may supply." % type(policy).__name__)
        self.env = env_client
        self.circulator = circulator
        self.policy = policy
        self._clock = clock or time.monotonic
        #: Set once the pre-declared fail-safe has been executed.
        self.safe_mode = False
        self.last_record: RelayRecord | None = None
        self._bad_since: float | None = None
        self._write_bad_since: float | None = None
        #: The last value handed over, used ONLY for the large-step quality
        #: flag. It is never re-sent: "hold the last value" is the 2026-01-11
        #: failure mode, where a saturated 30.000 was forwarded for hours.
        self._last_value: float | None = None

    # -- one iteration ----------------------------------------------------
    def step(self) -> RelayRecord:
        """Read the PLC once, forward DF9 once, or refuse once. Returns the record.

        Raises :class:`~.safety.CommsLost` when a watchdog fires (the safe-mode
        record is attached as ``exc.record``), and lets the circulator's
        :class:`~tools.circulator.api.SafetyError` propagate when DF9 is outside
        the command bound -- after executing the fail-safe. Both are events a
        phase script must see; swallowing either would leave the loop running on
        a state nobody observed.
        """
        now = float(self._clock())
        reading = self.env.read_block()
        value = reading.channels.get(SOURCE_CHANNEL)
        reason = self._why_unusable(reading, value)

        if reason is not None:
            return self._refuse(now, reading, reason)

        self._bad_since = None
        previous = self._last_value
        step_c = None if previous is None else abs(float(value) - previous)
        quality = {
            "large_step": step_c is not None and step_c > self.policy.large_step_c,
            "step_c": step_c,
            "large_step_c": self.policy.large_step_c,
            "previous_value_c": previous,
            "shaped": False,
            "shaping_note": "the forwarded value is the PLC's, unchanged. A large "
                            "step is a QUALITY FACT, not a reason to smooth: the "
                            "PLC is running the loop and would not know.",
        }

        try:
            # Raises, never clamps. Not swallowed: an out-of-bound PID output is
            # a safety event, and the prior system forwarded exactly this.
            sp = self.circulator.bound(float(value))
        except _circ.SafetyError as exc:
            record = self.safe(
                reason="DF9=%r was refused by the circulator command bound: %s"
                       % (value, exc))
            exc.record = record
            raise

        write = self.circulator.write_setpoint(sp)
        write_dict = write.as_dict() if hasattr(write, "as_dict") else dict(write)
        record = RelayRecord(
            t_utc=reading.t_utc, monotonic_s=reading.monotonic_s,
            forwarded=True, value_c=float(value), source_channel=SOURCE_CHANNEL,
            reading=reading.as_dict(), write=write_dict,
            safe_mode=self.safe_mode, quality=quality, reason=None)
        self.last_record = record

        if write_dict.get("outcome") == "failed":
            self._on_write_failure(now, write_dict)
        else:
            self._write_bad_since = None
            # Only the flag's baseline; see _last_value's comment.
            self._last_value = float(value)
        return record

    @staticmethod
    def _why_unusable(reading: EnvironmentReading, value: float | None) -> str | None:
        """Why this reading may not be forwarded, or ``None`` if it may be.

        The order matters and the middle two checks are the point of this
        method. ``read_ok`` is True for a SHORT block, so a reading missing the
        forwarded channel passes it -- see the module docstring. Do not reduce
        this to ``reading.read_ok``.
        """
        if not reading.read_ok:
            return ("the PLC read failed: no register pair arrived, so there is "
                    "no value to forward and nothing to substitute for one")
        if reading.partial:
            return ("the block is PARTIAL (%d of %d words arrived). read_ok is "
                    "True for a short block -- some pairs decoded -- so read_ok "
                    "alone is not evidence that %s is trustworthy, and this "
                    "reading is refused even when that channel happens to be "
                    "present." % (len(reading.raw_registers),
                                  registers.BLOCK_COUNT, SOURCE_CHANNEL))
        if value is None:
            return ("%s is absent from the reading's channels. The block "
                    "reported read_ok, and the channel still is not there."
                    % SOURCE_CHANNEL)
        if not math.isfinite(float(value)):
            return ("%s is not finite (%r). NaN encodes into a frame this device "
                    "would accept without error, which is the prior system's "
                    "defect: a failed upstream read still forwarded a value."
                    % (SOURCE_CHANNEL, value))
        return None

    def _refuse(self, now: float, reading: EnvironmentReading,
                reason: str) -> RelayRecord:
        """Record a non-forward, and fire the PLC watchdog if it has been too long."""
        if self._bad_since is None:
            self._bad_since = now
        elapsed = now - self._bad_since
        stale = elapsed > self.policy.plc_stale_s
        quality = {
            "large_step": False,
            "step_c": None,
            "large_step_c": self.policy.large_step_c,
            "previous_value_c": self._last_value,
            "plc_unusable_for_s": elapsed,
            "plc_stale_s": self.policy.plc_stale_s,
            "plc_stale": stale,
        }
        record = RelayRecord(
            t_utc=reading.t_utc, monotonic_s=reading.monotonic_s,
            forwarded=False, value_c=None, source_channel=SOURCE_CHANNEL,
            reading=reading.as_dict(), write=None,
            safe_mode=self.safe_mode or stale, quality=quality, reason=reason)
        self.last_record = record
        if not stale:
            return record

        safe_record = self.safe(
            reason="no usable PLC reading for %.3f s (plc_stale_s=%g): %s"
                   % (elapsed, self.policy.plc_stale_s, reason))
        error = CommsLost(
            "no usable PLC reading for %.3f s, past environment.plc_stale_s=%g. "
            "The safe setpoint %g C was written and the PID was disabled. The "
            "last value was NOT held: a real log from 2026-01-11 ends with the "
            "output pinned at exactly 30.000 while the measured temperature read "
            "11.4 C, so holding the last value can mean holding a saturated "
            "maximum. Last refusal: %s. Last transport error: %r"
            % (elapsed, self.policy.plc_stale_s, self.policy.safe_setpoint_c,
               reason, getattr(self.env, "last_error", None)))
        error.record = safe_record
        raise error

    def _on_write_failure(self, now: float, write: dict) -> None:
        """Fire the circulator watchdog if writes have been failing too long."""
        if self._write_bad_since is None:
            self._write_bad_since = now
        elapsed = now - self._write_bad_since
        if elapsed <= self.policy.circ_stale_s:
            return
        pid = self._disable_pid()
        self.safe_mode = True
        error = CommsLost(
            "no verified circulator write for %.3f s, past "
            "circulator.circ_stale_s=%g. The PID was disabled (%s). %s"
            % (elapsed, self.policy.circ_stale_s,
               pid.get("outcome", "unknown"), self.HONEST_RESIDUAL))
        error.record = self.last_record
        error.pid = pid
        raise error

    # -- the pre-declared fail-safe ---------------------------------------
    def safe(self, *, reason: str = "explicit safe() call") -> RelayRecord:
        """Command the declared safe setpoint and disable the PID. Never raises.

        This is the abort path, and **an abort path that can itself raise is not
        an abort path** -- the precedent is
        :meth:`~.plc.PlcClient.set_pid`, which reports an unobtainable read-back
        rather than raising for exactly that reason. So both actions are
        attempted, each failure is recorded as a fact, and the record comes back
        either way.

        The PID write is in a ``finally``, so C1 is driven False even if the
        setpoint write fails. That mirrors the one real interlock the prior
        system had (``try/finally`` writing ``C1 = False``) and is the property
        worth preserving from it.

        ``set_pid(False)`` is permitted with ``allow_actuation`` false: turning
        the loop off is the safe direction, so a supervisor that may not write
        must still be a supervisor that can stop.
        """
        t_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
        monotonic_s = time.monotonic()
        self.safe_mode = True
        write_dict: dict[str, Any] | None = None
        write_error: str | None = None
        try:
            try:
                sp = self.circulator.bound(float(self.policy.safe_setpoint_c))
                result = self.circulator.write_setpoint(sp)
                write_dict = (result.as_dict() if hasattr(result, "as_dict")
                              else dict(result))
            except Exception as exc:                                  # noqa: BLE001
                write_error = "%s: %s" % (type(exc).__name__, exc)
        finally:
            pid = self._disable_pid()

        quality = {
            "large_step": False,
            "step_c": None,
            "large_step_c": self.policy.large_step_c,
            "previous_value_c": self._last_value,
            "safe_setpoint_c": float(self.policy.safe_setpoint_c),
            "pid_disable": pid,
            "safe_write_error": write_error,
            "residual_risk": self.HONEST_RESIDUAL,
        }
        record = RelayRecord(
            t_utc=t_utc, monotonic_s=monotonic_s,
            forwarded=False, value_c=float(self.policy.safe_setpoint_c),
            source_channel="policy.safe_setpoint_c",
            reading={}, write=write_dict, safe_mode=True,
            quality=quality,
            reason="SAFE MODE: %s" % reason)
        self.last_record = record
        return record

    def _disable_pid(self) -> dict[str, Any]:
        """Drive C1 False and report what happened. Never raises.

        The result is flattened by ``WriteResult.as_dict()`` -- the same method
        the circulator's write result carries -- rather than by rebuilding a
        subset of its fields here. A hand-built dict is where ``ok`` and
        ``dry_run`` get dropped, and a dropped ``dry_run`` makes a plan and a
        confirmed write read identically in the run record.
        """
        try:
            result = self.env.set_pid(False)
        except Exception as exc:                                      # noqa: BLE001
            return {"outcome": "failed",
                    "error": "%s: %s" % (type(exc).__name__, exc)}
        if hasattr(result, "as_dict"):
            return dict(result.as_dict())
        # A stand-in transport that returns something else is a fact to report,
        # not to guess at: say what arrived instead of inventing an outcome.
        return {"outcome": getattr(result, "outcome", "unknown"),
                "ok": bool(getattr(result, "ok", False)),
                "error": getattr(result, "error", None),
                "note": "the PLC client returned %s, which carries no as_dict()"
                        % type(result).__name__}

    # -- session ----------------------------------------------------------
    def __enter__(self) -> Relay:
        return self

    def __exit__(self, *exc_info: object) -> bool:
        """Go safe on the way out, **including while an exception propagates**.

        Returns False, so nothing is suppressed: the phase script's exception is
        what the operator needs to see, and this method's job is only to make
        sure the bath and the PID are left in the declared state first.
        """
        self.safe(reason="relay context exited (exception=%s)"
                         % (type(exc_info[0]).__name__ if exc_info and exc_info[0]
                            else "none"))
        return False

    def __repr__(self) -> str:
        return "Relay(safe_setpoint_c=%g, plc_stale_s=%g, circ_stale_s=%g, %s)" % (
            self.policy.safe_setpoint_c, self.policy.plc_stale_s,
            self.policy.circ_stale_s, "SAFE MODE" if self.safe_mode else "running")


__all__ = [
    "DEFAULT_CIRC_STALE_S",
    "DEFAULT_LARGE_STEP_C",
    "DEFAULT_PLC_STALE_S",
    "POLICY_SECTION",
    "SOURCE_CHANNEL",
    "Relay",
    "RelayPolicy",
    "RelayRecord",
]
