"""Command channel between the dashboard and the ONE live controller.

THE CONTRACT (this module defines it; nothing else may re-implement it)
=======================================================================
Exactly one process writes to the PLC and the circulator at a time: the live
loop in ``scripts/environment/hold_environment.py`` (what ``start_kinetics
--execute`` runs). While it holds the ``environment`` occupancy claim, the
dashboard cannot open a session of its own -- so it *asks* instead, through
two files in :data:`~tools.environment.plc.PUBLISH_DIR`:

``command.json``  dashboard -> controller. ONE pending command, replaced
                  atomically (``os.replace``). A newer command overwrites an
                  unconsumed older one; that is deliberate -- the operator's
                  latest intent wins, and nothing is ever replayed.
``last_reading.json``  controller -> dashboard. The existing
                  :func:`~tools.environment.plc.publish_last_reading` record,
                  which the loop now writes EVERY step, extended with two keys:

    "controller": {"pid": int, "run_dir": str, "started_unix_s": float,
                   "valve_mode": "auto" | "open" | "close" | "unavailable",
                   "valve_note": str}
    "last_command": {"seq": int, "name": str, "args": dict, "source": str,
                     "outcome": "confirmed" | "planned" | "failed" | "refused"
                                | "stale",
                     "detail": str, "finished_unix_s": float}

Commands (``name`` -> ``args``)::

    "setpoint"  {"temp_c": float?, "rh_pct": float?}     at least one key
    "pid"       {"enabled": bool}                          a real bool
    "valve"     {"mode": "auto" | "open" | "close"}

WHAT THIS MODULE DOES AND DOES NOT DO
-------------------------------------
It validates *shape* (types, finiteness, allowed names) and moves JSON
atomically. It does NOT apply bounds, gates, rate limits, or decide anything:
the consumer runs every command through the same guards a CLI flag would --
``SetpointLimits.validate`` -> ``BoundedSetpoint`` -> ``RateLimiter`` ->
``write_float32`` with read-back; ``set_pid`` with read-back. Guards raise,
never clamp. A command older than :data:`STALE_AFTER_S` when taken is reported
as ``"stale"`` and never applied -- a queued intent must not fire after a
restart or a long stall.

``tools/`` reports; ``scripts/`` decides.
"""
from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

COMMAND_NAME = "command.json"
SEQ_NAME = "command.seq"
COMMAND_NAMES: tuple[str, ...] = ("setpoint", "pid", "valve")
VALVE_MODES: tuple[str, ...] = ("auto", "open", "close")
#: A command not consumed within this many seconds of being issued is stale.
STALE_AFTER_S = 30.0

OUTCOME_CONFIRMED = "confirmed"
OUTCOME_PLANNED = "planned"
OUTCOME_FAILED = "failed"
OUTCOME_REFUSED = "refused"
OUTCOME_STALE = "stale"
OUTCOMES: tuple[str, ...] = (OUTCOME_CONFIRMED, OUTCOME_PLANNED, OUTCOME_FAILED,
                             OUTCOME_REFUSED, OUTCOME_STALE)


@dataclass(frozen=True)
class Command:
    seq: int
    issued_unix_s: float
    name: str
    args: dict[str, Any]
    source: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class ChannelError(ValueError):
    """Malformed command (shape only). Bounds are the consumer's business."""


def channel_dir() -> Path:
    """Resolved at call time so tests may reassign ``plc.PUBLISH_DIR``."""
    from . import plc  # noqa: PLC0415  -- late, so a test's reassignment is seen
    return Path(plc.PUBLISH_DIR)


def _command_path() -> Path:
    return channel_dir() / COMMAND_NAME


def _seq_path() -> Path:
    return channel_dir() / SEQ_NAME


def _require_finite(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ChannelError("%s must be a real number, got %r" % (key, value))
    if not math.isfinite(value):
        raise ChannelError("%s must be finite, got %r" % (key, value))
    return float(value)


def validate(name: str, args: dict[str, Any]) -> dict[str, Any]:
    """Return a normalised copy of ``args`` or raise :class:`ChannelError`."""
    if name not in COMMAND_NAMES:
        raise ChannelError("unknown command %r (one of %s)" % (name, COMMAND_NAMES))
    if not isinstance(args, dict):
        raise ChannelError("args must be a mapping, got %r" % type(args).__name__)
    if name == "setpoint":
        out: dict[str, Any] = {}
        for key in ("temp_c", "rh_pct"):
            if key in args and args[key] is not None:
                out[key] = _require_finite(args[key], key)
        extra = set(args) - {"temp_c", "rh_pct"}
        if extra:
            raise ChannelError("setpoint: unexpected keys %s" % sorted(extra))
        if not out:
            raise ChannelError("setpoint needs temp_c and/or rh_pct")
        return out
    if name == "pid":
        enabled = args.get("enabled")
        if enabled is not True and enabled is not False:
            raise ChannelError("pid.enabled must be a real bool, got %r" % (enabled,))
        return {"enabled": enabled}
    mode = args.get("mode")
    if mode not in VALVE_MODES:
        raise ChannelError("valve.mode must be one of %s, got %r" % (VALVE_MODES, mode))
    return {"mode": mode}


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".%d.tmp" % os.getpid())
    temp.write_text(text, encoding="utf-8")
    os.replace(temp, path)


def _next_seq() -> int:
    path = _seq_path()
    try:
        current = int(path.read_text(encoding="utf-8").strip() or "0")
    except (FileNotFoundError, ValueError):
        current = 0
    nxt = current + 1
    _atomic_write(path, str(nxt))
    return nxt


def enqueue(name: str, args: dict[str, Any], *, source: str = "dashboard",
            now: Callable[[], float] = time.time) -> Command:
    """Validate shape, assign a seq, and atomically replace the pending command."""
    clean = validate(name, args)
    cmd = Command(seq=_next_seq(), issued_unix_s=float(now()), name=name,
                  args=clean, source=str(source))
    _atomic_write(_command_path(), json.dumps(cmd.as_dict(), indent=1))
    return cmd


def _load(path: Path) -> Command | None:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as exc:
        raise ChannelError("%s: unreadable command (%s)" % (path, exc)) from exc
    if not isinstance(raw, dict):
        raise ChannelError("%s: command is not a JSON object" % path)
    try:
        name = str(raw["name"])
        return Command(seq=int(raw["seq"]), issued_unix_s=float(raw["issued_unix_s"]),
                       name=name, args=validate(name, raw["args"]),
                       source=str(raw.get("source", "unknown")))
    except (KeyError, TypeError, ValueError) as exc:
        raise ChannelError("%s: malformed command (%s)" % (path, exc)) from exc


def peek() -> Command | None:
    """The pending command without consuming it (for the dashboard's 'queued' pill)."""
    return _load(_command_path())


def take() -> Command | None:
    """Consume the pending command: return it and remove the file, or ``None``.

    Removal happens BEFORE the caller applies anything, so a crash mid-apply
    cannot replay the command on restart. The caller decides staleness with
    :func:`is_stale` and reports the outcome through the published record.
    """
    path = _command_path()
    cmd = _load(path)
    if cmd is None:
        return None
    try:
        path.unlink()
    except FileNotFoundError:
        return None
    return cmd


def is_stale(cmd: Command, *, now: Callable[[], float] = time.time,
             stale_after_s: float = STALE_AFTER_S) -> bool:
    age = float(now()) - cmd.issued_unix_s
    return not math.isfinite(age) or age < 0 or age > stale_after_s


def outcome_record(cmd: Command, outcome: str, detail: str, *,
                   now: Callable[[], float] = time.time) -> dict[str, Any]:
    """The ``last_command`` block the controller publishes after handling ``cmd``."""
    if outcome not in OUTCOMES:
        raise ChannelError("unknown outcome %r" % outcome)
    rec = cmd.as_dict()
    rec.update(outcome=outcome, detail=str(detail), finished_unix_s=float(now()))
    return rec


__all__ = [
    "COMMAND_NAMES", "VALVE_MODES", "STALE_AFTER_S", "OUTCOMES",
    "OUTCOME_CONFIRMED", "OUTCOME_PLANNED", "OUTCOME_FAILED", "OUTCOME_REFUSED",
    "OUTCOME_STALE",
    "Command", "ChannelError", "channel_dir", "validate", "enqueue", "peek", "take",
    "is_stale", "outcome_record",
]
