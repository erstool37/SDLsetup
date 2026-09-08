"""Who is moving right now, and who is therefore not allowed to.

The problem this exists for
===========================

The UV-Vis plate carrier slides **out into the arm's workspace**. If the arm is
mid-traverse when the tray opens, or the tray is out when the arm swings
through, they collide. Nothing in the wiring prevents it: they are different
instruments on different interfaces, driven by different scripts.

And the scripts are **different processes**. ``scripts/calibration.py`` and
``scripts/plate_imaging.py`` and the dashboard each run on their own; an
in-memory flag in one of them is invisible to the others. So the record of who
is busy has to live on disk, and claiming it has to be atomic.

How it works
============

One small JSON file per instrument under ``~/.sdl_lab/occupancy/``. Creating it
is the claim, and it is created with ``O_CREAT | O_EXCL`` -- the filesystem
does the mutual exclusion, so two processes racing for the same instrument
cannot both win.

    from tools import occupancy

    with occupancy.claim("arm", doing="A1 focus sweep"):
        ...                       # the uv_vis carrier is refused for the duration

A claim carries the PID that made it. If that process is gone, the claim is
**stale** and is cleared automatically -- otherwise a crashed run would lock the
rig until someone deleted a file by hand.

What conflicts with what
========================

``CONFLICTS`` is the whole policy, and it is deliberately small enough to read:

* ``arm`` and ``uv_vis`` exclude each other -- they share physical space.
* every instrument excludes **itself** -- two processes must not drive one
  device, whether that is the arm's controller or the camera's USB endpoint.
* ``microscope`` conflicts with nothing but itself; the cameras do not move.
* ``environment`` conflicts with nothing but itself. The CLICK PLC does not
  move, but the self-conflict is load-bearing all the same: **the PLC accepts
  at most three concurrent Modbus TCP clients and refuses the fourth**, so two
  processes opening supervisor sessions is how a run loses its connection.
* ``circulator`` conflicts with nothing but itself, for a sharper reason:
  **opening its serial port hardware-resets the MCU** (FT232R with DTR wired to
  /RESET; the OS asserts DTR on open, before any byte is sent). A second
  process opening the port reboots the board under the first one.

Adding an instrument that occupies space means adding it here. An instrument
missing from the table conflicts only with itself, which is the safe default
for something that does not move -- so **check the table when you add a
device that does**.

What this is not
================

It is advisory between *cooperating* processes, like every lock of this kind.
It cannot stop someone driving the arm from a Python REPL, and it is not a
substitute for the motion envelope in :mod:`tools.arm.safety` -- that guards
*where* the arm may go, this guards *when*. Both apply.
"""
from __future__ import annotations

import contextlib
import dataclasses
import datetime as dt
import errno
import json
import os
import time
from collections.abc import Iterator
from pathlib import Path

#: Live state, beside the taught poses. Not in the repo: it describes this
#: machine right now, and a stale copy on another machine would be a lie.
OCCUPANCY_DIR = Path.home() / ".sdl_lab" / "occupancy"

#: Which instruments may not move at the same time. See the module docstring.
#: Every instrument implicitly conflicts with itself.
CONFLICTS: dict[str, frozenset[str]] = {
    # The UV-Vis carrier slides out into the arm's workspace.
    "arm": frozenset({"uv_vis"}),
    "uv_vis": frozenset({"arm"}),
    # The cameras do not move; they contend only for their own USB endpoint.
    "microscope": frozenset(),
    # The CLICK PLC does not move. It contends for its own session: only three
    # concurrent Modbus TCP clients are accepted, and the fourth is refused.
    "environment": frozenset(),
    # The bath does not move either -- but opening its serial port resets the
    # MCU, so a second opener reboots the board under the first.
    "circulator": frozenset(),
}

#: A claim older than this with a live PID is reported as long-running rather
#: than stale. It is not force-cleared -- a genuinely long sweep is normal.
LONG_RUNNING_S = 30 * 60


class Busy(RuntimeError):
    """Another instrument (or another process) is moving. Nothing was started."""


@dataclasses.dataclass(frozen=True)
class Claim:
    """One instrument's live state, as recorded on disk."""

    device: str
    doing: str
    pid: int
    since: float
    phase: str = ""

    @property
    def age_s(self) -> float:
        return max(0.0, time.time() - self.since)

    @property
    def alive(self) -> bool:
        """Is the process that made this claim still running?"""
        try:
            os.kill(self.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            # Owned by another user -- it exists, and we cannot signal it.
            return True
        except OSError:
            return True
        return True

    @property
    def stale(self) -> bool:
        return not self.alive

    def as_dict(self) -> dict:
        return {
            "device": self.device, "doing": self.doing, "pid": self.pid,
            "phase": self.phase,
            "since": dt.datetime.fromtimestamp(self.since).isoformat(timespec="seconds"),
            "age_s": round(self.age_s, 1),
            "alive": self.alive,
            "long_running": self.age_s > LONG_RUNNING_S,
        }


def _path(device: str) -> Path:
    if not device or "/" in device or device.startswith("."):
        raise ValueError(f"bad device name: {device!r}")
    return OCCUPANCY_DIR / f"{device}.json"


def _read(device: str) -> Claim | None:
    try:
        data = json.loads(_path(device).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    try:
        return Claim(device=device, doing=data.get("doing", ""), pid=int(data["pid"]),
                     since=float(data["since"]), phase=data.get("phase", ""))
    except (KeyError, TypeError, ValueError):
        return None


def current(device: str) -> Claim | None:
    """The live claim on ``device``, clearing it first if its process is gone."""
    claim = _read(device)
    if claim is None:
        return None
    if claim.stale:
        with contextlib.suppress(OSError):
            _path(device).unlink()
        return None
    return claim


def status() -> dict[str, dict]:
    """Every instrument's state. What the panel renders. Reports; decides nothing."""
    known = set(CONFLICTS)
    if OCCUPANCY_DIR.exists():
        known |= {p.stem for p in OCCUPANCY_DIR.glob("*.json")}
    out: dict[str, dict] = {}
    for device in sorted(known):
        claim = current(device)
        out[device] = ({"busy": False, "device": device}
                       if claim is None else {"busy": True, **claim.as_dict()})
    return out


def blockers(device: str) -> list[Claim]:
    """Live claims that forbid ``device`` from moving right now.

    A device conflicts with whatever ``CONFLICTS`` says, plus itself.
    """
    against = set(CONFLICTS.get(device, frozenset())) | {device}
    found = []
    for other in sorted(against):
        claim = current(other)
        if claim is not None:
            found.append(claim)
    return found


def require_free(device: str) -> None:
    """Raise :class:`Busy` if anything conflicting is moving. Starts nothing."""
    held = blockers(device)
    if not held:
        return
    lines = [
        f"{c.device} has been busy {c.age_s:.0f}s: {c.doing or 'unspecified'}"
        f" (pid {c.pid}{', ' + c.phase if c.phase else ''})"
        for c in held
    ]
    raise Busy(
        f"cannot start {device}: " + "; ".join(lines) + ". "
        "The arm and the UV-Vis carrier share physical space, so they are never "
        "allowed to move together. Wait for it to finish, or if that process is "
        "gone the claim clears itself."
    )


@contextlib.contextmanager
def claim(device: str, *, doing: str = "", phase: str = "") -> Iterator[Claim]:
    """Hold ``device`` for the duration of the block.

    Raises :class:`Busy` **before** doing anything if a conflicting instrument
    is already moving. Releases on the way out, including on exception -- an
    aborted run must not leave the rig locked.
    """
    require_free(device)
    OCCUPANCY_DIR.mkdir(parents=True, exist_ok=True)
    record = Claim(device=device, doing=doing, pid=os.getpid(), since=time.time(),
                   phase=phase or Path(__import__("sys").argv[0]).name)
    payload = json.dumps({"doing": record.doing, "pid": record.pid,
                          "since": record.since, "phase": record.phase}, indent=2)
    path = _path(device)
    try:
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            raise
        # Lost a race, or a stale file reappeared. current() clears stale ones.
        require_free(device)
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except OSError:
            raise Busy(f"cannot start {device}: another process claimed it first") from None
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(payload)
        yield record
    finally:
        # Only remove our own claim -- never someone else's.
        held = _read(device)
        if held is not None and held.pid == record.pid:
            with contextlib.suppress(OSError):
                path.unlink()


def release_stale() -> list[str]:
    """Clear claims whose process is gone. Returns what was cleared."""
    cleared = []
    if not OCCUPANCY_DIR.exists():
        return cleared
    for p in sorted(OCCUPANCY_DIR.glob("*.json")):
        claim_ = _read(p.stem)
        if claim_ is not None and claim_.stale:
            with contextlib.suppress(OSError):
                p.unlink()
            cleared.append(p.stem)
    return cleared


__all__ = [
    "Busy",
    "CONFLICTS",
    "Claim",
    "LONG_RUNNING_S",
    "OCCUPANCY_DIR",
    "blockers",
    "claim",
    "current",
    "release_stale",
    "require_free",
    "status",
]
