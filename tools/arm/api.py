"""The arm's main API -- what an orchestration script calls.

    from tools import arm

    arm.move(35, 34, 33, "config.yaml")          # one-off, uses the shared session
    arm.grip()

    with arm.session("config.yaml", live=True) as a:   # explicit, preferred
        a.move_to("microscope")
        a.grip()

Both forms end up in the same place: :class:`Arm`, holding one
:class:`~.driver.XArmConnection` and one :class:`~.safety.Envelope`. The
module-level shorthands exist because the operator asked for
``arm.move(x, y, z, config.yaml)`` to just work; they are a thin wrapper over a
single cached session, not a second implementation.

Two things this layer refuses to guess
======================================

**Whether motion is live.** ``live=False`` is the default and makes the call a
dry-run plan: validated, printed, never commanded. Live motion needs
``live=True`` or ``allow_motion: true`` under ``arm:`` in ``config.yaml``.

**Where the safety envelope comes from.** Every move is checked against an
envelope, and an :class:`Arm` with no envelope raises rather than falling back
to something permissive. The default envelope is a box around a taught anchor
(normally well A1, whose Z budget is the operator's 10 mm plate-to-objective
limit). Crossing the workspace is possible but has to be asked for explicitly,
with a reason and a Z corridor -- see :meth:`Arm.free_envelope`.

Coordinates are absolute, in the arm base frame, millimetres and degrees.
Orientation is inherited from the anchor: these procedures translate the plate,
they never re-orient it.
"""
from __future__ import annotations

import contextlib
import dataclasses
import math
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from .. import config as _config
from .. import occupancy
from .driver import (
    BIO_MODE_OPEN_CLOSE,
    BIO_MODE_POSITION,
    BIO_POS_MAX_MM,
    BIO_POS_MIN_MM,
    ArmError,
    XArmConnection,
    reachable,
)
from .safety import (
    BACKLASH_TAKEUP_MM,
    SPEED_MAX_MM_S,
    TRANSIT_SPEED_MAX_MM_S,
    XY_MAX_MM,
    Z_MAX_RISE_MM,
    Envelope,
    SafetyError,
)
from .workspace import DEFAULT_STORE, WorkspaceStore

#: Hardware inventory shipped beside this module (host, gripper model, wiring).
INVENTORY_PATH = Path(__file__).resolve().parent / "arm.json"

#: How far a commanded axis may end up from its target and still count as
#: ARRIVED. Deliberately NOT ``READBACK_SLACK_MM`` (0.5 mm): that constant lets a
#: boundary check breathe, and borrowing it here would let a 400 um shortfall
#: read as a completed move. Derived from measurement instead -- the worst
#: per-leg residual on this rig is 0.6 um (2026-08-05 six-leg transit) and
#: 1.3 um over 1047 mm (2026-08-04). 0.05 mm is ~80x that, and still 6x smaller
#: than the 0.30 mm Jacobian step, so a refused calibration move cannot hide
#: inside it.
ARRIVAL_TOL_MM = 0.05

#: The orientation counterpart. The anchored envelopes already work to
#: ``orient_tol_deg=0.5``; arrival is held to a tenth of that so a wrist that did
#: not turn is distinguishable from one that turned and settled.
ARRIVAL_TOL_DEG = 0.05
# A joint move that stopped short is indistinguishable from one that arrived
# unless the angles are read back. Measured 2026-08-07: the controller accepted
# floor -> home, moved ~23 mm of Z, stopped, and returned success -- the run
# printed "arrived home" with the arm still over the tray. 0.5 deg is well above
# the controller's own settling noise and well below any leg that matters.
JOINT_ARRIVAL_TOL_DEG = 0.5

# Longest single Cartesian leg commanded in one go. Measured 2026-08-07: a
# 340 mm linear X leg at y=440.6 z=185.3 was accepted, travelled 52 mm, and
# stopped with code=-9 and no latched fault -- while get_inverse_kinematics
# succeeded at all 36 sampled points along that exact line, so reachability was
# not the reason. Subdividing does two things: the controller gets a request it
# has repeatedly honoured at this size, and if one does stop, the readback names
# the millimetre rather than leaving a 340 mm interval to search.
# NOTE (operator decision 2026-08-11): this is NO LONGER THE DEFAULT. Cartesian
# axis legs are now commanded as a single move each -- "move in one line". The
# measurement below is kept because it records a real observed failure, and the
# parameter is kept so a caller can still ask for subdivision explicitly; what
# changed is only what happens when nobody asks.
#
# The mitigation for the refusal described below is now the bounded
# REFUSAL_CODE == -9 retry in tools/arm/driver.py:send(), which re-arms and
# re-issues rather than pre-emptively cutting every leg into pieces.
MAX_LEG_MM = 25.0

# The orientation counterpart of MAX_LEG_MM, and it exists for the same reason:
# a wrist turn is motion, so it gets the same treatment as a translation --
# bounded steps, each read back, so a stop names the degree rather than leaving
# a 90 deg interval to search. 15 deg is six steps for the tray->scope turn.
# NOT measured against a refusal the way MAX_LEG_MM was; chosen by analogy, and
# no rotation refusal has been observed on this rig yet.
MAX_ROT_STEP_DEG = 15.0

#: How far above the jaws' hard minimum they must stop for something to be
#: considered held. Measured 2026-08-11: a good grip on the 96-well plate reads
#: 125.0 mm; a close on empty air reads 71.0 mm, which is BIO_POS_MIN_MM
#: exactly. 5 mm is a tiny fraction of that 54 mm separation -- this asks only
#: "did the jaws bottom out", which is unambiguous, rather than guessing a
#: thickness band that has never been characterised across grip positions.
GRIP_EMPTY_MARGIN_MM = 5.0


def _ang_diff(target: float, current: float) -> float:
    """Signed shortest-path difference in degrees, wrapped to (-180, 180].

    Without the wrap, 179 -> -179 reads as -358 deg and the arm would unwind the
    long way round -- 358 deg of wrist travel to accomplish 2.
    """
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


#: Taught location used as the default safety anchor. On this rig the plate is
#: carried under a fixed objective, so "microscope" *is* well A1.
DEFAULT_ANCHOR = "microscope"

DEFAULT_HOST = "192.168.1.201"
DEFAULT_SPEED_MM_S = 10.0
DEFAULT_OPEN_SPEED = 2000
DEFAULT_CLOSE_SPEED = 1000

#: BIO Gripper G2 control mode. 0 (open/close) is the factory default and what
#: this rig ran until 2026-08-11; 1 is position mode, where ``set_opening``
#: works. It is device-persistent state, so it belongs in configuration -- see
#: :data:`tools.arm.driver.BIO_MODE_OPEN_CLOSE`.
DEFAULT_CONTROL_MODE = BIO_MODE_OPEN_CLOSE
#: Position-mode defaults. Force is a percentage of the 20 N maximum, and the
#: position path will not accept a speed below 500.
DEFAULT_POSITION_SPEED = 2000
DEFAULT_GRIP_FORCE = 100


@dataclasses.dataclass(frozen=True)

class ArmSettings:
    """Everything the arm needs, with the layer each value came from.

    ``sources`` is what a run logs so a later reader can tell whether a limit
    came from a flag, from ``config.yaml``, from ``arm.json``, or from the code
    default. A safety limit whose provenance is unknown is not a safety limit.
    """

    host: str = DEFAULT_HOST
    live: bool = False
    clear_errors: bool = False
    speed_mm_s: float = DEFAULT_SPEED_MM_S
    speed_max_mm_s: float = SPEED_MAX_MM_S
    transit_speed_max_mm_s: float = TRANSIT_SPEED_MAX_MM_S
    anchor: str = DEFAULT_ANCHOR
    envelope_kind: str = "anchored"          # "anchored" | "workspace"
    z_max_rise_mm: float = Z_MAX_RISE_MM
    z_min_rise_mm: float = 0.0
    xy_max_mm: float = XY_MAX_MM
    orient_tol_deg: float = 0.5
    backlash_takeup_mm: float = BACKLASH_TAKEUP_MM
    z_floor_mm: float | None = None       # workspace envelope only
    z_ceiling_mm: float | None = None     # workspace envelope only
    open_speed: int = DEFAULT_OPEN_SPEED
    close_speed: int = DEFAULT_CLOSE_SPEED
    control_mode: int = DEFAULT_CONTROL_MODE
    position_speed: int = DEFAULT_POSITION_SPEED
    grip_force: int = DEFAULT_GRIP_FORCE
    workspace_path: Path = DEFAULT_STORE
    sources: dict = dataclasses.field(default_factory=dict)

    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> ArmSettings:
        """Resolve settings through the precedence contract in :mod:`tools.config`.

        ``overrides`` are CLI-flag level (highest precedence). Passing None for
        an override means "not specified", so a flag that was not given never
        shadows the config file.
        """
        lab = _config.LabConfig.load(config)
        section = lab.section("arm")
        inventory = _config.load_inventory(INVENTORY_PATH)
        gripper_inv = inventory.get("gripper", {}) if isinstance(inventory, dict) else {}

        resolved: dict[str, Any] = {}
        sources: dict[str, str] = {}

        def take(key: str, default: Any, cast: type | None = None,
                 inv: dict | None = None) -> None:
            item = _config.resolve(
                key, argument=overrides.get(key), config=section,
                inventory=inventory if inv is None else inv,
                default=default, inventory_name="arm.json", cast=cast,
            )
            resolved[key] = item.value
            sources[key] = item.source

        take("host", DEFAULT_HOST, str)
        take("live", False, bool)
        take("clear_errors", False, bool)
        take("speed_mm_s", DEFAULT_SPEED_MM_S, float)
        take("speed_max_mm_s", SPEED_MAX_MM_S, float)
        take("transit_speed_max_mm_s", TRANSIT_SPEED_MAX_MM_S, float)
        take("anchor", DEFAULT_ANCHOR, str)
        take("envelope_kind", "anchored", str)
        take("z_max_rise_mm", Z_MAX_RISE_MM, float)
        take("z_min_rise_mm", 0.0, float)
        take("xy_max_mm", XY_MAX_MM, float)
        take("orient_tol_deg", 0.5, float)
        take("backlash_takeup_mm", BACKLASH_TAKEUP_MM, float)
        take("z_floor_mm", None, float)
        take("z_ceiling_mm", None, float)
        take("open_speed", DEFAULT_OPEN_SPEED, int, inv=gripper_inv)
        take("close_speed", DEFAULT_CLOSE_SPEED, int, inv=gripper_inv)
        take("control_mode", DEFAULT_CONTROL_MODE, int, inv=gripper_inv)
        take("position_speed", DEFAULT_POSITION_SPEED, int, inv=gripper_inv)
        take("grip_force", DEFAULT_GRIP_FORCE, int, inv=gripper_inv)

        # `allow_motion: true` in config.yaml is the file-level way to say live.
        if overrides.get("live") is None and section.get("allow_motion") is True:
            resolved["live"] = True
            sources["live"] = "config.yaml:allow_motion"

        store = overrides.get("workspace_path") or lab.path_for(
            section.get("workspace_path"), default=DEFAULT_STORE)
        resolved["workspace_path"] = Path(store)
        sources["workspace_path"] = (
            "argument" if overrides.get("workspace_path") else
            "config.yaml" if section.get("workspace_path") else "default")

        if resolved["control_mode"] not in (BIO_MODE_OPEN_CLOSE, BIO_MODE_POSITION):
            raise _config.ConfigError(
                f"arm gripper.control_mode must be {BIO_MODE_OPEN_CLOSE} (open/close) "
                f"or {BIO_MODE_POSITION} (position), got {resolved['control_mode']!r}")
        if resolved["envelope_kind"] not in ("anchored", "workspace"):
            raise _config.ConfigError(
                f"arm.envelope_kind must be 'anchored' or 'workspace', got "
                f"{resolved['envelope_kind']!r}")
        if resolved["envelope_kind"] == "workspace" and (
                resolved["z_floor_mm"] is None or resolved["z_ceiling_mm"] is None):
            raise _config.ConfigError(
                "arm.envelope_kind='workspace' removes the XY box, so it requires an "
                "explicit z_floor_mm and z_ceiling_mm corridor. Refusing to run "
                "unbounded.")
        # Hardware-clearance limits live in code. config.yaml may TIGHTEN them
        # for a run; it may not loosen them, because the numbers describe the
        # rig (10 mm of plate-to-objective clearance), not a preference.
        for key, ceiling in (("z_max_rise_mm", Z_MAX_RISE_MM),
                             ("xy_max_mm", XY_MAX_MM),
                             ("speed_max_mm_s", SPEED_MAX_MM_S),
                             ("transit_speed_max_mm_s", TRANSIT_SPEED_MAX_MM_S)):
            if resolved[key] > ceiling + 1e-9:
                raise _config.ConfigError(
                    f"arm.{key}={resolved[key]} exceeds the hardware limit {ceiling} set in "
                    f"tools.arm.safety. config.yaml may tighten a clearance "
                    f"limit, never loosen it -- if the rig really changed, change the "
                    f"constant and say why in .claude/rules/motion-safety.md.")
        if resolved["transit_speed_max_mm_s"] < resolved["speed_max_mm_s"]:
            raise _config.ConfigError(
                f"arm.transit_speed_max_mm_s ({resolved['transit_speed_max_mm_s']}) is below "
                f"the fine-positioning cap speed_max_mm_s ({resolved['speed_max_mm_s']}); "
                f"transit is the looser of the two, so this is almost certainly a swap")
        if resolved["speed_mm_s"] > resolved["speed_max_mm_s"]:
            raise _config.ConfigError(
                f"arm.speed_mm_s ({resolved['speed_mm_s']}) exceeds speed_max_mm_s "
                f"({resolved['speed_max_mm_s']})")

        return cls(sources=sources, **resolved)

    def describe(self) -> str:
        lines = [f"arm settings (host={self.host}, live={self.live})"]
        for field in dataclasses.fields(self):
            if field.name == "sources":
                continue
            value = getattr(self, field.name)
            lines.append(f"  {field.name:<20} {value!r:<28} [{self.sources.get(field.name, '-')}]")
        return "\n".join(lines)


@dataclasses.dataclass
class MoveResult:
    """What one commanded pose actually did. Data, not a verdict."""

    label: str
    target: tuple
    achieved: list | None     # None in dry-run: nothing was read back
    live: bool
    dipped: bool = False

    @property
    def dry_run(self) -> bool:
        return not self.live

    def verify(self, tol_mm: float = ARRIVAL_TOL_MM,
               tol_deg: float = ARRIVAL_TOL_DEG) -> dict:
        """Did the arm actually GO WHERE IT WAS SENT? Reports; decides nothing.

        THE FAILURE THIS EXISTS FOR, measured 2026-08-04: commanding y=290.625 at
        x=26.029 stopped the controller dead at y=319.855 -- state=4, code=-9,
        error_code=0, NO latched fault. The arm silently did not go.

        Until 2026-08-06 nothing compared 'target' with 'achieved'. The only
        post-move check was Envelope.check_readback, and an envelope is a
        REGION, not a destination: the pose an arm stops short at is usually
        still inside it -- necessarily so when the arm never moved, because it
        started inside. So a refused move returned success and every caller
        believed it.

        TOLERANCE PROVENANCE. Not READBACK_SLACK_MM (0.5 mm): that exists to let
        a boundary check breathe, and reusing it here would let a 400 um
        shortfall read as arrival. Measured settling on this rig is far tighter
        -- worst residual 0.6 um across the six legs of the 2026-08-05 transit,
        and 1.3 um over 1047 mm on 2026-08-04. ARRIVAL_TOL_MM is 0.05 mm, ~80x
        the observed settling and 10x below the smallest move the calibration
        layer makes (the 0.30 mm Jacobian step).

        Returns 'moved': True (arrived), False (did not), or **None** when
        there is nothing to compare -- a dry run reads nothing back, and that is
        not evidence of either outcome.
        """
        if self.achieved is None:
            return {"moved": None, "reason": "dry run: no pose was read back",
                    "worst_axis": None, "worst_gap": None,
                    "tol_mm": tol_mm, "tol_deg": tol_deg}

        names = ("x", "y", "z", "roll", "pitch", "yaw")
        worst_axis, worst_gap, worst_tol = None, 0.0, tol_mm
        for i, name in enumerate(names):
            if i >= len(self.achieved) or i >= len(self.target):
                break
            tol = tol_mm if i < 3 else tol_deg
            gap = abs(float(self.achieved[i]) - float(self.target[i]))
            # Rank by how far past its OWN tolerance each axis is, so a 0.2 deg
            # orientation miss cannot be hidden by a larger-but-fine mm number.
            if gap / tol > worst_gap / worst_tol:
                worst_axis, worst_gap, worst_tol = name, gap, tol

        arrived = worst_axis is None or worst_gap <= worst_tol
        return {
            "moved": bool(arrived),
            "worst_axis": worst_axis,
            "worst_gap": round(worst_gap, 6),
            "tol_mm": tol_mm,
            "tol_deg": tol_deg,
            "reason": "" if arrived else (
                "%s stopped %.4f %s short of its target (tolerance %.4f); the "
                "controller can refuse a move with no latched fault"
                % (worst_axis, worst_gap, "deg" if worst_axis in ("roll", "pitch", "yaw")
                   else "mm", worst_tol)),
        }

    def as_dict(self) -> dict:
        record = {"label": self.label, "target": list(self.target),
                  "achieved": self.achieved, "live": self.live, "dipped": self.dipped}
        record["arrival"] = self.verify()
        return record


class Arm:
    """One xArm session plus the envelope its moves are checked against."""

    def __init__(self, settings: ArmSettings, *, envelope: Envelope | None = None,
                 log=None) -> None:
        self.settings = settings
        self._emit = log or (lambda message: print(message, flush=True))
        self.connection = XArmConnection(
            settings.host, live=settings.live,
            clear_errors=settings.clear_errors, log=self._emit,
        )
        self._store: WorkspaceStore | None = None
        self._envelope = envelope
        self._held = None      # set while inside Arm.occupied()

    # -- construction -----------------------------------------------------
    @classmethod
    def from_config(cls, config: Any = None, *, envelope: Envelope | None = None,
                    log=None, **overrides: Any) -> Arm:
        return cls(ArmSettings.from_config(config, **overrides),
                   envelope=envelope, log=log)

    def __enter__(self) -> Arm:
        return self

    def __exit__(self, exc_type, *_exc: object) -> None:
        try:
            if exc_type is not None:
                self.retreat()
        finally:
            self.close()

    def close(self) -> None:
        self.connection.disconnect()

    # -- taught workspace ---------------------------------------------------
    @property
    def store(self) -> WorkspaceStore:
        if self._store is None:
            self._store = WorkspaceStore(self.settings.workspace_path)
        return self._store

    def locations(self) -> list[str]:
        try:
            return sorted(self.store.workspace.locations)
        except Exception:
            return []

    def location_pose(self, name: str) -> list[float]:
        """The taught 6-DOF pose of a named location."""
        loc = self.store.require_location(name)
        p = loc.pose
        return [p.x, p.y, p.z, p.roll, p.pitch, p.yaw]

    # -- envelope -----------------------------------------------------------
    @property
    def envelope(self) -> Envelope:
        """The active envelope, built from settings on first use.

        Raises rather than defaulting when the anchor it needs is not taught:
        a missing anchor is missing safety context, and the rule here is fail
        closed.
        """
        if self._envelope is None:
            self._envelope = self._build_envelope()
        return self._envelope

    def _build_envelope(self) -> Envelope:
        s = self.settings
        if s.envelope_kind == "workspace":
            return Envelope.free(
                reason=f"arm.envelope_kind='workspace' in "
                       f"{self.settings.sources.get('envelope_kind', 'config')}",
                z_floor_mm=float(s.z_floor_mm), z_ceiling_mm=float(s.z_ceiling_mm),
                speed_max_mm_s=s.speed_max_mm_s,
            )
        try:
            anchor = self.location_pose(s.anchor)
        except Exception as exc:
            raise SafetyError(
                f"no safety anchor: taught location {s.anchor!r} is not in "
                f"{self.settings.workspace_path}. Teach it first, pass an explicit "
                f"envelope=, or declare arm.envelope_kind='workspace' with a Z corridor. "
                f"Refusing to move without an envelope. ({exc})"
            ) from None
        return Envelope.anchored(
            anchor, name=s.anchor,
            z_max_rise_mm=s.z_max_rise_mm, z_min_rise_mm=s.z_min_rise_mm,
            xy_max_mm=s.xy_max_mm, orient_tol_deg=s.orient_tol_deg,
            speed_max_mm_s=s.speed_max_mm_s,
        )

    def with_envelope(self, envelope: Envelope) -> Arm:
        """A view of this session that validates against a different envelope.

        Shares the connection -- one session, one socket -- so a procedure can
        tighten the box for one phase without reconnecting.
        """
        other = Arm.__new__(Arm)
        other.settings = self.settings
        other._emit = self._emit
        other.connection = self.connection
        other._store = self._store
        other._envelope = envelope
        other._held = self._held
        return other

    def free_envelope(self, *, reason: str, z_floor_mm: float, z_ceiling_mm: float) -> Arm:
        """Explicitly drop the XY box for a workspace traversal. Requires a reason.

        Carries the *transit* speed cap, not the fine-positioning one: a leg
        that crosses the workspace is nowhere near the objective, and holding it
        to the approach cap would be a limit answering the wrong question.
        """
        return self.with_envelope(Envelope.free(
            reason=reason, z_floor_mm=z_floor_mm, z_ceiling_mm=z_ceiling_mm,
            speed_max_mm_s=self.settings.transit_speed_max_mm_s))

    # -- reads ---------------------------------------------------------------
    def pose(self) -> list[float]:
        """Fresh pose from the controller (live only)."""
        return self.connection.read_pose()

    def joints(self) -> list[float]:
        return self.connection.read_joints()

    def status(self) -> dict:
        info = self.connection.status()
        info.update(locations=self.locations(), anchor=self.settings.anchor,
                    gripper={"open_speed": self.settings.open_speed,
                             "close_speed": self.settings.close_speed})
        try:
            info["envelope"] = self.envelope.describe()
        except SafetyError as exc:
            info["envelope"] = f"unavailable: {exc}"
        return info

    def _verify_before_first_motion(self, label: str) -> None:
        """Check where the arm actually is, before this session commands anything.

        Without this, the very first ``move()`` of a session validates its
        *target* and sends it while the arm's real position is never read -- and
        if the rig has been relocated (it has been, by 340 mm), that first
        "step" is an unbounded 6-DOF traverse. The post-move readback cannot
        help: it happens after the motion.

        Runs once per session. Every later move is chained to it by its own
        post-move readback, so the guarantee carries forward without a second
        round-trip per step.
        """
        if not self.settings.live or self.connection.start_verified:
            return
        actual = self.connection.read_pose()
        self.envelope.check_readback(
            actual, where=f"before the first commanded move ({label!r})")
        self.connection.start_verified = True

    def verify_at_anchor(self, *, where: str = "before starting") -> list[float] | None:
        """Read the machine and confirm it is inside the envelope before moving.

        This is the guard that a pose built from the anchor can never provide:
        it compares the controller's own report to the anchor. Dry-run has
        nothing to read, so it returns None.
        """
        if not self.settings.live:
            return None
        actual = self.pose()
        self.envelope.check_readback(actual, where=where)
        self.connection.start_verified = True
        return actual

    # -- motion ---------------------------------------------------------------
    def move(self, x: float, y: float, z: float, config: Any = None, *,
             speed: float | None = None, label: str = "move",
             takeup: bool = True) -> MoveResult:
        """Move to an absolute (x, y, z), keeping the anchor's orientation.

        ``config`` is accepted positionally so the operator's shorthand
        ``arm.move(35, 34, 33, "config.yaml")`` reads the same on a bound
        :class:`Arm`; on an already-configured instance it is ignored with a
        note rather than silently re-loading a different file mid-session.
        """
        if config is not None:
            self._log("  [arm] note: this session is already configured; the config "
                      "argument to move() is ignored")
        envelope = self.envelope
        target = (envelope.pose_at(x=x, y=y, z=z) if envelope.is_anchored
                  else self._free_target(x, y, z))
        speed_value = float(speed) if speed is not None else self.settings.speed_mm_s

        # Announce before anything moves. The backlash dip is real motion, so a
        # console that printed only after it would show the operator a silent
        # move followed by a line for a different one.
        move = envelope.validate(target, speed_value, label)
        self._announce(move)

        dipped = False
        with self._occupy(f"move: {label}"):
            self._verify_before_first_motion(label)
            if takeup and self.settings.backlash_takeup_mm > 0:
                dip = envelope.backlash_dip(target, takeup_mm=self.settings.backlash_takeup_mm)
                if dip is not None:
                    self._send(envelope.validate(dip, speed_value, f"{label} (backlash dip)"))
                    dipped = True
            self._send(move)

            achieved = None
            if self.settings.live:
                achieved = self.pose()
                envelope.check_readback(achieved, where=f"readback after {label!r}")
                achieved = [round(v, 4) for v in achieved]
        return MoveResult(label, target, achieved, self.settings.live, dipped)

    def move_axiswise(self, x: float, y: float, z: float, *,
                      speed: float | None = None, label: str = "move",
                      order: str | None = None, max_leg_mm: float | None = None,
                      takeup: bool = True) -> list[MoveResult]:
        """Reach (x, y, z) as ONE SINGLE-AXIS LEG AT A TIME, never diagonally.

        Standing operator rule on this rig (2026-08-07). A diagonal Cartesian
        move sweeps the tool through a volume nobody surveyed: the straight line
        between two safe poses is not itself safe, and the arm has now proved it
        twice in one session -- a collision (controller error 31) crossing the
        bench, and a silent stop lifting out of the relocated tray.

        Decomposed, every leg is a motion along one axis whose clearance can be
        reasoned about on its own, and each leg is a full :meth:`move` -- so it
        gets its own envelope validation, its own readback check, and its own
        occupancy claim. A leg that is refused stops the sequence there instead
        of being averaged into a diagonal that hides it.

        ``order`` is a permutation of ``"xyz"``. The default encodes clearance,
        not preference:

        * **descending** (target Z below current) -- ``"yxz"``: travel
          horizontally at the height you already have, and give up height last,
          once you are over the destination.
        * **ascending or level** -- ``"zyx"``: buy height first, then travel.

        Y leads X because the tray and the scope are both displaced from the
        base primarily in Y, so the Y leg is the one that clears the bench.

        Each axis leg is ONE command by default (operator decision
        2026-08-11). Passing ``max_leg_mm`` subdivides it into steps of at most
        that many mm, every one read back -- see :data:`MAX_LEG_MM` for the
        measured reason that option exists.

        Returns one :class:`MoveResult` per commanded step, in execution order.
        Axes already within :data:`ARRIVAL_TOL_MM` are skipped and absent.
        """
        if not self.settings.live:
            raise SafetyError(
                "move_axiswise needs the arm's current pose to decide leg order "
                "and to skip axes already in place; that can only be read from a "
                "live controller.")
        here = self.pose()
        target = {"x": float(x), "y": float(y), "z": float(z)}

        if order is None:
            order = "yxz" if target["z"] < here[2] else "zyx"
        order = order.lower()
        if sorted(order) != ["x", "y", "z"]:
            raise SafetyError(
                f"move_axiswise order must be a permutation of 'xyz', got {order!r}")

        idx = {"x": 0, "y": 1, "z": 2}
        cursor = [here[0], here[1], here[2]]

        # An axis already AT its target is commanded as the TARGET value, not as
        # the value just measured. Every leg's pose carries all three axes, so a
        # measured coordinate on a skipped axis would otherwise be fed back as a
        # command on the axes that DO move.
        #
        # That is not hypothetical: the transit travels at exactly the taught
        # working height, the arm reads ~45 nm above it, and the corridor ceiling
        # is that taught height -- so the X leg was rejected for a Z it never
        # meant to request. The corridor is deliberately NOT widened to absorb
        # this (up is toward the objective, and widening would make a genuinely
        # too-high target legal); the drift is kept out of the command instead.
        for axis, k in idx.items():
            if abs(target[axis] - cursor[k]) <= ARRIVAL_TOL_MM:
                cursor[k] = target[axis]
        results: list[MoveResult] = []
        for n, axis in enumerate(order, 1):
            k = idx[axis]
            if abs(target[axis] - cursor[k]) <= ARRIVAL_TOL_MM:
                self._log(f"  [axis] {axis} already within {ARRIVAL_TOL_MM} mm; skipped")
                continue
            start = cursor[k]
            span = target[axis] - start
            # max_leg_mm=None (the default since 2026-08-11) means one command
            # for the whole span. See MAX_LEG_MM for what subdivision was for.
            if max_leg_mm is None:
                chunks = 1
            else:
                if not math.isfinite(max_leg_mm) or max_leg_mm <= 0:
                    raise SafetyError(
                        f"max_leg_mm must be a finite positive number or None, "
                        f"got {max_leg_mm!r}")
                chunks = max(1, math.ceil(abs(span) / max_leg_mm))
            if chunks > 1:
                self._log(f"  [axis] {axis} {abs(span):.1f} mm -> {chunks} steps "
                          f"of <= {max_leg_mm:g} mm")
            for c in range(1, chunks + 1):
                cursor[k] = start + span * (c / chunks)
                leg = (f"{label} [{n}/3 {axis}->{target[axis]:.3f}]"
                       if chunks == 1 else
                       f"{label} [{n}/3 {axis} {c}/{chunks}->{cursor[k]:.3f}]")
                res = self.move(cursor[0], cursor[1], cursor[2], speed=speed,
                                takeup=(takeup and axis == "z" and c == chunks),
                                label=leg)
                arrival = res.verify()
                if arrival["moved"] is False:
                    raise SafetyError(
                        f"axis leg {leg!r} did not arrive ({arrival['reason']}). "
                        f"Stopping: the remaining legs assume this one landed.")
                results.append(res)
        return results

    def rotate_in_place(self, roll: float, pitch: float, yaw: float, *,
                        speed: float | None = None, label: str = "rotate",
                        max_step_deg: float = MAX_ROT_STEP_DEG) -> list[MoveResult]:
        """Turn the wrist to an absolute orientation, translating nothing.

        THE DEFECT THIS EXISTS FOR. :meth:`move` and :meth:`move_axiswise` both
        inherit orientation -- an anchored envelope supplies the anchor's, and
        :meth:`_free_target` copies whatever the arm is holding right now.
        Neither can *change* it. So a route from the tray (taught yaw 0.003) to
        the scope (taught yaw 89.985) had no step that turned the wrist, and the
        plate would have arrived under the objective 90 deg out. The joint-space
        route it replaced did turn it, but only as an emergent property of
        interpolating seven angles -- nothing in the code ever said so, which is
        exactly why converting the route to Cartesian dropped it silently.

        Rotation is motion, so it gets what translation gets: bounded steps
        (``max_step_deg``), each one a fully-guarded :meth:`move_pose` through
        the same ``Envelope.validate``, each read back and compared against what
        was commanded. A step that does not arrive raises rather than rotating
        on -- the remaining steps assume a turn that did not happen.

        XYZ is pinned to where the arm actually is when the rotation starts and
        re-commanded identically on every step, so this cannot creep in
        translation even if a step settles slightly off.

        Note an **anchored** envelope will refuse this outright unless its
        ``orient_tol_deg`` admits the turn -- correct, and the reason the
        tray->scope rotation runs under the route's free envelope.

        Returns one :class:`MoveResult` per commanded step, in execution order;
        an orientation already within :data:`ARRIVAL_TOL_DEG` returns ``[]``.
        """
        if not self.settings.live:
            raise SafetyError(
                "rotate_in_place needs the arm's current orientation to decide "
                "how far to turn and to hold XYZ fixed; that can only be read "
                "from a live controller.")
        if not math.isfinite(max_step_deg) or max_step_deg <= 0:
            raise SafetyError(
                f"max_step_deg must be a finite positive number, got {max_step_deg!r}")

        here = self.pose()
        want = (float(roll), float(pitch), float(yaw))
        for n, value in zip(("roll", "pitch", "yaw"), want, strict=True):
            if not math.isfinite(value):
                raise SafetyError(f"rotate_in_place {n} must be finite, got {value!r}")

        deltas = [_ang_diff(want[i], here[3 + i]) for i in range(3)]
        worst = max(abs(d) for d in deltas)
        if worst <= ARRIVAL_TOL_DEG:
            self._log(f"  [rot] orientation already within {ARRIVAL_TOL_DEG} deg; "
                      f"nothing commanded")
            return []

        steps = max(1, math.ceil(worst / float(max_step_deg)))
        self._log(f"  [rot] {worst:.1f} deg -> {steps} step(s) of <= {max_step_deg:g} deg")

        results: list[MoveResult] = []
        for c in range(1, steps + 1):
            frac = c / steps
            pose = (here[0], here[1], here[2],
                    here[3] + deltas[0] * frac,
                    here[4] + deltas[1] * frac,
                    here[5] + deltas[2] * frac)
            leg = f"{label} [{c}/{steps} yaw->{pose[5]:.3f}]"
            # takeup=False: the backlash dip is a vertical translation, and this
            # leg is not supposed to move in Z at all.
            res = self.move_pose(pose, speed=speed, label=leg, takeup=False)
            results.append(res)

            # Verify ALL SIX axes, not just orientation. Under a free envelope
            # check_readback tests only the Z corridor -- its XY and orientation
            # branches are guarded by `if self.anchor is not None` -- so a step
            # that drifted laterally was previously invisible here. That matters
            # doubly for a rotation: XYZ is captured ONCE before the loop, so an
            # unnoticed drift makes the NEXT step command the original XYZ
            # alongside another turn, which is a diagonal translation+rotation,
            # exactly what the axis-sequential rule forbids.
            arrival = res.verify()
            if arrival["moved"] is False:
                raise SafetyError(
                    f"rotation {leg!r} did NOT arrive: {arrival['reason']} "
                    f"(worst axis {arrival['worst_axis']}). The controller can "
                    f"accept a move, travel part of it and report success; the "
                    f"remaining steps assume a turn that did not happen, and a "
                    f"lateral drift here would turn the next step into a "
                    f"diagonal.")
        return results

    def _free_target(self, x: float, y: float, z: float) -> tuple:
        """Target for an unanchored envelope: orientation must come from the machine."""
        if not self.settings.live:
            raise SafetyError(
                "a workspace (unanchored) move needs the arm's current orientation, "
                "which can only be read from a live controller. Use an anchored "
                "envelope for dry-run planning.")
        here = self.pose()
        return (float(x), float(y), float(z), here[3], here[4], here[5])

    def move_pose(self, pose: Sequence[float], *, speed: float | None = None,
                  label: str = "move", takeup: bool = False,
                  verify: bool = True, announce: bool = True) -> MoveResult:
        """Command a fully-specified 6-DOF pose, orientation included.

        :meth:`move` inherits orientation from the anchor, which is right for
        well-local work but cannot express a route whose legs each carry their
        own orientation -- the Y-approach into A1, for instance. This is that
        case. It is still fully guarded: the pose goes through the same
        ``Envelope.validate`` and the same ``ValidatedMove``.

        ``verify=False`` skips only the post-move readback re-check, for legs an
        envelope cannot meaningfully bound (a transit whose XY safety comes from
        a separately validated route). The pose is still read back and returned;
        what is skipped is re-checking it against limits that do not apply.

        ``announce=False`` suppresses this layer's ``[move]`` line for callers
        that print their own -- the line is skipped, never the guards.
        """
        envelope = self.envelope
        target = envelope.check_target(pose, what=f"target {label!r}")
        speed_value = float(speed) if speed is not None else self.settings.speed_mm_s

        move = envelope.validate(target, speed_value, label)
        if announce:
            self._announce(move)

        dipped = False
        with self._occupy(f"move_pose: {label}"):
            self._verify_before_first_motion(label)
            if takeup and self.settings.backlash_takeup_mm > 0:
                dip = envelope.backlash_dip(target, takeup_mm=self.settings.backlash_takeup_mm)
                if dip is not None:
                    self._send(envelope.validate(dip, speed_value, f"{label} (backlash dip)"))
                    dipped = True
            self._send(move)

        achieved = None
        if self.settings.live:
            achieved = self.pose()
            if verify:
                envelope.check_readback(achieved, where=f"readback after {label!r}")
            achieved = [round(v, 4) for v in achieved]
        return MoveResult(label, target, achieved, self.settings.live, dipped)

    def move_to(self, location: str, *, speed: float | None = None,
                takeup: bool = True) -> MoveResult:
        """Move to a taught location by name."""
        pose = self.location_pose(location)
        return self.move(pose[0], pose[1], pose[2], speed=speed,
                         label=f"goto {location}", takeup=takeup)

    def move_joints(self, angles: Sequence[float], *, speed_deg_s: float,
                    label: str = "joint move") -> dict:
        """Command taught joint angles. No Cartesian envelope applies in joint space.

        Only ever call this with angles read from a taught location -- never
        with angles computed from a pose. The procedures that use it print the
        whole plan first and require an explicit execute flag.
        """
        self._log(f"  [move] {label:<28} joints={[round(a, 3) for a in angles]} "
                  f"@ {speed_deg_s:g} deg/s")
        # No pre-check here: joint space has no Cartesian envelope to check
        # against, which is exactly why these angles must come from a taught
        # location and never from arithmetic on a pose.
        with self._occupy(f"joint move: {label}"):
            self.connection.send_joints(angles, speed_deg_s, label)

            # Read the angles back. send_joints() returning without raising is
            # NOT evidence of arrival: this controller can accept a joint move,
            # travel part of it, stop, and report success with no latched fault.
            arrival = {"verified": False, "reason": "dry-run", "worst_axis": None,
                       "worst_gap_deg": None}
            if self.settings.live:
                actual = [float(a) for a in self.connection.read_joints()]
                # strict=True: a controller that returns FEWER joints than were
                # commanded must not have the missing ones silently skipped --
                # the prefix matching is not evidence the arm arrived.
                if len(actual) != len(angles):
                    raise SafetyError(
                        f"joint move {label!r}: commanded {len(angles)} angles but "
                        f"read back {len(actual)}. Refusing to judge arrival from "
                        f"a partial readback.")
                gaps = [abs(a - t) for a, t in zip(actual, angles, strict=True)]
                worst = max(range(len(gaps)), key=lambda k: gaps[k]) if gaps else None
                arrival = {
                    "verified": True,
                    "arrived": bool(gaps) and max(gaps) <= JOINT_ARRIVAL_TOL_DEG,
                    "worst_axis": (worst + 1) if worst is not None else None,
                    "worst_gap_deg": round(max(gaps), 4) if gaps else None,
                    "tol_deg": JOINT_ARRIVAL_TOL_DEG,
                    "actual_deg": [round(a, 4) for a in actual],
                }
                if not arrival["arrived"]:
                    raise SafetyError(
                        f"joint move {label!r} did NOT arrive: joint "
                        f"{arrival['worst_axis']} is {arrival['worst_gap_deg']} deg "
                        f"from target (tolerance {JOINT_ARRIVAL_TOL_DEG} deg). The "
                        f"controller reported success. Something stopped the arm "
                        f"part-way -- inspect before commanding anything further.")
        return {"label": label, "angles": list(angles), "live": self.settings.live,
                "arrival": arrival}

    def _log(self, message: str, *, error: bool = False) -> None:
        """Emit one operator-visible line. Safety alerts also go to stderr, so a
        'LOWER THE ARM MANUALLY' is not buried in a normal run transcript."""
        self._emit(message)
        if error:
            print(message, file=sys.stderr, flush=True)

    def _announce(self, move) -> None:
        pose = move.pose
        envelope = move.envelope
        if envelope.is_anchored:
            a = envelope.anchor
            self._log(f"  [move] {move.label:<28} x={pose[0]:.4f} y={pose[1]:.4f} "
                      f"z={pose[2]:.4f}  ({envelope.anchor_name}{pose[0] - a[0]:+.3f},"
                      f"{pose[1] - a[1]:+.3f},{pose[2] - a[2]:+.3f}) @ "
                      f"{move.speed_mm_s:g} mm/s")
        else:
            self._log(f"  [move] {move.label:<28} x={pose[0]:.4f} y={pose[1]:.4f} "
                      f"z={pose[2]:.4f} @ {move.speed_mm_s:g} mm/s")

    @property
    def engaged(self) -> bool:
        """True once anything has been commanded through this session.

        Read from the connection so that Arm views sharing it (see
        :meth:`with_envelope`) cannot disagree about whether a retreat is owed.
        """
        return self.connection.engaged

    def _occupy(self, doing: str):
        """Hold the arm for one actuation, refusing if the UV-Vis carrier is out.

        Dry-run claims nothing: it commands no hardware, so it cannot collide
        with anything and must not block a real run that is under way.
        """
        if not self.settings.live:
            return contextlib.nullcontext()
        if self._held is not None:      # already inside an outer claim
            return contextlib.nullcontext()
        return occupancy.claim("arm", doing=doing)

    @contextlib.contextmanager
    def occupied(self, doing: str):
        """Hold the arm across a whole sequence rather than per move.

        A phase script should wrap its run in this: claiming once means the
        UV-Vis carrier cannot slip in *between* two of its moves, which
        per-move claiming would allow.
        """
        if not self.settings.live:
            yield
            return
        with occupancy.claim("arm", doing=doing) as held:
            self._held = held
            try:
                yield
            finally:
                self._held = None

    def _send(self, move) -> None:
        self.connection.send(move)

    # -- emergency --------------------------------------------------------------
    def retreat(self) -> None:
        """Lower the plate to the anchor height, away from the objective.

        Best effort and deliberately swallowing a second interrupt: once the
        plate is raised toward a fixed lens, getting it back down matters more
        than a clean exit code. Does nothing if this session never moved.
        """
        if not (self.settings.live and self.engaged):
            return
        envelope = self._envelope
        if envelope is None or not envelope.is_anchored:
            self._log("  [safety] no anchored envelope to retreat to -- LOWER THE ARM "
                      "MANUALLY if the plate is raised")
            return
        # Deliberately does NOT take a claim. retreat only lowers the plate away
        # from the objective, it runs on the abort path, and refusing it because
        # something else is busy would leave the plate up -- the exact state it
        # exists to escape.
        try:
            self.connection.halt()
            self.connection.recover()
            # retreat is allowed to be the first commanded motion of a session:
            # it only ever moves DOWN to the anchor height, away from the lens.
            self.connection.start_verified = True
            here = self.connection.read_pose()
            anchor = envelope.anchor
            self._log(f"  [safety] lowering to the taught {envelope.anchor_name} height "
                      f"z={anchor[2]:.4f}")
            # Two candidates, tried in order. Lowering in place is preferred --
            # it adds no lateral motion -- but it is only *valid* under an
            # envelope whose XY box has room, and the Z-only envelopes
            # (xy_max_mm=0) reject it because the machine's settled XY never
            # equals the anchor to within EPS. Falling back to the anchor's own
            # XY is what the retired focus_sweep.retreat() commanded, and it
            # always validates. A guard that can only ever refuse is not a
            # guard; a retreat that can only ever refuse leaves the plate up.
            candidates = (
                ("in place", (here[0], here[1], anchor[2], anchor[3], anchor[4], anchor[5])),
                (f"at {envelope.anchor_name} XY", tuple(anchor)),
            )
            last: BaseException | None = None
            for how, target in candidates:
                try:
                    move = envelope.validate(target, self.settings.speed_mm_s,
                                             f"retreat ({how})")
                except SafetyError as exc:
                    last = exc
                    continue
                self.connection.send(move)
                self._log(f"  [safety] retreat complete ({how})")
                return
            raise last if last is not None else SafetyError("no retreat target validated")
        except BaseException as exc:  # including a second KeyboardInterrupt
            self._log(f"  [safety] RETREAT FAILED: {exc!r} -- LOWER THE ARM MANUALLY",
                      error=True)

    # -- gripper ------------------------------------------------------------------
    def grip(self) -> dict:
        """Close the BIO Gripper G2 jaws. Jaw-only: the arm body does not move.

        REPORTS whether anything is held; it does not act on it. Per this
        surface's standing law a device returns measurements and quality flags
        and the phase script decides -- so a caller that lifts on a failed grip
        is the caller's defect, and now it has the fact it needs to avoid one.

        Adds to the returned dict:

        ``opening_mm``      jaws' position after closing, read back from the
                            gripper, or ``None`` if it could not be read.
        ``bottomed_out``    True when the jaws reached their hard minimum, i.e.
                            they closed on nothing. ``None`` when unknown --
                            never silently False, because "we could not tell"
                            and "it is holding" must not look alike.

        Measured 2026-08-11: plate held reads 125.0 mm, empty air reads 71.0 mm
        (= ``BIO_POS_MIN_MM``). The run that prompted this closed on empty air,
        reported success, and carried the nothing it was holding to the far side
        of the bench.
        """
        self._log("  [gripper] grip (close)")
        with self._occupy("gripper close"):
            result = self.connection.bio_move("close", self.settings.close_speed,
                                              mode=self.settings.control_mode)

        opening_mm, bottomed = None, None
        try:
            reading = self.opening()
            raw = reading.get("mm") if isinstance(reading, dict) else reading
            if raw is not None:
                value = float(raw)
                # A non-finite reading must stay UNKNOWN. `nan <= threshold` is
                # False, so without this a NaN jaw position reported itself as
                # "holding" and authorised a lift.
                if math.isfinite(value):
                    opening_mm = value
                    bottomed = value <= BIO_POS_MIN_MM + GRIP_EMPTY_MARGIN_MM
                else:
                    self._log(f"  [gripper] WARNING: jaw position read back as "
                              f"{value!r}; holding state is UNKNOWN, not assumed good")
        except Exception as exc:                       # noqa: BLE001
            self._log(f"  [gripper] WARNING: could not read jaw position after "
                      f"closing ({exc}); holding state is UNKNOWN, not assumed good")

        # Always return a mapping. The caller reads bottomed_out unconditionally
        # before it lifts; a non-dict result previously raised AttributeError
        # outside the (ArmError, SafetyError) handler, losing the controlled
        # abort this check exists to provide.
        result = dict(result) if isinstance(result, dict) else {"bio_result": result}
        result.update(opening_mm=opening_mm, bottomed_out=bottomed,
                      jaw_min_mm=BIO_POS_MIN_MM)
        if opening_mm is not None:
            self._log(f"  [gripper] jaws at {opening_mm:.1f} mm "
                      f"(min {BIO_POS_MIN_MM:.1f}) -- "
                      f"{'NOTHING HELD' if bottomed else 'holding'}")
        return result

    def release(self) -> dict:
        """Open the BIO Gripper G2 jaws."""
        self._log("  [gripper] release (open)")
        with self._occupy("gripper open"):
            return self.connection.bio_move("open", self.settings.open_speed,
                                            mode=self.settings.control_mode)

    def set_opening(self, opening_mm: float, *, speed: int | None = None,
                    force: int | None = None) -> dict:
        """Drive the jaws to a span of ``opening_mm``, between 71 and 150 mm.

        Requires ``gripper.control_mode: 1`` -- position mode. Both jaws move
        together about the tool axis; the gripper takes one span, not a pair of
        jaw positions, so there is no way to open one side only.

        Returns the commanded target alongside ``actual_mm`` read back from the
        gripper, and ``settled``, which is False when two readings taken a
        moment apart disagree -- the SDK's motion wait can return early, so
        ``actual_mm`` is only a measurement when ``settled`` is True.

        A gap between target and actual means the jaws stopped early -- on an
        object, but equally on a stall, too little force, or an obstruction.
        That is a measurement, not a verdict: deciding "this is a plate" needs a
        tolerance and an expected width, and belongs to the phase script.
        """
        speed = self.settings.position_speed if speed is None else int(speed)
        force = self.settings.grip_force if force is None else int(force)
        self._log(f"  [gripper] set opening {opening_mm} mm "
                  f"(speed={speed}, force={force}%)")
        with self._occupy(f"gripper position {opening_mm} mm"):
            return self.connection.bio_position(
                opening_mm, speed, force, mode=self.settings.control_mode)

    def opening(self) -> dict:
        """Read the current jaw span. Reads only; actuates nothing.

        Returns the value with the fact that qualifies it. Outside position mode
        the SDK's setter and getter disagree about units, so ``mm`` is reported
        with ``units_verified: False`` rather than presented as a measurement.
        """
        return {"mm": self.connection.bio_opening(),
                "control_mode": self.settings.control_mode,
                "units_verified": self.settings.control_mode == BIO_MODE_POSITION}


# ---------------------------------------------------------------------------
# module-level shorthand -- one cached session, explicitly closable
# ---------------------------------------------------------------------------

_DEFAULT: Arm | None = None
_DEFAULT_KEY: tuple | None = None


def session(config: Any = None, **overrides: Any) -> Arm:
    """A fresh, independent :class:`Arm`. Prefer this in scripts.

    Use it as a context manager so the connection is closed (and the plate
    lowered on error) deterministically::

        with arm.session("config.yaml", live=True) as a:
            a.move_to("microscope")
    """
    return Arm.from_config(config, **overrides)


def default(config: Any = None, **overrides: Any) -> Arm:
    """The process-wide session behind the module-level shorthands.

    Cached on (config, overrides): calling with different settings replaces it,
    closing the previous one first, so a shorthand call never silently keeps
    talking through a session configured for something else.
    """
    global _DEFAULT, _DEFAULT_KEY
    key = (repr(config), tuple(sorted(overrides.items(), key=lambda kv: kv[0])))
    if _DEFAULT is None or _DEFAULT_KEY != key:
        close()
        _DEFAULT = Arm.from_config(config, **overrides)
        _DEFAULT_KEY = key
    return _DEFAULT


def close() -> None:
    """Close the cached session, if any."""
    global _DEFAULT, _DEFAULT_KEY
    if _DEFAULT is not None:
        _DEFAULT.close()
    _DEFAULT = None
    _DEFAULT_KEY = None


def move(x: float, y: float, z: float, config: Any = None, **overrides: Any) -> MoveResult:
    """``arm.move(35, 34, 33, "config.yaml")`` -- move via the cached session.

    Dry-run unless ``live=True`` is passed or ``arm.allow_motion`` is true in
    the config file.
    """
    return default(config, **overrides).move(x, y, z)


def move_to(location: str, config: Any = None, **overrides: Any) -> MoveResult:
    return default(config, **overrides).move_to(location)


def grip(config: Any = None, **overrides: Any) -> dict:
    return default(config, **overrides).grip()


def release(config: Any = None, **overrides: Any) -> dict:
    return default(config, **overrides).release()


def set_opening(opening_mm: float, config: Any = None, *, speed: int | None = None,
                force: int | None = None, **overrides: Any) -> dict:
    """``arm.set_opening(96)`` -- partial jaw span, via the cached session."""
    return default(config, **overrides).set_opening(opening_mm, speed=speed, force=force)


def opening(config: Any = None, **overrides: Any) -> dict:
    return default(config, **overrides).opening()


def status(config: Any = None, **overrides: Any) -> dict:
    return default(config, **overrides).status()


__all__ = [
    "ARRIVAL_TOL_DEG",
    "GRIP_EMPTY_MARGIN_MM",
    "ARRIVAL_TOL_MM",
    "MAX_ROT_STEP_DEG",
    "Arm",
    "ArmError",
    "ArmSettings",
    "BIO_POS_MAX_MM",
    "BIO_POS_MIN_MM",
    "DEFAULT_ANCHOR",
    "Envelope",
    "INVENTORY_PATH",
    "MoveResult",
    "SafetyError",
    "close",
    "default",
    "grip",
    "move",
    "move_to",
    "opening",
    "reachable",
    "release",
    "session",
    "set_opening",
    "status",
]
