"""Motion guards for the xArm. Every commanded pose passes through here.

Why this module exists as a *type* and not a convention
=======================================================

Before the consolidation there were six independent copies of "connect, enable,
set_position" across the repo, each with its own subset of guards. Nothing
structurally prevented a seventh caller from reaching the SDK directly and
skipping all of them.

So the guard is enforced by construction, not by discipline:

* :class:`ValidatedMove` cannot be built except by :meth:`Envelope.validate`
  (its constructor demands a module-private token).
* :mod:`tools.arm.driver` refuses any argument that is not a
  ``ValidatedMove``.

A caller who wants to skip the guards has to edit this file, which is exactly
the visibility the previous arrangement lacked.

Reference frame and units
=========================

Every pose is a 6-tuple ``(x, y, z, roll, pitch, yaw)`` in the **arm base
frame**, millimetres and degrees, matching ``XArmAPI.get_position(is_radian=
False)``. An envelope's limits are expressed relative to its *anchor* -- the
taught pose the operation is defined around (normally well A1). There is no
implicit anchor: an operation that genuinely moves across the workspace must
ask for :meth:`Envelope.free` and say why, so "no box" is always a recorded
decision rather than the result of missing context.

Guard inventory (each one preserved verbatim from the implementation it came
from -- see ``test/test_arm_safety.py``):

===  ==========================================================================
G1   a target may only change the axes the operation declared
G2   target Z stays within ``[anchor_z + z_min_rise, anchor_z + z_max_rise]``
G3   the arm's **actual** pose is re-read after every move and re-checked
     against the anchor -- never against the arithmetic that produced it
G4   commanded speed is capped
G5   XY stays inside the declared box around the anchor
G6   orientation stays within tolerance of the anchor
G7   non-finite or wrong-length poses are rejected before anything is sent
===  ==========================================================================

G3 is the one that matters most. An earlier version compared a pose built
*from* the anchor back *to* the anchor -- a tautology that always passed while
the arm's real position was never read at all. :meth:`check_readback` takes a
freshly-read machine pose and nothing else.
"""
from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence

AXES = ("x", "y", "z", "roll", "pitch", "yaw")

#: Floating-point slop when comparing a commanded pose to a limit. Not a
#: tolerance the operator may tune -- it exists so an exactly-on-the-limit
#: target is not rejected by representation error.
EPS = 1e-6

#: Ceiling on commanded Cartesian speed **for fine positioning** -- approach
#: legs and well-local work under the objective, where a large ``--speed`` turns
#: a careful approach into a lunge. This is the default cap on an anchored
#: envelope.
SPEED_MAX_MM_S = 30.0

#: Ceiling for **workspace transit** -- carrying the plate between the floor
#: pick, the standoff, and the scope, far from the objective. A separate number
#: because it answers a different question: 30 mm/s is about not lunging into a
#: lens, and a transit leg is nowhere near one. Pass it explicitly to
#: :meth:`Envelope.free`; an anchored envelope never gets it.
TRANSIT_SPEED_MAX_MM_S = 100.0

#: Operator's hard limit: the plate may rise at most this far above the taught
#: A1 height, because raising Z drives the plate *toward* the fixed objective.
Z_MAX_RISE_MM = 10.0

#: Default XY box for closed-loop re-centring around a taught well.
XY_MAX_MM = 2.5

#: Readback slack. The controller settles a little short of the command; these
#: are the tolerances that distinguish "arrived" from "somewhere else entirely".
READBACK_SLACK_MM = 0.5
READBACK_SLACK_DEG = 0.5

#: Approach every Z from below so backlash is taken up the same way at every
#: sample. Zero disables the dip.
BACKLASH_TAKEUP_MM = 0.30


class SafetyError(RuntimeError):
    """A commanded pose violates the envelope. Always raised, never clamped.

    Clamping would silently turn an out-of-range request into a plausible
    in-range move, which is how a wrong number becomes a collision.
    """


_VALIDATION_TOKEN = object()


class ValidatedMove:
    """A pose that has passed :meth:`Envelope.validate`. The driver takes only these.

    Constructing one directly raises: the whole point is that the type itself
    is the evidence a guard ran.
    """

    __slots__ = ("pose", "speed_mm_s", "label", "envelope")

    def __init__(self, token: object, pose: Sequence[float], speed_mm_s: float,
                 label: str, envelope: Envelope) -> None:
        if token is not _VALIDATION_TOKEN:
            raise SafetyError(
                "ValidatedMove() may only be created by Envelope.validate(); "
                "constructing one directly would bypass every motion guard"
            )
        self.pose = tuple(float(v) for v in pose)
        self.speed_mm_s = float(speed_mm_s)
        self.label = str(label)
        self.envelope = envelope

    def __repr__(self) -> str:
        x, y, z = self.pose[0], self.pose[1], self.pose[2]
        return (f"ValidatedMove({self.label!r} x={x:.4f} y={y:.4f} z={z:.4f} "
                f"@ {self.speed_mm_s:g} mm/s)")


def check_pose_shape(pose: Sequence[float], *, what: str) -> tuple[float, ...]:
    """G7: reject a malformed pose before any limit arithmetic runs."""
    values = tuple(pose)
    if len(values) != 6:
        raise SafetyError(f"{what}: expected 6 pose values (x,y,z,roll,pitch,yaw), "
                          f"got {len(values)}")
    out = []
    for name, raw in zip(AXES, values, strict=False):
        try:
            value = float(raw)
        except (TypeError, ValueError) as exc:
            raise SafetyError(f"{what}: {name} is not a number: {raw!r}") from exc
        if not math.isfinite(value):
            raise SafetyError(f"{what}: {name} is not finite ({value!r})")
        out.append(value)
    return tuple(out)


@dataclasses.dataclass(frozen=True)
class Envelope:
    """The limits one operation is allowed to move within.

    Build it through :meth:`anchored` or :meth:`free` -- never by guessing at
    the fields -- so that an unanchored operation is always an explicit,
    justified choice.
    """

    anchor: tuple[float, ...] | None
    anchor_name: str = "anchor"
    z_min_rise_mm: float = 0.0
    z_max_rise_mm: float = Z_MAX_RISE_MM
    xy_max_mm: float = 0.0
    orient_tol_deg: float = 0.0
    readback_slack_mm: float = READBACK_SLACK_MM
    readback_slack_deg: float = READBACK_SLACK_DEG
    speed_max_mm_s: float = SPEED_MAX_MM_S
    z_floor_mm: float | None = None
    z_ceiling_mm: float | None = None
    free_reason: str = ""

    # -- constructors ---------------------------------------------------
    @classmethod
    def anchored(
        cls,
        anchor: Sequence[float],
        *,
        name: str = "A1",
        z_max_rise_mm: float = Z_MAX_RISE_MM,
        z_min_rise_mm: float = 0.0,
        xy_max_mm: float = 0.0,
        orient_tol_deg: float = 0.0,
        readback_slack_mm: float = READBACK_SLACK_MM,
        readback_slack_deg: float = READBACK_SLACK_DEG,
        speed_max_mm_s: float = SPEED_MAX_MM_S,
    ) -> Envelope:
        """A box around a taught pose. The normal case for well-local work."""
        pose = check_pose_shape(anchor, what=f"{name} anchor")
        if z_max_rise_mm < z_min_rise_mm:
            raise SafetyError(f"z_max_rise_mm ({z_max_rise_mm}) is below "
                              f"z_min_rise_mm ({z_min_rise_mm})")
        for field, value in (("z_max_rise_mm", z_max_rise_mm), ("xy_max_mm", xy_max_mm),
                             ("orient_tol_deg", orient_tol_deg),
                             ("speed_max_mm_s", speed_max_mm_s)):
            if not math.isfinite(value) or value < 0:
                raise SafetyError(f"{field} must be a finite non-negative number, got {value!r}")
        return cls(
            anchor=pose, anchor_name=name,
            z_min_rise_mm=float(z_min_rise_mm), z_max_rise_mm=float(z_max_rise_mm),
            xy_max_mm=float(xy_max_mm), orient_tol_deg=float(orient_tol_deg),
            readback_slack_mm=float(readback_slack_mm),
            readback_slack_deg=float(readback_slack_deg),
            speed_max_mm_s=float(speed_max_mm_s),
        )

    @classmethod
    def free(
        cls,
        *,
        reason: str,
        z_floor_mm: float,
        z_ceiling_mm: float,
        speed_max_mm_s: float = TRANSIT_SPEED_MAX_MM_S,
    ) -> Envelope:
        """Workspace traversal: no XY box, but an explicit Z corridor and a reason.

        Used by the plate-transport and pick sequences, which genuinely cross
        the workspace. ``reason`` is recorded in every error message so an
        unanchored move is never anonymous.
        """
        if not reason.strip():
            raise SafetyError("Envelope.free() requires a reason; an unanchored "
                              "envelope must always say what justified it")
        if not (math.isfinite(z_floor_mm) and math.isfinite(z_ceiling_mm)):
            raise SafetyError("Envelope.free() requires finite z_floor_mm and z_ceiling_mm")
        if z_ceiling_mm < z_floor_mm:
            raise SafetyError(f"z_ceiling_mm ({z_ceiling_mm}) is below z_floor_mm ({z_floor_mm})")
        return cls(anchor=None, anchor_name="workspace", free_reason=reason.strip(),
                   z_floor_mm=float(z_floor_mm), z_ceiling_mm=float(z_ceiling_mm),
                   speed_max_mm_s=float(speed_max_mm_s))

    # -- derived ---------------------------------------------------------
    @property
    def is_anchored(self) -> bool:
        return self.anchor is not None

    def z_window(self, *, slack_mm: float = 0.0) -> tuple[float, float]:
        """The permitted absolute Z range, optionally widened by readback slack."""
        if self.anchor is not None:
            lo = self.anchor[2] + self.z_min_rise_mm
            hi = self.anchor[2] + self.z_max_rise_mm
        else:
            lo, hi = float(self.z_floor_mm), float(self.z_ceiling_mm)
        return (lo - slack_mm, hi + slack_mm)

    def describe(self) -> str:
        if self.anchor is None:
            lo, hi = self.z_window()
            return (f"free envelope ({self.free_reason}): z in [{lo:.3f}, {hi:.3f}] mm, "
                    f"speed <= {self.speed_max_mm_s:g} mm/s")
        a = self.anchor
        return (f"{self.anchor_name} envelope: xy within +/-{self.xy_max_mm:.2f} mm of "
                f"({a[0]:.4f}, {a[1]:.4f}), z within [{a[2] + self.z_min_rise_mm:.4f}, "
                f"{a[2] + self.z_max_rise_mm:.4f}] (rise <= {self.z_max_rise_mm:.2f} mm), "
                f"orientation within {self.orient_tol_deg:.2f} deg, "
                f"speed <= {self.speed_max_mm_s:g} mm/s")

    # -- guards ----------------------------------------------------------
    def _check(self, pose: Sequence[float], *, what: str, slack_mm: float,
               slack_deg: float) -> tuple[float, ...]:
        values = check_pose_shape(pose, what=what)
        problems: list[str] = []

        lo, hi = self.z_window(slack_mm=slack_mm)
        if not (lo - EPS <= values[2] <= hi + EPS):
            if self.anchor is not None:
                problems.append(
                    f"z {values[2]:.4f} outside [{lo:.4f}, {hi:.4f}] "
                    f"({self.anchor_name} z={self.anchor[2]:.4f}, budget "
                    f"{self.z_max_rise_mm:.2f} mm)")
            else:
                problems.append(f"z {values[2]:.4f} outside the free corridor "
                                f"[{lo:.4f}, {hi:.4f}]")

        if self.anchor is not None:
            xy_tol = self.xy_max_mm + slack_mm
            for i, name in ((0, "x"), (1, "y")):
                delta = values[i] - self.anchor[i]
                if abs(delta) > xy_tol + EPS:
                    problems.append(
                        f"{name} is {delta:+.4f} mm from {self.anchor_name}, outside the "
                        f"+/-{xy_tol:.3f} mm box")
            deg_tol = self.orient_tol_deg + slack_deg
            for i in (3, 4, 5):
                delta = values[i] - self.anchor[i]
                if abs(delta) > deg_tol + EPS:
                    problems.append(
                        f"{AXES[i]} differs from {self.anchor_name} by {delta:+.4f} deg "
                        f"(tolerance {deg_tol:.3f})")

        if problems:
            raise SafetyError(f"{what}: " + "; ".join(problems))
        return values

    def check_target(self, pose: Sequence[float], *, what: str) -> tuple[float, ...]:
        """G1/G2/G5/G6/G7 on a pose about to be commanded. Raises; never clamps."""
        return self._check(pose, what=what, slack_mm=0.0, slack_deg=0.0)

    def check_readback(self, actual: Sequence[float], *, where: str) -> tuple[float, ...]:
        """G3: the arm's freshly-read pose must sit inside the envelope.

        ``actual`` must come from the controller. Passing a pose this process
        computed makes the check a tautology -- that exact defect shipped once
        and would have turned the first "Z step" into an unbounded traverse.
        """
        try:
            return self._check(actual, what=where, slack_mm=self.readback_slack_mm,
                               slack_deg=self.readback_slack_deg)
        except SafetyError as exc:
            if self.anchor is None:
                raise
            raise SafetyError(
                f"{exc}. The arm is not at {self.anchor_name}. Refusing to move: from "
                f"here the first commanded pose would be a full 6-DOF traverse, not a "
                f"step within the envelope. Drive to {self.anchor_name} along its "
                f"approach route first."
            ) from None

    def check_speed(self, speed_mm_s: float, *, what: str) -> float:
        """G4."""
        try:
            value = float(speed_mm_s)
        except (TypeError, ValueError) as exc:
            raise SafetyError(f"{what}: speed is not a number: {speed_mm_s!r}") from exc
        if not math.isfinite(value) or value <= 0:
            raise SafetyError(f"{what}: speed must be positive and finite, got {value!r}")
        if value > self.speed_max_mm_s + EPS:
            raise SafetyError(f"{what}: speed {value:g} mm/s exceeds the cap "
                              f"{self.speed_max_mm_s:g} mm/s")
        return value

    def validate(self, pose: Sequence[float], speed_mm_s: float, label: str) -> ValidatedMove:
        """The **only** way to produce a :class:`ValidatedMove`."""
        checked = self.check_target(pose, what=f"target {label!r}")
        speed = self.check_speed(speed_mm_s, what=f"target {label!r}")
        return ValidatedMove(_VALIDATION_TOKEN, checked, speed, label, self)

    # -- helpers used by procedures --------------------------------------
    def pose_at(self, *, x: float | None = None, y: float | None = None,
                z: float | None = None) -> tuple[float, ...]:
        """Anchor pose with selected axes replaced. Orientation always the anchor's."""
        if self.anchor is None:
            raise SafetyError("pose_at() needs an anchored envelope")
        pose = list(self.anchor)
        if x is not None:
            pose[0] = float(x)
        if y is not None:
            pose[1] = float(y)
        if z is not None:
            pose[2] = float(z)
        return tuple(pose)

    def backlash_dip(self, pose: Sequence[float], *,
                     takeup_mm: float = BACKLASH_TAKEUP_MM) -> tuple[float, ...] | None:
        """The pose to visit just below ``pose`` so Z is always approached from below.

        Returns None when no dip is needed or possible (already at the floor).
        """
        values = check_pose_shape(pose, what="backlash dip")
        if takeup_mm <= 0:
            return None
        floor = self.z_window()[0]
        under = max(floor, values[2] - takeup_mm)
        if under >= values[2] - 1e-9:
            return None
        dipped = list(values)
        dipped[2] = under
        return tuple(dipped)


__all__ = [
    "AXES",
    "BACKLASH_TAKEUP_MM",
    "EPS",
    "Envelope",
    "READBACK_SLACK_DEG",
    "READBACK_SLACK_MM",
    "SPEED_MAX_MM_S",
    "SafetyError",
    "TRANSIT_SPEED_MAX_MM_S",
    "ValidatedMove",
    "XY_MAX_MM",
    "Z_MAX_RISE_MM",
    "check_pose_shape",
]
