#!/usr/bin/env python3
"""
plate_imaging.py — UFACTORY xArm7 96-well plate imaging motion script.

=== Flow Summary ===
  1. Goto home         (ensure_safe_z then joint move to [0]*7; not holding -> transit speed)
  2. Approach floor    (Z-decoupled: ascend, XY-to-standoff, descend; not holding)
  3. Grab plate        (close gripper slowly; sets holding=True)
  4a. Floor->standoff  (ascend at floor XY to scope Z, then lateral to scope_standoff)
  4b. Push in to A1    (lateral move at fixed Z from standoff to A1; NEVER descends)
  5. Capture A1        (Leica K3C capture+verify; warn and continue on failure)
  6. Goto target well  (flat XY move at fixed Z, very slow; no standoff)
  6b. Capture well     (Leica K3C capture+verify; warn and continue on failure)
  7a. Return to A1     (flat XY move back to A1 at fixed Z)
  7b. Pull out         (lateral move from A1 to scope_standoff at fixed Z)
  8. Return to floor   (Z-decoupled approach, slow/holding -- bring the plate back)
  9. Release plate     (open gripper at floor -- put it back down)
  10. Return home      (ensure_safe_z, then joint to zero; gripper open/empty)

Captures grab the live-session's current Leica frame (a file copy of the stream
frame, post-arrival) so they never contend with the running live view for the
single-client K3C; a direct camera capture is used only if no live stream exists.

=== Lateral Microscope Entry/Exit via scope_standoff (CRITICAL SAFETY) ===
The microscope LENS sits directly above well A1. Vertical approach (ascending/
descending over A1) is a SERIOUS collision hazard and is PROHIBITED for microscope
access. The plate enters and exits the microscope ONLY by lateral XY motion at the
microscope's fixed working Z, via a taught staging pose: scope_standoff.

  scope_standoff: a safe staging pose OUTSIDE the scope, already at A1's working Z
                  (Z was clamped to A1.z when taught). Loaded from the workspace store.

  Approach (floor -> A1):
    1. Ascend straight up at floor XY to the scope working Z (scope_standoff.z)
    2. Lateral move to scope_standoff (outside the scope, at fixed Z)
    3. Lateral push-in from scope_standoff to A1 (never descends -- lens above A1)

  Departure (A1 -> home):
    1. Lateral pull-out from A1 to scope_standoff (at fixed Z)
    2. ensure_safe_z at standoff XY, then joint to home
    NEVER call ensure_safe_z above A1 -- that would ascend through the lens.

=== Z-Decoupled Safety Model (floor pick only) ===
The floor pick approach remains Z-decoupled (vertical). This is safe because there
is no lens above the floor. Every approach to the floor pick MUST be Z-decoupled:
  1. ensure_safe_z(target_z): ascend in place to max(current_z, target_z + standoff)
     at the CURRENT XY and orientation.
  2. Stored waypoint 1: XY move to over the target at the standoff Z.
  3. Stored waypoint 2: Pure-Z descend to the working Z (z_descend=True).

=== Routes Are Fixed Perpetually ===
A route (ordered list of waypoint dicts) from FROM->TO is computed once, saved to
~/.sdl_lab/robot_arm/routes.json, and replayed identically on every subsequent run.
This ensures reproducible, verifiable motion paths. Use --rebuild-routes to force
recomputation for any route touched this run.

Each waypoint dict fields:
  kind:       "cart" | "joint"
  pose:       [x,y,z,roll,pitch,yaw]   (mm/deg; for kind="cart")
  angles:     [j1..j7]                 (deg; for kind="joint")
  z_descend:  bool  (True only for the final descend step -> uses hold_z_mm_s)
  label:      str

=== Camera Capture and Verify ===
At A1 and at the target well, Executor.capture_and_verify() triggers a Leica K3C
image capture via tools.microscope. It parses the CaptureResult JSON
printed to stdout and checks the built-in signal verdict (ok field). Failures are
WARNINGS only -- the run continues. Results are emitted as display ops.

=== Display Ops Feed ===
Best-effort JSON events are POST-ed to <display_url>/ops (default
http://127.0.0.1:8770) throughout the run. Swallows all exceptions; never blocks
or aborts the run. Disable with --no-display.

=== Holding-Slowness Rule ===
Executor.holding tracks whether the plate is gripped. When True, all motion uses
HOLD speeds (hold_joint_deg_s, hold_cart_mm_s, hold_z_mm_s for z-descend moves).
When False, TRANSIT speeds are used. This rule applies automatically to every
waypoint and ensure_safe_z call throughout the run.

=== Dry-Run Safety ===
Without --execute the script prints every planned waypoint (label, kind, full
pose/angles, speed) and every action (grab, capture) without connecting to hardware
or moving anything. Review this output before going live.

=== Dependencies ===
  stdlib only + tools.arm (Arm/ArmSettings/Envelope -- see
  tools/devices/arm/__init__.py) + tools.microscope (used only by
  capture_and_verify(); that module was not found in this reorg'd tree at
  migration time -- left as-is per the migration brief, see UNCERTAIN below).
  The xarm SDK import itself stays deferred inside tools.arm.driver,
  never imported directly here.

=== Migrated Arm Plumbing (this reorg) ===
Executor no longer opens its own XArmAPI session. It builds ONE shared
tools.arm.Arm (see cmd_run) and drives every move through it:
  - Cartesian legs (floor approach, floor<->scope_standoff, A1<->well) go
    through Envelope.free() -- NOT the default anchored A1 box -- because this
    script genuinely crosses the workspace (floor to scope and back) and every
    waypoint already carries its own taught orientation rather than inheriting
    one from a single anchor. The Z corridor is derived from the four taught
    locations (home/floor/microscope/scope_standoff) plus z_standoff_mm; it is
    not an invented number.
  - The "home" joint move goes through Arm.move_joints() (joint space; no
    Cartesian envelope applies there).
  - The BIO gripper goes through Arm.grip() / Arm.release().
  - Host and gripper speeds come from ArmSettings (config.yaml > arm.json >
    code default) instead of a direct robot.json read.

TWO SPEED CAPS, and this script is why they are separate. The 30 mm/s ceiling
(`arm.speed_max_mm_s`) is the FINE-POSITIONING cap: it exists so an approach
under the objective is not a lunge. This script's transit legs -- floor pick to
standoff and back -- run at 100 mm/s and are nowhere near the lens, so they are
checked against `arm.transit_speed_max_mm_s` instead, carried automatically by
Arm.free_envelope() (see cmd_run). Neither cap was raised to make the other
pass; they answer different questions and are both explicit in config.yaml.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import argparse
import datetime as dt
import json
import subprocess
import sys
import time
import urllib.request
from collections.abc import Callable
from pathlib import Path
from typing import Any

from tools import config as _config
from tools.arm import Arm, ArmSettings, SafetyError
from tools.arm.approach import LENS_XY_RADIUS_MM
from tools.arm.driver import ArmError
from tools.arm.geometry import WELL_PITCH_MM
from tools.arm.workspace import WorkspaceStore

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

SDL_LAB_DIR = Path.home() / ".sdl_lab" / "robot_arm"
PLATE_CONFIG_PATH = SDL_LAB_DIR / "plate_config.json"
ROUTES_PATH = SDL_LAB_DIR / "routes.json"

# Host, gripper speeds, workspace-store path, and the global speed cap now
# come from tools.arm.ArmSettings (config.yaml > arm.json > code
# default) -- see cmd_run() -- rather than a direct robot.json read. The old
# ROBOT_JSON_PATH constant (-> src/tools/robot_arm/robot.json) is gone;
# the file it pointed at moved and was renamed to
# src/tools/devices/arm/arm.json, and ArmSettings already knows that path.

# Live-session stream frame (written continuously by the microscope live monitor).
# We grab THIS frame at each well instead of opening the camera directly, so we
# never contend with the live view for the single-client Leica K3C.
LIVE_STREAM_LEICA = Path("/tmp/sdl_microscope_stream/current_leica_k3c.jpg")
PLATE_OUTPUT_DIR = Path("/home/lamp/SDLsetup/dataset/captures")
SIGNAL_MIN_MEAN = 40.0


# ---------------------------------------------------------------------------
# (0) Display client — best-effort ops feed
# ---------------------------------------------------------------------------

class DisplayClient:
    """Best-effort display ops client. POSTs JSON events to <url>/ops.
    All exceptions are swallowed; the run is never affected by display failures."""

    def __init__(self, url: str, source: str = "microscope", enabled: bool = True) -> None:
        self.url     = url.rstrip("/")
        self.source  = source
        self.enabled = enabled

    def emit(self, message: str, level: str = "info") -> None:
        """POST {"source":..., "message":..., "level":...} to url/ops. Best-effort."""
        if not self.enabled:
            return
        try:
            payload = json.dumps(
                {"source": self.source, "message": message, "level": level}
            ).encode()
            req = urllib.request.Request(
                f"{self.url}/ops",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            urllib.request.urlopen(req, timeout=2)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# (1) PlateConfig — persisted configuration
# ---------------------------------------------------------------------------

class PlateConfig:
    """
    Persisted imaging geometry and speed configuration.
    Backed by ~/.sdl_lab/robot_arm/plate_config.json; created with defaults on
    first run if absent.
    """

    DEFAULTS: dict[str, Any] = {
        "well_pitch_mm": WELL_PITCH_MM,  # tools.arm.geometry, = 9.0
        "z_standoff_mm": 200.0,
        "speeds": {
            "transit_joint_deg_s": 20,
            "hold_joint_deg_s":    10,
            # Checked against arm.transit_speed_max_mm_s (config.yaml, = 100.0),
            # not the tighter fine-positioning arm.speed_max_mm_s (30.0) -- this
            # script's cart moves run inside a single Envelope.free() built with
            # the transit cap for the whole run. See the module docstring.
            "transit_cart_mm_s":   100,
            "hold_cart_mm_s":      30,
            "hold_z_mm_s":         20,
            "well_cart_mm_s":      10,   # very slow: A1<->well moves under the scope
        },
        "axis_map": {
            "col_axis":   "x",
            "col_sign":   1,
            "row_axis":   "y",
            "row_sign":   1,
            "calibrated": False,
        },
    }

    def __init__(self) -> None:
        self.well_pitch_mm: float = self.DEFAULTS["well_pitch_mm"]
        self.z_standoff_mm: float = self.DEFAULTS["z_standoff_mm"]
        self.speeds: dict[str, float] = dict(self.DEFAULTS["speeds"])
        self.axis_map: dict[str, Any] = dict(self.DEFAULTS["axis_map"])
        self.load()

    def load(self) -> None:
        if PLATE_CONFIG_PATH.exists():
            try:
                raw = json.loads(PLATE_CONFIG_PATH.read_text())
                self.well_pitch_mm = float(
                    raw.get("well_pitch_mm", self.DEFAULTS["well_pitch_mm"])
                )
                self.z_standoff_mm = float(
                    raw.get("z_standoff_mm", self.DEFAULTS["z_standoff_mm"])
                )
                speeds_raw = raw.get("speeds", {})
                for k, v in self.DEFAULTS["speeds"].items():
                    self.speeds[k] = float(speeds_raw.get(k, v))
                axis_raw = raw.get("axis_map", {})
                for k, v in self.DEFAULTS["axis_map"].items():
                    self.axis_map[k] = axis_raw.get(k, v)
            except Exception as exc:
                # FAIL CLOSED (2026-08-06). This used to warn, fall back to
                # DEFAULTS and then self.save() -- writing the defaults over the
                # calibrated file. DEFAULTS carries col_sign=+1; the calibrated
                # file on this rig carries col_sign=-1. So a transient read error
                # silently MIRRORED the column axis for all 96 wells and made the
                # mirror permanent, with a warning line as the only trace.
                # An axis map is a measurement, not a preference: if it cannot be
                # read, there is no safe default to substitute.
                raise SafetyError(
                    "plate config at %s exists but could not be read (%s). "
                    "Refusing to fall back to built-in defaults: DEFAULTS has "
                    "col_sign=%+d and a calibrated file may have the opposite, "
                    "which would mirror every well. Fix or restore the file."
                    % (PLATE_CONFIG_PATH, exc, self.DEFAULTS["axis_map"]["col_sign"])
                ) from exc
        else:
            self.save()

    def save(self) -> None:
        SDL_LAB_DIR.mkdir(parents=True, exist_ok=True)
        data: dict[str, Any] = {
            "well_pitch_mm": self.well_pitch_mm,
            "z_standoff_mm": self.z_standoff_mm,
            "speeds":        dict(self.speeds),
            "axis_map":      dict(self.axis_map),
        }
        PLATE_CONFIG_PATH.write_text(json.dumps(data, indent=2))

    def __repr__(self) -> str:
        return (
            f"PlateConfig(\n"
            f"  well_pitch_mm = {self.well_pitch_mm},\n"
            f"  z_standoff_mm = {self.z_standoff_mm},\n"
            f"  speeds        = {self.speeds},\n"
            f"  axis_map      = {self.axis_map}\n"
            f")"
        )


# ---------------------------------------------------------------------------
# (2) RouteStore — persisted, perpetually-fixed routes
# ---------------------------------------------------------------------------

Waypoint = dict[str, Any]
Route = list[Waypoint]


class RouteStore:
    """
    Routes between positions are FIXED PERPETUALLY: computed once, saved, and
    replayed identically on every later run.
    Use --rebuild-routes to force recomputation for any route touched this run.
    Backed by ~/.sdl_lab/robot_arm/routes.json.
    """

    def __init__(self, rebuild: bool = False) -> None:
        self._rebuild = rebuild
        self._data: dict[str, Route] = {}
        self._load()

    def _load(self) -> None:
        if ROUTES_PATH.exists():
            try:
                self._data = json.loads(ROUTES_PATH.read_text())
            except Exception as exc:
                print(f"[routes] Warning: could not load {ROUTES_PATH}: {exc}", flush=True)
                self._data = {}

    def _save(self) -> None:
        SDL_LAB_DIR.mkdir(parents=True, exist_ok=True)
        ROUTES_PATH.write_text(json.dumps(self._data, indent=2))

    def get_or_build(self, key: str, builder_fn: Callable[[], Route]) -> Route:
        """
        Return the stored route for key, or call builder_fn(), save, and return.
        If --rebuild-routes was set, always recompute and overwrite.
        """
        if key in self._data and not self._rebuild:
            n = len(self._data[key])
            print(f"[routes] Using saved route '{key}' ({n} waypoints)", flush=True)
            return self._data[key]
        print(f"[routes] Building route '{key}'...", flush=True)
        route = builder_fn()
        self._data[key] = route
        self._save()
        print(f"[routes] Saved route '{key}' ({len(route)} waypoints)", flush=True)
        return route

    def keys(self) -> list[str]:
        return list(self._data.keys())


# ---------------------------------------------------------------------------
# (3) Well addressing
# ---------------------------------------------------------------------------

def well_offset(
    well_str: str,
    a1_pose: list[float],
    cfg: PlateConfig,
) -> list[float]:
    """
    Return the Cartesian pose for a named well relative to the A1 (microscope) pose.

    Args:
        well_str: e.g. "C9" -- row letter A-H, column number 1-12.
        a1_pose:  [x, y, z, roll, pitch, yaw] of well A1.
        cfg:      PlateConfig with well_pitch_mm and axis_map.

    Returns:
        New pose list with x/y offset applied; z, roll, pitch, yaw unchanged.

    Raises:
        ValueError: on any invalid well string.
    """
    if not well_str or len(well_str) < 2:
        raise ValueError(f"Invalid well string: {well_str!r}")

    row_char = well_str[0].upper()
    col_str  = well_str[1:]

    if row_char not in "ABCDEFGH":
        raise ValueError(
            f"Invalid row '{row_char}' in well '{well_str}': must be A-H"
        )
    try:
        col_num = int(col_str)
    except ValueError as exc:
        raise ValueError(
            f"Invalid column '{col_str}' in well '{well_str}': must be an integer 1-12"
        ) from exc
    if col_num < 1 or col_num > 12:
        raise ValueError(
            f"Column {col_num} out of range in well '{well_str}': must be 1-12"
        )

    row_idx = ord(row_char) - ord("A")  # 0-7
    col_idx = col_num - 1               # 0-11

    col_axis = cfg.axis_map["col_axis"]   # "x" or "y"
    col_sign = cfg.axis_map["col_sign"]   # +1 or -1
    row_axis = cfg.axis_map["row_axis"]   # "x" or "y"
    row_sign = cfg.axis_map["row_sign"]   # +1 or -1
    pitch    = cfg.well_pitch_mm

    col_delta = col_sign * col_idx * pitch
    row_delta = row_sign * row_idx * pitch

    axis_idx = {"x": 0, "y": 1}
    pose = list(a1_pose)
    pose[axis_idx[col_axis]] += col_delta
    pose[axis_idx[row_axis]] += row_delta
    return pose


# ---------------------------------------------------------------------------
# (4) Route builders
# ---------------------------------------------------------------------------

def point_route(pose: list[float], label: str) -> Route:
    """Single-waypoint lateral Cartesian route. z_descend=False always."""
    return [{"kind": "cart", "pose": list(pose), "z_descend": False, "label": label}]


def floor_to_standoff_route(
    floor_pose: list[float],
    standoff_pose: list[float],
) -> Route:
    """
    Safe transit from floor pick pose to scope_standoff without crossing over A1.

    Waypoint 1 -- ascend at floor XY: straight-up move at the floor's XY/orientation
                  to the scope working Z (standoff_pose[2]). Keeps the arm outside
                  the scope footprint during ascent.
    Waypoint 2 -- lateral to standoff: move at fixed scope Z from floor XY to the
                  scope_standoff pose (its own XY and orientation).
    """
    ascend = [
        floor_pose[0], floor_pose[1], standoff_pose[2],
        floor_pose[3], floor_pose[4], floor_pose[5],
    ]
    return [
        {
            "kind":      "cart",
            "pose":      ascend,
            "z_descend": False,
            "label":     "ascend at floor to scope Z",
        },
        {
            "kind":      "cart",
            "pose":      list(standoff_pose),
            "z_descend": False,
            "label":     "lateral to scope standoff",
        },
    ]


def approach_waypoints(
    target_pose: list[float],
    cfg: PlateConfig,
    name: str,
) -> Route:
    """
    Build a 2-waypoint Z-decoupled approach route to target_pose.

    Waypoint 1 -- standoff: XY over target at (target_z + standoff), sets target
                             orientation at safe height. z_descend=False.
    Waypoint 2 -- descend:  straight-down Z move to working target_z. z_descend=True.

    IMPORTANT: The executor MUST call ensure_safe_z(target_z) IMMEDIATELY BEFORE
    replaying this route to ascend from current position to safe height in place.
    That step cannot be stored because it depends on the live current pose.
    """
    tx, ty, tz, t_roll, t_pitch, t_yaw = target_pose
    standoff_z = tz + cfg.z_standoff_mm
    return [
        {
            "kind":      "cart",
            "pose":      [tx, ty, standoff_z, t_roll, t_pitch, t_yaw],
            "z_descend": False,
            "label":     (
                f"{name} standoff "
                f"(xy over target, safe Z={standoff_z:.1f}mm)"
            ),
        },
        {
            "kind":      "cart",
            "pose":      [tx, ty, tz, t_roll, t_pitch, t_yaw],
            "z_descend": True,
            "label":     f"{name} descend to working Z={tz:.1f}mm",
        },
    ]


def home_route() -> Route:
    """
    Single-waypoint route: joint move to zero (home) configuration.
    The executor MUST call ensure_safe_z before replaying this route.
    """
    return [
        {
            "kind":      "joint",
            "angles":    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "z_descend": False,
            "label":     "home (zero joint angles)",
        }
    ]


def flat_well_route(well_pose: list[float], well_name: str) -> Route:
    """
    Single-waypoint flat Cartesian route for coplanar well-to-well moves.
    No standoff -- plate stays at A1's Z; only XY changes. (Direct diagonal.)
    """
    return [
        {
            "kind":      "cart",
            "pose":      list(well_pose),
            "z_descend": False,
            "label":     f"flat XY move to well {well_name}",
        }
    ]


def decomposed_well_route(a1_pose: list[float], well_pose: list[float],
                          well_name: str, forward: bool = True) -> Route:
    """
    Grid-aligned (axis-by-axis) well move at fixed Z -- NOT a diagonal.
    Forward (A1 -> well): move along columns (X) first, then rows (Y).
    Reverse (well -> A1): move along rows (Y) first, then columns (X).
    Both share the corner waypoint (well.x, A1.y), so the reverse exactly
    retraces the forward path.
    """
    z = well_pose[2]
    orient = well_pose[3:6]
    corner = [well_pose[0], a1_pose[1], z, *orient]   # (well X, A1 Y)
    if forward:
        return [
            {"kind": "cart", "pose": corner,            "z_descend": False,
             "label": f"to {well_name}: along columns (X)"},
            {"kind": "cart", "pose": list(well_pose),   "z_descend": False,
             "label": f"to {well_name}: along rows (Y)"},
        ]
    return [
        {"kind": "cart", "pose": corner,           "z_descend": False,
         "label": f"from {well_name}: along rows (Y)"},
        {"kind": "cart", "pose": list(a1_pose),    "z_descend": False,
         "label": f"from {well_name}: along columns (X) -> A1"},
    ]


# ---------------------------------------------------------------------------
# (5) Executor
# ---------------------------------------------------------------------------
# Host, gripper speeds, and BIO-gripper diagnostics all now come from the
# shared tools.arm.Arm session -- load_robot_json() and the old
# _bio_gripper_err() helper are gone, superseded by ArmSettings and
# tools.arm.driver's own bio_error().

class Executor:
    """
    Drives the xArm7 through routes and actions via the shared tools.arm.Arm.

    Dry-run mode (execute=False): prints every planned waypoint with label,
    kind, full pose/angles, and the speed that WOULD be used -- no connection.
    Live mode (execute=True): connects, enables motion, and every move is
    validated against `robot`'s Envelope.free() workspace corridor and re-checked
    for a controller error code after sending; aborts (SystemExit) on any guard
    failure (SafetyError) or controller failure (ArmError).

    State:
        holding (bool): True after grab(); selects HOLD speeds for all subsequent moves.
    """

    def __init__(
        self,
        cfg: PlateConfig,
        robot: Arm,
        execute: bool = False,
        display: DisplayClient | None = None,
    ) -> None:
        self.cfg     = cfg
        self.robot   = robot
        self.execute = execute
        self.display = display
        self.holding = False

    def _op(self, msg: str, level: str = "info") -> None:
        """Emit a display op (best-effort). Does not print anything extra."""
        if self.display is not None:
            self.display.emit(msg, level)

    # -- Connection ----------------------------------------------------------

    def connect(self) -> None:
        """Open the shared Arm session's socket and enable motion (live only).

        `robot` (an tools.arm.Arm) was already built in cmd_run with
        host/live/clear_errors resolved via ArmSettings; this just performs the
        actual connect + enable step eagerly, at the same point in the run the
        old script did, instead of lazily on first move.
        """
        print(f"[arm] Connecting to xArm7 at {self.robot.settings.host}...", flush=True)
        try:
            self.robot.connection.connect()
        except ArmError as exc:
            raise SystemExit(f"[arm] {exc}") from exc
        print("[arm] Connected and motion enabled.", flush=True)
        self._op("connected")

    def disconnect(self) -> None:
        self.robot.close()
        print("[arm] Disconnected.", flush=True)

    # -- Speed selection -----------------------------------------------------

    def _speed_for(self, z_descend: bool = False) -> dict[str, float]:
        """
        Return {"joint": deg/s, "cart": mm/s} based on holding state and move type.
        z_descend=True uses hold_z_mm_s (extra-slow) when holding.
        """
        s = self.cfg.speeds
        if self.holding:
            cart_spd = s["hold_z_mm_s"] if z_descend else s["hold_cart_mm_s"]
            return {"joint": s["hold_joint_deg_s"], "cart": cart_spd}
        return {"joint": s["transit_joint_deg_s"], "cart": s["transit_cart_mm_s"]}

    # -- Core move primitives ------------------------------------------------

    def _lens_xy(self):
        """A1's XY, or None if this executor has no taught A1 to compare against.

        Returns None rather than a default: a guard that invents an anchor is
        worse than one that says it cannot check. The caller treats None as
        "unknown", and the surrounding code path is unchanged in that case.
        """
        try:
            store = WorkspaceStore(self.robot.settings.workspace_path)
            pose = store.require_location("microscope").pose
            return (float(pose.x), float(pose.y))
        except Exception:
            return None

    def ensure_safe_z(self, target_z: float) -> None:
        """
        Ascend in place to max(current_z, target_z + standoff_mm) before an approach.
        Uses the current XY position and orientation -- does NOT change XY.

        This is an executor action, NOT stored in any route, because it depends on
        the live current pose which is unknowable at route-build time. The target
        pose is sent through robot.envelope.validate() + robot.connection.send(),
        the same guarded path move_cart() uses below.
        """
        standoff   = self.cfg.z_standoff_mm
        min_safe_z = target_z + standoff
        label = (
            f"ensure_safe_z: ascend to max(current_z, {min_safe_z:.1f}mm)  "
            f"[standoff={standoff:.0f}mm above target_z={target_z:.1f}mm]"
        )
        speeds = self._speed_for(z_descend=False)

        if not self.execute:
            print(
                f"  [dry-run] {label}  "
                f"speed={speeds['cart']:.1f}mm/s  holding={self.holding}",
                flush=True,
            )
            return

        cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw = self.robot.pose()

        # GUARD (2026-08-06). This routine ascends IN PLACE. Under the objective
        # that is a collision, and the module docstring has always said so:
        #   "NEVER call ensure_safe_z above A1 -- that would ascend through the
        #    lens."
        # It was a comment, not a check, while the only caller (step 1 of the
        # imaging loop) invokes it at whatever XY the arm is at -- and
        # find_target, calibration, sharpness_test, goto_a1 and pick_place all
        # leave the arm parked at A1. Refuse instead of trusting the caller.
        _a1 = self._lens_xy()
        if _a1 is not None:
            _dx, _dy = cur_x - _a1[0], cur_y - _a1[1]
            _r = (_dx * _dx + _dy * _dy) ** 0.5
            if _r <= LENS_XY_RADIUS_MM and min_safe_z > cur_z:
                raise SafetyError(
                    "ensure_safe_z refused: the arm is %.1f mm from A1's XY "
                    "(inside the %.0f mm lens radius) and this would ascend in "
                    "place from z=%.3f to z=%.3f -- %.1f mm TOWARD the fixed "
                    "objective. Move clear of the lens laterally first."
                    % (_r, LENS_XY_RADIUS_MM, cur_z, min_safe_z, min_safe_z - cur_z))

        safe_z = max(cur_z, min_safe_z)
        print(
            f"  [arm] {label}  "
            f"cur_z={cur_z:.1f} -> safe_z={safe_z:.1f}  "
            f"speed={speeds['cart']:.1f}mm/s",
            flush=True,
        )
        target = (cur_x, cur_y, safe_z, cur_roll, cur_pitch, cur_yaw)
        try:
            # move_pose(), not connection.send(): it carries the pre-first-motion
            # check (where is the arm actually?) and the post-move readback that
            # a direct send would skip.
            self.robot.move_pose(target, speed=speeds["cart"], label=label,
                                 announce=False)
        except (SafetyError, ArmError) as exc:
            raise SystemExit(f"[arm] ensure_safe_z failed: {exc}") from exc

    def move_cart(
        self,
        pose: list[float],
        speed: float,
        z_descend: bool = False,
        label: str = "",
    ) -> None:
        """Cartesian linear move through the shared guarded path.

        Sends the full taught 6-DOF pose as-is -- this rig's routes carry their
        own orientation per waypoint rather than inheriting one from a single
        anchor, which is what Arm.move_pose() exists for. It raises SafetyError
        for a guard violation (pose shape, the Z corridor, the speed cap, or the
        arm not being where it should be before the first command) and ArmError
        for a controller-reported failure; both become SystemExit here, matching
        the old set_position()-return-code check.
        """
        x, y, z, roll, pitch, yaw = pose
        tag    = "arm" if self.execute else "dry-run"
        suffix = " (z-descend)" if z_descend else ""
        print(
            f"  [{tag}] cart{suffix}: "
            f"[{x:.2f}, {y:.2f}, {z:.2f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}]  "
            f"speed={speed:.1f}mm/s  | {label}",
            flush=True,
        )
        try:
            self.robot.move_pose(pose, speed=speed, label=label, announce=False)
        except (SafetyError, ArmError) as exc:
            raise SystemExit(f"[arm] cart move failed: {exc}") from exc

    def move_joint(
        self,
        angles: list[float],
        speed: float,
        label: str = "",
    ) -> None:
        """Joint-space move via robot.move_joints() -- taught angles only, never
        computed from a pose (see home_route(), the only joint-kind waypoint)."""
        tag = "arm" if self.execute else "dry-run"
        print(
            f"  [{tag}] joint: {angles}  speed={speed:.1f}deg/s  | {label}",
            flush=True,
        )
        try:
            self.robot.move_joints(angles, speed_deg_s=speed, label=label)
        except ArmError as exc:
            raise SystemExit(f"[arm] joint move failed: {exc}") from exc

    # -- Route executor ------------------------------------------------------

    def execute_route(
        self,
        route: Route,
        pre_ensure_z: float | None = None,
        cart_speed_override: float | None = None,
    ) -> None:
        """
        Replay a saved route waypoint by waypoint.

        If pre_ensure_z is provided, call ensure_safe_z(pre_ensure_z) first.
        Required before any approach route to guarantee safe height clearance.

        cart_speed_override (mm/s), when set, forces that speed for every cart
        waypoint in this route (used for the very-slow A1<->well moves). Speed is
        a run parameter, not route geometry, so this does NOT alter saved routes.
        """
        if pre_ensure_z is not None:
            self.ensure_safe_z(pre_ensure_z)

        for wp in route:
            z_descend = wp.get("z_descend", False)
            speeds    = self._speed_for(z_descend=z_descend)
            label     = wp.get("label", "")
            kind      = wp["kind"]

            if kind == "cart":
                cart_speed = (
                    cart_speed_override
                    if cart_speed_override is not None
                    else speeds["cart"]
                )
                self.move_cart(
                    wp["pose"],
                    speed=cart_speed,
                    z_descend=z_descend,
                    label=label,
                )
            elif kind == "joint":
                self.move_joint(
                    wp["angles"],
                    speed=speeds["joint"],
                    label=label,
                )
            else:
                raise SystemExit(f"[arm] Unknown waypoint kind: {kind!r}")

    # -- Gripper -------------------------------------------------------------
    # Jaw-only actuation, always allowed (established operating policy) -- goes
    # through robot.grip() / robot.release() (tools.arm), which own
    # the BIO Gripper G2 enable/mode/speed/error-check sequence. Dry-run prints
    # a summary line instead of calling them, so dry-run never touches the
    # shared Arm session at all (matches the file's Dry-Run Safety contract).

    def grab(self) -> None:
        """
        Close the UFACTORY BIO Gripper G2 to hold the plate.

        Uses a SLOW close speed (ArmSettings.close_speed, from arm.json's
        gripper.close_speed) to protect the plate. This commands a **full
        close**: the plate stops the jaws at its own width (~85 mm, inside the
        gripper's 71-150 mm range) and the firmware holds it force-limited
        (<=20 N). That is a choice, not a limitation -- the plate's own width
        sets the span, so a setpoint would only add a way to get it wrong. The
        gripper *can* be driven to a specific span; see ``arm.set_opening`` and
        ``gripper.control_mode`` in ``tools/arm/arm.json``.
        Sets self.holding = True on success (and in dry-run).
        """
        close_speed = self.robot.settings.close_speed

        print(
            f"[plate] grab(): closing BIO gripper  close_speed={close_speed}",
            flush=True,
        )

        if not self.execute:
            print(
                f"  [dry-run] BIO gripper close via the shared Arm session "
                f"(enable, control_mode={self.robot.settings.control_mode}, "
                f"speed={close_speed}, close, wait=True)",
                flush=True,
            )
            self.holding = True
            print("[plate] grab(): holding=True (dry-run)", flush=True)
            self._op("grabbed plate")
            return

        try:
            self.robot.grip()
        except ArmError as exc:
            raise SystemExit(f"[arm] grab() failed: {exc}") from exc

        self.holding = True
        print("[plate] grab(): holding=True", flush=True)
        self._op("grabbed plate")

    def open_gripper(self) -> None:
        """
        Open the BIO Gripper G2 (prepare to pick / release plate).
        Sets self.holding = False. Called at the zero/home reset so the gripper is
        ready to receive the plate, and at release to set the plate back down.

        A full open, to the 150 mm end of the jaw travel.
        """
        open_speed = self.robot.settings.open_speed

        print(f"[plate] open_gripper(): opening BIO gripper  open_speed={open_speed}", flush=True)

        if not self.execute:
            print(
                f"  [dry-run] BIO gripper open via the shared Arm session "
                f"(enable, control_mode={self.robot.settings.control_mode}, "
                f"speed={open_speed}, open, wait=True)",
                flush=True,
            )
            self.holding = False
            print("[plate] open_gripper(): holding=False (dry-run)", flush=True)
            return

        try:
            self.robot.release()
        except ArmError as exc:
            raise SystemExit(f"[arm] open_gripper() failed: {exc}") from exc

        self.holding = False
        print("[plate] open_gripper(): done, holding=False", flush=True)

    # -- Camera capture + verify ---------------------------------------------

    def capture_and_verify(self, well_name: str, batch_id: str) -> dict[str, Any]:
        """
        Save a microscope image of the current well and check it passed the signal
        threshold. WARN AND CONTINUE on failure (never abort).

        Primary path: grab the live-session's CURRENT Leica frame (a file copy --
        no camera open), so it never contends with the running live view for the
        single-client Leica K3C. Waits for a frame that post-dates arrival so it
        shows the current well. Falls back to a direct camera capture ONLY when the
        live stream is stale/absent (in which case nothing else holds the camera).
        """
        out = PLATE_OUTPUT_DIR / f"plate_{well_name}_{batch_id}_leica_k3c.jpg"

        if not self.execute:
            print(
                f"[scope] dry-run: would save live frame for well {well_name} -> {out}",
                flush=True,
            )
            self._op(f"dry-run capture {well_name} (skipped)")
            return {"ok": None, "dry_run": True}

        res = self._capture_from_live_frame(well_name, out)
        if res is not None:
            return res
        print(
            f"[scope] live frame unavailable for {well_name}; falling back to direct capture",
            flush=True,
        )
        return self._capture_direct(well_name, batch_id)

    def _probe_mean(self, path: Path) -> float | None:
        """Best-effort mean-brightness probe of an image (the 'good photo' signal)."""
        try:
            p = subprocess.run(
                [sys.executable, "-m", "tools.microscope", "probe-image", str(path)],
                capture_output=True, text=True, timeout=30,
            )
            d = json.loads(p.stdout)
            m = d.get("mean")
            return float(m) if m is not None else None
        except Exception:
            return None

    def _capture_from_live_frame(self, well_name: str, out: Path) -> dict[str, Any] | None:
        """Copy the live-session's current Leica frame (post-arrival) and verify.
        Returns None (=> caller falls back to direct capture) if the stream is stale."""
        import shutil
        src = LIVE_STREAM_LEICA
        t0 = time.time()
        deadline = t0 + 8.0
        fresh = False
        while time.time() < deadline:
            try:
                if src.stat().st_mtime > t0 + 0.2:  # a frame captured AFTER we arrived
                    fresh = True
                    break
            except FileNotFoundError:
                pass
            time.sleep(0.3)
        if not fresh:
            return None  # live stream stale/absent -> let caller fall back

        try:
            shutil.copyfile(src, out)
        except Exception as exc:
            print(f"[scope] WARNING: copy live frame {well_name} failed "
                  f"(continuing): {exc}", flush=True)
            self._op(f"capture {well_name} live-frame copy error: {exc}", level="warn")
            return {"ok": False, "output": None, "source": "live_frame"}

        size = out.stat().st_size if out.exists() else 0
        mean = self._probe_mean(out)
        ok = (mean >= SIGNAL_MIN_MEAN) if mean is not None else (size > 20000)

        if ok:
            print(f"[scope] captured {well_name} (live frame): {out}  size={size} "
                  f"mean={mean}", flush=True)
            self._op(f"captured {well_name} (live frame): mean={mean}")
        else:
            print(
                f"[scope] WARNING: {well_name} live frame failed signal check "
                f"(continuing): size={size} mean={mean}",
                flush=True,
            )
            self._op(f"capture {well_name} FAILED signal: size={size} mean={mean}", level="warn")
        return {"ok": ok, "output": str(out), "mean": mean, "source": "live_frame"}

    def _capture_direct(self, well_name: str, batch_id: str) -> dict[str, Any]:
        """Direct camera capture (opens the Leica). Used only when no live stream
        is present, so there is no contention."""
        cmd = [
            sys.executable, "-m", "tools.microscope",
            "capture", "--camera", "leica_k3c",
            "--batch-id", f"plate_{well_name}_{batch_id}",
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        except Exception as exc:
            print(f"[scope] WARNING: capture {well_name} subprocess error "
                  f"(continuing): {exc}", flush=True)
            self._op(f"capture {well_name} subprocess error: {exc}", level="warn")
            return {"ok": False, "output": None}

        stdout = proc.stdout or ""
        parsed: dict[str, Any] | None = None
        try:
            parsed = json.loads(stdout)
        except Exception:
            start, end = stdout.find("{"), stdout.rfind("}")
            if start != -1 and end != -1 and end > start:
                try:
                    parsed = json.loads(stdout[start:end + 1])
                except Exception:
                    parsed = None

        ok = (proc.returncode == 0) and bool(parsed and parsed.get("ok"))
        if ok:
            output_path = parsed.get("output") if parsed else None
            print(f"[scope] captured {well_name} (direct): {output_path}", flush=True)
            self._op(f"captured {well_name} (direct): {output_path}")
        else:
            msg = parsed.get("message") if parsed else None
            print(
                f"[scope] WARNING: capture {well_name} did NOT pass signal check "
                f"(continuing): rc={proc.returncode} msg={msg}",
                flush=True,
            )
            self._op(f"capture {well_name} FAILED signal check: rc={proc.returncode}", level="warn")
        return parsed if parsed is not None else {"ok": ok, "output": None}


# ---------------------------------------------------------------------------
# (7) Main run subcommand
# ---------------------------------------------------------------------------

def ensure_display_and_browser(display_url: str, port: int) -> None:
    """Best-effort: ensure the lab dashboard is running on the server, then open
    a browser ON THE SERVER (WSL->Windows interop) pointing at it.

    Idempotent (skips launch if already up) and fully non-blocking: any failure
    is logged and ignored so it can never affect the robot run.
    """
    import urllib.request

    repo_dir = _config.PROJECT_ROOT  # /home/lamp/SDLsetup
    status_url = f"{display_url.rstrip('/')}/api/status"

    def _is_up() -> bool:
        try:
            urllib.request.urlopen(status_url, timeout=1.5)
            return True
        except Exception:
            return False

    up = _is_up()
    if not up:
        log = "/tmp/sdl_lab_display.log"
        launch = (
            f"cd {repo_dir} && nohup setsid ~/.pyenv/bin/pyenv exec python "
            f"scripts/lab.py --port {port} > {log} 2>&1 < /dev/null &"
        )
        try:
            subprocess.Popen(["bash", "-lc", launch],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[display] starting dashboard on :{port} (log {log}) ...", flush=True)
        except Exception as exc:
            print(f"[display] could not start dashboard: {exc}", flush=True)
        for _ in range(20):  # up to ~10s for it to bind
            if _is_up():
                up = True
                break
            time.sleep(0.5)

    print(f"[display] dashboard {'up' if up else 'not confirmed'} at {display_url}", flush=True)

    # Open the Windows default browser on the server desktop. Try several openers.
    open_url = f"http://localhost:{port}/"
    for opener in (
        ["cmd.exe", "/c", "start", "", open_url],
        ["powershell.exe", "-NoProfile", "-Command", f"Start-Process '{open_url}'"],
        ["wslview", open_url],
    ):
        try:
            subprocess.run(opener, cwd="/mnt/c",
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            print(f"[display] opened browser ({opener[0]}) -> {open_url}", flush=True)
            return
        except Exception:
            continue
    print("[display] could not open a server-side browser (continuing)", flush=True)


def cmd_run(args: argparse.Namespace) -> None:
    cfg      = PlateConfig()
    routes   = RouteStore(rebuild=args.rebuild_routes)
    well_str = args.well.upper()
    execute  = args.execute

    batch_id = args.batch_id or dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    # -- Build display client ------------------------------------------------
    display = DisplayClient(
        args.display_url,
        source="microscope",
        enabled=not args.no_display,
    )

    # -- Every run: bring up the dashboard + open a browser on the server -----
    if not args.no_display:
        from urllib.parse import urlparse as _urlparse
        _port = _urlparse(args.display_url).port or 8770
        ensure_display_and_browser(args.display_url, _port)

    # -- Arm settings (host, live gate, clear-errors, gripper speeds, the ----
    # -- global speed cap, and the taught-workspace path) resolved once via --
    # -- config.yaml > arm.json > code default. --execute drives `live` -----
    # -- directly (this script's own dry-run/--execute contract), not the ---
    # -- config.yaml allow_motion precedence. --------------------------------
    arm_overrides: dict[str, Any] = {"live": execute, "clear_errors": args.clear_errors}
    if args.host:
        arm_overrides["host"] = args.host
    settings = ArmSettings.from_config(**arm_overrides)

    # -- Load workspace locations --------------------------------------------
    store         = WorkspaceStore(settings.workspace_path)
    home_loc      = store.require_location("home")
    floor_loc     = store.require_location("floor")
    micro_loc     = store.require_location("microscope")
    standoff_loc  = store.require_location("scope_standoff")

    home_pose: list[float] = [
        home_loc.pose.x,  home_loc.pose.y,  home_loc.pose.z,
        home_loc.pose.roll, home_loc.pose.pitch, home_loc.pose.yaw,
    ]
    floor_pose: list[float] = [
        floor_loc.pose.x,  floor_loc.pose.y,  floor_loc.pose.z,
        floor_loc.pose.roll, floor_loc.pose.pitch, floor_loc.pose.yaw,
    ]
    a1_pose: list[float] = [
        micro_loc.pose.x,  micro_loc.pose.y,  micro_loc.pose.z,
        micro_loc.pose.roll, micro_loc.pose.pitch, micro_loc.pose.yaw,
    ]
    standoff_pose: list[float] = [
        standoff_loc.pose.x,  standoff_loc.pose.y,  standoff_loc.pose.z,
        standoff_loc.pose.roll, standoff_loc.pose.pitch, standoff_loc.pose.yaw,
    ]

    # -- Validate well string ------------------------------------------------
    try:
        well_pose = well_offset(well_str, a1_pose, cfg)
    except ValueError as exc:
        raise SystemExit(f"[plate] Bad --well argument: {exc}") from exc

    # -- Calibration warning (no gate; axis_map should already be calibrated) -
    if not bool(cfg.axis_map.get("calibrated", False)):
        print(
            "[plate] WARNING: axis_map.calibrated=False. Well offsets may be wrong.\n"
            "  Run `plate_imaging.py calibrate --set ...` to set and confirm axis mapping.\n"
            "  Proceeding anyway.",
            flush=True,
        )

    well_speed = cfg.speeds["well_cart_mm_s"]   # very slow near/under the scope

    # -- Free-workspace envelope ----------------------------------------------
    # plate_imaging genuinely crosses the workspace (floor pick <-> scope), not
    # a single taught anchor, so this uses Arm.free_envelope() -- an explicit Z
    # corridor, not the default anchored A1 box -- per the migration brief. It
    # also carries the TRANSIT speed cap (arm.transit_speed_max_mm_s, = 100.0),
    # not the tighter fine-positioning one (arm.speed_max_mm_s = 30.0): that one
    # exists so an approach under the objective is not a lunge, and this run's
    # floor<->standoff legs are nowhere near the lens. Both caps are explicit in
    # config.yaml under `arm:`; free_envelope() applies the transit cap itself.
    # The Z corridor is derived from the four taught locations actually used
    # below plus z_standoff_mm (the highest ensure_safe_z ever intentionally
    # targets) -- not an invented number.
    known_zs = [home_pose[2], floor_pose[2], a1_pose[2], standoff_pose[2]]
    robot = Arm(settings, log=lambda msg: print(msg, flush=True)).free_envelope(
        reason=(
            "plate_imaging transports the plate between home, floor pick, "
            "scope_standoff, and A1/well positions across the workspace; "
            "orientation is fixed throughout (translation only)"
        ),
        z_floor_mm=min(known_zs),
        z_ceiling_mm=max(known_zs) + cfg.z_standoff_mm,
    )

    # -- Build / load routes (perpetual) ------------------------------------

    # "anywhere->home": single joint waypoint. ensure_safe_z called by executor.
    route_home = routes.get_or_build("anywhere->home", home_route)

    # "home->floor": Z-decoupled approach to floor pick pose.
    route_to_floor = routes.get_or_build(
        "home->floor",
        lambda: approach_waypoints(floor_pose, cfg, "floor"),
    )

    # "floor->scope_standoff": ascend at floor XY to scope Z, lateral to standoff.
    route_floor_to_standoff = routes.get_or_build(
        "floor->scope_standoff",
        lambda: floor_to_standoff_route(floor_pose, standoff_pose),
    )

    # "scope_standoff->A1": lateral push-in at fixed Z (no descent -- lens above A1).
    route_pushin = routes.get_or_build(
        "scope_standoff->A1",
        lambda: point_route(a1_pose, "push-in to A1 (lateral, fixed Z)"),
    )

    # "A1->scope_standoff": lateral pull-out at fixed Z.
    route_pullout = routes.get_or_build(
        "A1->scope_standoff",
        lambda: point_route(standoff_pose, "pull-out to standoff (lateral, fixed Z)"),
    )

    # "A1->well:<WELL>": grid-aligned (columns then rows), NOT diagonal.
    route_to_well = routes.get_or_build(
        f"A1->well:{well_str}",
        lambda wp=well_pose, wn=well_str: decomposed_well_route(a1_pose, wp, wn, forward=True),
    )

    # "well:<WELL>->A1": grid-aligned reverse (rows then columns). Keyed per well
    # because the path's corner depends on the well.
    route_back_to_a1 = routes.get_or_build(
        f"well:{well_str}->A1",
        lambda wp=well_pose, wn=well_str: decomposed_well_route(a1_pose, wp, wn, forward=False),
    )

    # -- Executor ------------------------------------------------------------
    ex = Executor(cfg, robot, execute=execute, display=display)
    if not execute:
        print(
            "\n[plate] *** DRY-RUN MODE -- no hardware connection, no motion ***\n",
            flush=True,
        )

    if execute:
        ex.connect()

    try:
        # Step 1: Goto home
        print("\n[plate] === Step 1: Goto home ===", flush=True)
        ex._op("Step 1: Goto home")
        # home_pose[2] is typically 0; ensure_safe_z(0) => ascend to standoff height
        ex.ensure_safe_z(home_pose[2])
        ex.execute_route(route_home)

        # Step 1b: Open gripper at zero (reset; ready to receive the plate)
        print("\n[plate] === Step 1b: Open gripper (reset at zero) ===", flush=True)
        ex._op("Step 1b: Open gripper")
        ex.open_gripper()

        # Step 2: Approach floor pick pose
        print("\n[plate] === Step 2: Approach floor (pick pose) ===", flush=True)
        ex._op("Step 2: Approach floor")
        ex.execute_route(route_to_floor, pre_ensure_z=floor_pose[2])

        # Step 3: Grab plate
        print("\n[plate] === Step 3: Grab plate ===", flush=True)
        ex._op("Step 3: Grab plate")
        ex.grab()   # sets ex.holding = True

        # Step 4: Enter scope via lateral approach (NEVER vertical over A1)
        print("\n[plate] === Step 4: Enter scope (lateral approach via scope_standoff) ===",
              flush=True)
        ex._op("Step 4: Enter scope lateral approach")

        # 4a: Ascend at floor XY to scope Z, then lateral to standoff
        print("\n[plate] === Step 4a: Floor -> scope_standoff ===", flush=True)
        ex._op("Step 4a: floor->scope_standoff")
        ex.execute_route(
            route_floor_to_standoff,
            cart_speed_override=cfg.speeds["hold_cart_mm_s"],
        )

        # 4b: Lateral push-in from standoff to A1 at fixed Z (no descent)
        print("\n[plate] === Step 4b: Push in to A1 (lateral, fixed Z) ===", flush=True)
        ex._op("Step 4b: lateral push-in to A1")
        ex.execute_route(route_pushin, cart_speed_override=well_speed)

        # Step 5: Capture and verify at A1
        print("\n[plate] === Step 5: Capture and verify at A1 ===", flush=True)
        ex._op("Step 5: capture A1")
        ex.capture_and_verify("A1", batch_id)

        # Step 6: Goto target well (flat XY move, very slow)
        print(
            f"\n[plate] === Step 6: Goto well {well_str} "
            f"(flat XY move, very slow {well_speed:.0f}mm/s) ===",
            flush=True,
        )
        ex._op(f"Step 6: flat move to well {well_str}")
        ex.execute_route(route_to_well, cart_speed_override=well_speed)

        # Step 6b: Capture and verify at target well
        print(f"\n[plate] === Step 6b: Capture and verify at {well_str} ===", flush=True)
        ex._op(f"Step 6b: capture {well_str}")
        ex.capture_and_verify(well_str, batch_id)

        # Step 7: Exit scope via lateral retrace
        print("\n[plate] === Step 7: Exit scope (lateral retrace via scope_standoff) ===",
              flush=True)
        ex._op("Step 7: Exit scope lateral retrace")

        # 7a: Return from well to A1 (flat XY, very slow)
        print(
            f"\n[plate] === Step 7a: Return to A1 "
            f"(flat XY move, very slow {well_speed:.0f}mm/s) ===",
            flush=True,
        )
        ex._op("Step 7a: return well->A1")
        ex.execute_route(route_back_to_a1, cart_speed_override=well_speed)

        # 7b: Lateral pull-out from A1 to standoff at fixed Z
        print("\n[plate] === Step 7b: Pull out to scope_standoff (lateral, fixed Z) ===",
              flush=True)
        ex._op("Step 7b: lateral pull-out A1->standoff")
        ex.execute_route(route_pullout, cart_speed_override=well_speed)

        # Step 8: Return to floor with the plate (Z-decoupled approach, slow/holding)
        print(
            "\n[plate] === Step 8: Return to floor (put the plate back) ===",
            flush=True,
        )
        ex._op("Step 8: return to floor")
        ex.execute_route(route_to_floor, pre_ensure_z=floor_pose[2])

        # Step 9: Put the plate down (release at floor)
        print("\n[plate] === Step 9: Release plate at floor ===", flush=True)
        ex._op("Step 9: release plate at floor")
        ex.open_gripper()   # opens gripper, sets holding=False

        # Step 10: Return to zero/home (now empty -> transit speed)
        print("\n[plate] === Step 10: Return home (zero) ===", flush=True)
        ex._op("Step 10: return home")
        ex.execute_route(route_home, pre_ensure_z=home_pose[2])

        print(
            "\n[plate] Run complete. Plate returned to floor; arm at home (zero), gripper open.",
            flush=True,
        )
        ex._op("Run complete. Plate returned to floor; arm at home, gripper open.")

    finally:
        if execute:
            ex.disconnect()


# ---------------------------------------------------------------------------
# (8) Calibrate subcommand
# ---------------------------------------------------------------------------

def cmd_calibrate(args: argparse.Namespace) -> None:
    cfg = PlateConfig()

    if args.set:
        # Parse e.g. "col_axis=x,col_sign=1,row_axis=y,row_sign=-1"
        pairs      = [p.strip() for p in args.set.split(",") if p.strip()]
        valid_keys = {"col_axis", "col_sign", "row_axis", "row_sign"}
        updates: dict[str, Any] = {}

        for pair in pairs:
            if "=" not in pair:
                raise SystemExit(
                    f"[calibrate] Bad --set entry (expected key=value): {pair!r}"
                )
            k, v = pair.split("=", 1)
            k = k.strip()
            v = v.strip()
            if k not in valid_keys:
                raise SystemExit(
                    f"[calibrate] Unknown axis_map key: {k!r}  "
                    f"(valid: {sorted(valid_keys)})"
                )
            if k in ("col_sign", "row_sign"):
                try:
                    ival = int(v)
                    if ival not in (1, -1):
                        raise ValueError("must be 1 or -1")
                    updates[k] = ival
                except ValueError as exc:
                    raise SystemExit(
                        f"[calibrate] {k} must be 1 or -1, got: {v!r}"
                    ) from exc
            else:
                if v not in ("x", "y"):
                    raise SystemExit(
                        f"[calibrate] {k} must be 'x' or 'y', got: {v!r}"
                    )
                updates[k] = v

        cfg.axis_map.update(updates)
        cfg.axis_map["calibrated"] = True
        cfg.save()
        print("[calibrate] axis_map updated and calibrated=true:", flush=True)
        print(f"  {cfg.axis_map}", flush=True)

    else:
        # Print current config and instructions (no motion)
        print("[calibrate] Current axis_map:", flush=True)
        print(f"  {cfg.axis_map}", flush=True)
        print(
            "\n[calibrate] To set axis mapping:\n"
            "  plate_imaging.py calibrate "
            "--set 'col_axis=x,col_sign=1,row_axis=y,row_sign=1'\n"
            "  Adjust col_sign and/or row_sign to -1 as needed.",
            flush=True,
        )


# ---------------------------------------------------------------------------
# (9) Show-config subcommand
# ---------------------------------------------------------------------------

def cmd_show_config(args: argparse.Namespace) -> None:
    cfg    = PlateConfig()
    routes = RouteStore()
    print("[show-config] PlateConfig:", flush=True)
    print(f"  {cfg}", flush=True)
    saved_keys = routes.keys()
    print(f"\n[show-config] Saved route keys ({len(saved_keys)}):", flush=True)
    if saved_keys:
        for key in saved_keys:
            print(f"  {key}", flush=True)
    else:
        print("  (none -- routes will be built on first run)", flush=True)


# ---------------------------------------------------------------------------
# (10) CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="plate_imaging.py",
        description=(
            "xArm7 96-well plate imaging motion script.  "
            "Use --execute for live hardware; omit for dry-run."
        ),
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        default=False,
        help=(
            "Live mode: connect to arm and execute motion.  "
            "Default (without flag): dry-run -- prints planned waypoints only."
        ),
    )
    parser.add_argument(
        "--host",
        default=None,
        metavar="IP",
        help="Override arm host IP (default: from config.yaml/arm.json).",
    )
    parser.add_argument(
        "--clear-errors",
        action="store_true",
        default=False,
        help=(
            "Allow clearing a latched controller fault before connecting. Never "
            "automatic -- a latched error_code may record a previous collision, "
            "and now (via the shared Arm connection) blocks connecting outright "
            "unless this is passed."
        ),
    )
    parser.add_argument(
        "--rebuild-routes",
        action="store_true",
        default=False,
        help="Force recompute and overwrite any route touched this run.",
    )
    parser.add_argument(
        "--display-url",
        default="http://127.0.0.1:8770",
        metavar="URL",
        help="Display ops endpoint base URL (default: http://127.0.0.1:8770).",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        default=False,
        help="Disable display ops feed (no POST to display_url).",
    )

    sub = parser.add_subparsers(dest="subcommand", required=True)

    # run
    p_run = sub.add_parser(
        "run",
        help="Execute the full plate-imaging motion sequence.",
    )
    p_run.add_argument(
        "--well",
        default="C9",
        metavar="WELL",
        help="Target well (e.g. C9; default: C9).",
    )
    p_run.add_argument(
        "--batch-id",
        default=None,
        metavar="ID",
        help=(
            "Batch identifier for capture filenames "
            "(default: timestamp YYYYMMDD_HHMMSS)."
        ),
    )

    # calibrate
    p_cal = sub.add_parser(
        "calibrate",
        help="Show or update axis_map (col_axis/col_sign/row_axis/row_sign).",
    )
    p_cal.add_argument(
        "--set",
        default=None,
        metavar="KEY=VAL,...",
        help=(
            "Comma-separated axis_map assignments, e.g. "
            "'col_axis=x,col_sign=1,row_axis=y,row_sign=-1'.  "
            "Sets calibrated=true automatically."
        ),
    )

    # show-config
    sub.add_parser(
        "show-config",
        help="Print current PlateConfig and saved route keys.",
    )

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args   = parser.parse_args(argv)

    if args.subcommand == "run":
        cmd_run(args)
    elif args.subcommand == "calibrate":
        cmd_calibrate(args)
    elif args.subcommand == "show-config":
        cmd_show_config(args)
    else:
        parser.print_help()
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
