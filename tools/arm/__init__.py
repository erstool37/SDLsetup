"""UFACTORY xArm7 + BIO Gripper G2 -- everything needed to operate the arm.

    from tools import arm

    arm.move(35, 34, 33, "config.yaml")     # cached session, dry-run by default
    arm.grip()

    with arm.session("config.yaml", live=True) as a:
        a.move_to("microscope")

Layers, lowest to highest:

``safety``     Envelope + ValidatedMove -- the guards, enforced by type
``driver``     the only XArmAPI transport; takes ValidatedMove, nothing else
``workspace``  taught locations/routes JSON store (the safety anchor)
``geometry``   the 9 mm well grid, anchored on A1
``approach``   the 4-leg Y approach route to a well under the objective
``teach``      capture the current pose into workspace.json
``api``        Arm -- what orchestration calls
``node``       Lab node wrapper for the dashboard
"""
from . import approach, driver, geometry, safety, teach, workspace
from .api import (
    Arm,
    ArmError,
    ArmSettings,
    MoveResult,
    close,
    default,
    grip,
    move,
    move_to,
    opening,
    release,
    session,
    set_opening,
    status,
)
from .node import ArmNode
from .safety import Envelope, SafetyError, ValidatedMove
from .workspace import DEFAULT_STORE, Location, Pose, Route, RouteStep, WorkspaceStore

__all__ = [
    "Arm", "ArmError", "ArmNode", "ArmSettings", "DEFAULT_STORE", "Envelope",
    "Location", "MoveResult", "Pose", "Route", "RouteStep", "SafetyError",
    "ValidatedMove", "WorkspaceStore",
    "approach", "close", "default", "driver", "geometry", "grip", "move",
    "move_to", "opening", "release", "safety", "session", "set_opening",
    "status", "teach", "workspace",
]
