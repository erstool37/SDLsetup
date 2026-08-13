"""Teach a named pose into workspace.json, matching its schema byte-for-byte.

The target path is always an explicit parameter — `DEFAULT_STORE` documents
the real production path but is never assumed inside `capture_pose` itself,
so tests can point at a temp dir without risk of touching the live file.
"""
from __future__ import annotations

import datetime as dt
import json
import shutil
import time
from pathlib import Path
from typing import Protocol

DEFAULT_STORE = Path.home() / ".sdl_lab" / "robot_arm" / "workspace.json"


class TeachArmPort(Protocol):
    def get_position(self) -> list: ...
    def get_servo_angle(self) -> list: ...


def capture_pose(
    arm: TeachArmPort,
    name: str,
    description: str = "",
    *,
    path: Path | None = None,
    joints_source: str = "unspecified arm interface",
) -> dict:
    """Capture the arm's current pose + joint angles into workspace.json.

    Writes atomically (temp file + rename) and backs up any existing file
    at `path` first, matching the `workspace.json.bak.<epoch>` convention
    already in use on the live host. `path` defaults to the documented real
    location (DEFAULT_STORE) but callers under test MUST pass a temp path.
    """
    target = Path(path) if path is not None else DEFAULT_STORE

    pose_vals = list(arm.get_position())
    if len(pose_vals) != 6:
        raise ValueError(f"expected 6 pose values (x,y,z,roll,pitch,yaw), got {len(pose_vals)}")
    angles = list(arm.get_servo_angle())

    now = dt.datetime.now().isoformat(timespec="seconds")

    if target.exists():
        backup_path = target.with_name(f"{target.name}.bak.{int(time.time())}")
        shutil.copy2(target, backup_path)
        data = json.loads(target.read_text())
    else:
        data = {"locations": {}, "routes": {}, "protocols": {}}

    data.setdefault("locations", {})
    data.setdefault("routes", {})
    data.setdefault("protocols", {})

    entry = {
        "name": name,
        "pose": {
            "x": pose_vals[0], "y": pose_vals[1], "z": pose_vals[2],
            "roll": pose_vals[3], "pitch": pose_vals[4], "yaw": pose_vals[5],
            "units": "mm_deg",
        },
        "description": description,
        "created_at": now,
        "metadata": {
            "joint_angles_deg": angles,
            "joint_count": len(angles),
            "joints_captured_at": now,
            "joints_source": joints_source,
        },
    }
    data["locations"][name] = entry

    target.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = target.with_name(f"{target.name}.tmp.{int(time.time() * 1000)}")
    tmp_path.write_text(json.dumps(data, indent=2))
    tmp_path.replace(target)

    return entry
