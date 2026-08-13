"""Dashboard adapter for the arm. A thin shell over :class:`~.api.Arm`.

Before the consolidation this file carried its own SDK connection, its own
gripper enable sequence, and its own ``set_position`` call -- a seventh copy of
logic that now lives in :mod:`.driver` and :mod:`.safety`. It has none of that
any more: every command here goes through the same guarded path a script uses,
so the dashboard cannot move the arm in a way a procedure could not.

Motion is off by default. ``allow_motion=True`` makes ``move`` live; the
gripper is jaw-only and always available.
"""
from __future__ import annotations

from typing import Any

from ..node import Node
from .api import Arm, ArmSettings

#: The card's one-line description. Static: it names the hardware, not its
#: state, so it must not be built from a live reading that may be missing.
ARM_SUMMARY = "UFACTORY xArm7 with BIO Gripper G2 -- plate transport and positioning"


class ArmNode(Node):
    kind = "arm"

    def __init__(self, name: str = "arm", *, config: Any = None,
                 host: str | None = None, allow_motion: bool = False) -> None:
        super().__init__(name)
        self.allow_motion = bool(allow_motion)
        self.settings = ArmSettings.from_config(config, host=host, live=bool(allow_motion))
        self.arm = Arm(self.settings, log=lambda message: self.log(str(message).strip()))

    # -- status -------------------------------------------------------------
    def status(self) -> dict:
        try:
            info = self.arm.status()
        except Exception as exc:
            self.state = "offline"
            return self.base_status(connected=False, error=str(exc),
                                    summary=ARM_SUMMARY,
                                    motion_allowed=self.allow_motion)
        # The arm is "online" when its controller answers on the network, even
        # if this node is not allowed to command it. Motion permission is a
        # separate fact, reported as motion_allowed.
        self.state = "online" if (info.get("connected") or info.get("reachable")) else "offline"
        info["motion_allowed"] = self.allow_motion
        info.setdefault("summary", ARM_SUMMARY)
        return self.base_status(**info)

    # -- commands -----------------------------------------------------------
    def grip(self) -> dict:
        return self.arm.grip()

    def release(self) -> dict:
        return self.arm.release()

    def set_opening(self, mm: Any, *, speed: Any = None, force: Any = None) -> dict:
        """Partial jaw span. Refused unless the gripper is in position mode.

        ``speed`` and ``force`` are forwarded, not dropped: a caller that asks
        for a gentler grip and silently gets the configured 100 % has been
        handed a different grasp than the one it requested.
        """
        try:
            return self.arm.set_opening(
                float(mm),
                speed=None if speed is None else int(speed),
                force=None if force is None else int(force))
        except Exception as exc:  # report on the bus, never raise into it
            self.log(f"set_opening({mm!r}, speed={speed!r}, force={force!r}) "
                     f"refused: {exc}", "error")
            return {"action": "set_opening", "ok": False, "target_mm": mm,
                    "error": str(exc)}

    def move(self, to_location: str, **_ignored: Any) -> dict:
        """Move to a taught location. Dry-run plan unless allow_motion=True."""
        try:
            result = self.arm.move_to(to_location)
        except Exception as exc:  # report on the bus, never raise into it
            self.log(f"move to {to_location!r} refused: {exc}", "error")
            return {"action": "move", "ok": False, "to": to_location, "error": str(exc)}
        record = result.as_dict()
        record.update(action="move", ok=True, to=to_location, dry_run=result.dry_run)
        return record

    def commands(self) -> list[str]:
        return ["grip", "release", "set_opening", "move", "status"]

    def command(self, name: str, **kwargs: Any) -> dict:
        if name == "grip":
            return self.grip()
        if name == "release":
            return self.release()
        if name == "set_opening":
            mm = kwargs.get("mm", kwargs.get("opening_mm"))
            return self.set_opening(mm, speed=kwargs.get("speed"),
                                    force=kwargs.get("force"))
        if name == "move":
            return self.move(kwargs.get("to_location") or kwargs.get("to") or "microscope")
        if name == "status":
            return self.status()
        raise NotImplementedError(name)

    def stop(self) -> None:
        self.arm.close()
        super().stop()


__all__ = ["ArmNode"]
