"""Assemble the lab: register every device's node adapter on one orchestrator.

Each device package owns its own hardware; this file only says which of them
this rig has and whether their motion gates are open. Adding a device is one
import and one ``register`` line -- the dashboard picks it up automatically.
"""
from __future__ import annotations

from tools.arm import ArmNode
from tools.cctv import CctvNode
from tools.environment import EnvironmentNode
from tools.microscope import CameraNode
from tools.opentrons import OpentronsNode
from tools.uvvis import UvVisNode

from .registry import Lab


def build_lab(allow_arm_motion: bool = False,
              allow_uv_vis_motion: bool = False,
              config=None) -> Lab:
    lab = Lab()
    lab.register(CameraNode(config=config))                       # linked Leica + TIS
    lab.register(ArmNode(allow_motion=allow_arm_motion, config=config))
    # BMG SPECTROstar Nano; plate-carrier motion gated like the arm.
    lab.register(UvVisNode(allow_uv_vis_motion=allow_uv_vis_motion))
    lab.register(OpentronsNode())                    # vacant
    lab.register(EnvironmentNode())                  # vacant
    lab.register(CctvNode())                         # vacant
    return lab
