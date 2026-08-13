"""Opentrons liquid-handling robot control (vacant — see README.md)."""
from __future__ import annotations

from .._vacant import VacantNode


class OpentronsNode(VacantNode):
    def __init__(self, name: str = "opentrons") -> None:
        super().__init__(
            name=name,
            kind="liquidhandler",
            summary="Opentrons liquid-handling robot control",
            planned=["run_protocol", "home", "pause", "resume", "estop"],
        )
