"""Droplet-container humidity/temperature control (vacant — see README.md)."""
from __future__ import annotations

from .._vacant import VacantNode


class EnvironmentNode(VacantNode):
    def __init__(self, name: str = "environment") -> None:
        super().__init__(
            name=name,
            kind="environment",
            summary="Droplet-container humidity/temperature control",
            planned=["read", "set_humidity", "set_temperature"],
        )
