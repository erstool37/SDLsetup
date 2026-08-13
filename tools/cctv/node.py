"""Whole-setup overview (CCTV) camera (vacant — see README.md)."""
from __future__ import annotations

from .._vacant import VacantNode


class CctvNode(VacantNode):
    def __init__(self, name: str = "cctv") -> None:
        super().__init__(
            name=name,
            kind="cctv",
            summary="Whole-setup overview (CCTV) camera",
            planned=["snapshot", "stream"],
        )
