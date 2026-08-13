"""Base for not-yet-implemented device nodes.

A VacantNode reports state 'vacant' and advertises its planned commands so the
display shows it as a placeholder. Drop a real adapter into the package's
node.py (and remove VacantNode) when the SDK/hardware is wired. See each
package's README.md for the intended integration.
"""
from __future__ import annotations

from .node import Node


class VacantNode(Node):
    kind = "vacant"

    def __init__(self, name: str, kind: str, summary: str, planned=()) -> None:
        super().__init__(name)
        self.kind = kind
        self.state = "vacant"
        self.summary = summary
        self._planned = list(planned)

    def status(self) -> dict:
        return self.base_status(summary=self.summary, planned_commands=self._planned,
                                implemented=False)

    def commands(self) -> list[str]:
        return []  # nothing live yet
