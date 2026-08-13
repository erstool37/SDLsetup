"""Base class for every device's dashboard adapter.

Lives in :mod:`tools` and not in :mod:`dashboard` on purpose: every instrument
subclasses it, so putting it in the dashboard would make ``tools`` depend on
the dashboard and the dashboard depend on ``tools`` -- a cycle. The dependency
runs one way only: **dashboard imports tools.**
"""
from __future__ import annotations

import abc


class Node(abc.ABC):
    kind: str = "generic"

    def __init__(self, name: str) -> None:
        self.name = name
        self._bus = None
        self._oplog = None
        self.state: str = "init"  # init | online | offline | busy | error | vacant
        self.started = False

    # wired by Lab.register
    def attach(self, bus, oplog) -> None:
        self._bus = bus
        self._oplog = oplog

    def log(self, message: str, level: str = "info") -> None:
        if self._oplog is not None:
            self._oplog.emit(self.name, message, level)

    def publish_status(self) -> None:
        if self._bus is not None:
            self._bus.publish("status", {"node": self.name, "status": self.status()})

    # lifecycle (override as needed)
    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    # --- interface ---
    @abc.abstractmethod
    def status(self) -> dict:
        """Return a JSON-serialisable status dict."""

    def commands(self) -> list[str]:
        return []

    def command(self, name: str, **kwargs) -> dict:
        raise NotImplementedError(f"{self.name}: no command {name!r}")

    def base_status(self, **extra) -> dict:
        return {"name": self.name, "kind": self.kind, "state": self.state,
                "commands": self.commands(), **extra}
