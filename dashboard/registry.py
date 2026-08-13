"""Lab: the orchestrator/registry that owns the bus, oplog, and all nodes."""
from __future__ import annotations

import threading

from .bus import Bus
from .oplog import OpLog


class Lab:
    def __init__(self) -> None:
        self.bus = Bus()
        self.oplog = OpLog(self.bus)
        self.nodes: dict[str, object] = {}
        self._lock = threading.Lock()

    def register(self, node):
        node.attach(self.bus, self.oplog)
        with self._lock:
            self.nodes[node.name] = node
        self.oplog.emit("lab", f"registered '{node.name}' ({node.kind})")
        return node

    def start_all(self) -> None:
        for n in list(self.nodes.values()):
            try:
                n.start()
                self.oplog.emit("lab", f"started '{n.name}'")
            except Exception as exc:
                self.oplog.emit("lab", f"start failed '{n.name}': {exc}", "error")

    def stop_all(self) -> None:
        for n in list(self.nodes.values()):
            try:
                n.stop()
            except Exception:
                pass

    def status(self) -> dict:
        out = {}
        for name, n in list(self.nodes.items()):
            try:
                out[name] = n.status()
            except Exception as exc:
                out[name] = {"name": name, "kind": getattr(n, "kind", "?"),
                             "state": "error", "error": str(exc)}
        return out

    def command(self, node: str, name: str, **kwargs) -> dict:
        n = self.nodes.get(node)
        if n is None:
            raise KeyError(f"no node '{node}'")
        args = ", ".join(f"{k}={v}" for k, v in kwargs.items())
        self.oplog.emit(node, f"command: {name}({args})")
        result = n.command(name, **kwargs)
        n.publish_status()
        return result
