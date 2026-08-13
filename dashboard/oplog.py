"""Operation log: a ring buffer of human-readable activity, mirrored to the bus.

This is what the display's 'terminal' panel shows — which operation is being
conducted right now, across all nodes.
"""
from __future__ import annotations

import collections
import threading
import time


class OpLog:
    def __init__(self, bus, capacity: int = 1000) -> None:
        self.bus = bus
        self._buf: collections.deque = collections.deque(maxlen=capacity)
        self._lock = threading.Lock()

    def emit(self, source: str, message: str, level: str = "info") -> dict:
        entry = {"source": source, "message": message, "level": level, "ts": time.time()}
        with self._lock:
            self._buf.append(entry)
        self.bus.publish("ops", entry)
        return entry

    def recent(self, n: int = 300) -> list[dict]:
        with self._lock:
            return list(self._buf)[-n:]
