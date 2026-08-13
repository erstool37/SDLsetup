"""In-process publish/subscribe bus. Topics are strings; messages are dicts."""
from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable


class Bus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subs: dict[str, list[Callable[[dict], None]]] = {}
        self._firehoses: list[queue.Queue] = []

    def subscribe(self, topic: str, callback: Callable[[dict], None]) -> None:
        """Register a callback for a topic. Topic '*' receives everything."""
        with self._lock:
            self._subs.setdefault(topic, []).append(callback)

    def publish(self, topic: str, message: dict) -> dict:
        msg = {"topic": topic, "ts": time.time(), **message}
        with self._lock:
            subs = list(self._subs.get(topic, [])) + list(self._subs.get("*", []))
            hoses = list(self._firehoses)
        for cb in subs:
            try:
                cb(msg)
            except Exception:  # a bad subscriber must not break publishing
                pass
        for q in hoses:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass
        return msg

    def firehose(self, maxsize: int = 2000) -> queue.Queue:
        """A queue that receives every published message (used by SSE clients)."""
        q: queue.Queue = queue.Queue(maxsize=maxsize)
        with self._lock:
            self._firehoses.append(q)
        return q

    def drop_firehose(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._firehoses:
                self._firehoses.remove(q)
