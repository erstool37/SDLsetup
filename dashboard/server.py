"""Web display layer for the lab. Stdlib http.server + Server-Sent Events.

Serves a tabbed dashboard:
  - Overview: node status cards + a live 'terminal' (the operation feed)
  - Cameras:  the linked microscope views + record/photo buttons
  - CCTV:     whole-setup overview (vacant node placeholder)

Endpoints:
  GET  /                      dashboard
  GET  /api/status            {nodes: {...}, occupancy: {...}}
  GET  /api/occupancy         who is moving right now, and who is blocked
  GET  /api/bench             the bench layout, for the diagram
  GET  /api/ops?n=300         recent operation log
  GET  /events                SSE stream (ops + status)
  GET  /camera/<cam>          latest jpeg for a linked camera
  POST /command/<node>/<act>  run a node command (json body = kwargs)
"""
from __future__ import annotations

import http.server
import json
import queue
import time
from urllib.parse import parse_qs, urlparse

from tools import occupancy

from .bench import bench_spec
from .page import PAGE


class _Handler(http.server.BaseHTTPRequestHandler):
    lab = None

    def log_message(self, *args) -> None:  # quiet
        pass

    def _send(self, body, ctype="application/json", status=200) -> None:
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _camera_node(self):
        for n in self.lab.nodes.values():
            if getattr(n, "kind", "") == "camera":
                return n
        return None

    def do_GET(self) -> None:
        u = urlparse(self.path)
        path, q = u.path, parse_qs(u.query)
        if path in ("/", "/index.html"):
            return self._send(PAGE, "text/html; charset=utf-8")
        if path == "/api/status":
            return self._send({"nodes": self.lab.status(),
                               "occupancy": occupancy.status(),
                               "ts": time.time()})
        if path == "/api/occupancy":
            # What is moving right now. The same file-backed claims the arm and
            # the UV-Vis carrier take before they actuate -- this is a read of
            # the real interlock, not a separate bookkeeping copy that could
            # drift from it.
            return self._send({"occupancy": occupancy.status(),
                               "conflicts": {k: sorted(v) for k, v in
                                             occupancy.CONFLICTS.items()},
                               "ts": time.time()})
        if path == "/api/bench":
            # Where each instrument sits on the bench, so the panel can draw it.
            # Static; the live part comes from /api/status.
            return self._send(bench_spec())
        if path == "/api/ops":
            n = int((q.get("n") or ["300"])[0])
            return self._send(self.lab.oplog.recent(n))
        if path == "/events":
            return self._sse()
        if path.startswith("/camera/"):
            node = self._camera_node()
            data = node.latest_image(path[len("/camera/"):]) if node else None
            if not data:
                return self._send({"error": "no image"}, status=404)
            return self._send(data, "image/jpeg")
        return self._send({"error": "not found"}, status=404)

    def _sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        fh = self.lab.bus.firehose()
        try:
            for entry in self.lab.oplog.recent(50):
                self._emit({"topic": "ops", **entry})
            while True:
                try:
                    msg = fh.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                self._emit(msg)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.lab.bus.drop_firehose(fh)

    def _emit(self, msg: dict) -> None:
        self.wfile.write(b"data: " + json.dumps(msg).encode() + b"\n\n")
        self.wfile.flush()

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        ln = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(ln) if ln else b""
        if path == "/ops":
            # External processes (e.g. plate_imaging.py) push human-readable
            # operation lines into the dashboard terminal via the oplog.
            entry = {}
            if body:
                try:
                    entry = json.loads(body)
                except Exception:
                    entry = {}
            self.lab.oplog.emit(
                entry.get("source", "external"),
                entry.get("message", ""),
                entry.get("level", "info"),
            )
            return self._send({"ok": True})
        if path.startswith("/command/"):
            parts = path[len("/command/"):].split("/")
            if len(parts) >= 2 and parts[0] and parts[1]:
                kwargs = {}
                if body:
                    try:
                        kwargs = json.loads(body)
                    except Exception:
                        kwargs = {}
                try:
                    res = self.lab.command(parts[0], parts[1], **kwargs)
                    return self._send({"ok": True, "result": res})
                except Exception as exc:
                    return self._send({"ok": False, "error": str(exc)}, status=400)
        return self._send({"error": "not found"}, status=404)


def serve(lab, host: str = "0.0.0.0", port: int = 8770) -> None:
    handler = type("DisplayHandler", (_Handler,), {"lab": lab})
    server = http.server.ThreadingHTTPServer((host, port), handler)
    lab.oplog.emit("display", f"display serving on http://{host}:{port}/")
    try:
        server.serve_forever()
    finally:
        server.server_close()
