"""Dashboard adapter for the two cameras. A thin shell over :class:`~.api.Microscope`.

The dashboard reads the same published stream and drives the same live-session
HTTP API that a procedure does -- it holds no camera logic of its own.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from ..node import Node
from .api import LINKED_CAMERAS, Microscope, MicroscopeSettings

#: The card's one-line description. Names the hardware, not its state.
MICROSCOPE_SUMMARY = ("Leica K3C and The Imaging Source DFK 33UX264 over the "
                      "fixed objective")


class CameraNode(Node):
    kind = "camera"

    def __init__(self, name: str = "cameras", *, config: Any = None,
                 stream_dir: Path | None = None,
                 monitor: str | None = None) -> None:
        super().__init__(name)
        self.settings = MicroscopeSettings.from_config(
            config, stream_dir=stream_dir, monitor_url=monitor)
        self.scope = Microscope(self.settings, log=lambda message: self.log(str(message).strip()))
        self.cameras = LINKED_CAMERAS

    # -- image access (read-only, used by the display) ---------------------
    def image_path(self, cam: str) -> Path:
        return self.scope.stream_path(cam)

    def latest_image(self, cam: str) -> bytes | None:
        path = self.image_path(cam)
        return path.read_bytes() if path.exists() else None

    def status(self) -> dict:
        info = self.scope.status()
        fresh = any((c["age_s"] is not None and c["age_s"] < 15)
                    for c in info["cameras"].values())
        self.state = "online" if (info.get("recording") is not None or fresh) else "offline"
        info.setdefault("summary", MICROSCOPE_SUMMARY)
        return self.base_status(**info)

    def commands(self) -> list[str]:
        return ["take_photo", "start_record", "pause_record", "stop_record"]

    def command(self, name: str, **kwargs: Any) -> dict:
        if name == "take_photo":
            return self.scope.save_photo()
        if name == "start_record":
            return self.scope.start_recording()
        if name in ("pause_record", "stop_record"):
            return self.scope.stop_recording()
        raise NotImplementedError(name)


__all__ = ["CameraNode"]
