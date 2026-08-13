"""Dashboard adapter for the UV-Vis reader. A thin shell over :class:`SpectroStarNano`.

Rewritten 2026-08-03. The previous version called ``dde.send(cmd, self.cfg)``
and built ``self.cfg`` as a ``DdeResult`` — an API that no longer exists, so
constructing the node raised ``TypeError`` and **the dashboard could not start
at all** once this node was registered. It carried no test, and every check that
would have caught it (`--help`, the import sweep) stops short of building the
lab.

It now delegates to the same :class:`SpectroStarNano` a script would use, so
there is one device API and the dashboard cannot drive the reader in a way a
script could not.

Carrier motion is gated twice over: ``allow_uv_vis_motion`` here, and inside the
reader both the declared arm-clear handshake and the observed
:mod:`tools.occupancy` interlock — the arm and this carrier share physical
space.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from ..node import Node
from . import dde
from .config import UvVisConfig
from .reader import MotionNotAllowed, SpectroStarNano

DEFAULT_EXPORT_DIR = Path("/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/User/Data")
DEFAULT_LOG_DIR = Path(__file__).resolve().parents[2] / "dataset" / "uv_vis_runs"


class UvVisNode(Node):
    kind = "uv_vis"

    def __init__(
        self,
        name: str = "uv_vis",
        *,
        config: Any = None,
        reader: str = "SPECTROstar_Nano",
        export_dir: Path | str = DEFAULT_EXPORT_DIR,
        log_dir: Path | str = DEFAULT_LOG_DIR,
        allow_uv_vis_motion: bool = False,
        timeout_s: float = 180.0,
    ) -> None:
        super().__init__(name)
        self.reader_name = reader
        self.export_dir = Path(export_dir)
        self.log_dir = Path(log_dir)
        self.allow_motion = bool(allow_uv_vis_motion)
        self.timeout_s = float(timeout_s)
        self._last: dict | None = None
        self._reader: SpectroStarNano | None = None
        self._config_source = config

    # -- the device ---------------------------------------------------------
    @property
    def reader(self) -> SpectroStarNano:
        """Built on first use so importing this module needs no instrument."""
        if self._reader is None:
            cfg = UvVisConfig(
                reader=self.reader_name,
                allow_motion=self.allow_motion,
                timeout_s=self.timeout_s,
                export_dir=str(self.export_dir),
                log_dir=str(self.log_dir),
            )
            self._reader = SpectroStarNano.from_config(cfg)
        return self._reader

    # -- status -------------------------------------------------------------
    def status(self) -> dict:
        try:
            present = dde.reader_present()
        except Exception as exc:
            self.state = "offline"
            return self.base_status(implemented=True, present=False, error=str(exc),
                                    reader=self.reader_name)
        self.state = "online" if present else "offline"
        return self.base_status(
            summary="BMG SPECTROstar Nano UV-Vis absorbance reader",
            implemented=True,
            reader=self.reader_name,
            present=present,
            connected=present,
            interop=dde.available(),
            allow_uv_vis_motion=self.allow_motion,
            export_dir=str(self.export_dir),
            last_event=self._last,
        )

    # -- commands -----------------------------------------------------------
    def commands(self) -> list[str]:
        return ["status", "connect", "init", "plate_out", "plate_in", "set_temperature"]

    def command(self, name: str, **kwargs: Any) -> dict:
        try:
            if name == "status":
                return self.status()
            if name == "connect":
                return self._record("connect", self.reader.connect())
            if name == "init":
                return self._record("init", self.reader.init())
            if name == "plate_out":
                return self._record("plate_out", self.reader.plate_out())
            if name == "plate_in":
                return self._record("plate_in", self.reader.plate_in())
            if name == "set_temperature":
                target = float(kwargs["target_c"])
                return self._record("set_temperature",
                                    self.reader.set_temperature(target))
        except MotionNotAllowed as exc:
            self.log(f"{name} refused: {exc}", "warn")
            return {"action": name, "ok": False, "refused": str(exc)}
        except Exception as exc:  # report on the bus, never raise into it
            self.log(f"{name} failed: {exc}", "error")
            return {"action": name, "ok": False, "error": str(exc)}
        raise NotImplementedError(name)

    def _record(self, action: str, payload: Any) -> dict:
        out = {"action": action, "ok": True,
               "result": payload.as_dict() if hasattr(payload, "as_dict") else payload}
        self._last = out
        self.log(f"{action} ok")
        self.publish_status()
        return out


__all__ = ["UvVisNode"]
