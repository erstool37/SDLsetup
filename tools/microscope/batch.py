"""Batch capture modules -- one call per camera or per linked pair.

The three per-camera wrappers at the bottom used to be three separate files of
about twenty lines each. They are documented API (``docs/microscope-photo.md``)
so they are kept, but consolidated here beside the base classes they subclass.

For new orchestration code prefer :class:`tools.microscope.Microscope`,
which resolves paths and formats from ``config.yaml``; these are the direct,
config-free path.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

from .capture import (
    DEFAULT_CONFIG,
    CaptureResult,
    capture_batch,
    load_config,
    timestamp,
)


def runtime_config(
    *,
    config_path: Path = DEFAULT_CONFIG,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    config = load_config(config_path)
    if output_dir is None:
        return config
    patched = dict(config)
    patched["output_dir"] = str(output_dir.expanduser().resolve())
    return patched


def result_dicts(results: list[CaptureResult]) -> list[dict[str, Any]]:
    return [dataclasses.asdict(result) for result in results]


class CameraCaptureModule:
    """Small reusable capture module for one configured camera."""

    camera_name: str

    def __init__(
        self,
        *,
        config_path: Path = DEFAULT_CONFIG,
        output_dir: Path | None = None,
        start_timeout_s: float = 5.0,
    ) -> None:
        self.config_path = config_path
        self.output_dir = output_dir
        self.start_timeout_s = start_timeout_s

    def capture(
        self,
        *,
        batch_id: str | None = None,
        image_format: str | None = None,
    ) -> CaptureResult:
        config = runtime_config(config_path=self.config_path, output_dir=self.output_dir)
        results = capture_batch(
            config=config,
            camera_name=self.camera_name,
            image_format=image_format,
            batch_id=batch_id or timestamp(),
            start_timeout_s=self.start_timeout_s,
        )
        if len(results) != 1:
            raise RuntimeError(f"expected one result for {self.camera_name}, got {len(results)}")
        return results[0]


class MultiCameraCaptureModule:
    """Reusable wrapper for near-simultaneous software-triggered camera capture."""

    camera_name = "all"

    def __init__(
        self,
        *,
        config_path: Path = DEFAULT_CONFIG,
        output_dir: Path | None = None,
        start_timeout_s: float = 5.0,
    ) -> None:
        self.config_path = config_path
        self.output_dir = output_dir
        self.start_timeout_s = start_timeout_s

    def capture(
        self,
        *,
        batch_id: str | None = None,
        image_format: str | None = None,
    ) -> list[CaptureResult]:
        config = runtime_config(config_path=self.config_path, output_dir=self.output_dir)
        return capture_batch(
            config=config,
            camera_name=self.camera_name,
            image_format=image_format,
            batch_id=batch_id or timestamp(),
            start_timeout_s=self.start_timeout_s,
        )


# ---------------------------------------------------------------------------
# per-camera shortcuts
# ---------------------------------------------------------------------------

class LeicaK3CModule(CameraCaptureModule):
    """Leica K3C, through the Windows GenTL producer."""

    camera_name = "leica_k3c"


class TisDFK33Module(CameraCaptureModule):
    """The Imaging Source DFK33UX264, through the legacy tisgrabber stack."""

    camera_name = "tis_dfk33ux264"


class DualCameraModule(MultiCameraCaptureModule):
    """Both cameras, captured as near-simultaneously as the drivers allow."""

    camera_name = "all"


def capture_leica(*, batch_id: str | None = None, output_dir: Path | None = None,
                  config_path: Path = DEFAULT_CONFIG) -> CaptureResult:
    return LeicaK3CModule(config_path=config_path, output_dir=output_dir).capture(
        batch_id=batch_id)


def capture_tis(*, batch_id: str | None = None, output_dir: Path | None = None,
                config_path: Path = DEFAULT_CONFIG) -> CaptureResult:
    return TisDFK33Module(config_path=config_path, output_dir=output_dir).capture(
        batch_id=batch_id)


def capture_both(*, batch_id: str | None = None, output_dir: Path | None = None,
                 config_path: Path = DEFAULT_CONFIG,
                 start_timeout_s: float = 5.0) -> list[CaptureResult]:
    return DualCameraModule(config_path=config_path, output_dir=output_dir,
                            start_timeout_s=start_timeout_s).capture(batch_id=batch_id)


__all__ = [
    "CameraCaptureModule", "DualCameraModule", "LeicaK3CModule",
    "MultiCameraCaptureModule", "TisDFK33Module",
    "capture_both", "capture_leica", "capture_tis", "result_dicts", "runtime_config",
]
