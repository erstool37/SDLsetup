#!/usr/bin/env python
"""Capture Leica K3C frames through the installed GenTL producer on Windows."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np
from harvesters.core import Harvester
from PIL import Image

DEFAULT_CTI = Path(r"C:\Windows\twain_64\Leica Microsystems\bin64\bgapi2_usb.cti")


def device_info_dict(device: Any) -> dict[str, Any]:
    data = dict(device.property_dict)
    data.pop("parent", None)
    return data


def set_node_value(node_map: Any, name: str, value: Any) -> str:
    node = getattr(node_map, name)
    node.value = value
    return str(node.value)


def try_set_node_value(node_map: Any, name: str, value: Any) -> str | None:
    try:
        return set_node_value(node_map, name, value)
    except Exception:
        return None


def bayer_rg8_to_rgb(raw: np.ndarray) -> np.ndarray:
    """Small dependency-free BayerRG8 demosaic suitable for capture verification."""
    source = raw.astype(np.float32)
    height, width = source.shape
    rgb = np.zeros((height, width, 3), dtype=np.float32)

    rgb[0::2, 0::2, 0] = source[0::2, 0::2]
    rgb[0::2, 1::2, 1] = source[0::2, 1::2]
    rgb[1::2, 0::2, 1] = source[1::2, 0::2]
    rgb[1::2, 1::2, 2] = source[1::2, 1::2]

    for channel in range(3):
        plane = rgb[:, :, channel]
        missing = plane == 0
        sums = np.zeros_like(plane)
        counts = np.zeros_like(plane)
        for dy, dx in (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ):
            shifted = np.roll(np.roll(plane, dy, axis=0), dx, axis=1)
            valid = shifted > 0
            sums += shifted * valid
            counts += valid
        fillable = missing & (counts > 0)
        plane[fillable] = sums[fillable] / counts[fillable]
        rgb[:, :, channel] = plane

    return np.clip(rgb, 0, 255).astype(np.uint8)


def image_from_component(data: np.ndarray, pixel_format: str) -> Image.Image:
    if pixel_format == "BayerRG8":
        return Image.fromarray(bayer_rg8_to_rgb(data), "RGB")
    if pixel_format in {"Mono8", "BayerRG8Raw"}:
        return Image.fromarray(data, "L")
    # K3C currently reports BayerRG8. Fall back to grayscale for unknown 8-bit formats.
    return Image.fromarray(data, "L")


def save_image(image: Image.Image, path: Path, quality: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        image.save(path, quality=quality)
    else:
        image.save(path)


def list_devices(cti_path: Path) -> dict[str, Any]:
    with dll_directory(cti_path.parent):
        harvester = Harvester()
        try:
            harvester.add_file(str(cti_path))
            harvester.update()
            return {
                "ok": True,
                "cti_path": str(cti_path),
                "devices": [device_info_dict(device) for device in harvester.device_info_list],
            }
        finally:
            harvester.reset()


class dll_directory:
    def __init__(self, path: Path) -> None:
        self.path = str(path)
        self._handle: Any = None

    def __enter__(self) -> None:
        if hasattr(os, "add_dll_directory"):
            self._handle = os.add_dll_directory(self.path)

    def __exit__(self, *_exc: Any) -> None:
        if self._handle is not None:
            self._handle.close()


def choose_device(device_info_list: list[Any], serial: str | None) -> int:
    if not device_info_list:
        raise RuntimeError("No GenTL cameras were found")
    if not serial:
        return 0
    for index, device in enumerate(device_info_list):
        if device_info_dict(device).get("serial_number") == serial:
            return index
    raise RuntimeError(f"GenTL camera serial {serial!r} not found")


def capture_once(
    *,
    cti_path: Path,
    output: Path,
    online_output: Path | None,
    serial: str | None,
    exposure_us: float,
    gain: float,
    timeout_ms: int,
    quality: int,
) -> dict[str, Any]:
    if not cti_path.exists():
        raise RuntimeError(f"GenTL producer not found: {cti_path}")

    with dll_directory(cti_path.parent):
        harvester = Harvester()
        ia = None
        try:
            harvester.add_file(str(cti_path))
            harvester.update()
            device_index = choose_device(harvester.device_info_list, serial)
            device_info = device_info_dict(harvester.device_info_list[device_index])
            ia = harvester.create(device_index)
            node_map = ia.remote_device.node_map

            try_set_node_value(node_map, "TriggerMode", "Off")
            try_set_node_value(node_map, "ExposureAuto", "Off")
            try_set_node_value(node_map, "GainAuto", "Off")
            try_set_node_value(node_map, "AcquisitionFrameRateEnable", False)
            set_node_value(node_map, "ExposureTime", float(exposure_us))
            set_node_value(node_map, "Gain", float(gain))
            time.sleep(0.15)

            pixel_format = str(node_map.PixelFormat.value)
            ia.start()
            try:
                with ia.fetch(timeout=timeout_ms) as buffer:
                    component = buffer.payload.components[0]
                    data = component.data.reshape(component.height, component.width).copy()
                    width = int(component.width)
                    height = int(component.height)
            finally:
                ia.stop()

            image = image_from_component(data, pixel_format)
            save_image(image, output, quality)
            if online_output is not None:
                online_output.parent.mkdir(parents=True, exist_ok=True)
                online_output.write_bytes(output.read_bytes())

            stats = {
                "mean": float(data.mean()),
                "min": int(data.min()),
                "max": int(data.max()),
                "p95": float(np.percentile(data, 95)),
                "p99": float(np.percentile(data, 99)),
                "saturation_pct": float((data >= 250).mean() * 100.0),
            }
            return {
                "ok": output.exists() and output.stat().st_size > 0,
                "source": "Leica K3C GenTL",
                "cti_path": str(cti_path),
                "device": device_info,
                "output": str(output),
                "output_bytes": output.stat().st_size if output.exists() else 0,
                "online_output": str(online_output) if online_output else "",
                "online_output_bytes": (
                    online_output.stat().st_size
                    if online_output is not None and online_output.exists()
                    else 0
                ),
                "width": width,
                "height": height,
                "pixel_format": pixel_format,
                "exposure_us": float(node_map.ExposureTime.value),
                "gain": float(node_map.Gain.value),
                "stats": stats,
            }
        finally:
            if ia is not None:
                ia.destroy()
            harvester.reset()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Leica K3C GenTL capture helper")
    parser.add_argument("action", choices=["list", "capture"])
    parser.add_argument("--cti-path", default=str(DEFAULT_CTI))
    parser.add_argument("--serial")
    parser.add_argument("--output")
    parser.add_argument("--online-output")
    parser.add_argument("--exposure-us", type=float, default=60000.0)
    parser.add_argument("--gain", type=float, default=1.0)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--quality", type=int, default=95)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    cti_path = Path(args.cti_path)
    if args.action == "list":
        print(json.dumps(list_devices(cti_path), indent=2, default=str))
        return 0

    if not args.output:
        raise RuntimeError("--output is required for capture")
    result = capture_once(
        cti_path=cti_path,
        output=Path(args.output),
        online_output=Path(args.online_output) if args.online_output else None,
        serial=args.serial,
        exposure_us=args.exposure_us,
        gain=args.gain,
        timeout_ms=args.timeout_ms,
        quality=args.quality,
    )
    print(json.dumps(result, indent=2, default=str))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR_TYPE: {type(exc).__name__}", file=sys.stderr)
        print(f"ERROR_REPR: {exc!r}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        raise SystemExit(1) from exc
