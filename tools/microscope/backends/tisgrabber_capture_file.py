#!/usr/bin/env python
"""Capture one TIS frame through the legacy tisgrabber sample stack."""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


def declare_property_functions(ic: Any, tis: Any) -> None:
    """Add legacy VCD property setters and headless-open helpers omitted by bundled tisgrabber.py."""
    hgrabber = ctypes.POINTER(tis.HGRABBER)
    ic.IC_SetPropertySwitch.argtypes = (hgrabber, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int)
    ic.IC_SetPropertyValue.argtypes = (hgrabber, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int)
    ic.IC_SetPropertyAbsoluteValue.argtypes = (
        hgrabber,
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_float,
    )
    # Headless device-open helpers (not declared in the bundled tisgrabber.py).
    ic.IC_GetDeviceCount.restype = ctypes.c_int
    ic.IC_OpenDevByUniqueName.argtypes = (hgrabber, ctypes.c_char_p)
    ic.IC_OpenDevByUniqueName.restype = ctypes.c_int


def load_tis(sample_dir: Path) -> tuple[Any, Any]:
    if not sample_dir.exists():
        raise RuntimeError(f"legacy TIS sample directory not found: {sample_dir}")
    dll = sample_dir / "tisgrabber_x64.dll"
    if not dll.exists():
        raise RuntimeError(f"legacy TIS DLL not found: {dll}")
    device_xml = sample_dir / "device.xml"
    if not device_xml.exists():
        raise RuntimeError(f"legacy TIS device.xml not found: {device_xml}")

    sys.path.insert(0, str(sample_dir))
    import tisgrabber as tis  # type: ignore[import-not-found]

    ic = ctypes.cdll.LoadLibrary(str(dll))
    tis.declareFunctions(ic)
    declare_property_functions(ic, tis)
    ic.IC_InitLibrary(0)
    return ic, tis


def get_switch(ic: Any, tis: Any, h_grabber: Any, item: str, element: str) -> dict[str, Any]:
    value = ctypes.c_long()
    ret = ic.IC_GetPropertySwitch(h_grabber, tis.T(item), tis.T(element), ctypes.byref(value))
    return {"returncode": int(ret), "value": int(value.value)}


def get_value(ic: Any, tis: Any, h_grabber: Any, item: str, element: str) -> dict[str, Any]:
    value = ctypes.c_long()
    ret = ic.IC_GetPropertyValue(h_grabber, tis.T(item), tis.T(element), ctypes.byref(value))
    return {"returncode": int(ret), "value": int(value.value)}


def get_absolute(
    ic: Any,
    tis: Any,
    h_grabber: Any,
    item: str,
    element: str,
) -> dict[str, Any]:
    value = ctypes.c_float()
    ret = ic.IC_GetPropertyAbsoluteValue(
        h_grabber,
        tis.T(item),
        tis.T(element),
        ctypes.byref(value),
    )
    return {"returncode": int(ret), "value": float(value.value)}


def property_snapshot(ic: Any, tis: Any, h_grabber: Any) -> dict[str, Any]:
    snapshot: dict[str, Any] = {}
    for item, element in (
        ("Exposure", "Auto"),
        ("Exposure", "Value"),
        ("Gain", "Auto"),
        ("Gain", "Value"),
        ("Brightness", "Value"),
    ):
        key = f"{item}/{element}"
        snapshot[key] = {
            "switch": get_switch(ic, tis, h_grabber, item, element),
            "value": get_value(ic, tis, h_grabber, item, element),
            "absolute": get_absolute(ic, tis, h_grabber, item, element),
        }
    return snapshot


def decode_device_string(value: Any, tis: Any) -> str:
    if not value:
        return ""
    try:
        return tis.D(value)
    except Exception:
        try:
            return value.decode("utf-8", "ignore")
        except Exception:
            return str(value)


def device_snapshot(ic: Any, tis: Any, h_grabber: Any) -> dict[str, Any]:
    return {
        "device": decode_device_string(ic.IC_GetDevice(h_grabber), tis),
        "device_name": decode_device_string(ic.IC_GetDeviceName(0), tis),
    }


def apply_manual_profile(
    *,
    ic: Any,
    tis: Any,
    h_grabber: Any,
    exposure_s: float | None,
    gain: float | None,
    brightness: int | None,
) -> list[dict[str, Any]]:
    applied: list[dict[str, Any]] = []

    def record(kind: str, item: str, element: str, value: Any, returncode: int) -> None:
        applied.append(
            {
                "kind": kind,
                "item": item,
                "element": element,
                "value": value,
                "returncode": int(returncode),
            }
        )

    if exposure_s is not None:
        ret = ic.IC_SetPropertySwitch(h_grabber, tis.T("Exposure"), tis.T("Auto"), 0)
        record("switch", "Exposure", "Auto", 0, ret)
        ret = ic.IC_SetPropertyAbsoluteValue(
            h_grabber,
            tis.T("Exposure"),
            tis.T("Value"),
            ctypes.c_float(float(exposure_s)),
        )
        record("absolute", "Exposure", "Value", float(exposure_s), ret)

    if gain is not None:
        ret = ic.IC_SetPropertySwitch(h_grabber, tis.T("Gain"), tis.T("Auto"), 0)
        record("switch", "Gain", "Auto", 0, ret)
        ret = ic.IC_SetPropertyAbsoluteValue(
            h_grabber,
            tis.T("Gain"),
            tis.T("Value"),
            ctypes.c_float(float(gain)),
        )
        record("absolute", "Gain", "Value", float(gain), ret)

    if brightness is not None:
        ret = ic.IC_SetPropertyValue(
            h_grabber,
            tis.T("Brightness"),
            tis.T("Value"),
            int(brightness),
        )
        record("value", "Brightness", "Value", int(brightness), ret)

    return applied


def open_device_headless(ic: Any, tis: Any, *, device_unique_name: str | None = None) -> Any:
    """Open the TIS grabber without any GUI dialog.

    Strategy (cwd must already be sample_dir so device.xml is resolvable):
    1. Try IC_LoadDeviceStateFromFile from device.xml.
    2. If the resulting handle is not valid and a device_unique_name hint is
       provided, create a fresh grabber and open by unique name instead.
    3. Raise RuntimeError on failure — never call IC_ShowDeviceSelectionDialog.
    """
    h_grabber = ic.IC_LoadDeviceStateFromFile(None, tis.T("device.xml"))
    if ic.IC_IsDevValid(h_grabber):
        return h_grabber

    # device.xml did not yield a valid device — try by configured unique name.
    if device_unique_name:
        h_grabber2 = ic.IC_CreateGrabber()
        ret = ic.IC_OpenDevByUniqueName(h_grabber2, tis.T(device_unique_name))
        if ic.IC_IsDevValid(h_grabber2):
            return h_grabber2
        raise RuntimeError(
            f"IC_OpenDevByUniqueName({device_unique_name!r}) returned {ret} — "
            "device not found; check USB attachment and 'legacy_unique_name' in cameras.json"
        )

    raise RuntimeError(
        "IC_LoadDeviceStateFromFile returned an invalid handle and no "
        "device_unique_name hint was given — set 'legacy_unique_name' in cameras.json "
        "to the value of unique_name in device.xml (e.g. 'DFK 33UX264 48424295')"
    )


def capture_once(
    *,
    sample_dir: Path,
    output: Path,
    image_type: str,
    jpeg_quality: int,
    snap_timeout_ms: int,
    show_live: bool,
    settle_s: float,
    exposure_s: float | None,
    gain: float | None,
    brightness: int | None,
    device_unique_name: str | None = None,
) -> dict[str, Any]:
    output.parent.mkdir(parents=True, exist_ok=True)
    ic, tis = load_tis(sample_dir)
    old_cwd = Path.cwd()
    h_grabber = None
    os.chdir(sample_dir)
    try:
        h_grabber = open_device_headless(ic, tis, device_unique_name=device_unique_name)
        if not ic.IC_IsDevValid(h_grabber):
            raise RuntimeError("legacy TIS grabber did not open a valid device")

        device = device_snapshot(ic, tis, h_grabber)
        before_props = property_snapshot(ic, tis, h_grabber)
        applied_props = apply_manual_profile(
            ic=ic,
            tis=tis,
            h_grabber=h_grabber,
            exposure_s=exposure_s,
            gain=gain,
            brightness=brightness,
        )
        after_props = property_snapshot(ic, tis, h_grabber)

        ic.IC_StartLive(h_grabber, 1 if show_live else 0)
        if settle_s > 0:
            time.sleep(settle_s)
        live_props = property_snapshot(ic, tis, h_grabber)

        status = ic.IC_SnapImage(h_grabber, snap_timeout_ms)
        if status != tis.IC_SUCCESS:
            raise RuntimeError(f"IC_SnapImage failed with status {status}")
        snapped_props = property_snapshot(ic, tis, h_grabber)

        type_key = image_type.upper()
        if type_key not in tis.ImageFileTypes:
            raise RuntimeError(f"unsupported legacy TIS image type: {image_type}")
        ic.IC_SaveImage(h_grabber, tis.T(str(output)), tis.ImageFileTypes[type_key], jpeg_quality)
    finally:
        try:
            if h_grabber is not None:
                ic.IC_StopLive(h_grabber)
                ic.IC_ReleaseGrabber(h_grabber)
        finally:
            os.chdir(old_cwd)

    return {
        "ok": output.exists() and output.stat().st_size > 0,
        "output": str(output),
        "output_bytes": output.stat().st_size if output.exists() else 0,
        "sample_dir": str(sample_dir),
        "device_xml": str(sample_dir / "device.xml"),
        "device": device,
        "image_type": image_type,
        "captured_at": datetime.now().isoformat(timespec="microseconds"),
        "properties_before": before_props,
        "properties_applied": applied_props,
        "properties_after_apply": after_props,
        "properties_after_live_settle": live_props,
        "properties_after_snap": snapped_props,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Legacy TIS tisgrabber one-frame capture")
    parser.add_argument("--sample-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--image-type", choices=["JPEG", "BMP"], default="JPEG")
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--snap-timeout-ms", type=int, default=2000)
    parser.add_argument("--show-live", action="store_true")
    parser.add_argument("--settle-s", type=float, default=0.0)
    parser.add_argument("--exposure-s", type=float)
    parser.add_argument("--gain", type=float)
    parser.add_argument("--brightness", type=int)
    parser.add_argument(
        "--device-unique-name",
        default="",
        help=(
            "IC unique name of the device (e.g. 'DFK 33UX264 48424295'). "
            "Used as a headless fallback when device.xml does not produce a valid handle. "
            "Maps to 'legacy_unique_name' in cameras.json."
        ),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    result = capture_once(
        sample_dir=Path(args.sample_dir),
        output=Path(args.output),
        image_type=args.image_type,
        jpeg_quality=args.jpeg_quality,
        snap_timeout_ms=args.snap_timeout_ms,
        show_live=args.show_live,
        settle_s=args.settle_s,
        exposure_s=args.exposure_s,
        gain=args.gain,
        brightness=args.brightness,
        device_unique_name=args.device_unique_name or None,
    )
    print(json.dumps(result, indent=2, default=str))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
