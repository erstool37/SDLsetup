#!/usr/bin/env python
"""Capture one TIS frame through Windows OpenCV/DirectShow."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2


def backend_name(cap: Any) -> str:
    try:
        return str(cap.getBackendName())
    except Exception:
        return ""


def frame_stats(frame: Any) -> dict[str, Any]:
    return {
        "shape": list(frame.shape),
        "mean": float(frame.mean()),
        "min": int(frame.min()),
        "max": int(frame.max()),
    }


def list_devices(max_index: int) -> dict[str, Any]:
    devices: list[dict[str, Any]] = []
    for index in range(max_index + 1):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        try:
            opened = bool(cap.isOpened())
            data: dict[str, Any] = {
                "index": index,
                "opened": opened,
                "backend": backend_name(cap) if opened else "",
            }
            if opened:
                ok, frame = cap.read()
                data.update(
                    {
                        "read_ok": bool(ok),
                        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                        "fps": float(cap.get(cv2.CAP_PROP_FPS)),
                    }
                )
                if ok and frame is not None:
                    data["frame"] = frame_stats(frame)
            devices.append(data)
        finally:
            cap.release()
    return {"ok": True, "devices": devices}


def capture_once(
    *,
    index: int,
    output: Path,
    width: int | None,
    height: int | None,
    warmup_frames: int,
    settle_s: float,
) -> dict[str, Any]:
    output.parent.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"OpenCV DirectShow camera index {index} did not open")
        if width is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        if settle_s > 0:
            time.sleep(settle_s)

        last_frame = None
        read_ok = False
        for _ in range(max(1, warmup_frames)):
            read_ok, last_frame = cap.read()
            if not read_ok:
                time.sleep(0.05)
        if not read_ok or last_frame is None:
            raise RuntimeError(f"OpenCV DirectShow camera index {index} returned no frame")

        ok = cv2.imwrite(str(output), last_frame)
        return {
            "ok": bool(ok and output.exists() and output.stat().st_size > 0),
            "source": "OpenCV DirectShow",
            "index": index,
            "backend": backend_name(cap),
            "output": str(output),
            "output_bytes": output.stat().st_size if output.exists() else 0,
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": float(cap.get(cv2.CAP_PROP_FPS)),
            "frame": frame_stats(last_frame),
            "captured_at": datetime.now().isoformat(timespec="microseconds"),
        }
    finally:
        cap.release()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TIS OpenCV DirectShow capture helper")
    parser.add_argument("action", choices=["list", "capture"])
    parser.add_argument("--max-index", type=int, default=5)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--output")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--settle-s", type=float, default=0.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.action == "list":
        print(json.dumps(list_devices(args.max_index), indent=2, default=str))
        return 0
    if not args.output:
        raise RuntimeError("--output is required for capture")
    result = capture_once(
        index=args.index,
        output=Path(args.output),
        width=args.width,
        height=args.height,
        warmup_frames=args.warmup_frames,
        settle_s=args.settle_s,
    )
    print(json.dumps(result, indent=2, default=str))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
