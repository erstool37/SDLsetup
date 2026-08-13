#!/usr/bin/env python
"""Capture Leica TWAIN images from Windows Python.

This script is intentionally Windows-side: it uses the installed TWAIN DSM and
Leica camera driver, so run it with a Windows Python that can import `twain`.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
import tempfile
import traceback
from pathlib import Path
from typing import Any

import twain

CAPABILITY_NAMES = {
    value: name
    for name, value in vars(twain).items()
    if name.startswith(("CAP_", "ICAP_")) and isinstance(value, int)
}

INSPECT_CAPABILITIES = [
    "CAP_DEVICEONLINE",
    "CAP_UICONTROLLABLE",
    "CAP_XFERCOUNT",
    "ICAP_PIXELTYPE",
    "ICAP_BITDEPTH",
    "ICAP_IMAGEFILEFORMAT",
    "ICAP_PHYSICALWIDTH",
    "ICAP_PHYSICALHEIGHT",
    "ICAP_XRESOLUTION",
    "ICAP_YRESOLUTION",
    "ICAP_XNATIVERESOLUTION",
    "ICAP_YNATIVERESOLUTION",
    "ICAP_UNITS",
    "ICAP_XFERMECH",
    "ICAP_EXPOSURETIME",
    "ICAP_BRIGHTNESS",
    "ICAP_CONTRAST",
    "ICAP_GAMMA",
]


@contextlib.contextmanager
def twain_source_manager() -> Any:
    """Open TWAIN DSM with a real hidden parent window on Windows."""
    root = None
    parent: Any = None
    if sys.platform.startswith("win"):
        try:
            import tkinter as tk

            root = tk.Tk()
            root.withdraw()
            root.update_idletasks()
            parent = root
        except Exception:
            parent = None

    try:
        with twain.SourceManager(parent) as source_manager:
            yield source_manager
    finally:
        if root is not None:
            try:
                root.destroy()
            except Exception:
                pass


def twain_sources() -> list[str]:
    with twain_source_manager() as source_manager:
        return list(source_manager.GetSourceList())


def _capability_value(source: Any, cap_id: int) -> dict[str, Any]:
    data: dict[str, Any] = {"id": cap_id, "name": CAPABILITY_NAMES.get(cap_id, str(cap_id))}
    for label, method in (
        ("current", source.GetCapabilityCurrent),
        ("default", source.GetCapabilityDefault),
        ("container", source.GetCapability),
    ):
        try:
            item_type, value = method(cap_id)
        except Exception as exc:  # TWAIN raises source-specific capability errors.
            data[f"{label}_error"] = type(exc).__name__
            continue
        data[label] = {"item_type": item_type, "value": value}
    return data


def inspect_capabilities(source_name: str) -> dict[str, Any]:
    with twain_source_manager() as source_manager:
        sources = list(source_manager.GetSourceList())
        if source_name not in sources:
            raise RuntimeError(
                f"TWAIN source {source_name!r} not found. Available sources: {sources}"
            )
        source = source_manager.OpenSource(source_name)
        try:
            supported_type, supported_values = source.GetCapabilityCurrent(twain.CAP_SUPPORTEDCAPS)
            supported = [
                {"id": cap_id, "name": CAPABILITY_NAMES.get(cap_id, str(cap_id))}
                for cap_id in supported_values
            ]
            selected = []
            for name in INSPECT_CAPABILITIES:
                cap_id = getattr(twain, name)
                selected.append(_capability_value(source, cap_id))
        finally:
            source.close()

    return {
        "ok": True,
        "source": source_name,
        "supported_container_type": supported_type,
        "supported": supported,
        "selected": selected,
    }


def save_dib(handle: Any, output: Path) -> None:
    suffix = output.suffix.lower()
    if suffix in {".bmp", ".dib"}:
        twain.DIBToBMFile(handle, str(output))
        return

    if suffix not in {".jpg", ".jpeg", ".png"}:
        raise RuntimeError(
            f"Unsupported Leica TWAIN output extension {suffix!r}; use bmp, jpg, jpeg, or png"
        )

    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required to convert Leica TWAIN BMP output to JPEG/PNG"
        ) from exc

    tmp_name = ""
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".bmp") as tmp:
            tmp_name = tmp.name
        twain.DIBToBMFile(handle, tmp_name)
        with Image.open(tmp_name) as image:
            image.save(output)
    finally:
        if tmp_name:
            Path(tmp_name).unlink(missing_ok=True)


def capture_once(
    *,
    source_name: str,
    output: Path,
    online_output: Path | None,
    show_ui: bool,
    modal_ui: bool,
    transfer_mech: str,
) -> dict[str, Any]:
    output.parent.mkdir(parents=True, exist_ok=True)
    if online_output is not None:
        online_output.parent.mkdir(parents=True, exist_ok=True)

    with twain_source_manager() as source_manager:
        sources = list(source_manager.GetSourceList())
        if source_name not in sources:
            raise RuntimeError(
                f"TWAIN source {source_name!r} not found. Available sources: {sources}"
            )

        source = source_manager.OpenSource(source_name)
        try:
            if transfer_mech == "file":
                remaining = 0

                def before(_image_info: dict[str, Any]) -> str:
                    return str(output)

                def after(more: int) -> None:
                    nonlocal remaining
                    remaining = more

                source.acquire_file(before, after=after, show_ui=show_ui, modal=modal_ui)
                if online_output is not None and output.exists():
                    online_output.write_bytes(output.read_bytes())
            else:
                source.request_acquire(show_ui=show_ui, modal_ui=modal_ui)
                result = source.xfer_image_natively()
                if not result:
                    raise RuntimeError("TWAIN transfer returned no image handle")

                handle, remaining = result
                save_dib(handle, output)
                if online_output is not None:
                    save_dib(handle, online_output)
        finally:
            source.close()

    data: dict[str, Any] = {
        "ok": output.exists() and output.stat().st_size > 0,
        "source": source_name,
        "output": str(output),
        "output_bytes": output.stat().st_size if output.exists() else 0,
        "remaining_transfers": remaining,
    }
    if online_output is not None:
        data["online_output"] = str(online_output)
        data["online_output_bytes"] = online_output.stat().st_size if online_output.exists() else 0
    return data


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Leica TWAIN capture helper")
    parser.add_argument(
        "action",
        choices=["sources", "capabilities", "capture"],
        help="list TWAIN sources, inspect capabilities, or capture one frame",
    )
    parser.add_argument("--source", default="Leica Microsystems Camera")
    parser.add_argument("--output", help="exact image output path for capture")
    parser.add_argument(
        "--online-output",
        help="optional second path updated with the latest captured image",
    )
    parser.add_argument("--show-ui", action="store_true", help="show TWAIN UI")
    parser.add_argument("--modal-ui", action="store_true", help="request modal TWAIN UI")
    parser.add_argument(
        "--transfer-mech",
        choices=["native", "file"],
        default="native",
        help="TWAIN transfer mechanism to use for capture",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.action == "sources":
        print(json.dumps({"ok": True, "sources": twain_sources()}, indent=2))
        return 0
    if args.action == "capabilities":
        print(json.dumps(inspect_capabilities(args.source), indent=2, default=str))
        return 0

    if not args.output:
        raise RuntimeError("--output is required for capture")

    result = capture_once(
        source_name=args.source,
        output=Path(args.output),
        online_output=Path(args.online_output) if args.online_output else None,
        show_ui=args.show_ui,
        modal_ui=args.modal_ui,
        transfer_mech=args.transfer_mech,
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
