#!/usr/bin/env python3
"""Verify that the xArm Python SDK imports without connecting to hardware."""

from __future__ import annotations

import importlib.metadata
import io
import platform
from contextlib import redirect_stdout


def main() -> int:
    import_output = io.StringIO()
    with redirect_stdout(import_output):
        from xarm.wrapper import XArmAPI  # noqa: F401

    try:
        version = importlib.metadata.version("xarm-python-sdk")
    except importlib.metadata.PackageNotFoundError:
        version = "unknown"

    print("xArm SDK import: ok")
    print(f"xarm-python-sdk: {version}")
    print(f"python: {platform.python_version()}")
    print("hardware connection: not attempted")
    if import_output.getvalue().strip():
        print("sdk import output: captured")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
