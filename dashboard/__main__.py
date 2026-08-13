#!/usr/bin/env python3
"""Run every device node and serve the panel.

    python -m dashboard                 # http://127.0.0.1:8770/
    python -m dashboard --port 8770 --allow-arm-motion

Registers one node per instrument, starts them, and serves the status cards plus
the live operation terminal. Arm motion stays gated: without --allow-arm-motion
the arm node plans and never commands.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dashboard.app import build_lab  # noqa: E402
from dashboard.server import serve  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8770)
    p.add_argument("--config", default=None, help="config.yaml (default: repo root)")
    p.add_argument("--allow-arm-motion", action="store_true",
                   help="LIVE: let the dashboard's arm node command motion")
    p.add_argument("--allow-uv-vis-motion", action="store_true",
                   help="LIVE: let the UV-Vis node move the plate carrier")
    return p


def run(args: argparse.Namespace) -> int:
    lab = build_lab(allow_arm_motion=args.allow_arm_motion,
                    allow_uv_vis_motion=args.allow_uv_vis_motion,
                    config=args.config)
    lab.start_all()
    serve(lab, host=args.host, port=args.port)
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
