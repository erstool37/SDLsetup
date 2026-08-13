"""The arm's single command surface: ``python -m tools.arm``.

Also installed as ``sdl-robot``. It folds together what used to be two
programs -- the teach/route protocol CLI and a separate ``scripts/gripper.py``
-- because they drive one device and were each resolving the host and opening
their own connection.

Everything here that can move hardware goes through
:class:`tools.arm.Arm`, so the CLI is subject to exactly the same
envelope and readback guards a procedure is.

    python -m tools.arm status          # taught locations and routes
    python -m tools.arm settings        # resolved config + provenance
    python -m tools.arm controller      # read-only controller state
    python -m tools.arm gripper status  # read-only
    python -m tools.arm gripper read    # read-only: current jaw span in mm
    python -m tools.arm gripper open    # jaw-only actuation
    python -m tools.arm gripper set --mm 96   # partial span; needs control_mode 1
    python -m tools.arm dry-run <route>
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

from .api import Arm, ArmSettings
from .workspace import DEFAULT_STORE, Location, Pose, WorkspaceStore


def make_store(args: argparse.Namespace) -> WorkspaceStore:
    return WorkspaceStore(args.store)


def print_json(data: object) -> None:
    print(json.dumps(data, indent=2))


def cmd_status(args: argparse.Namespace) -> int:
    store = make_store(args)
    print_json(
        {
            "store": str(store.path),
            "locations": sorted(store.workspace.locations),
            "routes": sorted(store.workspace.routes),
            "protocols": sorted(store.workspace.protocols),
            "movement_mode": "dry-run unless --execute / arm.allow_motion",
        }
    )
    return 0


def cmd_teach_location(args: argparse.Namespace) -> int:
    store = make_store(args)
    location = Location(
        name=args.name,
        pose=Pose(
            x=args.x,
            y=args.y,
            z=args.z,
            roll=args.roll,
            pitch=args.pitch,
            yaw=args.yaw,
        ),
        description=args.description or "",
    )
    store.add_location(location)
    print_json({"saved": dataclasses.asdict(location), "store": str(store.path)})
    return 0


def cmd_route(args: argparse.Namespace) -> int:
    store = make_store(args)
    route = store.route_between(
        name=args.name,
        from_location=args.from_location,
        to_location=args.to_location,
        waypoints=args.waypoint,
        description=args.description or "",
    )
    store.add_route(route)
    print_json({"saved": dataclasses.asdict(route), "store": str(store.path)})
    return 0


def cmd_plate_protocol(args: argparse.Namespace) -> int:
    store = make_store(args)
    protocol = store.plate_to_microscope_protocol(
        name=args.name,
        plate_location=args.plate_location,
        microscope_location=args.microscope_location,
        approach_route=args.approach_route,
        transfer_route=args.transfer_route,
    )
    store.add_route(protocol, protocol=True)
    print_json({"saved": dataclasses.asdict(protocol), "store": str(store.path)})
    return 0


def cmd_dry_run(args: argparse.Namespace) -> int:
    store = make_store(args)
    route = store.require_route(args.name, protocol=args.protocol)
    print_json({"name": route.name, "dry_run": store.dry_run(route)})
    return 0


def cmd_move(args: argparse.Namespace) -> int:
    store = make_store(args)
    route = store.require_route(args.name, protocol=args.protocol)
    if args.execute:
        print(
            "error: no live robot hardware adapter is configured; refusing real movement",
            file=sys.stderr,
        )
        return 2
    print_json({"name": route.name, "dry_run": store.dry_run(route)})
    return 0


def cmd_settings(args: argparse.Namespace) -> int:
    """Print the resolved configuration and which layer each value came from."""
    print(ArmSettings.from_config(args.config, host=args.host).describe())
    return 0


def cmd_controller(args: argparse.Namespace) -> int:
    """Read-only controller state. Connects, reads, disconnects; moves nothing."""
    with Arm.from_config(args.config, live=True, host=args.host) as arm:
        print_json(arm.status())
    return 0


def cmd_gripper(args: argparse.Namespace) -> int:
    """BIO Gripper G2 control. Jaw-only: this never commands arm motion.

    The jaws are force-limited (<=20 N) over a 71-150 mm range and the plate is
    about 85 mm across, so on a full close the firmware stops on the plate and
    holds it.

    ``set --mm`` drives a specific span instead, which needs the gripper in
    position mode (``gripper.control_mode: 1`` in arm.json, or ``--control-mode
    1`` here). The mode is stored in the gripper and survives a power cycle.
    """
    overrides: dict = {}
    if args.speed is not None:
        # 'set' takes --speed as a direct argument below, so it is deliberately
        # not folded in here as well.
        overrides = {"open_speed": args.speed, "close_speed": args.speed}
    if args.control_mode is not None:
        overrides["control_mode"] = args.control_mode
    with Arm.from_config(args.config, live=True, host=args.host, **overrides) as arm:
        if args.action == "status":
            print_json({"controller": arm.connection.status(),
                        "bio_status": arm.connection.bio_status(),
                        "bio_error": arm.connection.bio_error()})
            return 0
        if args.action == "read":
            reading = arm.opening()
            if not reading["units_verified"]:
                print(f"note: gripper is in control_mode "
                      f"{reading['control_mode']} (open/close); the mm reading "
                      f"below is unverified in that mode", file=sys.stderr)
            print_json(reading)
            return 0
        if args.action == "set":
            if args.mm is None:
                print("gripper set requires --mm (71-150)", file=sys.stderr)
                return 2
            print_json(arm.set_opening(args.mm, speed=args.speed, force=args.force))
            return 0
        if args.action == "open":
            print_json(arm.release())
        elif args.action == "close":
            print_json(arm.grip())
        elif args.action == "cycle":
            print_json(arm.release())
            time.sleep(args.pause)
            print_json(arm.grip())
    return 0


def add_store_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--store",
        type=Path,
        default=DEFAULT_STORE,
        help=f"default: {DEFAULT_STORE}",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="xArm7 + BIO Gripper G2 control. Motion is dry-run unless "
                    "arm.allow_motion is set; the gripper is jaw-only and live.")
    add_store_arg(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    parser.add_argument("--config", default=None,
                        help="config.yaml to resolve settings from (default: repo root)")
    parser.add_argument("--host", default=None, help="override the controller address")

    status = sub.add_parser("status", help="show saved locations, routes, and protocols")
    status.set_defaults(func=cmd_status)

    settings = sub.add_parser("settings",
                              help="resolved configuration and where each value came from")
    settings.set_defaults(func=cmd_settings)

    controller = sub.add_parser("controller",
                                help="read-only controller state (connects, moves nothing)")
    controller.set_defaults(func=cmd_controller)

    gripper = sub.add_parser("gripper",
                             help="BIO Gripper G2 open/close/position "
                                  "(jaw-only; no arm motion)")
    gripper.add_argument("action",
                         choices=["status", "read", "open", "close", "cycle", "set"])
    gripper.add_argument("--mm", type=float, default=None,
                         help="'set' only: jaw span in mm (71-150). Both jaws move "
                              "together; there is no per-jaw command")
    gripper.add_argument("--force", type=int, default=None,
                         help="'set' only: 1-100 percent of the 20 N maximum; "
                              "default: grip_force from config")
    gripper.add_argument("--control-mode", type=int, default=None, dest="control_mode",
                         help="0 = open/close, 1 = position. Written to the gripper "
                              "and persists across a power cycle")
    gripper.add_argument("--speed", type=int, default=None,
                         help="0-4000; default: open_speed/close_speed from config "
                              "(the position path requires >=500)")
    gripper.add_argument("--pause", type=float, default=1.5,
                         help="seconds between open and close in 'cycle'")
    gripper.set_defaults(func=cmd_gripper)

    teach = sub.add_parser("teach-location", help="save a manually taught location pose")
    teach.add_argument("name")
    teach.add_argument("--x", type=float, required=True)
    teach.add_argument("--y", type=float, required=True)
    teach.add_argument("--z", type=float, required=True)
    teach.add_argument("--roll", type=float, required=True)
    teach.add_argument("--pitch", type=float, required=True)
    teach.add_argument("--yaw", type=float, required=True)
    teach.add_argument("--description")
    teach.set_defaults(func=cmd_teach_location)

    route = sub.add_parser("route", help="save a route through taught locations")
    route.add_argument("name")
    route.add_argument("--from-location", required=True)
    route.add_argument("--to-location", required=True)
    route.add_argument("--waypoint", action="append", default=[])
    route.add_argument("--description")
    route.set_defaults(func=cmd_route)

    plate = sub.add_parser("plate-to-microscope", help="save the 96 well plate imaging protocol")
    plate.add_argument("name")
    plate.add_argument("--plate-location", required=True)
    plate.add_argument("--microscope-location", required=True)
    plate.add_argument("--approach-route")
    plate.add_argument("--transfer-route")
    plate.set_defaults(func=cmd_plate_protocol)

    dry = sub.add_parser("dry-run", help="expand a saved route or protocol without moving hardware")
    dry.add_argument("name")
    dry.add_argument("--protocol", action="store_true")
    dry.set_defaults(func=cmd_dry_run)

    move = sub.add_parser(
        "move",
        help="dry-run a move; live execution is disabled until adapter setup",
    )
    move.add_argument("name")
    move.add_argument("--protocol", action="store_true")
    move.add_argument("--execute", action="store_true", help="reserved for future live adapter")
    move.set_defaults(func=cmd_move)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
