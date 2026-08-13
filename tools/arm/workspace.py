"""Taught locations, routes, and protocols -- the arm's persistent workspace.

This is the JSON store under ``~/.sdl_lab/robot_arm/workspace.json``: poses an
operator taught by hand, plus routes built from them. It is *state*, not
configuration -- it changes whenever the rig is re-taught, and it is the
source of the safety anchor every envelope is built around.

Reading and writing this file moves nothing; :class:`WorkspaceStore` has no
connection to the arm at all.
"""

from __future__ import annotations

import dataclasses
import datetime as dt
import json
from pathlib import Path
from typing import Any, Literal

DEFAULT_STORE = Path.home() / ".sdl_lab" / "robot_arm" / "workspace.json"


@dataclasses.dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    units: str = "mm_deg"


@dataclasses.dataclass
class Location:
    name: str
    pose: Pose
    description: str = ""
    created_at: str = dataclasses.field(
        default_factory=lambda: dt.datetime.now().isoformat(timespec="seconds")
    )
    metadata: dict[str, Any] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class RouteStep:
    kind: Literal["move", "grip", "release", "wait", "capture"]
    target: str = ""
    seconds: float = 0.0
    note: str = ""


@dataclasses.dataclass
class Route:
    name: str
    steps: list[RouteStep]
    description: str = ""
    created_at: str = dataclasses.field(
        default_factory=lambda: dt.datetime.now().isoformat(timespec="seconds")
    )


@dataclasses.dataclass
class Workspace:
    locations: dict[str, Location] = dataclasses.field(default_factory=dict)
    routes: dict[str, Route] = dataclasses.field(default_factory=dict)
    protocols: dict[str, Route] = dataclasses.field(default_factory=dict)


def _pose_from_dict(data: dict[str, Any]) -> Pose:
    return Pose(**data)


def _location_from_dict(data: dict[str, Any]) -> Location:
    copied = dict(data)
    copied["pose"] = _pose_from_dict(copied["pose"])
    return Location(**copied)


def _step_from_dict(data: dict[str, Any]) -> RouteStep:
    return RouteStep(**data)


def _route_from_dict(data: dict[str, Any]) -> Route:
    copied = dict(data)
    copied["steps"] = [_step_from_dict(step) for step in copied.get("steps", [])]
    return Route(**copied)


def _workspace_from_dict(data: dict[str, Any]) -> Workspace:
    return Workspace(
        locations={
            name: _location_from_dict(location)
            for name, location in data.get("locations", {}).items()
        },
        routes={name: _route_from_dict(route) for name, route in data.get("routes", {}).items()},
        protocols={
            name: _route_from_dict(route) for name, route in data.get("protocols", {}).items()
        },
    )


class WorkspaceStore:
    """Local JSON store for taught robot locations, routes, and protocols."""

    def __init__(self, path: Path = DEFAULT_STORE) -> None:
        self.path = path.expanduser()
        self.workspace = self._load()

    def _load(self) -> Workspace:
        if not self.path.exists():
            return Workspace()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        return _workspace_from_dict(data)

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = dataclasses.asdict(self.workspace)
        self.path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def add_location(self, location: Location) -> None:
        self.workspace.locations[location.name] = location
        self.save()

    def add_route(self, route: Route, *, protocol: bool = False) -> None:
        if protocol:
            self.workspace.protocols[route.name] = route
        else:
            self.workspace.routes[route.name] = route
        self.save()

    def require_location(self, name: str) -> Location:
        try:
            return self.workspace.locations[name]
        except KeyError as exc:
            raise KeyError(f"unknown location {name!r}") from exc

    def require_route(self, name: str, *, protocol: bool = False) -> Route:
        source = self.workspace.protocols if protocol else self.workspace.routes
        try:
            return source[name]
        except KeyError as exc:
            kind = "protocol" if protocol else "route"
            raise KeyError(f"unknown {kind} {name!r}") from exc

    def route_between(
        self,
        *,
        name: str,
        from_location: str,
        to_location: str,
        waypoints: list[str] | None = None,
        description: str = "",
    ) -> Route:
        ordered_locations = [from_location, *(waypoints or []), to_location]
        for location_name in ordered_locations:
            self.require_location(location_name)
        return Route(
            name=name,
            description=description,
            steps=[
                RouteStep(kind="move", target=location_name)
                for location_name in ordered_locations
            ],
        )

    def plate_to_microscope_protocol(
        self,
        *,
        name: str,
        plate_location: str,
        microscope_location: str,
        approach_route: str | None = None,
        transfer_route: str | None = None,
    ) -> Route:
        self.require_location(plate_location)
        self.require_location(microscope_location)
        steps: list[RouteStep] = []
        if approach_route:
            steps.append(
                RouteStep(
                    kind="move",
                    target=f"route:{approach_route}",
                    note="go to plate area",
                )
            )
        else:
            steps.append(RouteStep(kind="move", target=plate_location, note="go to plate area"))
        steps.extend(
            [
                RouteStep(kind="grip", target="96_well_plate", note="pick plate"),
                RouteStep(kind="wait", seconds=0.5, note="settle after gripping"),
            ]
        )
        if transfer_route:
            steps.append(
                RouteStep(
                    kind="move",
                    target=f"route:{transfer_route}",
                    note="carry plate to microscope",
                )
            )
        else:
            steps.append(
                RouteStep(kind="move", target=microscope_location, note="carry plate to microscope")
            )
        steps.extend(
            [
                RouteStep(kind="release", target="96_well_plate", note="place plate on microscope"),
                RouteStep(kind="wait", seconds=1.0, note="settle before imaging"),
                RouteStep(
                    kind="capture",
                    target="camera:all",
                    note="near-simultaneous dual capture",
                ),
            ]
        )
        return Route(
            name=name,
            description="Pick a 96 well plate from location 1 and place it on the microscope.",
            steps=steps,
        )

    def dry_run(self, route: Route) -> list[dict[str, Any]]:
        expanded: list[dict[str, Any]] = []
        for index, step in enumerate(route.steps, start=1):
            record = dataclasses.asdict(step)
            record["index"] = index
            if step.kind == "move" and step.target.startswith("route:"):
                nested = self.require_route(step.target.removeprefix("route:"))
                record["route_steps"] = [
                    dataclasses.asdict(nested_step) for nested_step in nested.steps
                ]
            elif step.kind == "move":
                location = self.require_location(step.target)
                record["pose"] = dataclasses.asdict(location.pose)
            expanded.append(record)
        return expanded


#: Former name, kept so older notes and transcripts still resolve.
RobotProtocolStore = WorkspaceStore
