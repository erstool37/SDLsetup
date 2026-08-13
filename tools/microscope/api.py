"""The microscope's main API -- what an orchestration script calls.

    from tools import microscope

    scope = microscope.Microscope.from_config("config.yaml")
    frame = scope.grab_frame(Path("shot.jpg"))       # next published live frame
    sample = scope.measure_focus(frame.path)         # sharpness + quality flags
    results = scope.capture(batch_id="A1")           # full-resolution, both cameras

One device package, two cameras: the Leica K3C and the TIS DFK33UX264 are
captured together, published together, and calibrated together, so they are one
lifecycle-managed aggregate rather than two.

Frames come from one of two places and the difference matters
=============================================================

``capture()`` drives the cameras directly through their vendor backends: full
control, full resolution, several seconds, and it needs exclusive access to the
device.

``grab_frame()`` copies a frame the **live session** already published to
``/tmp/sdl_microscope_stream``. It cannot configure anything, but it does not
contend for the camera, which is what makes it the right source during a closed
loop while the live monitor is running.

Two measured facts about the published stream, both load-bearing:

* the file's mtime is its **publish** time, not its exposure time -- publishing
  runs about 3.0 s apart while a capture takes about 2 s, so the first frame to
  appear after a move may have been exposed *before* that move finished. Taking
  the **second** frame clears it with a measured 4.84 s margin against 1.89 s.
* a frame appears before it is complete, so it is copied only once its size and
  mtime have stopped changing and the JPEG end-of-image marker is present.

This layer reports frames and measurements with quality flags. It does not
retry a measurement, choose the next well, or decide a plane is in focus --
that is :mod:`tools`.
"""
from __future__ import annotations

import dataclasses
import datetime
import json
import shutil
import time
from pathlib import Path
from typing import Any

from .. import config as _config
from . import focus as _focus
from .focus import FocusSample

#: Hardware inventory shipped beside this module.
INVENTORY_PATH = Path(__file__).resolve().parent / "cameras.json"

#: Where the live session publishes its newest frame per camera.
DEFAULT_STREAM_DIR = Path("/tmp/sdl_microscope_stream")

#: The live monitor's HTTP API (record toggle, save-latest).
DEFAULT_MONITOR_URL = "http://127.0.0.1:8766"

#: Published file name per camera. The Leica is the one through the objective.
STREAM_FILES = {
    "leica": "current_leica_k3c.jpg",
    "tis": "current_tis_dfk33ux264.jpg",
}
LINKED_CAMERAS = ("leica", "tis")

#: The camera looking through the microscope objective -- the one every
#: focus and centring procedure measures.
OBJECTIVE_CAMERA = "leica"

#: Frames published after the request, of which to take the n-th. See above.
DEFAULT_FRAME_ORDINAL = 2

DEFAULT_GRAB_TIMEOUT_S = 25.0


class MicroscopeError(RuntimeError):
    """A frame could not be obtained, or a camera refused a request."""


@dataclasses.dataclass
class Frame:
    """One saved frame and where it came from. Data, not a verdict."""

    ok: bool
    path: Path | None = None
    camera: str = OBJECTIVE_CAMERA
    published_at: float | None = None
    reason: str = ""

    def require(self) -> Path:
        """The path, or raise with the recorded reason."""
        if not self.ok or self.path is None:
            raise MicroscopeError(self.reason or "no frame")
        return self.path

    def as_dict(self) -> dict:
        return {"ok": self.ok, "path": str(self.path) if self.path else None,
                "camera": self.camera, "published_at": self.published_at,
                "reason": self.reason}


def jpeg_complete(path: Path) -> bool:
    """True if the file ends with the JPEG end-of-image marker FF D9."""
    try:
        if path.stat().st_size < 4:
            return False
        with path.open("rb") as handle:
            handle.seek(-2, 2)
            return handle.read(2) == b"\xff\xd9"
    except OSError:
        return False


def frame_exposure_start(source: Path) -> float | None:
    """When the published frame began exposing, epoch seconds, or None.

    The live session writes a JSON sidecar beside every frame carrying
    ``started_at`` and ``ended_at``. That is strictly better information than
    the file's mtime, which only says when writing finished: it lets a caller
    ask "was this frame exposed after my move?" instead of inferring it by
    counting publish cycles.

    Returns None -- never raises and never guesses -- if the sidecar is absent,
    truncated, or has no parseable ``started_at``. Callers must treat None as
    "unknown" and fall back, because a fabricated timestamp here would silently
    pair a frame with the wrong arm pose.

    Tolerates trailing bytes after the first JSON object: the TIS sidecar has
    been observed carrying several concatenated documents, which ``json.load``
    rejects outright with "Extra data".
    """
    try:
        raw = Path(str(source) + ".json").read_text()
    except OSError:
        return None
    try:
        payload, _end = json.JSONDecoder().raw_decode(raw.lstrip())
        stamp = payload.get("started_at")
        if not stamp:
            return None
        return datetime.datetime.fromisoformat(str(stamp)).timestamp()
    except (ValueError, AttributeError, TypeError):
        return None


@dataclasses.dataclass(frozen=True)
class MicroscopeSettings:
    """Resolved camera-layer settings, with the layer each value came from."""

    stream_dir: Path = DEFAULT_STREAM_DIR
    monitor_url: str = DEFAULT_MONITOR_URL
    output_dir: Path = _config.PROJECT_ROOT / "dataset" / "captures"
    camera_config: Path = INVENTORY_PATH
    objective_camera: str = OBJECTIVE_CAMERA
    frame_ordinal: int = DEFAULT_FRAME_ORDINAL
    grab_timeout_s: float = DEFAULT_GRAB_TIMEOUT_S
    focus_method: str = "tenengrad"
    focus_min_mean: float = _focus.BRIGHTNESS_MIN_MEAN
    focus_masked: bool = True
    sources: dict = dataclasses.field(default_factory=dict)

    @classmethod
    def from_config(cls, config: Any = None, **overrides: Any) -> MicroscopeSettings:
        lab = _config.LabConfig.load(config)
        live = lab.section("live_session")
        scope = lab.section("microscope")
        resolved: dict[str, Any] = {}
        sources: dict[str, str] = {}

        def take(key: str, default: Any, section: dict, cast: type | None = None) -> None:
            item = _config.resolve(key, argument=overrides.get(key), config=section,
                                   default=default, inventory_name="cameras.json", cast=cast)
            resolved[key] = item.value
            sources[key] = item.source

        take("objective_camera", OBJECTIVE_CAMERA, scope, str)
        take("frame_ordinal", DEFAULT_FRAME_ORDINAL, scope, int)
        take("grab_timeout_s", DEFAULT_GRAB_TIMEOUT_S, scope, float)
        take("focus_method", "tenengrad", scope, str)
        take("focus_min_mean", _focus.BRIGHTNESS_MIN_MEAN, scope, float)
        take("focus_masked", True, scope, bool)
        take("monitor_url", DEFAULT_MONITOR_URL, scope, str)

        stream = overrides.get("stream_dir") or live.get("stream_dir") or DEFAULT_STREAM_DIR
        resolved["stream_dir"] = Path(stream)
        sources["stream_dir"] = ("argument" if overrides.get("stream_dir")
                                 else "config.yaml" if live.get("stream_dir") else "default")
        output = overrides.get("output_dir") or lab.raw.get("output_dir")
        default_out = _config.PROJECT_ROOT / "dataset" / "captures"
        resolved["output_dir"] = Path(output) if output else default_out
        sources["output_dir"] = ("argument" if overrides.get("output_dir")
                                 else "config.yaml" if lab.raw.get("output_dir") else "default")
        cameras = overrides.get("camera_config") or lab.path_for(
            lab.raw.get("camera_config"), default=INVENTORY_PATH)
        resolved["camera_config"] = Path(cameras)
        sources["camera_config"] = ("argument" if overrides.get("camera_config")
                                    else "config.yaml" if lab.raw.get("camera_config")
                                    else "default")

        if resolved["objective_camera"] not in STREAM_FILES:
            raise _config.ConfigError(
                f"microscope.objective_camera must be one of {sorted(STREAM_FILES)}, "
                f"got {resolved['objective_camera']!r}")
        if resolved["frame_ordinal"] < 1:
            raise _config.ConfigError("microscope.frame_ordinal must be >= 1")
        if resolved["focus_method"] not in _focus.FOCUS_METHODS:
            raise _config.ConfigError(
                f"microscope.focus_method must be one of {_focus.FOCUS_METHODS}, "
                f"got {resolved['focus_method']!r}")
        return cls(sources=sources, **resolved)


class Microscope:
    """The two cameras, their published stream, and focus measurement."""

    def __init__(self, settings: MicroscopeSettings | None = None, *, log=None) -> None:
        self.settings = settings or MicroscopeSettings()
        self._log = log or (lambda message: print(message, flush=True))

    @classmethod
    def from_config(cls, config: Any = None, *, log=None, **overrides: Any) -> Microscope:
        return cls(MicroscopeSettings.from_config(config, **overrides), log=log)

    # -- the published live stream ------------------------------------------
    def stream_path(self, camera: str | None = None) -> Path:
        cam = camera or self.settings.objective_camera
        try:
            return self.settings.stream_dir / STREAM_FILES[cam]
        except KeyError:
            raise MicroscopeError(
                f"unknown camera {cam!r}; known: {sorted(STREAM_FILES)}") from None

    def latest_frame(self, camera: str | None = None) -> Frame:
        """The newest published frame, however old. No waiting."""
        cam = camera or self.settings.objective_camera
        path = self.stream_path(cam)
        if not path.exists():
            return Frame(False, camera=cam,
                         reason=f"no published frame at {path}; is the live session running?")
        return Frame(True, path=path, camera=cam, published_at=path.stat().st_mtime)

    def stream_age_s(self, camera: str | None = None) -> float | None:
        """Seconds since the newest frame was published, or None if there is none."""
        path = self.stream_path(camera)
        try:
            return time.time() - path.stat().st_mtime
        except OSError:
            return None

    def grab_frame(self, dest: Path, *, camera: str | None = None,
                   ordinal: int | None = None,
                   after: float | None = None,
                   timeout_s: float | None = None) -> Frame:
        """Copy a published frame that is known to post-date the arm's move.

        Two ways to be sure of that, and they cost very different amounts of
        time. Measured on this rig 2026-08-04: the stream publishes every
        **3.05 s**, each capture round-trip takes **2.12 s**, and the sensor
        exposure inside it is 60 ms.

        ``ordinal=2`` (the historical default) waits for the *second* frame to
        appear, because the first one may have begun exposing before the move
        finished. It is correct and needs nothing but the file's mtime, but it
        pays a whole extra publish cycle to establish something the publisher
        already recorded: an average **4.57 s** per frame.

        ``after=<epoch seconds>`` instead accepts the first frame whose sidecar
        says it **started exposing** at or after that moment -- normally the
        instant the move returned. That is the same guarantee, checked rather
        than assumed, and it costs an average **3.66 s**. Verified before being
        relied on: the sidecar's ``ended_at`` matches the file's own mtime to
        within 0.02 s across consecutive frames, so the two clocks agree.

        ``after`` falls back to the ordinal rule for any frame whose sidecar is
        missing or unparseable, so a publisher that stops writing sidecars costs
        speed and never correctness.
        """
        cam = camera or self.settings.objective_camera
        source = self.stream_path(cam)
        want = self.settings.frame_ordinal if ordinal is None else int(ordinal)
        limit = self.settings.grab_timeout_s if timeout_s is None else float(timeout_s)
        gate = None if after is None else float(after)

        started = time.time()
        deadline = started + limit
        seen, last_mtime = 0, None

        while time.time() < deadline:
            try:
                mtime = source.stat().st_mtime
            except FileNotFoundError:
                time.sleep(0.2)
                continue
            except OSError as exc:
                return Frame(False, camera=cam,
                             reason=f"cannot stat the live frame: {exc!r}")
            if mtime <= started or mtime == last_mtime:
                time.sleep(0.2)
                continue
            last_mtime, seen = mtime, seen + 1
            if gate is not None:
                exposed = frame_exposure_start(source)
                if exposed is not None:
                    # The publisher told us when this frame began exposing. Use
                    # it, and ignore the ordinal entirely -- that rule exists
                    # only to approximate this one.
                    if exposed < gate:
                        continue
                elif seen < want:
                    continue      # no sidecar: fall back to counting frames
            elif seen < want:
                continue
            # Settled-copy: a frame appears before it is finished being written.
            for _ in range(10):
                try:
                    first = source.stat()
                    time.sleep(0.25)
                    second = source.stat()
                    if (first.st_size, first.st_mtime) != (second.st_size, second.st_mtime):
                        continue
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, dest)
                except OSError:
                    time.sleep(0.25)
                    continue
                if jpeg_complete(dest):
                    return Frame(True, path=dest, camera=cam,
                                 published_at=second.st_mtime)
            return Frame(False, camera=cam,
                         reason="the live frame never settled into a complete JPEG")
        return Frame(False, camera=cam,
                     reason=(f"only {seen} of {want} frames arrived within {limit:.0f} s; "
                             f"is the live session running?"))

    # -- measurement -----------------------------------------------------------
    def measure_focus(self, path: Path, *, method: str | None = None,
                      min_mean: float | None = None,
                      masked: bool | None = None) -> FocusSample:
        """Sharpness of a saved frame, with the flags needed to trust it."""
        return _focus.score_frame(
            Path(path),
            method=method or self.settings.focus_method,
            min_mean=self.settings.focus_min_mean if min_mean is None else min_mean,
            masked=self.settings.focus_masked if masked is None else masked,
        )

    def detect_marker(self, path: Path, *, mode: str = "dark") -> dict:
        """Locate the operator's alignment mark. Reports both estimates + disagreement."""
        from . import marker
        return marker.detect(Path(path), mode=mode)

    # -- direct capture (vendor backends) ---------------------------------------
    def capture(self, *, camera: str = "all", batch_id: str | None = None,
                image_format: str | None = None,
                output_dir: Path | None = None) -> list:
        """Full-resolution capture through the vendor backends.

        Needs exclusive device access -- do not call this while a closed loop is
        reading the published stream from the same camera.
        """
        from . import capture as _capture

        config = _capture.load_config(self.settings.camera_config)
        return _capture.capture_batch(
            config=config, camera_name=camera, image_format=image_format,
            batch_id=batch_id or time.strftime("%Y%m%d_%H%M%S"),
            start_timeout_s=5.0,
            output_dir=Path(output_dir) if output_dir else self.settings.output_dir,
        )

    # -- the live monitor -------------------------------------------------------
    def _monitor(self, path: str, method: str = "GET", timeout_s: float = 3.0) -> dict:
        import json
        import urllib.request

        request = urllib.request.Request(self.settings.monitor_url + path, method=method)
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.load(response)

    def monitor_status(self) -> dict:
        return self._monitor("/api/status")

    def save_photo(self) -> dict:
        """Ask the live session to save the current frame from both cameras."""
        self._log("  [scope] take photo (both cameras)")
        return self._monitor("/api/save-latest", "POST")

    def start_recording(self) -> dict:
        self._log("  [scope] start recording")
        return self._monitor("/api/record/start", "POST")

    def stop_recording(self) -> dict:
        self._log("  [scope] stop recording")
        return self._monitor("/api/record/stop", "POST")

    # -- status ------------------------------------------------------------------
    def status(self) -> dict:
        cameras = {}
        for cam in LINKED_CAMERAS:
            age = self.stream_age_s(cam)
            cameras[cam] = {"online": age is not None,
                            "age_s": round(age, 1) if age is not None else None}
        info = {"linked": list(LINKED_CAMERAS), "cameras": cameras,
                "stream_dir": str(self.settings.stream_dir),
                "monitor": self.settings.monitor_url,
                "objective_camera": self.settings.objective_camera}
        try:
            info["recording"] = (self.monitor_status().get("recording") or {}).get("recording")
        except Exception:
            info["recording"] = None
        return info


# ---------------------------------------------------------------------------
# module-level shorthand
# ---------------------------------------------------------------------------

_DEFAULT: Microscope | None = None
_DEFAULT_KEY: tuple | None = None


def default(config: Any = None, **overrides: Any) -> Microscope:
    """The process-wide microscope session behind the shorthands."""
    global _DEFAULT, _DEFAULT_KEY
    key = (repr(config), tuple(sorted(overrides.items(), key=lambda kv: kv[0])))
    if _DEFAULT is None or _DEFAULT_KEY != key:
        _DEFAULT = Microscope.from_config(config, **overrides)
        _DEFAULT_KEY = key
    return _DEFAULT


def grab_frame(dest: Path, config: Any = None, **overrides: Any) -> Frame:
    return default(config).grab_frame(Path(dest), **overrides)


def latest_frame(camera: str | None = None, config: Any = None) -> Frame:
    return default(config).latest_frame(camera)


def measure_focus(path: Path, config: Any = None, **overrides: Any) -> FocusSample:
    return default(config).measure_focus(Path(path), **overrides)


# NOTE: there is deliberately no module-level ``capture()`` shorthand. The name
# belongs to the ``capture`` submodule, and a function of the same name exported
# from the package would shadow it -- ``from .. import capture`` would silently
# hand out the function. Use ``Microscope.capture()`` or ``batch.capture_both()``.


def status(config: Any = None) -> dict:
    return default(config).status()


__all__ = [
    "DEFAULT_FRAME_ORDINAL",
    "DEFAULT_MONITOR_URL",
    "DEFAULT_STREAM_DIR",
    "Frame",
    "INVENTORY_PATH",
    "LINKED_CAMERAS",
    "Microscope",
    "MicroscopeError",
    "MicroscopeSettings",
    "OBJECTIVE_CAMERA",
    "STREAM_FILES",
    "default",
    "frame_exposure_start",
    "grab_frame",
    "jpeg_complete",
    "latest_frame",
    "measure_focus",
    "status",
]
