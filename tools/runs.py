"""Per-run data collection: one folder per run, one sub-folder per device.

**``dataset/`` mirrors ``scripts/``**: one folder per phase, named after the
script that writes it, then one folder per run inside. Finding a run means going
to the folder named after the script you ran.

Every run writes into a single directory holding everything it produced -- each
device's measurements, each device's log, the operator-visible transcript, and a
manifest saying how to reproduce it::

    dataset/calibration/2026-08-03_143012/
      manifest.json              what this run was, and how to regenerate it
      run.log                    the full operator-visible transcript
      devices/
        arm/
          arm.log                this device's own log
          moves.jsonl            one record per commanded pose
          poses.json             taught/measured poses this run used
        microscope/
          microscope.log
          frames/                every frame captured, with its metadata sidecar
          focus.jsonl            one record per focus measurement
        uv_vis/
          uv_vis.log
          raw/                   exports pulled off the reader
          absorbance.csv
      results/                   derived artifacts: plots, reports, fitted models

Why per-device sub-folders and not one flat pile: the devices are operated
independently and are added to over time, so a run's shape should follow the rig
rather than a fixed schema. A device that was not used simply has no folder.

Why a manifest is mandatory: months later the only questions asked of a run are
"what produced this number" and "how do I regenerate this figure", and neither
the code, the summary, nor git history answers them. Unknown fields are written
as ``"UNKNOWN"`` with a note -- never guessed, because an invented commit hash
turns an open question into a false certainty.

Usage::

    from tools import runs

    with runs.Run.create("align-a1", config="config.yaml", argv=sys.argv) as run:
        run.log("starting")
        run.record("arm", "moves", {"label": "z step", "z": 177.2})
        frame = run.device_dir("microscope", "frames") / "a1_000.jpg"
        ...
        run.metric("best_z_mm", 180.2)

The context manager writes the manifest on the way out, including on failure --
a run that crashed is exactly the one whose record matters most.
"""
from __future__ import annotations

import contextlib
import dataclasses
import datetime as dt
import io
import json
import os
import platform
import socket
import subprocess
import sys
import time
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from . import config as _config

#: Default root for collected run data: ``dataset/`` beside the phase scripts.
#: It sits inside the repo so a run's output is next to the code that made it,
#: and ``dataset/.gitignore`` excludes every byte of it -- the data grows without
#: bound and is machine-specific. Override with ``data.root`` in config.yaml or
#: the ``SDL_DATA_ROOT`` environment variable.
DEFAULT_DATA_ROOT = _config.PROJECT_ROOT / "dataset"

#: Written into every run folder so a stray `git add` has something to trip on.
_GITIGNORE = "# Collected run data. Never commit.\n*\n"

STATUS_RUNNING = "running"
STATUS_OK = "ok"
STATUS_FAILED = "failed"
STATUS_ABORTED = "aborted"

UNKNOWN = "UNKNOWN"


def data_root(config: Any = None) -> Path:
    """Resolve the run-data root: argument > config.yaml > env > default."""
    lab = _config.LabConfig.load(config)
    section = lab.section("data")
    configured = section.get("root") or lab.raw.get("data_root")
    if configured:
        # Resolved against the CONFIG FILE, never the process working directory:
        # the same config launched from main.sh, a service, or a shell must put
        # its runs in the same place.
        return lab.path_for(configured, default=DEFAULT_DATA_ROOT)
    env = os.environ.get("SDL_DATA_ROOT")
    if env:
        return Path(env).expanduser()
    return DEFAULT_DATA_ROOT


def _git_provenance(repo: Path) -> dict:
    """Commit and dirty-state of the code that produced a run.

    A dirty tree is recorded as dirty *and* its diff is saved beside the run --
    "dirty" without the diff is not reproducible.
    """
    def git(*args: str) -> str | None:
        try:
            out = subprocess.run(["git", "-C", str(repo), *args], capture_output=True,
                                 text=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            return None
        return out.stdout.strip() if out.returncode == 0 else None

    commit = git("rev-parse", "HEAD")
    if commit is None:
        return {"commit": UNKNOWN, "note": "not a git checkout, or git unavailable",
                "dirty": UNKNOWN}
    status = git("status", "--porcelain")
    return {"commit": commit, "branch": git("rev-parse", "--abbrev-ref", "HEAD") or UNKNOWN,
            "dirty": bool(status), "dirty_files": len(status.splitlines()) if status else 0}


def _environment() -> dict:
    versions = {}
    for name in ("numpy", "cv2", "PIL", "xarm"):
        try:
            # The xArm SDK prints its version banner on import; probing for a
            # manifest field should not put a line in the run transcript.
            with contextlib.redirect_stdout(io.StringIO()):
                module = __import__(name)
            versions[name] = getattr(module, "__version__", UNKNOWN)
        except Exception:
            versions[name] = "not installed"
    return {
        "python": sys.version.split()[0],
        "executable": sys.executable,
        "host": socket.gethostname(),
        "platform": platform.platform(),
        "packages": versions,
    }


@dataclasses.dataclass
class Run:
    """One run's folder, its per-device sub-folders, and its manifest."""

    run_id: str
    path: Path
    name: str
    started_at: float
    config_path: Path | None = None
    description: str = ""
    argv: list = dataclasses.field(default_factory=list)
    status: str = STATUS_RUNNING
    metrics: dict = dataclasses.field(default_factory=dict)
    devices: list = dataclasses.field(default_factory=list)
    produced: list = dataclasses.field(default_factory=list)
    notes: list = dataclasses.field(default_factory=list)
    _log_handle: Any = dataclasses.field(default=None, repr=False)
    _device_logs: dict = dataclasses.field(default_factory=dict, repr=False)
    _saved_diff: str | None = dataclasses.field(default=None, repr=False)

    # -- creation ---------------------------------------------------------
    @classmethod
    def create(cls, name: str, *, config: Any = None, root: Path | None = None,
               description: str = "", argv: Iterable[str] | None = None,
               devices: Iterable[str] = ()) -> Run:
        """Make a new run folder. The name is a slug, not a path."""
        slug = "".join(c if (c.isalnum() or c in "-_") else "-" for c in name).strip("-")
        if not slug:
            raise ValueError(f"run name {name!r} produces an empty slug")
        base = Path(root) if root is not None else data_root(config)
        now = dt.datetime.now()
        # dataset/ mirrors scripts/: one folder per phase, then one per run.
        # scripts/calibration.py writes into dataset/calibration/<stamp>/, so
        # finding a run means going to the folder named after the script.
        stamp = now.strftime("%Y-%m-%d_%H%M%S")
        path = base / slug / stamp
        suffix = 1
        while path.exists():
            suffix += 1
            stamp = f"{now.strftime('%Y-%m-%d_%H%M%S')}-{suffix}"
            path = base / slug / stamp
        run_id = f"{slug}/{stamp}"

        path.mkdir(parents=True)
        (path / "devices").mkdir()
        (path / "results").mkdir()
        (base / ".gitignore").write_text(_GITIGNORE, encoding="utf-8")

        lab = _config.LabConfig.load(config)
        run = cls(run_id=run_id, path=path, name=slug,
                  started_at=time.time(), config_path=lab.path,
                  description=description,
                  argv=list(argv) if argv is not None else list(sys.argv))
        run._log_handle = (path / "run.log").open("a", encoding="utf-8")

        # Freeze the resolved config into the run: naming a config file is not
        # enough, because the file changes and the run does not.
        if lab.raw:
            (path / "config.resolved.json").write_text(
                json.dumps(lab.raw, indent=2, default=str), encoding="utf-8")
        for device in devices:
            run.device_dir(device)
        run.log(f"run {run.run_id} started ({run.name})")
        return run

    # -- layout -------------------------------------------------------------
    def device_dir(self, device: str, *parts: str) -> Path:
        """This run's folder for one device, created on first use."""
        target = self.path / "devices" / device
        for part in parts:
            target = target / part
        target.mkdir(parents=True, exist_ok=True)
        if device not in self.devices:
            self.devices.append(device)
        return target

    @property
    def results_dir(self) -> Path:
        return self.path / "results"

    # -- logging --------------------------------------------------------------
    def log(self, message: str, *, device: str | None = None,
            level: str = "info", echo: bool = True) -> None:
        """Append to the run transcript, and to that device's own log.

        Everything an operator would have seen on stdout ends up in ``run.log``;
        a device's own lines additionally land in its folder, so one device's
        behaviour can be read without untangling it from the rest.
        """
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        source = device or "run"
        line = f"{stamp} [{source}] {message}" if level == "info" else \
               f"{stamp} [{source}] {level.upper()}: {message}"
        if self._log_handle is not None:
            self._log_handle.write(line + "\n")
            self._log_handle.flush()
        if device is not None:
            handle = self._device_logs.get(device)
            if handle is None:
                handle = (self.device_dir(device) / f"{device}.log").open("a", encoding="utf-8")
                self._device_logs[device] = handle
            handle.write(line + "\n")
            handle.flush()
        if echo:
            print(message, flush=True)

    def logger(self, device: str):
        """A callable to hand a device as its ``log=``.

        Accepts ``(message)`` OR ``(message, level)``. A device's log callback
        may pass a level alongside the message -- the circulator link's
        ``_emit`` calls ``log_fn(message, level)`` -- so the returned callable
        must take one too, or a warn-level line from the device crashes the run
        with ``TypeError: <lambda>() takes 1 positional argument but 2 were
        given``. The level defaults to ``"info"`` for callers that pass only a
        message, and is threaded into :meth:`log` so a device warning stays a
        warning in the transcript.
        """
        return lambda message, level="info": self.log(
            str(message).rstrip(), device=device, level=level)

    # -- records ----------------------------------------------------------------
    def record(self, device: str, stream: str, payload: dict) -> Path:
        """Append one JSON record to ``devices/<device>/<stream>.jsonl``.

        JSONL because a run appends as it goes: a crash halfway through leaves
        every record written so far readable, which a single JSON document would
        not.
        """
        target = self.device_dir(device) / f"{stream}.jsonl"
        entry = {"ts": time.time(), **payload}
        with target.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(entry, default=str) + "\n")
        return target

    def write_json(self, device: str | None, name: str, payload: Any) -> Path:
        """Write one JSON document into a device folder, or into ``results/``."""
        base = self.device_dir(device) if device else self.results_dir
        target = base / (name if name.endswith(".json") else f"{name}.json")
        target.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
        return self.produce(target)

    def produce(self, path: Path, *, note: str = "") -> Path:
        """Register an artifact this run produced, for the manifest."""
        entry = {"path": str(path)}
        if note:
            entry["note"] = note
        if entry not in self.produced:
            self.produced.append(entry)
        return path

    def metric(self, key: str, value: Any) -> None:
        """Record a number this run asserts. These land in the manifest."""
        self.metrics[key] = value

    def note(self, text: str) -> None:
        """Something a later reader needs to know -- especially what was NOT checked."""
        self.notes.append(text)

    # -- finish -------------------------------------------------------------------
    def manifest(self) -> dict:
        repo = _config.PROJECT_ROOT
        provenance = _git_provenance(repo)
        if provenance.get("dirty") is True:
            provenance["diff_saved_as"] = (
                self._saved_diff or "UNKNOWN (git diff unavailable at finish time)")
        finished = time.time()
        return {
            "run_id": self.run_id,
            "name": self.name,
            "description": self.description,
            "status": self.status,
            "started_at": dt.datetime.fromtimestamp(self.started_at).isoformat(timespec="seconds"),
            "finished_at": dt.datetime.fromtimestamp(finished).isoformat(timespec="seconds"),
            "duration_s": round(finished - self.started_at, 1),
            "entrypoint": " ".join(self.argv) if self.argv else UNKNOWN,
            "code": {"repo": str(repo), **provenance},
            "config": {"path": str(self.config_path) if self.config_path else UNKNOWN,
                       "resolved_copy": "config.resolved.json"
                       if (self.path / "config.resolved.json").exists() else None},
            "environment": _environment(),
            "devices": sorted(self.devices),
            "metrics": self.metrics,
            "produced": self.produced,
            "notes": self.notes,
        }

    def _save_dirty_diff(self) -> str | None:
        """A dirty tree is only reproducible if its diff is saved with the run."""
        repo = _config.PROJECT_ROOT
        try:
            out = subprocess.run(["git", "-C", str(repo), "diff", "HEAD"],
                                 capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.SubprocessError):
            return None
        if out.returncode != 0 or not out.stdout.strip():
            return None
        target = self.path / "code.dirty.diff"
        target.write_text(out.stdout, encoding="utf-8")
        return target.name

    def finish(self, status: str = STATUS_OK, **metrics: Any) -> Path:
        self.status = status
        self.metrics.update(metrics)
        self._saved_diff = self._save_dirty_diff()
        target = self.path / "manifest.json"
        target.write_text(json.dumps(self.manifest(), indent=2, default=str), encoding="utf-8")
        self.log(f"run {self.run_id} finished: {status}")
        self.close()
        return target

    def close(self) -> None:
        for handle in [self._log_handle, *self._device_logs.values()]:
            with contextlib.suppress(Exception):
                if handle is not None:
                    handle.close()
        self._log_handle = None
        self._device_logs = {}

    def __enter__(self) -> Run:
        return self

    def __exit__(self, exc_type, exc, _tb) -> None:
        if exc_type is None:
            status = STATUS_OK
        elif issubclass(exc_type, KeyboardInterrupt):
            status = STATUS_ABORTED
        else:
            status = STATUS_FAILED
        if exc is not None:
            self.note(f"{status}: {exc_type.__name__}: {exc}")
        self.finish(status)


# ---------------------------------------------------------------------------
# finding runs afterwards
# ---------------------------------------------------------------------------

def list_runs(config: Any = None, *, root: Path | None = None,
              limit: int = 20) -> list[dict]:
    """Recent runs, newest first, read from their manifests."""
    base = Path(root) if root is not None else data_root(config)
    if not base.exists():
        return []
    found = []
    for phase in sorted(base.iterdir()):
        if not phase.is_dir() or phase.name.startswith("."):
            continue
        for run_path in sorted(phase.iterdir(), reverse=True):
            manifest = run_path / "manifest.json"
            if manifest.exists():
                try:
                    found.append(json.loads(manifest.read_text(encoding="utf-8")))
                except (OSError, json.JSONDecodeError):
                    found.append({"run_id": f"{phase.name}/{run_path.name}",
                                  "status": "unreadable manifest"})
            elif run_path.is_dir():
                found.append({"run_id": f"{phase.name}/{run_path.name}",
                              "status": STATUS_RUNNING, "note": "no manifest yet"})
    found.sort(key=lambda r: r.get("started_at", ""), reverse=True)
    return found[:limit]


def latest(config: Any = None, *, root: Path | None = None) -> dict | None:
    runs = list_runs(config, root=root, limit=1)
    return runs[0] if runs else None


__all__ = [
    "DEFAULT_DATA_ROOT",
    "Run",
    "STATUS_ABORTED",
    "STATUS_FAILED",
    "STATUS_OK",
    "STATUS_RUNNING",
    "UNKNOWN",
    "data_root",
    "latest",
    "list_runs",
]
