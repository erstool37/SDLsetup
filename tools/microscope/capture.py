#!/usr/bin/env python3
"""USB camera capture CLI for the WSL SDL lab."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import dataclasses
import datetime as dt
import html
import http.server
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .. import config as _config

if TYPE_CHECKING:  # the annotation is a string; the import would be circular at runtime
    from .viewer import RecordingState

ROOT = Path(__file__).resolve().parent
#: Vendor capture programs and their PowerShell bridges. They resolve their own
#: helper .py files with $PSScriptRoot, so the pair must stay in this directory.
BACKENDS = ROOT / "backends"
PROJECT_ROOT = _config.PROJECT_ROOT
DEFAULT_CONFIG = ROOT / "cameras.json"
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "config.yaml"
DEFAULT_OUTPUT_DIR = Path("/home/lamp/SDLsetup/dataset/captures")
DEFAULT_USER_SAVED_DIR = DEFAULT_OUTPUT_DIR / "user-saved"
IMAGE_SIGNAL = BACKENDS / "image_signal.ps1"
LEICA_LASX_UI = BACKENDS / "leica_lasx_ui.ps1"
LEICA_TWAIN = BACKENDS / "leica_twain_capture.ps1"
TISGRABBER = BACKENDS / "tisgrabber_capture.ps1"
DEFAULT_SIGNAL_MIN_MEAN = 40.0
LEGACY_LASX_ROOT = Path("/mnt/c/Users/Public/MicroscopeAutomationTransfer/installers/LAS X")
LEGACY_TIS_ROOT = Path(
    "/mnt/c/Users/Public/MicroscopeAutomationTransfer/code/HorizontalPython/tisgrabber/samples"
)
DEFAULT_LIVE_STREAM_DIR = Path("/tmp/sdl_microscope_stream")
DEFAULT_LIVE_PID_FILE = Path.home() / ".sdl_lab" / "microscope_monitor" / "live_session.pid"
DEFAULT_LIVE_LOG_FILE = Path.home() / ".sdl_lab" / "microscope_monitor" / "live_session.log"


@dataclasses.dataclass
class CaptureResult:
    camera: str
    ok: bool
    output: str | None
    batch_id: str
    started_at: str
    ended_at: str
    elapsed_s: float
    message: str


def run_command(
    command: list[str],
    *,
    timeout_s: float | None = None,
    check: bool = False,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=timeout_s,
        check=check,
    )


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


#: The YAML subset parser lives in :mod:`tools.config` so every device reads
#: ``config.yaml`` through one implementation. These names are kept because the
#: camera CLI and its tests have always used them.
_strip_yaml_comment = _config._strip_comment
_parse_yaml_scalar = _config._parse_scalar
load_simple_yaml = _config.load_yaml


def load_root_config(path: Path) -> dict[str, Any]:
    """Parse the repo-root runtime config. Missing file -> empty mapping."""
    path = path.expanduser()
    if not path.exists():
        return {}
    config = load_simple_yaml(path)
    if not isinstance(config, dict):
        raise ValueError(f"runtime config must be a mapping: {path}")
    return config


def _config_section(runtime_config: dict[str, Any], name: str) -> dict[str, Any]:
    value = runtime_config.get(name, {})
    return value if isinstance(value, dict) else {}


def _config_default(
    runtime_config: dict[str, Any],
    section: str,
    key: str,
    default: Any,
) -> Any:
    section_config = _config_section(runtime_config, section)
    if key in section_config:
        return section_config[key]
    return default


def _path_from_config(value: Any, *, base_dir: Path) -> Path:
    path = value if isinstance(value, Path) else Path(str(value))
    path = path.expanduser()
    if not path.is_absolute():
        path = (base_dir / path).resolve()
    return path


def _config_path_default(
    runtime_config: dict[str, Any],
    runtime_config_path: Path,
    section: str,
    key: str,
    default: Path | None,
) -> Path | None:
    section_config = _config_section(runtime_config, section)
    if key not in section_config:
        return default
    return _path_from_config(section_config[key], base_dir=runtime_config_path.parent)


def _root_path_default(
    runtime_config: dict[str, Any],
    runtime_config_path: Path,
    key: str,
    default: Path,
) -> Path:
    if key not in runtime_config:
        return default
    return _path_from_config(runtime_config[key], base_dir=runtime_config_path.parent)


def _user_saved_dir_default(
    runtime_config: dict[str, Any],
    runtime_config_path: Path,
    section: str,
) -> Path:
    section_default = _config_path_default(
        runtime_config, runtime_config_path, section, "user_saved_dir", None
    )
    if section_default is not None:
        return section_default
    if section != "monitor":
        monitor_default = _config_path_default(
            runtime_config, runtime_config_path, "monitor", "user_saved_dir", None
        )
        if monitor_default is not None:
            return monitor_default
    output_root = _root_path_default(
        runtime_config, runtime_config_path, "output_dir", DEFAULT_OUTPUT_DIR
    )
    return output_root / "user-saved"


def load_runtime_config(args: argparse.Namespace) -> dict[str, Any]:
    config = load_config(args.config)
    output_dir = getattr(args, "output_dir", None)
    if output_dir is not None:
        config = dict(config)
        config["output_dir"] = str(output_dir.expanduser().resolve())
    return config


def wsl_to_windows_path(path: Path) -> str:
    cp = run_command(["wslpath", "-w", str(path)], timeout_s=5)
    if cp.returncode != 0:
        raise RuntimeError(cp.stderr.strip() or cp.stdout.strip() or "wslpath failed")
    return cp.stdout.strip()


def timestamp() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def read_text_if_exists(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def ensure_output_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def write_metadata(
    output: Path,
    result: CaptureResult,
    extra: dict[str, Any] | None = None,
) -> None:
    meta = dataclasses.asdict(result)
    if extra:
        meta.update(extra)
    meta_path = output.with_suffix(output.suffix + ".json")
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def image_signal_threshold(cfg: dict[str, Any]) -> float:
    return float(cfg.get("signal_min_mean", DEFAULT_SIGNAL_MIN_MEAN))


def should_check_signal(cfg: dict[str, Any]) -> bool:
    return bool(cfg.get("signal_check", True))


def probe_image_signal(path: Path, *, step: int = 64) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {"ok": False, "error": f"image file missing or empty: {path}"}
    if not IMAGE_SIGNAL.exists():
        return {"ok": False, "error": f"missing image signal probe: {IMAGE_SIGNAL}"}

    cp = run_command(
        [
            "powershell.exe",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            wsl_to_windows_path(IMAGE_SIGNAL),
            "-Path",
            wsl_to_windows_path(path),
            "-Step",
            str(step),
        ],
        timeout_s=30,
    )
    payload = (cp.stdout or cp.stderr).strip()
    if cp.returncode != 0:
        return {
            "ok": False,
            "returncode": cp.returncode,
            "error": payload or "image signal probe failed",
        }
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return {"ok": False, "error": "image signal probe returned non-JSON output", "raw": payload}
    data["ok"] = bool(data.get("ok", True))
    return data


def apply_signal_check(
    *,
    result: CaptureResult,
    cfg: dict[str, Any],
    output: Path,
) -> dict[str, Any] | None:
    if not result.ok or not should_check_signal(cfg):
        return None

    signal = probe_image_signal(output)
    if not signal.get("ok"):
        result.ok = False
        result.message = append_message(
            result.message,
            f"image signal check failed: {signal.get('error')}",
        )
        return signal

    mean = float(signal.get("mean", 0.0))
    threshold = image_signal_threshold(cfg)
    signal["min_required_mean"] = threshold
    if mean < threshold:
        result.ok = False
        result.message = append_message(
            result.message,
            f"low image signal: mean brightness {mean:.2f} < {threshold:.2f}",
        )
    return signal


def append_message(message: str, extra: str) -> str:
    if not message:
        return extra
    return f"{message}\n{extra}"


def wsl_usb_devices() -> list[dict[str, str]]:
    devices: list[dict[str, str]] = []
    for dev in sorted(Path("/sys/bus/usb/devices").glob("*")):
        vendor = read_text_if_exists(dev / "idVendor")
        product = read_text_if_exists(dev / "idProduct")
        if not vendor or not product:
            continue
        devices.append(
            {
                "sysfs": str(dev),
                "vid_pid": f"{vendor}:{product}".lower(),
                "manufacturer": read_text_if_exists(dev / "manufacturer") or "",
                "product": read_text_if_exists(dev / "product") or "",
                "serial": read_text_if_exists(dev / "serial") or "",
            }
        )
    return devices


def parse_usbipd_devices(stdout: str) -> list[dict[str, str]]:
    devices: list[dict[str, str]] = []
    in_connected = False
    for raw_line in stdout.splitlines():
        line = raw_line.strip()
        if line == "Connected:":
            in_connected = True
            continue
        if line == "Persisted:":
            break
        if not in_connected or not line or line.startswith("BUSID"):
            continue

        parts = line.split()
        if len(parts) < 4:
            continue
        busid = parts[0]
        vid_pid = parts[1].lower()
        if len(parts) >= 5 and parts[-2:] == ["Not", "shared"]:
            state = "Not shared"
            device = " ".join(parts[2:-2])
        else:
            state = parts[-1]
            device = " ".join(parts[2:-1])
        devices.append(
            {
                "busid": busid,
                "vid_pid": vid_pid,
                "device": device,
                "state": state,
            }
        )
    return devices


def usbipd_status() -> dict[str, Any]:
    usbipd = Path("/mnt/c/Program Files/usbipd-win/usbipd.exe")
    if not usbipd.exists():
        return {"available": False, "path": str(usbipd), "message": "usbipd.exe not found"}

    cp = run_command([str(usbipd), "list"], timeout_s=15)
    return {
        "available": cp.returncode == 0,
        "path": str(usbipd),
        "returncode": cp.returncode,
        "stdout": cp.stdout,
        "stderr": cp.stderr,
        "devices": parse_usbipd_devices(cp.stdout),
    }


class Camera:
    def __init__(self, name: str, cfg: dict[str, Any], output_dir: Path) -> None:
        self.name = name
        self.cfg = cfg
        self.output_dir = output_dir

    @property
    def label(self) -> str:
        return str(self.cfg.get("label", self.name))

    @property
    def enabled(self) -> bool:
        return bool(self.cfg.get("enabled", True))

    def status(self) -> dict[str, Any]:
        vid_pid = str(self.cfg.get("vid_pid", "")).lower()
        wsl_matches = [dev for dev in wsl_usb_devices() if vid_pid and dev["vid_pid"] == vid_pid]
        return {
            "name": self.name,
            "label": self.label,
            "type": self.cfg.get("type"),
            "enabled": self.enabled,
            "vid_pid": self.cfg.get("vid_pid"),
            "usbipd_busid": self.cfg.get("usbipd_busid"),
            "wsl_usb_matches": wsl_matches,
            "notes": self.cfg.get("notes"),
        }

    def capture(
        self,
        *,
        image_format: str | None = None,
        batch_id: str | None = None,
    ) -> CaptureResult:
        raise NotImplementedError

    def props(self, names: list[str]) -> int:
        raise NotImplementedError(f"{self.name} does not implement property access")

    def set_props(self, assignments: list[str]) -> int:
        raise NotImplementedError(f"{self.name} does not implement property access")

    def stream_frame(self, *, image_format: str, timeout_ms: int) -> bytes:
        raise NotImplementedError(f"{self.name} does not implement live stream frames")


class TisIc4CliCamera(Camera):
    def _ic4_ctrl(self) -> str:
        return str(self.cfg["ic4_ctrl"])

    def _device_id(self) -> str:
        return str(self.cfg["device_id"])

    def status(self) -> dict[str, Any]:
        data = super().status()
        cp = run_command([self._ic4_ctrl(), "device", self._device_id()], timeout_s=10)
        data.update(
            {
                "device_id": self._device_id(),
                "available": cp.returncode == 0 and "ModelName:" in cp.stdout,
                "detail": (cp.stdout or cp.stderr).strip(),
            }
        )
        return data

    def capture(
        self,
        *,
        image_format: str | None = None,
        batch_id: str | None = None,
    ) -> CaptureResult:
        fmt = image_format or str(self.cfg.get("format", "png"))
        timeout_ms = int(self.cfg.get("timeout_ms", 3000))
        ts = batch_id or timestamp()
        ensure_output_dir(self.output_dir)
        output = self.output_dir / f"{ts}_{self.name}.{fmt}"
        output_win = wsl_to_windows_path(output)

        started_perf = time.perf_counter()
        started = dt.datetime.now().isoformat(timespec="microseconds")
        cp = run_command(
            [
                self._ic4_ctrl(),
                "image",
                "-f",
                output_win,
                "--count",
                "1",
                "--timeout",
                str(timeout_ms),
                "--type",
                fmt,
                self._device_id(),
            ],
            timeout_s=max(5, timeout_ms / 1000 + 10),
        )
        ended = dt.datetime.now().isoformat(timespec="microseconds")
        elapsed = time.perf_counter() - started_perf
        ok = cp.returncode == 0 and output.exists() and output.stat().st_size > 0
        message = (cp.stdout or cp.stderr).strip()
        if not ok and not message:
            message = f"capture failed; expected output was {output}"

        result = CaptureResult(
            camera=self.name,
            ok=ok,
            output=str(output) if output.exists() else str(output),
            batch_id=ts,
            started_at=started,
            ended_at=ended,
            elapsed_s=elapsed,
            message=message,
        )
        signal = apply_signal_check(result=result, cfg=self.cfg, output=output)
        self._write_metadata(output, result, cp, signal)
        return result

    def stream_frame(self, *, image_format: str, timeout_ms: int) -> bytes:
        suffix = f".{image_format}"
        fd, tmp_name = tempfile.mkstemp(prefix=f"{self.name}_stream_", suffix=suffix)
        tmp_path = Path(tmp_name)
        try:
            os.close(fd)
            tmp_path.unlink(missing_ok=True)
            output_win = wsl_to_windows_path(tmp_path)
            cp = run_command(
                [
                    self._ic4_ctrl(),
                    "image",
                    "-f",
                    output_win,
                    "--count",
                    "1",
                    "--timeout",
                    str(timeout_ms),
                    "--type",
                    image_format,
                    self._device_id(),
                ],
                timeout_s=max(5, timeout_ms / 1000 + 10),
            )
            if cp.returncode != 0 or not tmp_path.exists() or tmp_path.stat().st_size == 0:
                message = (cp.stdout or cp.stderr).strip()
                raise RuntimeError(message or f"stream frame capture failed: {tmp_path}")
            return tmp_path.read_bytes()
        finally:
            try:
                tmp_path.unlink(missing_ok=True)
            except OSError:
                pass

    def props(self, names: list[str]) -> int:
        command = [self._ic4_ctrl(), "prop", self._device_id(), *names]
        cp = run_command(command, timeout_s=15)
        print((cp.stdout or cp.stderr).rstrip())
        return cp.returncode

    def set_props(self, assignments: list[str]) -> int:
        command = [self._ic4_ctrl(), "prop", self._device_id(), *assignments]
        cp = run_command(command, timeout_s=15)
        print((cp.stdout or cp.stderr).rstrip())
        return cp.returncode

    def _write_metadata(
        self,
        output: Path,
        result: CaptureResult,
        cp: subprocess.CompletedProcess[str],
        signal: dict[str, Any] | None,
    ) -> None:
        extra: dict[str, Any] = {
            "command_stdout": cp.stdout,
            "command_stderr": cp.stderr,
            "returncode": cp.returncode,
        }
        if signal is not None:
            extra["image_signal"] = signal
        write_metadata(output, result, extra)


class TisGrabberLegacyCamera(Camera):
    def _ic4_ctrl(self) -> str | None:
        value = self.cfg.get("ic4_ctrl")
        return str(value) if value else None

    def _device_id(self) -> str | None:
        value = self.cfg.get("device_id")
        return str(value) if value else None

    def _sample_dir(self) -> Path:
        return Path(str(self.cfg["sample_dir"]))

    def status(self) -> dict[str, Any]:
        data = super().status()
        sample_dir = self._sample_dir()
        data.update(
            {
                "legacy_stack": "tisgrabber_x64.dll",
                "sample_dir": str(sample_dir),
                "sample_dir_exists": sample_dir.exists(),
                "device_xml": str(sample_dir / "device.xml"),
                "device_xml_exists": (sample_dir / "device.xml").exists(),
                "dll": str(sample_dir / "tisgrabber_x64.dll"),
                "dll_exists": (sample_dir / "tisgrabber_x64.dll").exists(),
            }
        )
        ic4_ctrl = self._ic4_ctrl()
        device_id = self._device_id()
        if ic4_ctrl and device_id:
            cp = run_command([ic4_ctrl, "device", device_id], timeout_s=10)
            data.update(
                {
                    "device_id": device_id,
                    "available": cp.returncode == 0 and "ModelName:" in cp.stdout,
                    "detail": (cp.stdout or cp.stderr).strip(),
                }
            )
        return data

    def capture(
        self,
        *,
        image_format: str | None = None,
        batch_id: str | None = None,
    ) -> CaptureResult:
        if not TISGRABBER.exists():
            raise RuntimeError(f"missing legacy TIS bridge: {TISGRABBER}")

        started_perf = time.perf_counter()
        started = dt.datetime.now().isoformat(timespec="microseconds")
        ts = batch_id or timestamp()
        fmt = (image_format or str(self.cfg.get("format", "jpg"))).lower()
        if fmt == "jpeg":
            fmt = "jpg"
        if fmt not in {"jpg", "bmp"}:
            raise RuntimeError("legacy TIS capture supports jpg/jpeg or bmp output")

        ensure_output_dir(self.output_dir)
        output = self.output_dir / f"{ts}_{self.name}.{fmt}"
        pre_props = self._configured_prop_assignments("pre_capture_props")
        post_props = self._configured_prop_assignments("post_capture_props")
        pre_cp = self._apply_props(pre_props) if pre_props else None
        post_cp: subprocess.CompletedProcess[str] | None = None

        if pre_cp is not None and pre_cp.returncode != 0:
            command: list[str] = []
            cp = pre_cp
        else:
            command = self._capture_command(output=output, fmt=fmt)
            try:
                cp = run_command(command, timeout_s=float(self.cfg.get("timeout_s", 30)))
            finally:
                if post_props:
                    post_cp = self._apply_props(post_props)
        ended = dt.datetime.now().isoformat(timespec="microseconds")
        ok = cp.returncode == 0 and output.exists() and output.stat().st_size > 0
        result = CaptureResult(
            camera=self.name,
            ok=ok,
            output=str(output),
            batch_id=ts,
            started_at=started,
            ended_at=ended,
            elapsed_s=time.perf_counter() - started_perf,
            message=(cp.stdout or cp.stderr).strip(),
        )
        signal = apply_signal_check(result=result, cfg=self.cfg, output=output)
        extra: dict[str, Any] = {
            "command": command,
            "command_stdout": cp.stdout,
            "command_stderr": cp.stderr,
            "returncode": cp.returncode,
            "legacy_stack": "tisgrabber_x64.dll",
            "sample_dir": str(self._sample_dir()),
        }
        if pre_cp is not None:
            extra["pre_capture_props"] = pre_props
            extra["pre_capture_props_stdout"] = pre_cp.stdout
            extra["pre_capture_props_stderr"] = pre_cp.stderr
            extra["pre_capture_props_returncode"] = pre_cp.returncode
        if post_cp is not None:
            extra["post_capture_props"] = post_props
            extra["post_capture_props_stdout"] = post_cp.stdout
            extra["post_capture_props_stderr"] = post_cp.stderr
            extra["post_capture_props_returncode"] = post_cp.returncode
        if signal is not None:
            extra["image_signal"] = signal
        write_metadata(output, result, extra)
        return result

    def stream_frame(self, *, image_format: str, timeout_ms: int) -> bytes:
        fmt = "jpg" if image_format in {"jpeg", "jpg"} else "bmp"
        fd, tmp_name = tempfile.mkstemp(prefix=f"{self.name}_stream_", suffix=f".{fmt}")
        tmp_path = Path(tmp_name)
        try:
            os.close(fd)
            tmp_path.unlink(missing_ok=True)
            command = self._capture_command(output=tmp_path, fmt=fmt, snap_timeout_ms=timeout_ms)
            cp = run_command(command, timeout_s=max(10, timeout_ms / 1000 + 10))
            if cp.returncode != 0 or not tmp_path.exists() or tmp_path.stat().st_size == 0:
                message = (cp.stdout or cp.stderr).strip()
                raise RuntimeError(message or f"legacy TIS stream frame failed: {tmp_path}")
            return tmp_path.read_bytes()
        finally:
            try:
                tmp_path.unlink(missing_ok=True)
            except OSError:
                pass

    def props(self, names: list[str]) -> int:
        ic4_ctrl = self._ic4_ctrl()
        device_id = self._device_id()
        if not ic4_ctrl or not device_id:
            raise RuntimeError(f"{self.name} has no ic4_ctrl/device_id configured")
        cp = run_command([ic4_ctrl, "prop", device_id, *names], timeout_s=15)
        print((cp.stdout or cp.stderr).rstrip())
        return cp.returncode

    def set_props(self, assignments: list[str]) -> int:
        ic4_ctrl = self._ic4_ctrl()
        device_id = self._device_id()
        if not ic4_ctrl or not device_id:
            raise RuntimeError(f"{self.name} has no ic4_ctrl/device_id configured")
        cp = run_command([ic4_ctrl, "prop", device_id, *assignments], timeout_s=15)
        print((cp.stdout or cp.stderr).rstrip())
        return cp.returncode

    def _configured_prop_assignments(self, key: str) -> list[str]:
        raw = self.cfg.get(key, [])
        if isinstance(raw, dict):
            return [f"{name}={value}" for name, value in raw.items()]
        if isinstance(raw, list):
            return [str(item) for item in raw]
        if raw:
            raise RuntimeError(f"{self.name} config field {key!r} must be a list or object")
        return []

    def _apply_props(self, assignments: list[str]) -> subprocess.CompletedProcess[str]:
        ic4_ctrl = self._ic4_ctrl()
        device_id = self._device_id()
        if not ic4_ctrl or not device_id:
            raise RuntimeError(f"{self.name} has no ic4_ctrl/device_id configured")
        return run_command([ic4_ctrl, "prop", device_id, *assignments], timeout_s=15)

    def _capture_command(
        self,
        *,
        output: Path,
        fmt: str,
        snap_timeout_ms: int | None = None,
    ) -> list[str]:
        command = [
            "powershell.exe",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            wsl_to_windows_path(TISGRABBER),
            "-Output",
            wsl_to_windows_path(output),
            "-SampleDir",
            wsl_to_windows_path(self._sample_dir()),
            "-ImageType",
            "JPEG" if fmt == "jpg" else "BMP",
            "-JpegQuality",
            str(int(self.cfg.get("jpeg_quality", 90))),
            "-SnapTimeoutMs",
            str(snap_timeout_ms or int(self.cfg.get("snap_timeout_ms", 2000))),
            "-SettleSeconds",
            str(float(self.cfg.get("settle_s", 0))),
        ]
        if bool(self.cfg.get("show_live", True)):
            command.append("-ShowLive")
        if self.cfg.get("legacy_exposure_s") is not None:
            command.extend(["-ExposureSeconds", str(float(self.cfg["legacy_exposure_s"]))])
        if self.cfg.get("legacy_gain") is not None:
            command.extend(["-Gain", str(float(self.cfg["legacy_gain"]))])
        if self.cfg.get("legacy_brightness") is not None:
            command.extend(["-Brightness", str(int(self.cfg["legacy_brightness"]))])
        legacy_unique_name = self.cfg.get("legacy_unique_name", "")
        if legacy_unique_name:
            command.extend(["-DeviceUniqueName", str(legacy_unique_name)])
        return command


class ExternalCommandCamera(Camera):
    def capture(
        self,
        *,
        image_format: str | None = None,
        batch_id: str | None = None,
    ) -> CaptureResult:
        started_perf = time.perf_counter()
        started = dt.datetime.now().isoformat(timespec="microseconds")
        ts = batch_id or timestamp()
        fmt = image_format or str(self.cfg.get("format", "png"))
        ensure_output_dir(self.output_dir)
        output = self.output_dir / f"{ts}_{self.name}.{fmt}"
        output_win = wsl_to_windows_path(output)
        output_dir_win = wsl_to_windows_path(self.output_dir)
        tool_dir_win = wsl_to_windows_path(BACKENDS)

        if not self.enabled:
            ended = dt.datetime.now().isoformat(timespec="microseconds")
            return CaptureResult(
                camera=self.name,
                ok=False,
                output=str(output),
                batch_id=ts,
                started_at=started,
                ended_at=ended,
                elapsed_s=time.perf_counter() - started_perf,
                message=str(self.cfg.get("notes", "camera is disabled in config")),
            )

        command_template = self.cfg.get("command", [])
        if not command_template:
            ended = dt.datetime.now().isoformat(timespec="microseconds")
            return CaptureResult(
                camera=self.name,
                ok=False,
                output=str(output),
                batch_id=ts,
                started_at=started,
                ended_at=ended,
                elapsed_s=time.perf_counter() - started_perf,
                message="external command camera has no command configured",
            )

        mapping = {
            "output": str(output),
            "output_win": output_win,
            "output_dir": str(self.output_dir),
            "output_dir_win": output_dir_win,
            "tool_dir": str(BACKENDS),
            "tool_dir_win": tool_dir_win,
            "timestamp": ts,
            "batch_id": ts,
            "camera": self.name,
        }
        command = [str(part).format(**mapping) for part in command_template]
        cp = run_command(command, timeout_s=float(self.cfg.get("timeout_s", 60)))
        ended = dt.datetime.now().isoformat(timespec="microseconds")
        ok = cp.returncode == 0 and output.exists() and output.stat().st_size > 0
        result = CaptureResult(
            camera=self.name,
            ok=ok,
            output=str(output),
            batch_id=ts,
            started_at=started,
            ended_at=ended,
            elapsed_s=time.perf_counter() - started_perf,
            message=(cp.stdout or cp.stderr).strip(),
        )
        signal = apply_signal_check(result=result, cfg=self.cfg, output=output)
        extra: dict[str, Any] = {
            "command": command,
            "command_stdout": cp.stdout,
            "command_stderr": cp.stderr,
            "returncode": cp.returncode,
        }
        if signal is not None:
            extra["image_signal"] = signal
        write_metadata(output, result, extra)
        return result


def build_cameras(config: dict[str, Any]) -> dict[str, Camera]:
    output_dir = Path(config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures")).expanduser()
    cameras: dict[str, Camera] = {}
    for name, cfg in config.get("cameras", {}).items():
        cam_type = cfg.get("type")
        if cam_type == "ic4_cli":
            cameras[name] = TisIc4CliCamera(name, cfg, output_dir)
        elif cam_type == "tisgrabber_legacy":
            cameras[name] = TisGrabberLegacyCamera(name, cfg, output_dir)
        elif cam_type == "external_command":
            cameras[name] = ExternalCommandCamera(name, cfg, output_dir)
        else:
            raise ValueError(f"Unsupported camera type for {name}: {cam_type}")
    return cameras


def selected_cameras(cameras: dict[str, Camera], name: str) -> list[Camera]:
    if name in {"all", "both"}:
        return [cam for cam in cameras.values() if cam.enabled]
    if name not in cameras:
        raise KeyError(f"Unknown camera {name!r}. Known cameras: {', '.join(cameras)}")
    return [cameras[name]]


def capture_batch(
    *,
    config: dict[str, Any],
    camera_name: str,
    image_format: str | None,
    batch_id: str,
    start_timeout_s: float,
) -> list[CaptureResult]:
    cameras = build_cameras(config)
    cams = selected_cameras(cameras, camera_name)
    if not cams:
        raise RuntimeError("No enabled cameras selected")

    barrier: threading.Barrier | None = None
    if len(cams) > 1:
        barrier = threading.Barrier(len(cams) + 1)

    def capture_one(cam: Camera) -> CaptureResult:
        if barrier is not None:
            barrier.wait(timeout=start_timeout_s)
        return cam.capture(image_format=image_format, batch_id=batch_id)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(cams)) as pool:
        futures = [pool.submit(capture_one, cam) for cam in cams]
        if barrier is not None:
            barrier.wait(timeout=start_timeout_s)
        return [future.result() for future in futures]


def print_capture_results(results: list[CaptureResult]) -> bool:
    ok = True
    for result in results:
        ok = ok and result.ok
        print(json.dumps(dataclasses.asdict(result), indent=2))
    return ok


def parse_xml_attrs(path: Path, tag: str) -> dict[str, str] | None:
    if not path.exists():
        return None
    try:
        root = ET.fromstring(path.read_text(encoding="utf-8-sig", errors="replace"))
    except ET.ParseError:
        return None
    for element in root.iter(tag):
        return dict(element.attrib)
    return None


def lasx_user_dir(user: str) -> Path:
    return Path(f"/mnt/c/Users/{user}/AppData/Roaming/Leica Microsystems/LAS X")


def lasx_user_settings(user: str) -> dict[str, Any]:
    base = lasx_user_dir(user)
    userdata = base / "UserData.lcf"
    profile = base / "UserCameraProfiles/K3C.xml"
    loadsave = parse_xml_attrs(userdata, "LoadSave")
    basic = parse_xml_attrs(profile, "Basic")
    fmt = parse_xml_attrs(profile, "Format")
    return {
        "user": user,
        "userdata": str(userdata),
        "userdata_exists": userdata.exists(),
        "k3c_profile": str(profile),
        "k3c_profile_exists": profile.exists(),
        "loadsave": loadsave,
        "k3c_basic": basic,
        "k3c_format": fmt,
        "has_tif_autosave": bool(
            loadsave
            and any(
                "Type: TIF" in str(value)
                for key, value in loadsave.items()
                if "XLEFDataTypeConfigurations" in key
            )
        ),
        "has_lof_autosave": bool(
            loadsave
            and any(
                "Type: LOF" in str(value)
                for key, value in loadsave.items()
                if "XLEFDataTypeConfigurations" in key
            )
        ),
    }


def windows_lasx_processes() -> Any:
    cp = run_command(
        [
            "powershell.exe",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            (
                "Get-Process LMSApplication,CAMServer -ErrorAction SilentlyContinue | "
                "Select-Object ProcessName,Id,SessionId,Responding,"
                "MainWindowHandle,MainWindowTitle | "
                "ConvertTo-Json -Depth 3"
            ),
        ],
        timeout_s=10,
    )
    payload = cp.stdout.strip()
    if cp.returncode != 0 or not payload:
        return {"ok": False, "returncode": cp.returncode, "stderr": cp.stderr.strip()}
    try:
        return {"ok": True, "processes": json.loads(payload)}
    except json.JSONDecodeError:
        return {"ok": False, "raw": payload}


def tis_legacy_settings() -> dict[str, Any]:
    device_xml = LEGACY_TIS_ROOT / "device.xml"
    ic4_json = Path("/mnt/c/Users/KWON MIN KYUNG/AppData/Roaming/ic4-demoapp/device.json")
    ic_capture = Path("/mnt/c/Users/KWON MIN KYUNG/AppData/Roaming/IC Capture 2.5/default.iccf")
    props: dict[str, Any] | None = None
    if ic4_json.exists():
        try:
            data = json.loads(ic4_json.read_text(encoding="utf-8"))
            raw_props = data.get("properties", {})
            keys = (
                "PixelFormat",
                "Width",
                "Height",
                "AcquisitionFrameRate",
                "Brightness",
                "BlackLevel",
                "ExposureAuto",
                "ExposureAutoReference",
                "GainAuto",
                "GainAutoUpperLimit",
                "BalanceWhiteAuto",
                "Gamma",
            )
            props = {key: raw_props[key] for key in keys if key in raw_props}
        except (OSError, json.JSONDecodeError):
            props = None
    return {
        "legacy_sample_dir": str(LEGACY_TIS_ROOT),
        "device_xml": str(device_xml),
        "device_xml_exists": device_xml.exists(),
        "ic_capture_default": str(ic_capture),
        "ic_capture_default_exists": ic_capture.exists(),
        "ic4_demo_device_json": str(ic4_json),
        "ic4_demo_device_json_exists": ic4_json.exists(),
        "ic4_demo_properties": props,
    }


def newest_matching_file(root: Path, suffixes: tuple[str, ...]) -> str | None:
    if not root.exists():
        return None
    newest: Path | None = None
    newest_mtime = -1.0
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in suffixes:
            continue
        try:
            mtime = path.stat().st_mtime
        except OSError:
            continue
        if mtime > newest_mtime:
            newest = path
            newest_mtime = mtime
    return str(newest) if newest else None


def cmd_diagnose(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    output_dir = Path(config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures")).expanduser()
    diagnosis = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "summary": (
            "Legacy Leica publication data used LAS X XLEF export with TIF+LOF metadata. "
            "Legacy TIS horizontal data used tisgrabber_x64.dll with device.xml. "
            "A capture file is treated as failed when the image signal check reports "
            "low mean brightness."
        ),
        "output_dir": str(output_dir),
        "signal_min_mean": DEFAULT_SIGNAL_MIN_MEAN,
        "windows_lasx_processes": windows_lasx_processes(),
        "lasx_users": {
            "lamp": lasx_user_settings("lamp"),
            "KWON MIN KYUNG": lasx_user_settings("KWON MIN KYUNG"),
        },
        "legacy_leica_expected": {
            "camera": "K3C-700011170655",
            "successful_export_stack": "LAS X XLEF export",
            "expected_data_types": ["TIF", "LOF"],
            "example_tif": newest_matching_file(LEGACY_LASX_ROOT, (".tif", ".tiff")),
        },
        "legacy_tis_expected": tis_legacy_settings(),
    }
    print(json.dumps(diagnosis, indent=2))
    return 0


def cmd_probe_image(args: argparse.Namespace) -> int:
    signal = probe_image_signal(args.image, step=args.step)
    threshold = float(args.min_mean)
    signal["min_required_mean"] = threshold
    signal["passes_signal_check"] = (
        bool(signal.get("ok")) and float(signal.get("mean", 0.0)) >= threshold
    )
    print(json.dumps(signal, indent=2))
    return 0 if signal["passes_signal_check"] else 1


def cmd_list(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    cameras = build_cameras(config)
    for cam in cameras.values():
        print(json.dumps(cam.status(), indent=2))
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    cameras = build_cameras(config)
    usbipd = usbipd_status()
    camera_statuses = [cam.status() for cam in cameras.values()]
    usbipd_devices = usbipd.get("devices", [])
    for camera_status in camera_statuses:
        vid_pid = str(camera_status.get("vid_pid", "")).lower()
        busid = camera_status.get("usbipd_busid")
        camera_status["usbipd_matches"] = [
            dev
            for dev in usbipd_devices
            if (vid_pid and dev.get("vid_pid") == vid_pid) or (busid and dev.get("busid") == busid)
        ]
    status = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "config": str(args.config),
        "output_dir": str(Path(config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures"))),
        "cameras": camera_statuses,
        "usbipd": usbipd,
        "wsl_usb_devices": wsl_usb_devices(),
    }
    print(json.dumps(status, indent=2))
    return 0


def cmd_capture(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    batch_id = args.batch_id or timestamp()
    results = capture_batch(
        config=config,
        camera_name=args.camera,
        image_format=args.format,
        batch_id=batch_id,
        start_timeout_s=float(args.start_timeout_s),
    )
    return 0 if print_capture_results(results) else 1


def cmd_schedule(args: argparse.Namespace) -> int:
    if args.count < 1:
        print("error: --count must be at least 1", file=sys.stderr)
        return 2
    if args.interval_s < 0:
        print("error: --interval-s must be non-negative", file=sys.stderr)
        return 2

    config = load_runtime_config(args)
    output_dir = Path(config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures")).expanduser()
    ensure_output_dir(output_dir)
    schedule_id = args.schedule_id or f"schedule_{timestamp()}"
    manifest: dict[str, Any] = {
        "schedule_id": schedule_id,
        "camera": args.camera,
        "count": args.count,
        "interval_s": args.interval_s,
        "started_at": dt.datetime.now().isoformat(timespec="microseconds"),
        "output_dir": str(output_dir),
        "batches": [],
    }

    all_ok = True
    for index in range(args.count):
        batch_started = time.perf_counter()
        batch_id = f"{schedule_id}_{index + 1:04d}"
        results = capture_batch(
            config=config,
            camera_name=args.camera,
            image_format=args.format,
            batch_id=batch_id,
            start_timeout_s=float(args.start_timeout_s),
        )
        batch_ok = all(result.ok for result in results)
        all_ok = all_ok and batch_ok
        batch_record = {
            "index": index + 1,
            "batch_id": batch_id,
            "ok": batch_ok,
            "results": [dataclasses.asdict(result) for result in results],
        }
        manifest["batches"].append(batch_record)
        print(json.dumps(batch_record, indent=2))

        if index + 1 < args.count:
            elapsed = time.perf_counter() - batch_started
            time.sleep(max(0.0, args.interval_s - elapsed))

    manifest["ended_at"] = dt.datetime.now().isoformat(timespec="microseconds")
    manifest["ok"] = all_ok
    manifest_path = output_dir / f"{schedule_id}_manifest.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    print(json.dumps({"manifest": str(manifest_path), "ok": all_ok}, indent=2))
    return 0 if all_ok else 1


def cmd_autoshoot(args: argparse.Namespace) -> int:
    if args.interval_s <= 0:
        print("error: --interval-s must be positive", file=sys.stderr)
        return 2
    if args.max_batches is not None and args.max_batches < 1:
        print("error: --max-batches must be at least 1", file=sys.stderr)
        return 2

    config = load_runtime_config(args)
    prefix = args.batch_prefix or "autoshoot"
    index = 0
    while args.max_batches is None or index < args.max_batches:
        index += 1
        batch_started = time.perf_counter()
        batch_id = f"{prefix}_{timestamp()}"
        try:
            results = capture_batch(
                config=config,
                camera_name=args.camera,
                image_format=args.format,
                batch_id=batch_id,
                start_timeout_s=float(args.start_timeout_s),
            )
            batch_ok = all(result.ok for result in results)
            record = {
                "batch_id": batch_id,
                "index": index,
                "ok": batch_ok,
                "interval_s": args.interval_s,
                "elapsed_s": time.perf_counter() - batch_started,
                "results": [dataclasses.asdict(result) for result in results],
            }
            print(json.dumps(record, indent=2), flush=True)
        except KeyboardInterrupt:
            print(json.dumps({"stopped": True, "batches": index - 1}, indent=2), flush=True)
            return 130
        except Exception as exc:
            print(
                json.dumps(
                    {
                        "batch_id": batch_id,
                        "index": index,
                        "ok": False,
                        "error": str(exc),
                    },
                    indent=2,
                ),
                flush=True,
            )

        elapsed = time.perf_counter() - batch_started
        time.sleep(max(0.0, args.interval_s - elapsed))
    return 0


def _live_stage_dir(stream_dir: Path) -> Path:
    return stream_dir / ".staging"


def _atomic_copy(src: Path, dst: Path) -> None:
    ensure_output_dir(dst.parent)
    tmp = dst.with_name(f".{dst.name}.tmp")
    shutil.copy2(src, tmp)
    os.replace(tmp, dst)


def _clear_live_tmp_files(root: Path) -> None:
    if not root.exists():
        return
    patterns = (
        "current_*",
        "vertical_online_image.*",
        "*.json",
        ".*.tmp",
    )
    for pattern in patterns:
        for path in root.glob(pattern):
            if path.is_file():
                path.unlink(missing_ok=True)


def _live_capture_config(
    *,
    config: dict[str, Any],
    output_dir: Path,
    stream_settle_s: float,
    stream_snap_timeout_ms: int,
) -> dict[str, Any]:
    live_config = copy.deepcopy(config)
    live_config["output_dir"] = str(output_dir)
    for cam_cfg in live_config.get("cameras", {}).values():
        cam_cfg["signal_check"] = False
        if cam_cfg.get("type") == "tisgrabber_legacy":
            cam_cfg["settle_s"] = stream_settle_s
            cam_cfg["snap_timeout_ms"] = stream_snap_timeout_ms
    return live_config


def _publish_live_results(
    *,
    results: list[CaptureResult],
    stream_dir: Path,
    stage_dir: Path,
) -> list[dict[str, str]]:
    published: list[dict[str, str]] = []
    for result in results:
        if not result.output:
            continue
        src = Path(result.output)
        if not src.exists() or not src.is_file():
            continue
        dst = stream_dir / f"current_{result.camera}{src.suffix.lower()}"
        _atomic_copy(src, dst)

        src_meta = src.with_suffix(src.suffix + ".json")
        if src_meta.exists():
            _atomic_copy(src_meta, dst.with_suffix(dst.suffix + ".json"))

        published.append(
            {
                "camera": result.camera,
                "source": str(src),
                "published": str(dst),
            }
        )

    online = stage_dir / "vertical_online_image.jpg"
    if online.exists():
        _atomic_copy(online, stream_dir / online.name)
    return published


def _archive_live_results(
    *,
    archive_dir: Path,
    archive_batch_id: str,
    published: list[dict[str, str]],
    results: list[CaptureResult],
) -> list[dict[str, str]]:
    by_camera = {result.camera: result for result in results}
    archived: list[dict[str, str]] = []
    ensure_output_dir(archive_dir)
    for record in published:
        camera = record["camera"]
        current = Path(record["published"])
        if not current.exists():
            continue
        archive = archive_dir / f"{archive_batch_id}_{camera}{current.suffix.lower()}"
        _atomic_copy(current, archive)

        source_result = by_camera.get(camera)
        if source_result is not None:
            archive_result = dataclasses.replace(
                source_result,
                output=str(archive),
                batch_id=archive_batch_id,
            )
            write_metadata(
                archive,
                archive_result,
                {
                    "archive_kind": "live_session_copy",
                    "source_live_output": str(current),
                    "source_capture_output": source_result.output,
                },
            )
        archived.append(
            {
                "camera": camera,
                "output": str(archive),
                "source_live_output": str(current),
            }
        )
    return archived


def _run_live_capture_loop(
    *,
    args: argparse.Namespace,
    stop_event: threading.Event,
    recording_state: RecordingState,
) -> None:
    base_config = load_config(args.config)
    stream_dir = args.stream_dir.expanduser().resolve()
    stage_dir = _live_stage_dir(stream_dir)
    archive_dir = (
        args.archive_dir.expanduser().resolve()
        if args.archive_dir
        else Path(base_config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures"))
        .expanduser()
        .resolve()
    )
    ensure_output_dir(stream_dir)
    ensure_output_dir(stage_dir)
    if not args.no_archive:
        ensure_output_dir(archive_dir)
    _clear_live_tmp_files(stream_dir)
    _clear_live_tmp_files(stage_dir)

    live_config = _live_capture_config(
        config=base_config,
        output_dir=stage_dir,
        stream_settle_s=float(args.stream_settle_s),
        stream_snap_timeout_ms=int(args.stream_snap_timeout_ms),
    )
    next_save_at = time.monotonic() + float(args.save_interval_s)
    last_generation = recording_state.generation
    index = 0

    while not stop_event.is_set():
        index += 1
        batch_started = time.perf_counter()
        batch_id = "current"
        archived: list[dict[str, str]] = []
        try:
            results = capture_batch(
                config=live_config,
                camera_name=args.camera,
                image_format=args.format,
                batch_id=batch_id,
                start_timeout_s=float(args.start_timeout_s),
            )
            published = _publish_live_results(
                results=results,
                stream_dir=stream_dir,
                stage_dir=stage_dir,
            )
            _clear_live_tmp_files(stage_dir)

            now = time.monotonic()
            # Recording (auto-save) is toggled at runtime from the browser. The
            # live stream above always runs; only archiving is gated here.
            if recording_state.generation != last_generation:
                last_generation = recording_state.generation
                if recording_state.is_recording():
                    next_save_at = now  # just started recording: archive promptly
            recording = recording_state.is_recording() and not args.no_archive
            if recording and now >= next_save_at:
                archive_batch_id = f"{args.save_prefix}_{timestamp()}"
                archived = _archive_live_results(
                    archive_dir=archive_dir,
                    archive_batch_id=archive_batch_id,
                    published=published,
                    results=results,
                )
                recording_state.note_saved(archived)
                while next_save_at <= now:
                    next_save_at += float(args.save_interval_s)

            record = {
                "mode": "live-session",
                "index": index,
                "ok": all(result.ok for result in results),
                "stream_interval_s": args.stream_interval_s,
                "recording": recording_state.is_recording(),
                "save_interval_s": None if args.no_archive else args.save_interval_s,
                "elapsed_s": time.perf_counter() - batch_started,
                "stream_dir": str(stream_dir),
                "archive_dir": None if args.no_archive else str(archive_dir),
                "published": published,
                "archived": archived,
                "results": [dataclasses.asdict(result) for result in results],
            }
            print(json.dumps(record, indent=2), flush=True)
        except Exception as exc:
            print(
                json.dumps(
                    {
                        "mode": "live-session",
                        "index": index,
                        "ok": False,
                        "error": str(exc),
                    },
                    indent=2,
                ),
                flush=True,
            )

        elapsed = time.perf_counter() - batch_started
        stop_event.wait(max(0.0, float(args.stream_interval_s) - elapsed))


def _read_pid_file(path: Path) -> int | None:
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    try:
        return int(text)
    except ValueError:
        return None


def _pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _live_session_run_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "tools.microscope",
        "--config",
        str(args.config.expanduser().resolve()),
        "live-session",
        "run",
        "--camera",
        args.camera,
        "--format",
        args.format,
        "--stream-dir",
        str(args.stream_dir.expanduser().resolve()),
        "--user-saved-dir",
        str(args.user_saved_dir.expanduser().resolve()),
        "--stream-interval-s",
        str(args.stream_interval_s),
        "--save-interval-s",
        str(args.save_interval_s),
        "--save-prefix",
        args.save_prefix,
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--refresh-ms",
        str(args.refresh_ms),
        "--stream-settle-s",
        str(args.stream_settle_s),
        "--stream-snap-timeout-ms",
        str(args.stream_snap_timeout_ms),
        "--start-timeout-s",
        str(args.start_timeout_s),
    ]
    if args.archive_dir:
        command.extend(["--archive-dir", str(args.archive_dir.expanduser().resolve())])
    if args.lan:
        command.append("--lan")
    if args.https:
        command.append("--https")
    if args.cert_file:
        command.extend(["--cert-file", str(args.cert_file.expanduser().resolve())])
    if args.key_file:
        command.extend(["--key-file", str(args.key_file.expanduser().resolve())])
    if args.no_archive:
        command.append("--no-archive")
    if args.save_on_start:
        command.append("--save-on-start")
    if args.quiet:
        command.append("--quiet")
    return command


def cmd_live_session_run(args: argparse.Namespace) -> int:
    if args.stream_interval_s <= 0:
        print("error: --stream-interval-s must be positive", file=sys.stderr)
        return 2
    if not args.no_archive and args.save_interval_s <= 0:
        print(
            "error: --save-interval-s must be positive unless --no-archive is used",
            file=sys.stderr,
        )
        return 2
    if args.refresh_ms < 100:
        print("error: --refresh-ms must be at least 100", file=sys.stderr)
        return 2

    from .viewer import RecordingState, serve_monitor

    stream_dir = args.stream_dir.expanduser().resolve()
    ensure_output_dir(stream_dir)

    base_config = load_config(args.config)
    archive_dir = (
        args.archive_dir.expanduser().resolve()
        if args.archive_dir
        else Path(base_config.get("output_dir", "/home/lamp/SDLsetup/dataset/captures"))
        .expanduser()
        .resolve()
    )
    recording_state = RecordingState(
        save_interval_s=float(args.save_interval_s),
        save_prefix=args.save_prefix,
        archive_dir=None if args.no_archive else str(archive_dir),
        enabled=bool(args.save_on_start),
        can_record=not args.no_archive,
    )

    stop_event = threading.Event()
    worker = threading.Thread(
        target=_run_live_capture_loop,
        kwargs={
            "args": args,
            "stop_event": stop_event,
            "recording_state": recording_state,
        },
        daemon=True,
    )
    worker.start()
    host = "0.0.0.0" if args.lan else args.host
    try:
        serve_monitor(
            root=stream_dir,
            host=host,
            port=args.port,
            refresh_ms=args.refresh_ms,
            quiet=args.quiet,
            https=args.https,
            cert_file=args.cert_file,
            key_file=args.key_file,
            user_saved_dir=args.user_saved_dir,
            recording_state=recording_state,
        )
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        worker.join(timeout=2)
    return 0


def cmd_live_session_start(args: argparse.Namespace) -> int:
    pid_file = args.pid_file.expanduser()
    log_file = args.log_file.expanduser()
    existing_pid = _read_pid_file(pid_file)
    if existing_pid is not None and _pid_is_alive(existing_pid):
        print(
            json.dumps(
                {
                    "already_running": True,
                    "pid": existing_pid,
                    "pid_file": str(pid_file),
                    "log_file": str(log_file),
                },
                indent=2,
            )
        )
        return 0

    ensure_output_dir(pid_file.parent)
    ensure_output_dir(log_file.parent)
    command = _live_session_run_command(args)
    with log_file.open("ab") as log:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            cwd=str(Path.cwd()),
        )
    pid_file.write_text(f"{process.pid}\n", encoding="utf-8")
    time.sleep(1.0)
    if process.poll() is not None:
        tail = ""
        try:
            lines = log_file.read_text(encoding="utf-8", errors="replace").splitlines()
            tail = "\n".join(lines[-40:])
        except OSError:
            pass
        print(
            json.dumps(
                {
                    "started": False,
                    "returncode": process.returncode,
                    "pid": process.pid,
                    "pid_file": str(pid_file),
                    "log_file": str(log_file),
                    "log_tail": tail,
                },
                indent=2,
            ),
            file=sys.stderr,
        )
        return 1

    print(
        json.dumps(
            {
                "started": True,
                "pid": process.pid,
                "pid_file": str(pid_file),
                "log_file": str(log_file),
                "stream_dir": str(args.stream_dir.expanduser().resolve()),
                "monitor": f"http://127.0.0.1:{args.port}/",
                "command": command,
            },
            indent=2,
        )
    )
    return 0


def cmd_live_session_stop(args: argparse.Namespace) -> int:
    pid_file = args.pid_file.expanduser()
    pid = _read_pid_file(pid_file)
    if pid is None:
        print(json.dumps({"running": False, "message": "no live-session pid file"}, indent=2))
        return 0
    if not _pid_is_alive(pid):
        pid_file.unlink(missing_ok=True)
        print(json.dumps({"running": False, "stale_pid": pid, "pid_file_removed": True}, indent=2))
        return 0

    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    except PermissionError:
        os.kill(pid, signal.SIGTERM)

    deadline = time.monotonic() + float(args.timeout_s)
    while time.monotonic() < deadline and _pid_is_alive(pid):
        time.sleep(0.2)
    if _pid_is_alive(pid) and args.force:
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except PermissionError:
            os.kill(pid, signal.SIGKILL)
    stopped = not _pid_is_alive(pid)
    if stopped:
        pid_file.unlink(missing_ok=True)
    print(json.dumps({"stopped": stopped, "pid": pid, "pid_file": str(pid_file)}, indent=2))
    return 0 if stopped else 1


def cmd_live_session_status(args: argparse.Namespace) -> int:
    pid_file = args.pid_file.expanduser()
    pid = _read_pid_file(pid_file)
    running = bool(pid is not None and _pid_is_alive(pid))
    print(
        json.dumps(
            {
                "running": running,
                "pid": pid,
                "pid_file": str(pid_file),
                "log_file": str(args.log_file.expanduser()),
                "stream_dir": str(args.stream_dir.expanduser()),
            },
            indent=2,
        )
    )
    return 0


def cmd_stream(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    cameras = build_cameras(config)
    if args.camera not in cameras:
        print(
            f"Unknown camera {args.camera!r}. Known cameras: {', '.join(cameras)}",
            file=sys.stderr,
        )
        return 2

    cam = cameras[args.camera]
    if not cam.enabled:
        print(f"Camera {args.camera!r} is disabled in config", file=sys.stderr)
        return 2

    frame_format = "jpeg"
    content_type = "image/jpeg"

    class StreamHandler(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *values: object) -> None:
            if not args.quiet:
                super().log_message(fmt, *values)

        def do_GET(self) -> None:
            if self.path in {"/", "/index.html"}:
                self._write_index()
                return
            if self.path == "/snapshot.jpg":
                self._write_snapshot()
                return
            if self.path == "/stream.mjpg":
                self._write_stream()
                return
            self.send_error(404, "Not found")

        def _write_index(self) -> None:
            title = html.escape(f"{cam.label} live stream")
            body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; background: #111; color: #eee; }}
    header {{ padding: 12px 16px; background: #1f2933; }}
    main {{ padding: 16px; }}
    img {{ max-width: 100%; height: auto; background: #000; }}
    a {{ color: #8bd3ff; }}
  </style>
</head>
<body>
  <header>{title}</header>
  <main>
    <img src="/stream.mjpg" alt="{title}">
    <p><a href="/snapshot.jpg">Open one JPEG frame</a></p>
  </main>
</body>
</html>
"""
            body_bytes = body.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body_bytes)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body_bytes)

        def _capture_frame(self) -> bytes:
            return cam.stream_frame(image_format=frame_format, timeout_ms=args.timeout_ms)

        def _write_snapshot(self) -> None:
            try:
                frame = self._capture_frame()
            except Exception as exc:
                self.send_error(503, str(exc))
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(frame)

        def _write_stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            while True:
                started = time.perf_counter()
                try:
                    frame = self._capture_frame()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(f"Content-Type: {content_type}\r\n".encode("ascii"))
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                except Exception as exc:
                    try:
                        self.wfile.write(f"--frame\r\nX-Error: {exc}\r\n\r\n".encode())
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    time.sleep(max(0.25, args.interval_s))
                    continue

                elapsed = time.perf_counter() - started
                time.sleep(max(0.0, args.interval_s - elapsed))

    server = http.server.ThreadingHTTPServer((args.host, args.port), StreamHandler)
    url_host = "localhost" if args.host in {"0.0.0.0", "127.0.0.1"} else args.host
    print(f"Streaming {cam.label} at http://{url_host}:{args.port}/")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def cmd_props(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    cameras = build_cameras(config)
    return cameras[args.camera].props(args.names)


def cmd_set_props(args: argparse.Namespace) -> int:
    config = load_runtime_config(args)
    cameras = build_cameras(config)
    return cameras[args.camera].set_props(args.assignments)


def cmd_twain(args: argparse.Namespace) -> int:
    if not LEICA_TWAIN.exists():
        print(f"error: missing Leica TWAIN bridge: {LEICA_TWAIN}", file=sys.stderr)
        return 2
    script_win = wsl_to_windows_path(LEICA_TWAIN)
    command = [
        "powershell.exe",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        script_win,
        "-Action",
        args.action,
        "-Source",
        args.source,
    ]
    cp = run_command(command, timeout_s=30)
    if cp.stdout:
        print(cp.stdout.rstrip())
    if cp.stderr:
        print(cp.stderr.rstrip(), file=sys.stderr)
    return cp.returncode


def cmd_lasx(args: argparse.Namespace) -> int:
    if not LEICA_LASX_UI.exists():
        print(f"error: missing Leica LAS X bridge: {LEICA_LASX_UI}", file=sys.stderr)
        return 2
    script_win = wsl_to_windows_path(LEICA_LASX_UI)
    command = [
        "powershell.exe",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        script_win,
        args.action,
        "-WaitSeconds",
        str(args.wait_seconds),
    ]
    cp = run_command(command, timeout_s=max(15, args.wait_seconds + 20))
    if cp.stdout:
        print(cp.stdout.rstrip())
    if cp.stderr:
        print(cp.stderr.rstrip(), file=sys.stderr)
    return cp.returncode


def cmd_monitor(args: argparse.Namespace) -> int:
    from .viewer import serve_monitor

    host = "0.0.0.0" if args.lan else args.host
    serve_monitor(
        root=args.output_dir,
        host=host,
        port=args.port,
        refresh_ms=args.refresh_ms,
        quiet=args.quiet,
        https=args.https,
        cert_file=args.cert_file,
        key_file=args.key_file,
        user_saved_dir=args.user_saved_dir,
    )
    return 0


def cmd_lan_info(args: argparse.Namespace) -> int:
    from .viewer import monitor_urls

    host = "0.0.0.0" if args.lan else args.host
    scheme = "https" if args.https else "http"
    print(json.dumps(monitor_urls(host, args.port, scheme=scheme), indent=2))
    return 0


def add_live_session_run_args(
    parser: argparse.ArgumentParser,
    runtime_config: dict[str, Any] | None = None,
    runtime_config_path: Path = DEFAULT_RUNTIME_CONFIG,
) -> None:
    runtime_config = runtime_config or {}
    parser.add_argument(
        "--camera",
        default=_config_default(runtime_config, "live_session", "camera", "all"),
        help="camera name, all, or both",
    )
    parser.add_argument(
        "--format",
        default=_config_default(runtime_config, "live_session", "format", "jpg"),
        choices=["jpg", "jpeg"],
    )
    parser.add_argument(
        "--stream-dir",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "stream_dir",
            DEFAULT_LIVE_STREAM_DIR,
        ),
        help=f"temporary live image root, default: {DEFAULT_LIVE_STREAM_DIR}",
    )
    parser.add_argument(
        "--archive-dir",
        type=Path,
        help="slow-save archive root, default: config output_dir",
    )
    parser.add_argument(
        "--user-saved-dir",
        type=Path,
        default=_user_saved_dir_default(runtime_config, runtime_config_path, "live_session"),
        help="folder for monitor Take Photo saves; default: output_dir/user-saved",
    )
    parser.add_argument(
        "--stream-interval-s",
        type=float,
        default=_config_default(runtime_config, "live_session", "stream_interval_s", 0.5),
        help="minimum seconds between live capture starts; default comes from config.yaml or 0.5",
    )
    parser.add_argument(
        "--save-interval-s",
        type=float,
        default=_config_default(runtime_config, "live_session", "save_interval_s", 31.0),
        help="seconds between archived copies of live frames; default comes from config.yaml or 31",
    )
    parser.add_argument(
        "--save-prefix",
        default=_config_default(runtime_config, "live_session", "save_prefix", "live_save"),
    )
    parser.add_argument(
        "--no-archive",
        action="store_true",
        default=bool(_config_default(runtime_config, "live_session", "no_archive", False)),
        help="only update temporary live images; do not copy periodic archive files",
    )
    parser.add_argument(
        "--save-on-start",
        action="store_true",
        default=bool(_config_default(runtime_config, "live_session", "save_on_start", False)),
        help="archive the first successful live batch immediately",
    )
    parser.add_argument(
        "--stream-settle-s",
        type=float,
        default=_config_default(runtime_config, "live_session", "stream_settle_s", 0.0),
        help="temporary TIS settle seconds for live preview; default comes from config.yaml or 0",
    )
    parser.add_argument(
        "--stream-snap-timeout-ms",
        type=int,
        default=_config_default(runtime_config, "live_session", "stream_snap_timeout_ms", 3000),
        help="temporary TIS snap timeout for live preview; default comes from config.yaml or 3000",
    )
    parser.add_argument(
        "--start-timeout-s",
        type=float,
        default=_config_default(runtime_config, "live_session", "start_timeout_s", 5.0),
        help="worker start barrier timeout for multi-camera capture",
    )
    parser.add_argument(
        "--host",
        default=_config_default(runtime_config, "live_session", "host", "0.0.0.0"),
        help="bind address; default comes from config.yaml or 0.0.0.0",
    )
    parser.add_argument(
        "--lan",
        action="store_true",
        help="bind to all WSL interfaces; equivalent to --host 0.0.0.0",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=_config_default(runtime_config, "live_session", "port", 8766),
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=_config_default(runtime_config, "live_session", "refresh_ms", 500),
        help="browser monitor image refresh milliseconds; default comes from config.yaml or 500",
    )
    parser.add_argument("--https", action="store_true", help="serve HTTPS with a local cert")
    parser.add_argument("--cert-file", type=Path, help="HTTPS certificate path")
    parser.add_argument("--key-file", type=Path, help="HTTPS private key path")
    parser.add_argument(
        "--quiet",
        action="store_true",
        default=bool(_config_default(runtime_config, "live_session", "quiet", False)),
        help="suppress request logs",
    )


def build_parser(
    runtime_config: dict[str, Any] | None = None,
    runtime_config_path: Path = DEFAULT_RUNTIME_CONFIG,
) -> argparse.ArgumentParser:
    runtime_config = runtime_config or {}
    runtime_config_path = runtime_config_path.expanduser()
    parser = argparse.ArgumentParser(description="USB camera capture/control CLI")
    parser.add_argument(
        "--runtime-config",
        type=Path,
        default=runtime_config_path,
        help=f"root runtime YAML config, default: {DEFAULT_RUNTIME_CONFIG}",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_root_path_default(
            runtime_config, runtime_config_path, "camera_config", DEFAULT_CONFIG
        ),
        help=f"camera hardware config JSON, default: {DEFAULT_CONFIG}",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    list_p = sub.add_parser("list", help="list configured and detected cameras")
    list_p.set_defaults(func=cmd_list)

    status_p = sub.add_parser("status", help="show camera and USB diagnostics")
    status_p.set_defaults(func=cmd_status)

    diagnose_p = sub.add_parser(
        "diagnose",
        help="compare current camera/LAS X settings with legacy successful capture paths",
    )
    diagnose_p.add_argument("--output-dir", type=Path, help="override output directory")
    diagnose_p.set_defaults(func=cmd_diagnose)

    probe_p = sub.add_parser("probe-image", help="measure mean brightness of an image file")
    probe_p.add_argument("image", type=Path)
    probe_p.add_argument("--step", type=int, default=64, help="pixel sampling stride")
    probe_p.add_argument(
        "--min-mean",
        type=float,
        default=DEFAULT_SIGNAL_MIN_MEAN,
        help="minimum acceptable sampled mean brightness",
    )
    probe_p.set_defaults(func=cmd_probe_image)

    capture_p = sub.add_parser("capture", help="capture one image")
    capture_camera_default = _config_default(runtime_config, "capture", "camera", None)
    capture_p.add_argument(
        "--camera",
        required=capture_camera_default is None,
        default=capture_camera_default,
        help="camera name, all, or both",
    )
    capture_p.add_argument(
        "--format",
        default=_config_default(runtime_config, "capture", "format", None),
        choices=["bmp", "png", "jpg", "jpeg", "tiff"],
    )
    capture_p.add_argument(
        "--output-dir",
        type=Path,
        default=_root_path_default(runtime_config, runtime_config_path, "output_dir", None)
        if "output_dir" in runtime_config
        else _config_path_default(
            runtime_config, runtime_config_path, "capture", "output_dir", None
        ),
        help="override output directory",
    )
    capture_p.add_argument(
        "--batch-id",
        help="filename/metadata prefix used for every camera in this capture batch",
    )
    capture_p.add_argument(
        "--start-timeout-s",
        type=float,
        default=_config_default(runtime_config, "capture", "start_timeout_s", 5.0),
        help="worker start barrier timeout for multi-camera capture",
    )
    capture_p.set_defaults(func=cmd_capture)

    schedule_p = sub.add_parser("schedule", help="capture repeated batches on a timer")
    schedule_camera_default = _config_default(runtime_config, "schedule", "camera", None)
    schedule_interval_default = _config_default(runtime_config, "schedule", "interval_s", None)
    schedule_p.add_argument(
        "--camera",
        required=schedule_camera_default is None,
        default=schedule_camera_default,
        help="camera name, all, or both",
    )
    schedule_p.add_argument("--count", type=int, required=True, help="number of batches")
    schedule_p.add_argument(
        "--interval-s",
        type=float,
        required=schedule_interval_default is None,
        default=schedule_interval_default,
        help="seconds between scheduled batch starts",
    )
    schedule_p.add_argument(
        "--format",
        default=_config_default(runtime_config, "schedule", "format", None),
        choices=["bmp", "png", "jpg", "jpeg", "tiff"],
    )
    schedule_p.add_argument(
        "--output-dir",
        type=Path,
        default=_root_path_default(runtime_config, runtime_config_path, "output_dir", None)
        if "output_dir" in runtime_config
        else _config_path_default(
            runtime_config, runtime_config_path, "schedule", "output_dir", None
        ),
        help="override output directory",
    )
    schedule_p.add_argument("--schedule-id", help="prefix used for scheduled batch IDs")
    schedule_p.add_argument(
        "--start-timeout-s",
        type=float,
        default=_config_default(runtime_config, "schedule", "start_timeout_s", 5.0),
        help="worker start barrier timeout for multi-camera capture",
    )
    schedule_p.set_defaults(func=cmd_schedule)

    autoshoot_p = sub.add_parser(
        "autoshoot",
        help="continuously capture batches at a fixed start interval",
    )
    autoshoot_p.add_argument(
        "--camera",
        default=_config_default(runtime_config, "autoshoot", "camera", "all"),
        help="camera name, all, or both",
    )
    autoshoot_p.add_argument(
        "--interval-s",
        type=float,
        default=_config_default(runtime_config, "autoshoot", "interval_s", 10.0),
        help="seconds between batch starts; default comes from config.yaml or 10",
    )
    autoshoot_p.add_argument(
        "--format",
        default=_config_default(runtime_config, "autoshoot", "format", None),
        choices=["bmp", "png", "jpg", "jpeg", "tiff"],
    )
    autoshoot_p.add_argument(
        "--output-dir",
        type=Path,
        default=_root_path_default(runtime_config, runtime_config_path, "output_dir", None)
        if "output_dir" in runtime_config
        else _config_path_default(
            runtime_config, runtime_config_path, "autoshoot", "output_dir", None
        ),
        help="override output directory",
    )
    autoshoot_p.add_argument(
        "--batch-prefix",
        default=_config_default(runtime_config, "autoshoot", "batch_prefix", "autoshoot"),
    )
    autoshoot_p.add_argument("--max-batches", type=int, help="optional stop after N batches")
    autoshoot_p.add_argument(
        "--start-timeout-s",
        type=float,
        default=_config_default(runtime_config, "autoshoot", "start_timeout_s", 5.0),
        help="worker start barrier timeout for multi-camera capture",
    )
    autoshoot_p.set_defaults(func=cmd_autoshoot)

    live_p = sub.add_parser(
        "live-session",
        help="run fast temporary live preview plus slower archive saves",
    )
    live_sub = live_p.add_subparsers(dest="live_action", required=True)

    live_run_p = live_sub.add_parser("run", help="run the live session in the foreground")
    add_live_session_run_args(live_run_p, runtime_config, runtime_config_path)
    live_run_p.set_defaults(func=cmd_live_session_run)

    live_start_p = live_sub.add_parser("start", help="start the live session in the background")
    add_live_session_run_args(live_start_p, runtime_config, runtime_config_path)
    live_start_p.add_argument(
        "--pid-file",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "pid_file",
            DEFAULT_LIVE_PID_FILE,
        ),
        help=f"background pid file, default: {DEFAULT_LIVE_PID_FILE}",
    )
    live_start_p.add_argument(
        "--log-file",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "log_file",
            DEFAULT_LIVE_LOG_FILE,
        ),
        help=f"background log file, default: {DEFAULT_LIVE_LOG_FILE}",
    )
    live_start_p.set_defaults(func=cmd_live_session_start)

    live_stop_p = live_sub.add_parser("stop", help="stop the background live session")
    live_stop_p.add_argument(
        "--pid-file",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "pid_file",
            DEFAULT_LIVE_PID_FILE,
        ),
        help=f"background pid file, default: {DEFAULT_LIVE_PID_FILE}",
    )
    live_stop_p.add_argument("--timeout-s", type=float, default=5.0)
    live_stop_p.add_argument("--force", action="store_true", help="send SIGKILL if SIGTERM fails")
    live_stop_p.set_defaults(func=cmd_live_session_stop)

    live_status_p = live_sub.add_parser("status", help="show background live session status")
    live_status_p.add_argument(
        "--pid-file",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "pid_file",
            DEFAULT_LIVE_PID_FILE,
        ),
        help=f"background pid file, default: {DEFAULT_LIVE_PID_FILE}",
    )
    live_status_p.add_argument(
        "--log-file",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "log_file",
            DEFAULT_LIVE_LOG_FILE,
        ),
        help=f"background log file, default: {DEFAULT_LIVE_LOG_FILE}",
    )
    live_status_p.add_argument(
        "--stream-dir",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "live_session",
            "stream_dir",
            DEFAULT_LIVE_STREAM_DIR,
        ),
        help=f"temporary live image root, default: {DEFAULT_LIVE_STREAM_DIR}",
    )
    live_status_p.set_defaults(func=cmd_live_session_status)

    stream_p = sub.add_parser("stream", help="serve a browser MJPEG live stream")
    stream_camera_default = _config_default(runtime_config, "stream", "camera", None)
    stream_p.add_argument(
        "--camera",
        required=stream_camera_default is None,
        default=stream_camera_default,
        help="camera name to stream",
    )
    stream_p.add_argument(
        "--host",
        default=_config_default(runtime_config, "stream", "host", "0.0.0.0"),
        help="bind address; default comes from config.yaml or 0.0.0.0",
    )
    stream_p.add_argument(
        "--port",
        type=int,
        default=_config_default(runtime_config, "stream", "port", 8765),
        help="HTTP port; default comes from config.yaml or 8765",
    )
    stream_p.add_argument(
        "--interval-s",
        type=float,
        default=_config_default(runtime_config, "stream", "interval_s", 0.1),
        help="minimum delay between frames; default comes from config.yaml or 0.1",
    )
    stream_p.add_argument(
        "--timeout-ms",
        type=int,
        default=_config_default(runtime_config, "stream", "timeout_ms", 1000),
        help="per-frame capture timeout in milliseconds; default comes from config.yaml or 1000",
    )
    stream_p.add_argument("--quiet", action="store_true", help="suppress request logs")
    stream_p.set_defaults(func=cmd_stream)

    props_p = sub.add_parser("props", help="read camera properties")
    props_p.add_argument("camera")
    props_p.add_argument("names", nargs="*")
    props_p.set_defaults(func=cmd_props)

    set_p = sub.add_parser("set-prop", help="set camera properties")
    set_p.add_argument("camera")
    set_p.add_argument("assignments", nargs="+")
    set_p.set_defaults(func=cmd_set_props)

    twain_p = sub.add_parser("twain", help="inspect Leica TWAIN source/capabilities")
    twain_p.add_argument("action", choices=["sources", "capabilities"])
    twain_p.add_argument("--source", default="Leica Microsystems Camera")
    twain_p.set_defaults(func=cmd_twain)

    lasx_p = sub.add_parser("lasx", help="control Leica LAS X UI automation bridge")
    lasx_p.add_argument(
        "action",
        choices=["status", "capture", "acquire", "stop"],
        help="LAS X UI action to run",
    )
    lasx_p.add_argument(
        "--wait-seconds",
        type=int,
        default=3,
        help="seconds to leave capture/acquire toggled on before stopping",
    )
    lasx_p.set_defaults(func=cmd_lasx)

    monitor_p = sub.add_parser("monitor", help="serve latest captured images in a browser")
    monitor_p.add_argument(
        "--output-dir",
        type=Path,
        default=_config_path_default(
            runtime_config,
            runtime_config_path,
            "monitor",
            "output_dir",
            _root_path_default(
                runtime_config,
                runtime_config_path,
                "output_dir",
                Path("/home/lamp/SDLsetup/dataset/captures"),
            ),
        ),
        help="folder containing captured image files",
    )
    monitor_p.add_argument(
        "--user-saved-dir",
        type=Path,
        default=_user_saved_dir_default(runtime_config, runtime_config_path, "monitor"),
        help="folder for monitor Take Photo saves; default: output_dir/user-saved",
    )
    monitor_p.add_argument(
        "--host",
        default=_config_default(runtime_config, "monitor", "host", "127.0.0.1"),
    )
    monitor_p.add_argument(
        "--lan",
        action="store_true",
        help="bind to all WSL interfaces and print LAN candidate URLs",
    )
    monitor_p.add_argument(
        "--port",
        type=int,
        default=_config_default(runtime_config, "monitor", "port", 8765),
    )
    monitor_p.add_argument(
        "--refresh-ms",
        type=int,
        default=_config_default(runtime_config, "monitor", "refresh_ms", 2000),
    )
    monitor_p.add_argument("--https", action="store_true", help="serve HTTPS with a local cert")
    monitor_p.add_argument("--cert-file", type=Path, help="HTTPS certificate path")
    monitor_p.add_argument("--key-file", type=Path, help="HTTPS private key path")
    monitor_p.add_argument("--quiet", action="store_true", help="suppress request logs")
    monitor_p.set_defaults(func=cmd_monitor)

    lan_p = sub.add_parser("lan-info", help="print monitor URLs for localhost and LAN access")
    lan_p.add_argument("--host", default="127.0.0.1")
    lan_p.add_argument("--lan", action="store_true", help="show URLs for a 0.0.0.0 bind")
    lan_p.add_argument("--port", type=int, default=8765)
    lan_p.add_argument("--https", action="store_true", help="print HTTPS URLs")
    lan_p.set_defaults(func=cmd_lan_info)

    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--runtime-config", type=Path, default=DEFAULT_RUNTIME_CONFIG)
    pre_args, _ = pre_parser.parse_known_args(argv)
    runtime_config_path = pre_args.runtime_config.expanduser()
    runtime_config = load_root_config(runtime_config_path)
    parser = build_parser(runtime_config, runtime_config_path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        return int(args.func(args))
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
