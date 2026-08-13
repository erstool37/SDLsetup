"""BMG LABTECH reader backend — WSL→Windows ActiveX bridge.

WSL cannot host COM, so every call shells out to 32-bit Windows PowerShell
running ``bmg_bridge.ps1`` beside this file. The bridge prints one JSON object;
we parse it and raise :class:`BmgError` on failure.

Interop caveat (same failure mode as the cameras, see the surface CLAUDE.md):
WSL→Windows interop is bound to the Windows *logon* session. Calls made from an
ssh-launched process can fail with ``UtilAcceptVsock accept4 failed 110`` once
that session ends. For unattended runs, drive this from the operator's desktop
login rather than from automation.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

# 32-bit PowerShell. The 64-bit host cannot load the x86 BMG_ActiveX.ocx.
PWSH32 = "/mnt/c/Windows/SysWOW64/WindowsPowerShell/v1.0/powershell.exe"

# Windows-side PnP id of the reader. Used only for the presence check.
READER_HWID = "VID_0483&PID_A29A"

_BRIDGE = Path(__file__).with_name("bmg_bridge.ps1")


class BmgError(RuntimeError):
    """A bridge call returned ok=false, or the bridge itself could not run."""


@dataclass
class BmgConfig:
    reader: str = "SPECTROstar Nano"
    timeout_s: float = 120.0
    # Default True. An earlier revision set this False, blaming a stale control
    # process for the 0x8000FFFF E_UNEXPECTED failures -- that diagnosis was
    # WRONG (the real cause was passing the command as a bare string instead of
    # an array; see bmg_bridge.ps1 rule 5). Terminating the software after every
    # call costs a ~45 s relaunch on the next one, which made plate_in time out.
    # Leaving it running keeps consecutive commands fast.
    keep_alive: bool = True


def _win_path(p: Path) -> str:
    """Translate a WSL path to a Windows path for powershell.exe."""
    out = subprocess.run(["wslpath", "-w", str(p)], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if out.returncode != 0:
        raise BmgError(f"wslpath failed for {p}: {out.stderr.strip()}")
    return out.stdout.strip()


def interop_available() -> bool:
    """True if the Windows PowerShell host is reachable from this WSL session."""
    return Path(PWSH32).exists() and shutil.which("wslpath") is not None


def reader_present() -> bool:
    """True if Windows currently enumerates the reader as a present device.

    Cheap pre-flight: avoids a 30 s COM timeout when the instrument is simply
    powered off or unplugged, which is by far the most common failure.
    """
    ps = (
        "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; "
        f"$d = Get-PnpDevice -PresentOnly | Where-Object {{ $_.InstanceId -match "
        f"'{READER_HWID}' }}; "
        "if ($d) { 'present' } else { 'absent' }"
    )
    try:
        out = subprocess.run(
            [PWSH32, "-NoProfile", "-Command", ps],
            capture_output=True, text=True, timeout=30,
            encoding="utf-8", errors="replace",
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return "present" in out.stdout


def call(action: str, cfg: BmgConfig | None = None, **kwargs) -> dict:
    """Invoke one bridge action and return its parsed JSON payload."""
    cfg = cfg or BmgConfig()
    if not interop_available():
        raise BmgError(
            "Windows interop unavailable from this WSL session "
            f"(missing {PWSH32} or wslpath)"
        )

    argv = [
        PWSH32, "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", _win_path(_BRIDGE),
        "-Action", action,
        "-Reader", cfg.reader,
    ]
    if cfg.keep_alive:
        argv.append("-KeepAlive")
    for key, value in kwargs.items():
        if value in (None, ""):
            continue
        # -PlateId1 etc. -- PowerShell params are PascalCase.
        param = "".join(part.capitalize() for part in key.split("_"))
        argv += [f"-{param}", str(value)]

    try:
        proc = subprocess.run(
            argv, capture_output=True, text=True, timeout=cfg.timeout_s,
            encoding="utf-8", errors="replace",
        )
    except subprocess.TimeoutExpired as exc:
        raise BmgError(f"{action}: bridge timed out after {cfg.timeout_s}s") from exc
    except OSError as exc:
        raise BmgError(f"{action}: could not start bridge: {exc}") from exc

    stdout = (proc.stdout or "").strip()
    if not stdout:
        raise BmgError(
            f"{action}: bridge produced no output "
            f"(rc={proc.returncode}) {(proc.stderr or '').strip()[:200]}"
        )
    try:
        payload = json.loads(stdout)
    except json.JSONDecodeError as exc:
        raise BmgError(f"{action}: unparseable bridge output: {stdout[:200]}") from exc

    if not payload.get("ok"):
        raise BmgError(
            f"{action}: {payload.get('error', 'unknown')}: {payload.get('detail', '')}"
        )
    return payload
