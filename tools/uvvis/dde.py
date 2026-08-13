"""DDE control path — the one that can run measurements.

Why this exists
---------------
The ActiveX path (:mod:`backend`) can only issue **parameterless** commands.
``PlateOut``/``PlateIn``/``Init`` work; ``Run <protocol>`` and ``Temp <degC>``
return success and never reach the instrument, because passing a parameter
requires the ``sExecute*`` family whose arguments are ``VT_LPSTR`` — a raw
``char*`` that PowerShell cannot construct (it only produces ``BSTR``, which the
control rejects with ``0x80020008``).

DDE sidesteps the whole problem: BMG's own ``Cln\\DDEClient.exe`` takes *"the DDE
command and its parameters as command line parameters"*, so the protocol name
travels as plain text and never has to be marshalled.

Verified 2026-07-29 by log evidence, which is the only trustworthy signal here:

===============================  ==========================================
``DDEClient.exe PlateOut``       ``(Plate command)`` count 0 -> 1
``DDEClient.exe Run Protein``    ``(Run command)`` count 0 -> 1, then a full
                                 96-well sweep ending "End of test run.
                                 Meas. data transfer finished"
===============================  ==========================================

Requirements: the control software must be running as a DDE server, i.e.
``[ControlApp] AsDDEserver=True`` in its INI — the *opposite* of what script
mode needs. :func:`~.script.set_mode` manages that flag.

DDE service ``SPECTROstar_Nano``, topic ``DDEServerConv1``, status items
prefixed ``DdeServer`` (e.g. ``DdeServerDeviceBusy``, ``DdeServerDeviceError``,
``DdeServerMeasureData``).
"""
from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

DDE_CLIENT = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/Cln/DDEClient.exe"
)
RUN_LOG = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/SPECTROstar Nano.log"
)
CONTROL_EXE = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/ExeDLL/SPECTROstar_Nano.exe"
)

DDE_SERVICE = "SPECTROstar_Nano"
DDE_TOPIC = "DDEServerConv1"

#: Run-log markers, per command, used to prove a command actually landed.
LOG_MARKERS = {
    "Run": "(Run command)",
    "PlateOut": "(Plate command)",
    "PlateIn": "(Plate command)",
    "Init": "(Init command",
    "Temp": "(Temperature command",
}

RUN_FINISHED = "End of test run"


class DdeError(RuntimeError):
    """A DDE command failed, or could not be proven to have run."""


def log_count(needle: str, log: Path = RUN_LOG) -> int:
    """Occurrences of ``needle`` in the control software's run log."""
    try:
        return log.read_bytes().decode("cp949", errors="replace").count(needle)
    except OSError:
        return 0


def control_running() -> bool:
    out = subprocess.run(
        ["/mnt/c/Windows/System32/tasklist.exe", "/FI",
         "IMAGENAME eq SPECTROstar_Nano.exe"],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=60,
    )
    return "SPECTROstar_Nano.exe" in (out.stdout or "")


def ensure_control_running(settle_s: float = 45.0) -> bool:
    """Start the control software if it is not already up. Returns True if started."""
    if control_running():
        return False
    subprocess.Popen(
        [str(CONTROL_EXE)], cwd=str(CONTROL_EXE.parent),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(settle_s)
    return True


@dataclass
class DdeResult:
    command: str
    args: tuple[str, ...]
    returncode: int
    log_confirmed: bool
    marker: str = ""
    elapsed_s: float = 0.0

    def as_dict(self) -> dict:
        return {
            "command": self.command, "args": list(self.args),
            "returncode": self.returncode, "log_confirmed": self.log_confirmed,
            "marker": self.marker, "elapsed_s": round(self.elapsed_s, 1),
            "ok": self.log_confirmed,
        }


def send(
    command: str,
    *args: str,
    verify: bool = True,
    settle_s: float = 8.0,
    timeout_s: float = 300.0,
) -> DdeResult:
    """Send one DDE command and prove it landed.

    ``DDEClient.exe`` returns 0 whether or not the command reached the
    instrument, so the return code is meaningless on its own. With ``verify``
    (default) this compares the command's run-log marker before and after and
    raises :class:`DdeError` if it did not increase — never reporting a success
    it cannot evidence.
    """
    if not DDE_CLIENT.exists():
        raise DdeError(f"DDE client not found: {DDE_CLIENT}")

    marker = LOG_MARKERS.get(command, "")
    before = log_count(marker) if marker else 0
    started = time.time()

    proc = subprocess.run(
        [str(DDE_CLIENT), command, *args],
        cwd=str(DDE_CLIENT.parent), capture_output=True,
        text=True, encoding="utf-8", errors="replace", timeout=timeout_s,
    )
    time.sleep(settle_s)
    confirmed = (log_count(marker) > before) if marker else False
    result = DdeResult(command, tuple(args), proc.returncode, confirmed,
                       marker, time.time() - started)

    if verify and marker and not confirmed:
        raise DdeError(
            f"DDE {command} {' '.join(args)} returned rc={proc.returncode} but "
            f"the run log shows no new {marker!r} — the command did not reach "
            "the reader. Check that the control software is running with "
            "AsDDEserver=True (script mode disables DDE)."
        )
    return result


# -- convenience wrappers --------------------------------------------------

def plate_out(**kw) -> DdeResult:
    return send("PlateOut", **kw)


def plate_in(**kw) -> DdeResult:
    return send("PlateIn", **kw)


def init(**kw) -> DdeResult:
    return send("Init", **kw)


def run_protocol(
    protocol: str,
    *ids: str,
    wait_for_finish: bool = True,
    timeout_s: float = 900.0,
    poll_s: float = 5.0,
) -> DdeResult:
    """Run a protocol and, by default, wait for the instrument to finish.

    A 96-well spectrum sweep takes minutes; ``DDEClient.exe`` returns as soon as
    the command is accepted, so without the wait a caller would read the run log
    mid-sweep and see partial data.
    """
    if not protocol:
        raise DdeError("protocol name is required")
    finished_before = log_count(RUN_FINISHED)
    result = send("Run", protocol, *ids)

    if wait_for_finish:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if log_count(RUN_FINISHED) > finished_before:
                result.elapsed_s = time.time() - (time.time() - result.elapsed_s)
                return result
            time.sleep(poll_s)
        raise DdeError(
            f"run of {protocol!r} started but no {RUN_FINISHED!r} appeared "
            f"within {timeout_s}s — check the instrument."
        )
    return result


def wells_measured(log: Path = RUN_LOG) -> list[str]:
    """Wells the control software reported values for, in log order.

    Parsed from ``Got measurement values for <well>`` lines. This proves *which*
    wells a run covered; it does not give the values themselves, which stay in
    the proprietary ``MeasurementData.abs``.
    """
    try:
        text = log.read_bytes().decode("cp949", errors="replace")
    except OSError:
        return []
    out: list[str] = []
    for line in text.splitlines():
        if "Got measurement values for" in line:
            well = line.split("Got measurement values for")[-1].strip()
            if well:
                out.append(well.upper())
    return out


# --------------------------------------------------------------------------
# Instrument presence (not an ActiveX concern -- plain Windows enumeration)
# --------------------------------------------------------------------------

PWSH = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
READER_HWID = "VID_0483&PID_A29A"


def reader_present() -> bool:
    """True if Windows currently enumerates the reader.

    Cheap pre-flight so a powered-off instrument fails fast instead of after a
    long DDE timeout. Uses plain PnP enumeration — no COM, so the 32-bit
    constraint that applied to the old ActiveX bridge does not apply here.
    """
    ps = (
        "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; "
        f"$d = Get-PnpDevice -PresentOnly | Where-Object {{ $_.InstanceId -match "
        f"'{READER_HWID}' }}; "
        "if ($d) { 'present' } else { 'absent' }"
    )
    try:
        out = subprocess.run(
            [PWSH, "-NoProfile", "-Command", ps],
            capture_output=True, text=True, timeout=60,
            encoding="utf-8", errors="replace",
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return "present" in (out.stdout or "")


def available() -> bool:
    """True if the DDE client and the control software executable are present."""
    return DDE_CLIENT.exists() and CONTROL_EXE.exists()
