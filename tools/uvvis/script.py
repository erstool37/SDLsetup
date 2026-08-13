"""BMG `.btc` script generation — the well-selection workaround.

Why this exists
---------------
The ActiveX interface can *select* a protocol by name but cannot define which
wells it reads. The `.btc` script language can: ``R_EditLayout`` rewrites a
protocol's layout, so the wells and their roles become a runtime parameter
instead of something hand-drawn in the UI.

**The catch, and it is a hard one.** BMG's manual states verbatim: *"The script
mode is not available when the program is used in ActiveX or DDE mode, e.g. as
part of a robotic system."* So a single control-software session is either
ActiveX-driven or script-driven, never both. That is why
:class:`~.reader.SpectroStarNano` (ActiveX: carrier motion, fast handshakes) and
this module (scripts: layout + measurement + data extraction) are separate
entry points, and why running a script requires the ActiveX session to be
closed first — see :func:`run_script`.

Script command reference (from the decompiled vendor help, *The Script
Language*):

===============================  ==========================================
``R_GetProtocolNames {sep}``     list existing protocols
``R_EditLayout {p} {actions}``   set well contents; ``EmptyLayout`` clears
``R_EditConcAndVol {p} {acts}``  set standard concentrations
``R_Run {protocol}``             run a measurement
``R_GetData {well} {cyc} {wl}``  read one well's value back
``R_PlateOut`` / ``R_PlateIn``   carrier motion (script-mode equivalent)
``AddToMemo "text"``             emit a line into the Run Log
===============================  ==========================================

Well content codes used by ``R_EditLayout``: ``B`` blank, ``S<n>`` standard n,
``X<n>`` sample n, ``Empty``/``-`` unused.
"""
from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from .config import AssayConfig, WellSelection

# Where generated scripts and their Windows-visible copies live.
WIN_SCRATCH = Path("/mnt/c/Users/lamp")
CONTROL_EXE = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/ExeDLL/SPECTROstar_Nano.exe"
)
RUN_LOG = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/SPECTROstar Nano.log"
)
CONTROL_INI = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/SPECTROstar Nano.ini"
)

DATA_MARKER = "DATA"


class ScriptError(RuntimeError):
    """A script could not be generated or run."""


# --------------------------------------------------------------------------
# Control-software mode
# --------------------------------------------------------------------------

def current_mode(ini: Path = CONTROL_INI) -> str:
    """Return ``"activex"`` or ``"script"`` per the control software's INI."""
    try:
        text = ini.read_bytes().decode("cp949", errors="replace")
    except OSError as exc:
        raise ScriptError(f"cannot read {ini}: {exc}") from exc
    for line in text.splitlines():
        if line.strip().lower().startswith("asddeserver"):
            _, _, val = line.partition("=")
            return "activex" if val.strip().lower() == "true" else "script"
    return "script"      # key absent => not a DDE server


def set_mode(mode: str, ini: Path = CONTROL_INI) -> str:
    """Switch the control software between ActiveX/DDE and script mode.

    This is THE reason script mode appeared broken. ``[ControlApp]
    AsDDEserver=True`` makes the software enter ActiveX/DDE mode on every
    launch, and BMG's manual states script mode is unavailable in that mode — so
    a ``/s`` launch initialises the reader and then silently ignores every
    script line, with no error anywhere.

    The setting is read at startup, so the software must be restarted (which
    :func:`run_script` does) for a change to take effect. Returns the previous
    mode so a caller can restore it.

    Switching to script mode disables the ActiveX path used by
    :class:`~.reader.SpectroStarNano` — the two are mutually exclusive.
    """
    if mode not in ("activex", "script"):
        raise ScriptError(f"mode must be 'activex' or 'script', got {mode!r}")
    previous = current_mode(ini)
    if previous == mode:
        return previous

    want = "True" if mode == "activex" else "False"
    text = ini.read_bytes().decode("cp949", errors="replace")
    out, seen = [], False
    for line in text.splitlines():
        if line.strip().lower().startswith("asddeserver"):
            out.append(f"AsDDEserver={want}")
            seen = True
        else:
            out.append(line)
    if not seen:                      # key absent: add it under [ControlApp]
        for i, line in enumerate(out):
            if line.strip().lower() == "[controlapp]":
                out.insert(i + 1, f"AsDDEserver={want}")
                break
        else:
            out = ["[ControlApp]", f"AsDDEserver={want}"] + out
    ini.write_bytes(("\r\n".join(out) + "\r\n").encode("cp949", errors="replace"))
    return previous


@dataclass
class ScriptBuilder:
    """Assemble a `.btc` script line by line.

    Scripts are written with CRLF line endings — this is a Windows-native
    interpreter and LF-only files are not reliably parsed.
    """

    lines: list[str] = field(default_factory=list)

    def add(self, line: str = "") -> ScriptBuilder:
        self.lines.append(line)
        return self

    def comment(self, text: str) -> ScriptBuilder:
        return self.add(f"; {text}")

    def memo(self, text: str) -> ScriptBuilder:
        """Emit a line into the Run Log — our only data channel out."""
        return self.add(f'AddToMemo "{text}"')

    def wait(self, seconds: float) -> ScriptBuilder:
        return self.add(f"wait for {seconds} s")

    def wait_ready(self) -> ScriptBuilder:
        return self.add("wait for ready")

    # -- reader operations -------------------------------------------------

    def plate_out(self) -> ScriptBuilder:
        return self.add("R_PlateOut")

    def plate_in(self) -> ScriptBuilder:
        return self.add("R_PlateIn")

    def set_layout(self, protocol: str, actions: str) -> ScriptBuilder:
        return self.add(f'R_EditLayout "{protocol}" "{actions}"')

    def set_concentrations(self, protocol: str, actions: str) -> ScriptBuilder:
        return self.add(f'R_EditConcAndVol "{protocol}" "{actions}"')

    def run(self, protocol: str) -> ScriptBuilder:
        return self.add(f'R_Run "{protocol}"')

    def read_well(self, well: str, cycle: int = 1, wavelength: int = 1,
                  missing: float = -1) -> ScriptBuilder:
        """Read one well and echo it into the Run Log as ``DATA <well>=<v>``."""
        var = "v"
        self.add(f"{var}:=R_GetData {well} {cycle} {wavelength} {missing}")
        return self.memo(f"{DATA_MARKER} {well}=<{var}>")

    def read_wells(self, wells, cycle: int = 1, wavelength: int = 1) -> ScriptBuilder:
        for w in wells:
            self.read_well(w, cycle=cycle, wavelength=wavelength)
        return self

    def list_protocols(self) -> ScriptBuilder:
        """Ask the software to enumerate its protocols into the Run Log."""
        self.add('st1:=R_GetProtocolNames ","')
        return self.memo("PROTOCOLS <st1>")

    # -- output ------------------------------------------------------------

    def text(self) -> str:
        return "\r\n".join(self.lines) + "\r\n"

    def write(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(self.text().encode("ascii", errors="replace"))
        return p


# --------------------------------------------------------------------------
# Layout construction
# --------------------------------------------------------------------------

def layout_actions(
    samples: WellSelection | list[str] | None = None,
    blanks: list[str] | None = None,
    standards: list[str] | None = None,
    clear_first: bool = True,
) -> str:
    """Build the ``R_EditLayout`` action string.

    Wells are numbered in the order given: samples become ``X1..Xn``, standards
    ``S1..Sn``, blanks all ``B``. ``clear_first`` prepends ``EmptyLayout`` so the
    protocol's previous layout cannot leak into the run — strongly recommended,
    since a stale layout is invisible in the results.
    """
    parts: list[str] = ["EmptyLayout"] if clear_first else []
    for i, w in enumerate(standards or [], start=1):
        parts.append(f"{w}=S{i}")
    for w in (blanks or []):
        parts.append(f"{w}=B")
    sample_list = list(samples) if samples is not None else []
    for i, w in enumerate(sample_list, start=1):
        parts.append(f"{w}=X{i}")
    if len(parts) <= (1 if clear_first else 0):
        raise ScriptError("layout is empty — give at least one sample/standard/blank")
    return " ".join(parts)


def concentration_actions(standard_concentrations: list[float]) -> str:
    """Build the ``R_EditConcAndVol`` action string for standards S1..Sn."""
    return " ".join(
        f"S{i}: C={c}" for i, c in enumerate(standard_concentrations, start=1)
    )


def build_assay_script(
    assay: AssayConfig,
    samples: WellSelection | list[str],
    *,
    read_cycle: int = 1,
    wavelength_index: int = 1,
    move_plate: bool = False,
) -> ScriptBuilder:
    """Full measure-and-report script for one assay.

    Sequence: rewrite the layout → set standard concentrations → run → read
    every well back → emit each value into the Run Log for
    :func:`~.analysis.parse_run_log_data`.

    ``move_plate`` adds a ``R_PlateOut``/``R_PlateIn`` pair around the run; leave
    it False when the robot arm has already loaded the plate and closed the
    carrier through the ActiveX path.
    """
    if not assay.protocol:
        raise ScriptError(
            "assay has no protocol name. Discover existing ones with "
            "build_list_protocols_script(), or author one in the control software."
        )
    sample_wells = list(samples)
    b = ScriptBuilder()
    b.comment(f"generated assay script: {assay.name if hasattr(assay, 'name') else ''}"
              f" protocol={assay.protocol}")
    b.comment(f"samples={len(sample_wells)} standards={len(assay.standard_wells)} "
              f"blanks={len(assay.blank_wells)}")
    b.add()

    if move_plate:
        b.plate_out().wait(3).plate_in().wait_ready()

    b.set_layout(
        assay.protocol,
        layout_actions(samples=sample_wells, blanks=assay.blank_wells,
                       standards=assay.standard_wells),
    )
    if assay.standard_concentrations:
        b.set_concentrations(
            assay.protocol, concentration_actions(assay.standard_concentrations)
        )
    b.add()
    b.memo(f"RUNSTART {assay.protocol}")
    b.run(assay.protocol)
    b.wait_ready()
    b.add()

    every = list(assay.standard_wells) + list(assay.blank_wells) + sample_wells
    seen: set[str] = set()
    ordered = [w for w in every if not (w in seen or seen.add(w))]
    b.read_wells(ordered, cycle=read_cycle, wavelength=wavelength_index)
    b.memo("RUNEND")
    return b


def build_list_protocols_script() -> ScriptBuilder:
    """A script that just enumerates the software's protocols into the Run Log."""
    b = ScriptBuilder()
    b.comment("enumerate protocols; result appears in the Run Log as 'PROTOCOLS ...'")
    b.list_protocols()
    return b


# --------------------------------------------------------------------------
# Execution
# --------------------------------------------------------------------------

def run_script(
    builder: ScriptBuilder,
    name: str = "sdl_generated",
    timeout_s: float = 600.0,
    kill_existing: bool = True,
) -> dict:
    """Write the script to the Windows filesystem and run it in script mode.

    The control software is launched as ``SPECTROstar_Nano.exe /s <script>``,
    which executes the script in the background straight after initialisation.

    ``kill_existing`` terminates any running control software first. This is not
    optional housekeeping: an instance already in ActiveX/DDE mode has script
    mode DISABLED, so the script would be silently ignored — the exact failure
    observed on 2026-07-29 (the software started, initialised, and never ran a
    single script line).

    The script must live on the Windows filesystem; PowerShell/the control
    software cannot reliably execute from a ``\\\\wsl.localhost`` UNC path.
    """
    script_path = WIN_SCRATCH / f"{name}.btc"
    builder.write(script_path)
    win_path = f"C:\\Users\\lamp\\{name}.btc"

    # Script mode is only honoured when the software is NOT a DDE server, and
    # the INI is read at startup -- so flip it BEFORE (re)launching.
    previous_mode = set_mode("script")

    if kill_existing:
        subprocess.run(
            ["/mnt/c/Windows/System32/taskkill.exe", "/IM",
             "SPECTROstar_Nano.exe", "/F"],
            capture_output=True, timeout=60,
        )

    proc = subprocess.Popen(
        [str(CONTROL_EXE), "/s", win_path],
        cwd=str(CONTROL_EXE.parent),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return {
        "script": str(script_path),
        "windows_path": win_path,
        "pid": proc.pid,
        "lines": len(builder.lines),
        "mode": "script",
        "previous_mode": previous_mode,
        "note": ("runs asynchronously; poll the Run Log for RUNEND. "
                 "Call set_mode('activex') to restore the ActiveX path."),
    }


def wait_for_marker(
    marker: str = "RUNEND",
    timeout_s: float = 900.0,
    poll_s: float = 5.0,
    log_path: Path = RUN_LOG,
) -> bool:
    """Block until ``marker`` appears in the Run Log, or the timeout expires."""
    import time
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            text = log_path.read_bytes().decode("cp949", errors="replace")
            if marker in text:
                return True
        except OSError:
            pass
        time.sleep(poll_s)
    return False
