"""Configuration, well addressing, and the robot-arm interlock.

Everything an orchestrating layer needs to drive the reader is declared here, so
a workflow is "load a config, call functions" rather than "remember which
arguments are safe".

    from tools.nodes.uv_vis import UvVisConfig, SpectroStarNano

    cfg = UvVisConfig.from_dict(yaml_section)      # or UvVisConfig()
    reader = SpectroStarNano.from_config(cfg)

    reader.interlock.set_arm_clear(True)          # arm reports it has retreated
    reader.interlock.set_plate_loaded(True)       # arm reports the plate is seated
    if reader.interlock.ready_to_measure:
        reader.run_protocol(cfg.assays["bca"].protocol)

The interlock exposes both a plain boolean and a single encoded integer
(:attr:`Interlock.code`), so a controller that can only exchange numbers with
the arm still gets the full state in one value.
"""
from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .plates import PlateDefinition

# --------------------------------------------------------------------------
# Plate geometry and well addressing
# --------------------------------------------------------------------------

_WELL_RE = re.compile(r"^([A-Za-z]{1,2})(\d{1,2})$")


@dataclass(frozen=True)
class PlateSpec:
    """A microplate format. 96-well SBS is the default.

    ``definition`` optionally links this spec to a vendor :class:`~.plates.PlateDefinition`
    (physical geometry — heights, well diameters, corner coordinates) recovered
    from the control software's plate library. Row/column/name still work
    exactly as before when ``definition`` is absent; nothing here requires it.
    """

    rows: int = 8
    columns: int = 12
    name: str = "96-well"
    definition: PlateDefinition | None = None

    @classmethod
    def wells96(cls) -> PlateSpec:
        return cls(rows=8, columns=12, name="96-well")

    @classmethod
    def wells384(cls) -> PlateSpec:
        return cls(rows=16, columns=24, name="384-well")

    @classmethod
    def from_library(cls, name: str) -> PlateSpec:
        """Build a spec from the vendor plate library (see :mod:`.plates`).

        Looks the plate up by name (case-insensitive, whitespace-tolerant) and
        carries its full geometry along in :attr:`definition`.
        """
        from .plates import find  # deferred: plates.py imports PlateSpec from here

        d = find(name)
        return cls(rows=d.rows, columns=d.columns, name=d.name, definition=d)

    @property
    def well_count(self) -> int:
        return self.rows * self.columns

    @property
    def row_labels(self) -> list[str]:
        # A..H for 96, A..P for 384. Two-letter labels are not needed below 676.
        return [chr(ord("A") + i) for i in range(self.rows)]

    def validate(self, well: str) -> str:
        """Normalise a well id (``a1`` -> ``A1``) and bounds-check it."""
        m = _WELL_RE.match(well.strip())
        if not m:
            raise ValueError(f"malformed well id {well!r} (expected e.g. 'A1', 'H12')")
        row, col = m.group(1).upper(), int(m.group(2))
        if row not in self.row_labels:
            raise ValueError(
                f"row {row!r} outside {self.name} (rows {self.row_labels[0]}"
                f"–{self.row_labels[-1]})"
            )
        if not 1 <= col <= self.columns:
            raise ValueError(
                f"column {col} outside {self.name} (1–{self.columns})"
            )
        return f"{row}{col}"

    def all_wells(self, by: str = "row") -> list[str]:
        """Every well, ordered row-major (default) or column-major."""
        if by == "row":
            return [f"{r}{c}" for r in self.row_labels
                    for c in range(1, self.columns + 1)]
        if by == "column":
            return [f"{r}{c}" for c in range(1, self.columns + 1)
                    for r in self.row_labels]
        raise ValueError("by must be 'row' or 'column'")


@dataclass
class WellSelection:
    """Which wells a run should use.

    Built from whichever form is most natural, and always normalised to an
    explicit ordered list:

        WellSelection.parse("A1:C12", plate)     # rectangular block
        WellSelection.parse("A1,A2,B7", plate)   # explicit list
        WellSelection.rows("A", "B", plate=p)    # whole rows
        WellSelection.columns(1, 2, plate=p)     # whole columns
        WellSelection.all(plate=p)
    """

    wells: list[str] = field(default_factory=list)
    plate: PlateSpec = field(default_factory=PlateSpec.wells96)

    def __post_init__(self) -> None:
        self.wells = [self.plate.validate(w) for w in self.wells]

    # -- constructors -----------------------------------------------------

    @classmethod
    def all(cls, plate: PlateSpec | None = None, by: str = "row") -> WellSelection:
        p = plate or PlateSpec.wells96()
        return cls(wells=p.all_wells(by=by), plate=p)

    @classmethod
    def rows(cls, *rows: str, plate: PlateSpec | None = None) -> WellSelection:
        p = plate or PlateSpec.wells96()
        out: list[str] = []
        for r in rows:
            r = r.upper()
            if r not in p.row_labels:
                raise ValueError(f"row {r!r} outside {p.name}")
            out += [f"{r}{c}" for c in range(1, p.columns + 1)]
        return cls(wells=out, plate=p)

    @classmethod
    def columns(cls, *cols: int, plate: PlateSpec | None = None) -> WellSelection:
        p = plate or PlateSpec.wells96()
        out: list[str] = []
        for c in cols:
            if not 1 <= c <= p.columns:
                raise ValueError(f"column {c} outside {p.name}")
            out += [f"{r}{c}" for r in p.row_labels]
        return cls(wells=out, plate=p)

    @classmethod
    def parse(cls, spec: str, plate: PlateSpec | None = None) -> WellSelection:
        """Parse a compact spec.

        Accepts comma-separated terms, each either a single well (``A1``) or a
        rectangular block (``A1:C12`` / ``A1-C12``). Blocks are inclusive and
        expand row-major. ``all`` selects the whole plate.
        """
        p = plate or PlateSpec.wells96()
        spec = spec.strip()
        if spec.lower() in ("all", "*"):
            return cls.all(plate=p)
        out: list[str] = []
        for term in (t.strip() for t in spec.split(",") if t.strip()):
            if ":" in term or "-" in term:
                sep = ":" if ":" in term else "-"
                start, end = (s.strip() for s in term.split(sep, 1))
                out += cls._expand_block(start, end, p)
            else:
                out.append(p.validate(term))
        # de-duplicate, preserving order
        seen: set[str] = set()
        return cls(wells=[w for w in out if not (w in seen or seen.add(w))], plate=p)

    @staticmethod
    def _expand_block(start: str, end: str, plate: PlateSpec) -> list[str]:
        s, e = plate.validate(start), plate.validate(end)
        sm, em = _WELL_RE.match(s), _WELL_RE.match(e)
        r0, c0 = sm.group(1), int(sm.group(2))          # type: ignore[union-attr]
        r1, c1 = em.group(1), int(em.group(2))          # type: ignore[union-attr]
        i0, i1 = sorted((plate.row_labels.index(r0), plate.row_labels.index(r1)))
        c0, c1 = sorted((c0, c1))
        return [f"{plate.row_labels[i]}{c}"
                for i in range(i0, i1 + 1) for c in range(c0, c1 + 1)]

    # -- use --------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.wells)

    def __iter__(self):
        return iter(self.wells)

    def to_list(self) -> list[str]:
        return list(self.wells)

    def as_spec(self) -> str:
        return ",".join(self.wells)


# --------------------------------------------------------------------------
# Robot-arm interlock
# --------------------------------------------------------------------------

# Bit positions in Interlock.code. Stable — an orchestrator may hardcode these.
BIT_READER_PRESENT = 1 << 0     # 1   instrument enumerated by Windows
BIT_CARRIER_OUT    = 1 << 1     # 2   carrier commanded OUT (open)
BIT_CARRIER_IN     = 1 << 2     # 4   carrier commanded IN (closed)
BIT_PLATE_LOADED   = 1 << 3     # 8   arm reports a plate is seated
BIT_ARM_CLEAR      = 1 << 4     # 16  arm reports it has retreated
BIT_READY          = 1 << 5     # 32  derived: safe to measure
BIT_BUSY           = 1 << 6     # 64  a command is in flight
BIT_ERROR          = 1 << 7     # 128 last operation failed


@dataclass
class Interlock:
    """Handshake state between the reader and the robot arm.

    The arm (or the orchestrator on its behalf) *declares* two facts that the
    reader cannot sense for itself — whether a plate is seated, and whether the
    arm has retreated clear of the drawer. Everything else is derived.

    This is a software interlock, not a safety rated one: the reader has no
    presence sensor exposed through the ActiveX interface, so
    :attr:`plate_loaded` and :attr:`arm_clear` are trusted assertions. Treat
    them as a coordination protocol, never as a guard against collision.
    """

    reader_present: bool = False
    carrier: str = "unknown"          # unknown | in | out
    plate_loaded: bool = False
    arm_clear: bool = True
    busy: bool = False
    error: str | None = None
    updated_at: str | None = None

    # -- arm-facing setters ----------------------------------------------

    def set_arm_clear(self, clear: bool) -> Interlock:
        """Arm declares whether it is clear of the reader."""
        self.arm_clear = bool(clear)
        self._stamp()
        return self

    def set_plate_loaded(self, loaded: bool) -> Interlock:
        """Arm declares whether a plate is seated on the carrier."""
        self.plate_loaded = bool(loaded)
        self._stamp()
        return self

    def _stamp(self) -> None:
        self.updated_at = datetime.now(timezone.utc).isoformat(timespec="seconds")

    # -- derived ----------------------------------------------------------

    @property
    def carrier_may_move(self) -> bool:
        """True when it is safe to command the carrier: the arm must be clear."""
        return self.reader_present and self.arm_clear

    @property
    def ready_to_measure(self) -> bool:
        """True when a measurement may start.

        Requires: instrument present, carrier closed, a plate seated, the arm
        retreated, nothing in flight, no error outstanding.
        """
        return (
            self.reader_present
            and self.carrier == "in"
            and self.plate_loaded
            and self.arm_clear
            and not self.busy
            and self.error is None
        )

    @property
    def code(self) -> int:
        """The whole state as one integer, for controllers that exchange numbers.

        Bit layout is the ``BIT_*`` constants in this module; ``code == 63``
        (present|out|in is impossible, so in practice 1|4|8|16|32 = 61) means
        fully ready. Decode with :meth:`describe` rather than by eye.
        """
        c = 0
        if self.reader_present:
            c |= BIT_READER_PRESENT
        if self.carrier == "out":
            c |= BIT_CARRIER_OUT
        if self.carrier == "in":
            c |= BIT_CARRIER_IN
        if self.plate_loaded:
            c |= BIT_PLATE_LOADED
        if self.arm_clear:
            c |= BIT_ARM_CLEAR
        if self.ready_to_measure:
            c |= BIT_READY
        if self.busy:
            c |= BIT_BUSY
        if self.error:
            c |= BIT_ERROR
        return c

    def describe(self) -> dict:
        """Human- and machine-readable state, including the encoded code."""
        return {
            "code": self.code,
            "ready_to_measure": self.ready_to_measure,
            "carrier_may_move": self.carrier_may_move,
            "reader_present": self.reader_present,
            "carrier": self.carrier,
            "plate_loaded": self.plate_loaded,
            "arm_clear": self.arm_clear,
            "busy": self.busy,
            "error": self.error,
            "updated_at": self.updated_at,
        }

    @staticmethod
    def decode(code: int) -> dict[str, bool]:
        """Expand an integer code back into named flags."""
        return {
            "reader_present": bool(code & BIT_READER_PRESENT),
            "carrier_out":    bool(code & BIT_CARRIER_OUT),
            "carrier_in":     bool(code & BIT_CARRIER_IN),
            "plate_loaded":   bool(code & BIT_PLATE_LOADED),
            "arm_clear":      bool(code & BIT_ARM_CLEAR),
            "ready":          bool(code & BIT_READY),
            "busy":           bool(code & BIT_BUSY),
            "error":          bool(code & BIT_ERROR),
        }


# --------------------------------------------------------------------------
# Incubation and shaking
# --------------------------------------------------------------------------


@dataclass
class Incubation:
    """An incubator setpoint request, translated to the ``Temp`` DDE argument.

    The standard SPECTROstar Nano incubator supports roughly 25.0–45.0 °C;
    BMG advise a target at least ``ambient_margin_c`` above ambient (not
    enforced here — no ambient sensor is exposed to check it against).
    ``target_c=None`` means "leave the incubator alone" and is not itself a
    command; use :meth:`~.reader.SpectroStarNano.temperature_off` for
    ``Temp 0`` and :meth:`~.reader.SpectroStarNano.monitor_temperature` for
    ``Temp 0.1`` (sensor readback with no heating).
    """

    MIN_C = 25.0
    MAX_C = 45.0

    target_c: float | None = None
    monitor_only: bool = False
    ambient_margin_c: float = 5.0

    def validate(self) -> None:
        if self.target_c is not None and not (self.MIN_C <= self.target_c <= self.MAX_C):
            raise ValueError(
                f"incubator target {self.target_c} °C is outside the "
                f"supported {self.MIN_C}–{self.MAX_C} °C range"
            )

    def to_command_value(self) -> str:
        """The string to send as ``Temp``'s argument: ``"0"`` off, ``"0.1"``
        monitor-only, else the one-decimal target in °C."""
        self.validate()
        if self.monitor_only:
            return "0.1"
        if self.target_c is None:
            return "0"
        return f"{self.target_c:.1f}"


#: Shake-mode name <-> DDE integer code (fact 6). Both spellings accepted.
_SHAKE_MODE_CODES: dict[str, int] = {
    "orbital": 0,
    "linear": 1,
    "double_orbital": 2,
    "double orbital": 2,
    "meander": 3,
    "meander_corner": 3,
    "meander corner well": 3,
}
_SHAKE_MODE_NAMES = ("orbital", "linear", "double_orbital", "meander")


@dataclass
class Shaking:
    """A ``Shake`` request: ``Shake <mode> <rpm> <time_s> [<pos_x> <pos_y>]``.

    UNVERIFIED-ON-DDE: only the script-language form (``R_Shake``) is
    documented in the vendor help; the exact DDE positional-argument shape
    implemented by :meth:`to_args` (and used by
    :meth:`~.reader.SpectroStarNano.shake`) has not been confirmed against a
    live instrument.

    Caps (fact 6): orbital/double-orbital 100–700 rpm, or up to 1100 rpm with
    ``high_speed=True``; linear 100–700 rpm, or up to 800 rpm with
    ``high_speed=True``; meander (corner well) 100–300 rpm regardless of
    ``high_speed``. ``time_s`` must be 1–3600. ``position_x`` (250–3100) and
    ``position_y`` (125–800) are optional; ``9999`` on either means "random
    position" and is what :meth:`to_args` sends when only one of the pair is
    given.
    """

    mode: str | int = "orbital"
    frequency_rpm: int = 300
    time_s: int = 10
    position_x: int | None = None
    position_y: int | None = None
    high_speed: bool = False

    def _mode_code(self) -> int:
        key = self.mode.strip().lower() if isinstance(self.mode, str) else self.mode
        if key in _SHAKE_MODE_CODES:
            return _SHAKE_MODE_CODES[key]
        if key in (0, 1, 2, 3):
            return int(key)
        raise ValueError(
            f"unknown shake mode {self.mode!r}; expected one of "
            f"{_SHAKE_MODE_NAMES} or their integer codes 0-3"
        )

    def validate(self) -> None:
        code = self._mode_code()
        if not (1 <= self.time_s <= 3600):
            raise ValueError(f"shake time {self.time_s} s is outside the 1-3600 s range")

        if code == 3:  # meander corner well
            lo, hi = 100, 300
            if not (lo <= self.frequency_rpm <= hi):
                raise ValueError(
                    f"meander shaking frequency {self.frequency_rpm} rpm is "
                    f"outside the {lo}-{hi} rpm cap (meander never goes high-speed)"
                )
        elif code == 1:  # linear
            hi = 800 if self.high_speed else 700
            if not (100 <= self.frequency_rpm <= hi):
                raise ValueError(
                    f"linear shaking frequency {self.frequency_rpm} rpm is "
                    f"outside the 100-{hi} rpm cap"
                    + ("" if self.high_speed else " (700 rpm normal, 800 "
                                                  "rpm needs high_speed=True)")
                )
        else:  # orbital, double orbital
            hi = 1100 if self.high_speed else 700
            if not (100 <= self.frequency_rpm <= hi):
                raise ValueError(
                    f"shaking frequency {self.frequency_rpm} rpm is outside "
                    f"the 100-{hi} rpm cap"
                    + ("" if self.high_speed else " (700 rpm normal, 1100 "
                                                  "rpm needs high_speed=True)")
                )

        if self.position_x is not None and not (250 <= self.position_x <= 3100):
            raise ValueError(f"shake position_x {self.position_x} is outside 250-3100")
        if self.position_y is not None and not (125 <= self.position_y <= 800):
            raise ValueError(f"shake position_y {self.position_y} is outside 125-800")

    def to_args(self) -> list[str]:
        """Positional DDE arguments: ``[mode, rpm, time_s]`` plus position if set."""
        self.validate()
        args = [str(self._mode_code()), str(self.frequency_rpm), str(self.time_s)]
        if self.position_x is not None or self.position_y is not None:
            args.append(str(self.position_x if self.position_x is not None else 9999))
            args.append(str(self.position_y if self.position_y is not None else 9999))
        return args


# --------------------------------------------------------------------------
# Assay and top-level configuration
# --------------------------------------------------------------------------


@dataclass
class AssayConfig:
    """One assay, bound to a control-software protocol.

    ``protocol`` must name a protocol that exists in the SPECTROstar Nano
    control software — this package selects one, it cannot define measurement
    parameters. Leave it empty until the protocol has been authored.
    """

    protocol: str = ""
    wavelength_nm: int = 562
    model: str = "linear"                 # linear | quadratic
    blank_wells: list[str] = field(default_factory=list)
    standard_wells: list[str] = field(default_factory=list)
    standard_concentrations: list[float] = field(default_factory=list)
    sample_wells: list[str] = field(default_factory=list)
    dilution_factor: float = 1.0

    def validate(self, plate: PlateSpec) -> None:
        for group in ("blank_wells", "standard_wells", "sample_wells"):
            setattr(self, group, [plate.validate(w) for w in getattr(self, group)])
        if self.standard_wells and (
            len(self.standard_wells) != len(self.standard_concentrations)
        ):
            raise ValueError(
                f"standard_wells ({len(self.standard_wells)}) and "
                f"standard_concentrations ({len(self.standard_concentrations)}) "
                "must be the same length"
            )


@dataclass
class UvVisConfig:
    """Everything the UV-Vis layer needs, in one object.

    Load from the ``uv_vis:`` section of ``config.yaml`` with
    :meth:`from_dict`, or construct directly for a one-off run.
    """

    reader: str = "SPECTROstar_Nano"
    allow_motion: bool = False
    timeout_s: float = 240.0
    log_dir: str = "/home/lamp/SDLsetup/dataset/uv_vis_runs"
    export_dir: str = (
        "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/User/Data"
    )
    plate: PlateSpec = field(default_factory=PlateSpec.wells96)
    wells: WellSelection | None = None
    require_interlock: bool = True
    assays: dict[str, AssayConfig] = field(default_factory=dict)
    incubation: Incubation | None = None
    shaking: Shaking | None = None
    protocol_db_path: str = (
        r"C:\Program Files (x86)\BMG\SPECTROstar Nano\User\Definit"
    )
    layout_dir: str = "/home/lamp/SDLsetup/dataset/uv_vis_runs/layouts"

    def __post_init__(self) -> None:
        if self.wells is None:
            self.wells = WellSelection.all(plate=self.plate)
        for a in self.assays.values():
            a.validate(self.plate)

    # -- (de)serialisation -------------------------------------------------

    @classmethod
    def from_dict(cls, data: dict | None) -> UvVisConfig:
        """Build from a plain dict, e.g. the ``uv_vis:`` block of config.yaml.

        Unknown keys are ignored so the repo's hand-rolled YAML parser and this
        class can evolve independently.
        """
        data = dict(data or {})

        plate_raw = data.get("plate", 96)
        if isinstance(plate_raw, dict):
            plate = PlateSpec(**{k: v for k, v in plate_raw.items()
                                 if k in ("rows", "columns", "name")})
        else:
            plate = (PlateSpec.wells384() if str(plate_raw).strip() in ("384", "384-well")
                     else PlateSpec.wells96())

        wells_raw = data.get("wells")
        wells = (WellSelection.parse(str(wells_raw), plate)
                 if wells_raw else WellSelection.all(plate=plate))

        assays: dict[str, AssayConfig] = {}
        for key, raw in (data.get("assays") or {}).items():
            raw = dict(raw or {})
            assays[key] = AssayConfig(
                **{k: v for k, v in raw.items()
                   if k in AssayConfig.__dataclass_fields__}
            )

        incubation_raw = data.get("incubation")
        incubation = (
            Incubation(**{k: v for k, v in dict(incubation_raw).items()
                         if k in Incubation.__dataclass_fields__})
            if incubation_raw else None
        )

        shaking_raw = data.get("shaking")
        shaking = (
            Shaking(**{k: v for k, v in dict(shaking_raw).items()
                      if k in Shaking.__dataclass_fields__})
            if shaking_raw else None
        )

        known = ("reader", "allow_motion", "timeout_s", "log_dir",
                 "export_dir", "require_interlock",
                 "protocol_db_path", "layout_dir")
        kwargs = {k: data[k] for k in known if k in data}
        # tolerate the older key name used by the node
        if "allow_uv_vis_motion" in data:
            kwargs["allow_motion"] = data["allow_uv_vis_motion"]
        return cls(plate=plate, wells=wells, assays=assays,
                  incubation=incubation, shaking=shaking, **kwargs)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["plate"] = {"rows": self.plate.rows, "columns": self.plate.columns,
                      "name": self.plate.name}
        d["wells"] = self.wells.as_spec() if self.wells else ""
        d["incubation"] = asdict(self.incubation) if self.incubation else None
        d["shaking"] = asdict(self.shaking) if self.shaking else None
        return d

    @property
    def log_path(self) -> Path:
        return Path(self.log_dir)
