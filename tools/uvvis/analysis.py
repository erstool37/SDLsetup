"""Result retrieval, saturation analysis, and report writing.

This is the layer the orchestrator reads. It answers three questions:

1. **Did the measurement land in a usable range?** — :class:`SaturationReport`.
   An absorbance can fail in both directions: above the detector's linear range
   (saturated, the value is a floor-bound lie) or in the noise (below detection).
   Either way the concentration derived from it is not trustworthy, and the
   orchestrator needs to know to dilute and re-read rather than record a number.

2. **What is the concentration?** — via a :class:`~.assays.StandardCurve` or
   Beer–Lambert, with the saturation verdict attached so a caller cannot use a
   concentration without seeing its reliability.

3. **What do I persist?** — :func:`write_report` emits JSON or YAML.

Absorbance is logarithmic: A = log10(I0/I). At A = 2 only 1% of light reaches
the detector, at A = 3 only 0.1%. Photometric linearity therefore degrades well
before the electronics clip, which is why the default ceiling here is 2.5 rather
than the instrument's nominal maximum.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .assays import AssayError, StandardCurve, a280_concentration

# Photometric limits, absorbance units.
LINEAR_CEILING = 2.5      # above this, treat the reading as unreliable
WARN_CEILING = 2.0        # above this, warn and suggest dilution
DETECTION_FLOOR = 0.01    # below this, indistinguishable from blank noise

STATUS_OK = "ok"
STATUS_NEAR_SATURATION = "near_saturation"
STATUS_SATURATED = "saturated"
STATUS_BELOW_DETECTION = "below_detection"
STATUS_ABOVE_CURVE = "above_standard_curve"
STATUS_BELOW_CURVE = "below_standard_curve"

_UNRELIABLE = {STATUS_SATURATED, STATUS_BELOW_DETECTION, STATUS_ABOVE_CURVE}


@dataclass
class WellResult:
    """One well's absorbance, verdict, and derived concentration."""

    well: str
    absorbance: float
    status: str = STATUS_OK
    concentration: float | None = None
    unit: str = ""
    reliable: bool = True
    recommended_dilution: float | None = None
    note: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class SaturationReport:
    """Verdict over a whole plate read."""

    generated_at: str = ""
    assay: str = ""
    wavelength_nm: int = 0
    protocol: str = ""
    wells: list[WellResult] = field(default_factory=list)
    curve: dict | None = None
    calibration: dict = field(default_factory=dict)
    summary: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.generated_at:
            self.generated_at = datetime.now(timezone.utc).isoformat(timespec="seconds")

    # -- views -------------------------------------------------------------

    @property
    def saturated(self) -> list[str]:
        return [w.well for w in self.wells if w.status == STATUS_SATURATED]

    @property
    def unreliable(self) -> list[str]:
        return [w.well for w in self.wells if not w.reliable]

    @property
    def all_reliable(self) -> bool:
        return all(w.reliable for w in self.wells)

    def needs_dilution(self) -> dict[str, float]:
        """Wells that should be diluted and re-read, with a suggested factor."""
        return {w.well: w.recommended_dilution for w in self.wells
                if w.recommended_dilution}

    # -- per-well accessors (the orchestrator's usual entry points) ---------

    def booleans(self, key: str = "reliable") -> dict[str, bool]:
        """``{well: bool}`` for one boolean property.

        ``key`` may be ``reliable`` (default), ``saturated``, ``in_range``, or
        ``has_concentration`` — enough to branch per well without parsing the
        full record.
        """
        out: dict[str, bool] = {}
        for w in self.wells:
            if key == "reliable":
                out[w.well] = w.reliable
            elif key == "saturated":
                out[w.well] = w.status == STATUS_SATURATED
            elif key == "in_range":
                out[w.well] = w.status == STATUS_OK
            elif key == "has_concentration":
                out[w.well] = w.concentration is not None
            else:
                raise ValueError(
                    f"unknown boolean key {key!r} (reliable | saturated | "
                    "in_range | has_concentration)"
                )
        return out

    def concentrations(self, only_reliable: bool = True) -> dict[str, float | None]:
        """``{well: concentration}``.

        With ``only_reliable`` (default) unreliable wells map to ``None`` rather
        than to a number derived from a saturated or out-of-range reading.
        """
        return {
            w.well: (w.concentration if (w.reliable or not only_reliable) else None)
            for w in self.wells
        }

    def absorbances(self) -> dict[str, float]:
        return {w.well: w.absorbance for w in self.wells}

    def by_well(self) -> dict[str, dict]:
        """``{well: full record}`` — everything known about each well."""
        return {w.well: w.as_dict() for w in self.wells}

    def wells_where(self, **criteria) -> list[str]:
        """Wells matching every given field, e.g. ``wells_where(status="ok")``."""
        out = []
        for w in self.wells:
            d = w.as_dict()
            if all(d.get(k) == v for k, v in criteria.items()):
                out.append(w.well)
        return out

    def as_dict(self) -> dict:
        return {
            "generated_at": self.generated_at,
            "assay": self.assay,
            "wavelength_nm": self.wavelength_nm,
            "protocol": self.protocol,
            "curve": self.curve,
            "calibration": self.calibration,
            "summary": self.summary,
            "wells": [w.as_dict() for w in self.wells],
        }

    # -- the orchestrator-facing verdict -----------------------------------

    def orchestrator_view(self) -> dict:
        """Compact decision payload for the orchestration layer.

        Deliberately small and flat: an orchestrator should be able to branch on
        this without parsing the whole report.
        """
        return {
            "ok": self.all_reliable,
            "n_wells": len(self.wells),
            "n_saturated": len(self.saturated),
            "n_unreliable": len(self.unreliable),
            "saturated_wells": self.saturated,
            "needs_dilution": self.needs_dilution(),
            "action": (
                "proceed" if self.all_reliable
                else ("dilute_and_reread" if self.needs_dilution() else "review")
            ),
        }


def classify_absorbance(
    absorbance: float,
    linear_ceiling: float = LINEAR_CEILING,
    warn_ceiling: float = WARN_CEILING,
    detection_floor: float = DETECTION_FLOOR,
) -> tuple[str, float | None, str]:
    """Classify one absorbance. Returns ``(status, suggested_dilution, note)``.

    The suggested dilution aims to land the re-read near A≈1.0, comfortably
    inside the linear range.
    """
    if absorbance >= linear_ceiling:
        factor = max(2.0, round(absorbance / 1.0, 1))
        return (STATUS_SATURATED, factor,
                f"A={absorbance:.3f} at/above the linear ceiling "
                f"({linear_ceiling}); the value is not trustworthy — dilute "
                f"~{factor}x and re-read")
    if absorbance >= warn_ceiling:
        factor = max(2.0, round(absorbance / 1.0, 1))
        return (STATUS_NEAR_SATURATION, factor,
                f"A={absorbance:.3f} above the comfortable ceiling "
                f"({warn_ceiling}); usable but a ~{factor}x dilution is safer")
    if absorbance <= detection_floor:
        return (STATUS_BELOW_DETECTION, None,
                f"A={absorbance:.3f} at/below the detection floor "
                f"({detection_floor}); indistinguishable from blank")
    return (STATUS_OK, None, "")


def analyse_absorbances(
    absorbances: dict[str, float],
    *,
    assay: str = "",
    wavelength_nm: int = 0,
    protocol: str = "",
    curve: StandardCurve | None = None,
    blank: float | None = None,
    dilution_factor: float = 1.0,
    unit: str = "ug/mL",
    extinction_coefficient: float | None = None,
    molecular_weight: float | None = None,
    mass_extinction_coefficient: float | None = None,
    path_length_cm: float = 1.0,
    linear_ceiling: float = LINEAR_CEILING,
) -> SaturationReport:
    """Turn raw well absorbances into a full report.

    Supply ``curve`` for standard-curve assays (BCA/Bradford), or
    ``extinction_coefficient`` + ``molecular_weight`` for direct A280. With
    neither, absorbance is reported without a concentration.

    A concentration is only ever attached to a well whose absorbance passed the
    saturation check — a saturated reading is reported as unreliable with the
    concentration left ``None``, never quietly converted.
    """
    report = SaturationReport(
        assay=assay, wavelength_nm=wavelength_nm, protocol=protocol
    )
    # Beer-Lambert path reports mg/mL; fix the unit up front so unreliable
    # wells carry the right unit too, not just the ones that got a value.
    if curve is None and (mass_extinction_coefficient or
                          (extinction_coefficient and molecular_weight)):
        unit = "mg/mL"
    if curve is not None:
        report.curve = {
            "model": curve.model,
            "coefficients": list(curve.coeffs),
            "r_squared": round(curve.r_squared, 6),
            "standards": list(curve.concentrations),
        }

    for well, raw in absorbances.items():
        corrected = raw - blank if blank is not None else raw
        status, dilution, note = classify_absorbance(
            corrected, linear_ceiling=linear_ceiling
        )
        res = WellResult(
            well=well, absorbance=round(corrected, 6), status=status,
            recommended_dilution=dilution, note=note, unit=unit,
        )

        if status not in _UNRELIABLE:
            try:
                if curve is not None:
                    if not curve.in_range(corrected):
                        hi = max(curve.absorbances)
                        res.status = (STATUS_ABOVE_CURVE if corrected > hi
                                      else STATUS_BELOW_CURVE)
                        res.note = (
                            f"A={corrected:.3f} outside the standard curve "
                            f"({min(curve.absorbances):.3f}–{hi:.3f})"
                        )
                        if res.status == STATUS_ABOVE_CURVE:
                            res.recommended_dilution = max(2.0, round(corrected / hi, 1))
                    else:
                        res.concentration = round(
                            curve.concentration(corrected) * dilution_factor, 4
                        )
                elif mass_extinction_coefficient or (
                    extinction_coefficient and molecular_weight
                ):
                    res.concentration = round(
                        a280_concentration(
                            corrected,
                            extinction_coefficient=extinction_coefficient,
                            molecular_weight=molecular_weight,
                            mass_extinction_coefficient=mass_extinction_coefficient,
                            path_length_cm=path_length_cm,
                        ) * dilution_factor, 6
                    )
                    res.unit = "mg/mL"
            except AssayError as exc:
                res.note = str(exc)
                res.status = STATUS_ABOVE_CURVE

        res.reliable = res.status not in _UNRELIABLE
        report.wells.append(res)

    # Record the calibration that produced these numbers. A concentration whose
    # coefficient is not written down cannot be checked later, which is exactly
    # the failure mode that made the instrument's own prompt unsafe.
    report.calibration = {
        "mass_extinction_coefficient_L_per_g_cm": mass_extinction_coefficient,
        "molar_extinction_coefficient_M_per_cm": extinction_coefficient,
        "molecular_weight_Da": molecular_weight,
        "path_length_cm": path_length_cm,
        "blank_subtracted": blank,
        "dilution_factor": dilution_factor,
        "approximate": not (mass_extinction_coefficient or
                            (extinction_coefficient and molecular_weight)) and curve is None,
    }
    conc = [w.concentration for w in report.wells if w.concentration is not None]
    report.summary = {
        "n_wells": len(report.wells),
        "n_ok": sum(1 for w in report.wells if w.status == STATUS_OK),
        "n_saturated": len(report.saturated),
        "n_unreliable": len(report.unreliable),
        "absorbance_min": round(min(absorbances.values()), 6) if absorbances else None,
        "absorbance_max": round(max(absorbances.values()), 6) if absorbances else None,
        "concentration_min": round(min(conc), 4) if conc else None,
        "concentration_max": round(max(conc), 4) if conc else None,
        "concentration_mean": round(sum(conc) / len(conc), 4) if conc else None,
        "unit": unit,
    }
    return report


# --------------------------------------------------------------------------
# Retrieval from the control software's Run Log
# --------------------------------------------------------------------------

def parse_run_log_data(
    log_path: str | Path,
    marker: str = "DATA",
    encoding: str = "cp949",
) -> dict[str, float]:
    """Recover ``well -> absorbance`` from the control software's Run Log.

    The generated .btc scripts emit one ``AddToMemo "DATA <well>=<value>"`` per
    well after ``R_GetData``, and every script command is echoed into the Run
    Log. Parsing that log is currently the only route out of the instrument:
    measurements are stored in ``MeasurementData.abs`` (a proprietary database)
    rather than per-run export files.

    The log is CP949-encoded on this Korean-locale host.
    """
    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"run log not found: {p}")
    text = p.read_bytes().decode(encoding, errors="replace")
    out: dict[str, float] = {}
    for line in text.splitlines():
        if marker not in line:
            continue
        _, _, payload = line.partition(marker)
        payload = payload.strip().strip('"').strip()
        if "=" not in payload:
            continue
        well, _, value = payload.partition("=")
        try:
            out[well.strip().upper()] = float(value.strip())
        except ValueError:
            continue
    return out


# --------------------------------------------------------------------------
# Report writing
# --------------------------------------------------------------------------

def _to_yaml(obj, indent: int = 0) -> str:
    """Minimal YAML emitter.

    Hand-rolled because PyYAML is not installed in the lab's ``main`` env and
    the repo already parses its own YAML subset. Handles the dict/list/scalar
    shapes this module produces — not a general serialiser.
    """
    pad = "  " * indent
    if isinstance(obj, dict):
        if not obj:
            return pad + "{}\n"
        out = ""
        for k, v in obj.items():
            if isinstance(v, (dict, list)) and v:
                out += f"{pad}{k}:\n{_to_yaml(v, indent + 1)}"
            else:
                out += f"{pad}{k}: {_scalar(v)}\n"
        return out
    if isinstance(obj, list):
        if not obj:
            return pad + "[]\n"
        out = ""
        for item in obj:
            if isinstance(item, dict):
                body = _to_yaml(item, indent + 1)
                first, *rest = body.splitlines()
                out += f"{pad}- {first.strip()}\n"
                for line in rest:
                    out += line + "\n"
            else:
                out += f"{pad}- {_scalar(item)}\n"
        return out
    return pad + _scalar(obj) + "\n"


def _scalar(v) -> str:
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return str(v)
    s = str(v)
    return f'"{s}"' if (s == "" or any(c in s for c in ":#{}[],&*?|-<>=!%@`")) else s


def write_report(
    report: SaturationReport,
    path: str | Path,
    fmt: str = "json",
) -> Path:
    """Write the report to disk as JSON or YAML. Returns the path written."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    data = report.as_dict()
    if fmt == "json":
        p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    elif fmt in ("yaml", "yml"):
        p.write_text(_to_yaml(data), encoding="utf-8")
    else:
        raise ValueError(f"unknown format {fmt!r} (use 'json' or 'yaml')")
    return p


def write_reports(
    report: SaturationReport,
    directory: str | Path,
    stem: str | None = None,
) -> dict[str, Path]:
    """Write both JSON and YAML alongside each other, timestamped."""
    d = Path(directory)
    stem = stem or (
        f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
        f"_{report.assay or 'read'}"
    )
    return {
        "json": write_report(report, d / f"{stem}.json", "json"),
        "yaml": write_report(report, d / f"{stem}.yaml", "yaml"),
    }


# --------------------------------------------------------------------------
# MARS CSV export — the working data channel
# --------------------------------------------------------------------------

#: Where MARS automatic mode drops its per-run CSV.
REPORT_DIR = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/User/Report"
)


def parse_mars_csv(path: str | Path) -> dict:
    """Parse a MARS automatic-mode CSV export into metadata + per-well values.

    This is the route that actually gets numbers off the instrument. The binary
    ``MeasurementData.abs`` is a ComponentAce Absolute Database (paged and
    compressed, no public reader), so rather than fight the format we let MARS
    export each run — it runs headless in automatic mode via
    ``MARS.exe SP <user> "<userdir>" -3``.

    Real header, for reference::

        User: USER,Path: ...\\User\\Data,Test run no.: 14
        Test name: Protein,Date: 2026-07-29,Time:  5:04:33

        Absorbance spectrum, Absorbance values are pathlength corrected ...

        Well,Content,Protein concentration in g/l,
        A01,Sample X1,0.094791663,

    The value column is named by the protocol, so it is read from the header
    rather than assumed — ``Protein concentration in g/l`` here, something else
    for Bradford or an ELISA. Wells are normalised ``A01`` -> ``A1`` to match
    :class:`~.config.WellSelection`.

    Returns ``{meta, column, unit, values, wells, path}``.
    """
    p = Path(path)
    text = p.read_bytes().decode("cp949", errors="replace")
    lines = [row.rstrip() for row in text.splitlines()]

    meta: dict[str, str] = {}
    for line in lines[:4]:
        for cell in line.split(","):
            if ":" in cell:
                k, _, v = cell.partition(":")
                meta[k.strip()] = v.strip()

    header_idx = next(
        (i for i, row in enumerate(lines) if row.lower().startswith("well,")), None
    )
    if header_idx is None:
        raise ValueError(f"no 'Well,' header row in {p.name} — not a MARS export?")

    cols = [c.strip() for c in lines[header_idx].split(",")]
    value_col = cols[2] if len(cols) > 2 else "value"
    unit = ""
    if " in " in value_col:
        unit = value_col.rsplit(" in ", 1)[-1].strip()

    values: dict[str, float] = {}
    contents: dict[str, str] = {}
    unreadable: dict[str, str] = {}
    for line in lines[header_idx + 1:]:
        parts = [c.strip() for c in line.split(",")]
        if len(parts) < 3 or not parts[0]:
            continue
        well = parts[0].upper()
        # A01 -> A1 so it matches WellSelection / PlateSpec addressing
        if len(well) >= 3 and well[1] == "0":
            well = well[0] + well[2:]
        contents[well] = parts[1]
        try:
            values[well] = float(parts[2])
        except ValueError:
            # The instrument writes 'n.a.' when it could not compute a well.
            # Record it rather than dropping the row: a silently missing well
            # is indistinguishable from one that was never measured, and this
            # layer must report facts, not quietly shrink the dataset.
            unreadable[well] = parts[2]

    return {
        "meta": meta,
        "column": value_col,
        "unit": unit,
        "values": values,
        "contents": contents,
        "unreadable": unreadable,
        "wells": sorted(values),
        "wells_reported": sorted(set(values) | set(unreadable)),
        "path": str(p),
    }


def latest_mars_csv(report_dir: str | Path = REPORT_DIR,
                    protocol: str | None = None) -> Path | None:
    """Newest MARS export, optionally filtered to one protocol name."""
    d = Path(report_dir)
    if not d.is_dir():
        return None
    files = [f for f in d.glob("*.CSV")] + [f for f in d.glob("*.csv")]
    if protocol:
        files = [f for f in files if protocol.lower() in f.name.lower()]
    return max(files, key=lambda f: f.stat().st_mtime, default=None)
