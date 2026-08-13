"""One-call workflows — the plug-and-play surface.

These compose the lower layers (reader → measurement → retrieval → analysis →
report files) into single functions an orchestrator can call without knowing how
the instrument works.

Consistent with the surface's *Sensors report, they do not decide* law, every
function here returns **facts plus an advisory**. Nothing retries, re-runs,
dilutes, or discards a reading on its own judgement — the orchestrator decides
what to do with `action`.

    from tools.nodes.uv_vis import UvVisConfig, measure_protein

    result = measure_protein(UvVisConfig.from_dict(cfg["uv_vis"]),
                             extinction_coefficient=38940,
                             molecular_weight=14300)
    result["orchestrator"]      # -> {'ok': ..., 'action': ..., ...}
    result["reports"]["yaml"]   # -> path to the YAML record
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from . import protocols
from .analysis import (
    SaturationReport,
    analyse_absorbances,
    parse_run_log_data,
    write_reports,
)
from .assays import StandardCurve
from .config import UvVisConfig, WellSelection
from .layout import LayoutPlan
from .reader import SpectroStarNano

RUN_LOG = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/SPECTROstar Nano.log"
)


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def read_plate(
    config: UvVisConfig,
    protocol: str = protocols.PROTEIN,
    *,
    wells: WellSelection | list[str] | None = None,
    plate_id: str = "",
    require_interlock: bool | None = None,
) -> dict:
    """Run one protocol and return the raw absorbances. No interpretation.

    This is the pure sensing step. It returns what the instrument measured plus
    provenance; it makes no judgement about whether the numbers are good.

    Raises :class:`ReaderError` if the control software logged no
    ``(Run command)`` — a returned success is not proof on this interface.
    """
    reader = SpectroStarNano.from_config(config)
    if require_interlock is not None:
        reader.require_interlock = require_interlock

    reader.require_present()
    run = reader.run_protocol(protocol, plate_id1=plate_id)

    absorbances = parse_run_log_data(RUN_LOG)
    selected = list(wells) if wells is not None else (
        list(config.wells) if config.wells else []
    )
    if selected:
        absorbances = {w: v for w, v in absorbances.items() if w in set(selected)}

    return {
        "protocol": protocol,
        "protocol_info": (protocols.describe(protocol).description
                          if protocols.describe(protocol) else ""),
        "plate_id": plate_id,
        "run": run,
        "absorbances": absorbances,
        "n_values": len(absorbances),
        "measured_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }


def analyse_plate(
    absorbances: dict[str, float],
    *,
    assay: str = "A280",
    wavelength_nm: int = 280,
    protocol: str = "",
    extinction_coefficient: float | None = None,
    molecular_weight: float | None = None,
    standard_concentrations: list[float] | None = None,
    standard_absorbances: list[float] | None = None,
    curve_model: str = "linear",
    blank: float | None = None,
    dilution_factor: float = 1.0,
    report_dir: str | Path | None = None,
) -> dict:
    """Turn absorbances into a saturation/concentration report + report files.

    Pure computation — safe to run offline on numbers from any source, which is
    what makes the analysis testable without the instrument.
    """
    curve = None
    if standard_concentrations and standard_absorbances:
        curve = StandardCurve(
            concentrations=standard_concentrations,
            absorbances=standard_absorbances,
            model=curve_model,
        )

    report: SaturationReport = analyse_absorbances(
        absorbances, assay=assay, wavelength_nm=wavelength_nm, protocol=protocol,
        curve=curve, blank=blank, dilution_factor=dilution_factor,
        extinction_coefficient=extinction_coefficient,
        molecular_weight=molecular_weight,
    )

    out = {
        "report": report,
        "orchestrator": report.orchestrator_view(),
        "per_well": report.by_well(),
        "concentrations": report.concentrations(),
        "reliable": report.booleans("reliable"),
        "saturated": report.booleans("saturated"),
        "summary": report.summary,
    }
    if report_dir:
        out["reports"] = {k: str(v) for k, v in
                          write_reports(report, report_dir,
                                        stem=f"{_stamp()}_{assay}").items()}
    return out


def measure_protein(
    config: UvVisConfig,
    *,
    protocol: str = protocols.PROTEIN,
    extinction_coefficient: float | None = None,
    molecular_weight: float | None = None,
    wells: WellSelection | list[str] | None = None,
    plate_id: str = "",
    blank: float | None = None,
    dilution_factor: float = 1.0,
    require_interlock: bool | None = None,
) -> dict:
    """Measure a protein plate end to end and write JSON + YAML records.

    Sequence: run the protocol → recover absorbances from the run log → classify
    saturation → derive concentrations → persist. Returns facts plus an advisory
    ``action``; it never acts on that advice itself.

    Supply ``extinction_coefficient`` (M⁻¹cm⁻¹) and ``molecular_weight`` (Da) for
    a real A280 concentration. Without them the report still gives absorbances
    and reliability, and any concentration falls back to the crude
    1 AU ≈ 1 mg/mL approximation, flagged as approximate.
    """
    read = read_plate(config, protocol, wells=wells, plate_id=plate_id,
                      require_interlock=require_interlock)
    analysed = analyse_plate(
        read["absorbances"], assay="A280", wavelength_nm=280, protocol=protocol,
        extinction_coefficient=extinction_coefficient,
        molecular_weight=molecular_weight, blank=blank,
        dilution_factor=dilution_factor,
        report_dir=Path(config.log_dir) / "reports",
    )
    analysed["measurement"] = {k: v for k, v in read.items() if k != "absorbances"}
    analysed["absorbances"] = read["absorbances"]
    return analysed


def measure_wells(
    config: UvVisConfig,
    plan_or_wells: LayoutPlan | WellSelection | list[str],
    *,
    protocol: str = protocols.PROTEIN,
    mass_extinction_coefficient: float | None = None,
    path_length_cm: float = 1.0,
    assay: str = "A280",
    wavelength_nm: int = 280,
    plate_id: str = "",
    blank: float | None = None,
    dilution_factor: float = 1.0,
    require_interlock: bool | None = None,
) -> dict:
    """Apply a layout (or a plain well list), measure it, and analyse.

    ``plan_or_wells`` is either a :class:`~.layout.LayoutPlan` — imported into
    the control software via :meth:`SpectroStarNano.apply_layout` before the
    run — or a plain well list/:class:`WellSelection`, which skips the
    layout-import step and measures whatever layout is already loaded.

    ``mass_extinction_coefficient`` (mL·mg⁻¹·cm⁻¹) is an alternative to the
    molar ``extinction_coefficient``/``molecular_weight`` pair used by
    :func:`measure_protein`, for assays calibrated by mass rather than molarity:
    ``concentration = A / (mass_extinction_coefficient * path_length_cm)``.
    Reported alongside, not folded into, the :class:`SaturationReport` — see
    ``result["mass_ec_concentrations"]``.

    Same *facts plus advisory* contract as :func:`measure_protein`: this
    function never retries, re-runs, dilutes, or discards a reading on its own
    judgement. The orchestrator decides what to do with ``action``.
    """
    reader = SpectroStarNano.from_config(config)
    if require_interlock is not None:
        reader.require_interlock = require_interlock
    reader.require_present()

    layout_path: Path | None = None
    if isinstance(plan_or_wells, LayoutPlan):
        layout_path = reader.apply_layout(protocol, plan_or_wells)
        selected = plan_or_wells.measured_wells()
    else:
        selected = list(plan_or_wells) if plan_or_wells is not None else []

    run = reader.run_protocol(protocol, plate_id1=plate_id)
    absorbances = parse_run_log_data(RUN_LOG)
    if selected:
        absorbances = {w: v for w, v in absorbances.items() if w in set(selected)}

    analysed = analyse_plate(
        absorbances, assay=assay, wavelength_nm=wavelength_nm, protocol=protocol,
        blank=blank, dilution_factor=dilution_factor,
        report_dir=Path(config.log_dir) / "reports",
    )

    if mass_extinction_coefficient:
        analysed["mass_ec_concentrations"] = {
            w: (v / (mass_extinction_coefficient * path_length_cm)
                if v is not None else None)
            for w, v in absorbances.items()
        }

    analysed["measurement"] = {
        "protocol": protocol,
        "protocol_info": (protocols.describe(protocol).description
                          if protocols.describe(protocol) else ""),
        "plate_id": plate_id,
        "layout_path": str(layout_path) if layout_path else None,
        "run": run,
        "n_values": len(absorbances),
        "measured_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    analysed["absorbances"] = absorbances
    return analysed


def dry_run(config: UvVisConfig, absorbances: dict[str, float], **kwargs) -> dict:
    """Exercise the whole analysis path on supplied numbers, no instrument.

    For validating an orchestration flow, or for re-analysing a past run with
    different calibration, without touching hardware.
    """
    return analyse_plate(
        absorbances, report_dir=Path(config.log_dir) / "reports", **kwargs
    )


def plan_protein_grid(
    config: UvVisConfig,
    samples: int = 5,
    replicates: int = 3,
    *,
    replicates_along: str = "column",
    rows=None,
    columns=None,
    blanks: list[str] | None = None,
) -> LayoutPlan:
    """Build the plate layout for a protein assay from just the grid shape.

    The common case is ``samples`` distinct samples each run ``replicates``
    times. With the default ``replicates_along="column"`` that lays each sample
    along a row: ``plan_protein_grid(cfg, samples=5, replicates=3)`` gives
    A1-A3 as sample 1, B1-B3 as sample 2, ... E1-E3 as sample 5 — 15 wells, and
    the reader measures those 15 only.

    Pass explicit ``rows``/``columns`` (labels or counts) instead to place the
    block somewhere other than the top-left corner, and ``blanks`` for buffer
    wells, which the analysis layer subtracts.

    Returns the plan without touching the instrument, so it can be inspected
    (``plan.measured_wells()``, ``plan.to_lb_text()``) before anything moves.
    """
    if rows is None or columns is None:
        if replicates_along == "column":
            rows, columns = samples, replicates
        elif replicates_along == "row":
            rows, columns = replicates, samples
        else:
            raise ValueError("replicates_along must be 'row' or 'column'")
    plan = LayoutPlan.grid(
        rows, columns, replicates_along=replicates_along, plate=config.plate
    )
    if blanks:
        plan.blanks(blanks)
    return plan


def measure_protein_grid(
    config: UvVisConfig,
    samples: int = 5,
    replicates: int = 3,
    *,
    replicates_along: str = "column",
    rows=None,
    columns=None,
    blanks: list[str] | None = None,
    protocol: str = protocols.PROTEIN,
    mass_extinction_coefficient: float | None = None,
    path_length_cm: float = 1.0,
    plate_id: str = "",
    blank: float | None = None,
    dilution_factor: float = 1.0,
    require_interlock: bool | None = None,
) -> dict:
    """Measure a protein plate given only the grid shape. The one-call entry point.

        result = measure_protein_grid(cfg, samples=5, replicates=3,
                                      mass_extinction_coefficient=eps)

    Builds the layout, imports it into the control software, runs the protocol
    over exactly those wells, and analyses the result. ``result["layout"]``
    carries the plan that was applied, so the report says which wells were
    asked for as well as which were read.

    ``mass_extinction_coefficient`` is the per-run value supplied by the
    orchestrator, in mL/(mg*cm); concentration is ``A / (eps * path_length_cm)``.
    Omit it to get absorbances and reliability flags without concentrations.

    Same *facts plus advisory* contract as the rest of this module: it reports,
    it does not decide. No retry, no re-run, no discarded readings.
    """
    plan = plan_protein_grid(
        config, samples, replicates, replicates_along=replicates_along,
        rows=rows, columns=columns, blanks=blanks,
    )
    result = measure_wells(
        config, plan, protocol=protocol,
        mass_extinction_coefficient=mass_extinction_coefficient,
        path_length_cm=path_length_cm, plate_id=plate_id, blank=blank,
        dilution_factor=dilution_factor, require_interlock=require_interlock,
    )
    result["layout"] = {
        "requested_wells": plan.measured_wells(),
        "n_requested": len(plan.measured_wells()),
        "lb_text": plan.to_lb_text(),
        "samples": samples,
        "replicates": replicates,
    }
    return result
