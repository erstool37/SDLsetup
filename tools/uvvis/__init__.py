"""UV-Vis absorbance reader — BMG LABTECH SPECTROstar Nano.

Layers, lowest to highest:

``dde``       control transport: BMG's DDEClient.exe, run-log verified
``config``    PlateSpec / WellSelection / Interlock / UvVisConfig
``reader``    SpectroStarNano — the device API SDL workflows should use
``assays``    quantitation maths + assay drivers (BCA, Bradford, A280, turbidity)
``node``      Lab node wrapper, for the dashboard and the node bus

Typical orchestration::

    from tools.nodes.uv_vis import UvVisConfig, SpectroStarNano, A280Assay

    reader = SpectroStarNano.from_config(UvVisConfig.from_dict(cfg["uv_vis"]))
    reader.interlock.set_arm_clear(True)
    with reader.open_carrier():          # opens, and always closes again
        ...                              # arm places the plate
    reader.interlock.set_plate_loaded(True)
    if reader.interlock.ready_to_measure:
        reader.run_protocol("A280 scan")
"""
from . import dde, protocols
from .analysis import (
    DETECTION_FLOOR,
    LINEAR_CEILING,
    REPORT_DIR,
    WARN_CEILING,
    SaturationReport,
    WellResult,
    analyse_absorbances,
    classify_absorbance,
    latest_mars_csv,
    parse_mars_csv,
    parse_run_log_data,
    write_report,
    write_reports,
)
from .assays import (
    A280_NM,
    BCA_NM,
    BRADFORD_NM,
    SCATTER_CORRECTION_NM,
    A280Assay,
    AssayError,
    BCAAssay,
    BradfordAssay,
    ProteinAssay,
    StandardCurve,
    TurbidityAssay,
    a280_concentration,
)
from .config import (
    BIT_ARM_CLEAR,
    BIT_BUSY,
    BIT_CARRIER_IN,
    BIT_CARRIER_OUT,
    BIT_ERROR,
    BIT_PLATE_LOADED,
    BIT_READER_PRESENT,
    BIT_READY,
    AssayConfig,
    Incubation,
    Interlock,
    PlateSpec,
    Shaking,
    UvVisConfig,
    WellSelection,
)
from .dde import (
    DdeError,
    DdeResult,
    control_running,
    ensure_control_running,
    wells_measured,
)
from .dde import (
    plate_in as dde_plate_in,
)
from .dde import (
    plate_out as dde_plate_out,
)
from .dde import (
    run_protocol as dde_run_protocol,
)
from .dde import (
    send as dde_send,
)
from .layout import LayoutPlan, WellContent, windows_path
from .node import UvVisNode
from .plates import (
    TALLEST_DEFINED_MM,
    PlateDefinition,
    clearance_report,
    load_library,
)
from .plates import (
    find as find_plate,
)
from .plates import (
    search as search_plates,
)
from .protocols import (
    BRADFORD,
    CATALOGUE,
    DSDNA,
    ELISA_405,
    ELISA_450,
    ELISA_492,
    LOWRY,
    MTT,
    NADH,
    PROTEIN,
    PROTEIN_PROTOCOLS,
    QC_PROTOCOLS,
    SSDNA,
    ProtocolInfo,
)
from .protocols import (
    describe as describe_protocol,
)
from .reader import CarrierState, MotionNotAllowed, ReaderError, SpectroStarNano
from .script import (
    ScriptBuilder,
    ScriptError,
    build_assay_script,
    build_list_protocols_script,
    concentration_actions,
    layout_actions,
    run_script,
    wait_for_marker,
)
from .workflows import (
    analyse_plate,
    dry_run,
    measure_protein,
    measure_protein_grid,
    measure_wells,
    plan_protein_grid,
    read_plate,
)

__all__ = [
    "UvVisNode",
    "SpectroStarNano", "ReaderError", "MotionNotAllowed", "CarrierState",
    "PlateSpec", "WellSelection", "Interlock", "UvVisConfig", "AssayConfig",
    "Incubation", "Shaking",
    "BIT_READER_PRESENT", "BIT_CARRIER_OUT", "BIT_CARRIER_IN",
    "BIT_PLATE_LOADED", "BIT_ARM_CLEAR", "BIT_READY", "BIT_BUSY", "BIT_ERROR",
    "StandardCurve", "a280_concentration", "AssayError",
    "ProteinAssay", "BCAAssay", "BradfordAssay", "A280Assay", "TurbidityAssay",
    "BCA_NM", "BRADFORD_NM", "A280_NM", "SCATTER_CORRECTION_NM",
    "SaturationReport", "WellResult", "analyse_absorbances",
    "classify_absorbance", "parse_run_log_data", "parse_mars_csv", "latest_mars_csv",
    "write_report", "write_reports", "REPORT_DIR",
    "LINEAR_CEILING", "WARN_CEILING", "DETECTION_FLOOR",
    "ScriptBuilder", "ScriptError", "build_assay_script",
    "build_list_protocols_script", "layout_actions", "concentration_actions",
    "run_script", "wait_for_marker",
    "dde", "DdeError", "DdeResult", "dde_send", "dde_run_protocol",
    "dde_plate_in", "dde_plate_out", "wells_measured",
    "ensure_control_running", "control_running",
    "protocols", "CATALOGUE", "ProtocolInfo", "describe_protocol",
    "PROTEIN", "BRADFORD", "LOWRY", "DSDNA", "SSDNA",
    "ELISA_405", "ELISA_450", "ELISA_492", "NADH", "MTT",
    "PROTEIN_PROTOCOLS", "QC_PROTOCOLS",
    "read_plate", "analyse_plate", "measure_protein", "measure_wells", "dry_run",
    "plan_protein_grid", "measure_protein_grid",
    "PlateDefinition", "load_library", "find_plate", "search_plates",
    "TALLEST_DEFINED_MM", "clearance_report",
    "WellContent", "LayoutPlan", "windows_path",
]
