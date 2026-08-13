"""Vendor plate geometry library — SPECTROstar Nano plate definitions.

Source: 73 plate definitions recovered from the control software's own plate
library (``Stamm/Platedef.DB``) and shipped beside this module as
``plate_library.json``. Read it, do not regenerate it (fact 7).

**On maximum plate height.** BMG do not publish a maximum accepted plate
height anywhere in the vendor documentation. :data:`TALLEST_DEFINED_MM` is the
tallest plate the vendor *ships a definition for* (20.2 mm) — that is an
empirical **lower bound** on what the drawer will accept, not a clearance
limit. A robot integrator sizing arm/gripper clearance around this instrument
must not treat this constant as a ceiling; it only proves the drawer accepts
plates at least this tall. Use :func:`clearance_report` per-plate geometry
instead of a single global number.
"""
from __future__ import annotations

import difflib
import functools
import json
from dataclasses import dataclass
from pathlib import Path

from .config import PlateSpec

_LIBRARY_PATH = Path(__file__).with_name("plate_library.json")


@dataclass(frozen=True)
class PlateDefinition:
    """One vendor plate's physical geometry, as recorded in the plate library.

    Not every field is populated for every plate: several of the 73 entries
    (e.g. ``BMG LVis Cuvette``, ``SARSTEDT 96``, the 1536-well formats) ship
    with ``null`` height/depth/shape fields in the vendor's own database, and
    ``SBS STANDARD 1536`` carries a clearly-sentinel ``well_diameter_mm`` of
    ``-327.68``. These are recorded verbatim rather than guessed at — do not
    backfill a missing value with an inferred number.
    """

    name: str
    columns: int
    rows: int
    well_count: int
    length_mm: float | None
    width_mm: float | None
    corner_well_x_mm: float | None
    corner_well_y_mm: float | None
    corner_well_xn_mm: float | None
    corner_well_yn_mm: float | None
    plate_height_mm: float | None
    well_depth_mm: float | None
    well_shape: str | None
    well_diameter_mm: float | None
    border_height_mm: float | None
    plate_to_well_bottom_mm: float | None
    stack_height_mm: float | None

    @property
    def row_labels(self) -> list[str]:
        return self.as_plate_spec().row_labels

    def all_wells(self, by: str = "row") -> list[str]:
        return self.as_plate_spec().all_wells(by=by)

    def as_plate_spec(self) -> PlateSpec:
        """This definition's rows/columns/name as a plain :class:`PlateSpec`."""
        return PlateSpec(rows=self.rows, columns=self.columns, name=self.name,
                         definition=self)


@functools.lru_cache(maxsize=1)
def load_library() -> dict[str, PlateDefinition]:
    """All 73 vendor plate definitions, keyed by their exact library name."""
    data = json.loads(_LIBRARY_PATH.read_text())
    out: dict[str, PlateDefinition] = {}
    for raw in data["plates"]:
        d = PlateDefinition(**raw)
        out[d.name] = d
    return out


def _norm(name: str) -> str:
    return " ".join(name.strip().split()).lower()


def find(name: str) -> PlateDefinition:
    """Look up a plate by name — case-insensitive, tolerant of extra whitespace.

    Raises :class:`KeyError` naming near matches (substring, then fuzzy) when
    there is no exact hit, so a typo does not read as "plate does not exist".
    """
    lib = load_library()
    target = _norm(name)
    for d in lib.values():
        if _norm(d.name) == target:
            return d

    near = [d.name for d in lib.values()
            if target in _norm(d.name) or _norm(d.name) in target]
    if not near:
        near = difflib.get_close_matches(name, list(lib.keys()), n=5, cutoff=0.4)

    hint = f" Near matches: {near}" if near else " No near matches found in the library."
    raise KeyError(f"no plate definition named {name!r}.{hint}")


def search(
    *,
    well_count: int | None = None,
    min_height_mm: float | None = None,
    max_height_mm: float | None = None,
    shape: str | None = None,
) -> list[PlateDefinition]:
    """Filter the library. All filters are ANDed; omit any to skip it.

    A plate whose ``plate_height_mm``/``well_shape`` is ``null`` in the vendor
    data (see :class:`PlateDefinition`) never matches a height or shape filter
    — there is nothing to compare, so it is excluded rather than guessed into
    either bucket.
    """
    out: list[PlateDefinition] = []
    for d in load_library().values():
        if well_count is not None and d.well_count != well_count:
            continue
        if min_height_mm is not None and (
            d.plate_height_mm is None or d.plate_height_mm < min_height_mm
        ):
            continue
        if max_height_mm is not None and (
            d.plate_height_mm is None or d.plate_height_mm > max_height_mm
        ):
            continue
        if shape is not None and (
            d.well_shape is None or d.well_shape.lower() != shape.lower()
        ):
            continue
        out.append(d)
    return out


#: Tallest plate the vendor ships a *definition* for — see the module
#: docstring: this is an empirical lower bound on drawer clearance, not a
#: maximum. BMG publish no maximum plate height. Computed only over entries
#: that actually carry a ``plate_height_mm`` (11 of the 73 do not — see
#: :class:`PlateDefinition`).
TALLEST_DEFINED_MM: float = max(
    d.plate_height_mm for d in load_library().values() if d.plate_height_mm is not None
)


def clearance_report(plate: PlateDefinition) -> dict:
    """Geometry an arm-integration layer needs to plan clearance around this plate.

    Returns raw facts only — headroom is computed against
    :data:`TALLEST_DEFINED_MM`, which itself is only a lower bound (see module
    docstring). This function does not decide whether a plate fits; it reports.
    ``headroom_vs_tallest_defined_mm`` is ``None`` when this plate has no
    recorded ``plate_height_mm`` (vendor data gap, not inferred).
    """
    return {
        "name": plate.name,
        "plate_height_mm": plate.plate_height_mm,
        "stack_height_mm": plate.stack_height_mm,
        "border_height_mm": plate.border_height_mm,
        "plate_to_well_bottom_mm": plate.plate_to_well_bottom_mm,
        "headroom_vs_tallest_defined_mm": (
            (TALLEST_DEFINED_MM - plate.plate_height_mm)
            if plate.plate_height_mm is not None else None
        ),
    }
