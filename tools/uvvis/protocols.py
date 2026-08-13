"""The control software's built-in measurement protocols.

``R_Run`` / the ActiveX ``Run`` command select a protocol **by name**, so a valid
name is the one thing without which nothing measures. These were recovered by
parsing ``User/Definit/Testdef.DB`` (a Paradox table — the names and their
descriptions are stored as plain text records inside it), because there is no
API that lists them without script mode.

Treat this as a *snapshot of what shipped*, not a live registry: a user can add,
rename, or delete protocols in the control software's UI at any time. Call
:func:`verify` before relying on a name, and
:func:`~.script.build_list_protocols_script` for an authoritative live list once
script mode works.

For the droplet-crystallization SDL the protocol that matters is
:data:`PROTEIN` — a 220–360 nm absorbance spectrum, which contains both the
280 nm quantitation peak and the 320 nm scatter reference in a single read.
"""
from __future__ import annotations

from dataclasses import dataclass

# Recovered from Testdef.DB on 2026-07-29.
PROTEIN = "Protein"
BRADFORD = "Bradford"
LOWRY = "Lowry"
DSDNA = "dsDNA"
SSDNA = "ssDNA"
ELISA_405 = "Elisa 405"
ELISA_450 = "Elisa 450"
ELISA_492 = "Elisa 492"
NADH = "NADH"
MTT = "MTT toxicity"
LVIS_HOLMIUM = "LVis Holmium filter"
LVIS_ND = "LVis ND filters"


@dataclass(frozen=True)
class ProtocolInfo:
    """What a shipped protocol measures.

    ``wavelengths_nm`` lists the wavelengths in the order the protocol records
    them — the index into this list (1-based) is what ``R_GetData``'s
    *wavelength number* argument selects. For spectrum protocols the list gives
    the scan bounds rather than discrete points.
    """

    name: str
    description: str
    wavelengths_nm: tuple[int, ...] = ()
    spectrum: tuple[int, int] | None = None      # (start, end) for scans
    kind: str = "endpoint"                        # endpoint | kinetic | spectrum
    notes: str = ""


CATALOGUE: dict[str, ProtocolInfo] = {
    PROTEIN: ProtocolInfo(
        name=PROTEIN,
        description="Absorbance spectrum of a protein sample, 220-360 nm.",
        spectrum=(220, 360), kind="spectrum",
        notes="The SDL workhorse: one read yields A280 for quantitation and "
              "A320 for the scatter/turbidity correction.",
    ),
    BRADFORD: ProtocolInfo(
        name=BRADFORD, wavelengths_nm=(595,),
        description="Protein dyed with Coomassie blue, measured at 595 nm.",
        notes="Needs a standard curve. Detergent-sensitive.",
    ),
    LOWRY: ProtocolInfo(
        name=LOWRY,
        description="Protein via Folin-Ciocalteu reagent.",
        notes="Needs a standard curve.",
    ),
    DSDNA: ProtocolInfo(
        name=DSDNA, spectrum=(220, 360), kind="spectrum",
        description="Double-stranded DNA; spectrum 220-360 nm, corrected "
                    "260/280 ratio, concentration by extinction coefficient.",
    ),
    SSDNA: ProtocolInfo(
        name=SSDNA, spectrum=(220, 360), kind="spectrum",
        description="Single-stranded DNA; spectrum 220-360 nm.",
    ),
    ELISA_405: ProtocolInfo(
        name=ELISA_405, wavelengths_nm=(405, 630),
        description="ABTS endpoint; 405 nm signal against a 630 nm reference.",
    ),
    ELISA_450: ProtocolInfo(
        name=ELISA_450, wavelengths_nm=(450, 630),
        description="TMB endpoint; 450 nm signal against a 630 nm reference.",
    ),
    ELISA_492: ProtocolInfo(
        name=ELISA_492, wavelengths_nm=(492, 630),
        description="OPD endpoint; 492 nm signal against a 630 nm reference.",
    ),
    NADH: ProtocolInfo(
        name=NADH, wavelengths_nm=(340,), kind="kinetic",
        description="NADH to NAD+ conversion at 340 nm, by extinction coefficient.",
    ),
    MTT: ProtocolInfo(
        name=MTT, wavelengths_nm=(570,),
        description="Cell death via formazan formation from MTT, ~570 nm.",
    ),
    LVIS_HOLMIUM: ProtocolInfo(
        name=LVIS_HOLMIUM, kind="spectrum",
        description="Wavelength-accuracy check using the LVis Plate.",
        notes="Instrument QC, not a sample assay.",
    ),
    LVIS_ND: ProtocolInfo(
        name=LVIS_ND,
        description="Absorbance-accuracy check using the LVis Plate.",
        notes="Instrument QC, not a sample assay.",
    ),
}

#: Protocols that answer "how much protein is in this well?".
PROTEIN_PROTOCOLS = (PROTEIN, BRADFORD, LOWRY)

#: Protocols that are instrument QC rather than sample measurement.
QC_PROTOCOLS = (LVIS_HOLMIUM, LVIS_ND)


def describe(name: str) -> ProtocolInfo | None:
    """Look up a protocol, case-insensitively."""
    if name in CATALOGUE:
        return CATALOGUE[name]
    lowered = name.strip().lower()
    for key, info in CATALOGUE.items():
        if key.lower() == lowered:
            return info
    return None


def wavelength_index(name: str, wavelength_nm: int) -> int | None:
    """1-based index of a wavelength within a protocol's recorded set.

    This is the *wavelength number* argument of ``R_GetData``. Returns ``None``
    for spectrum protocols, where the index maps to a scan point rather than a
    named wavelength and must be derived from the scan's step size.
    """
    info = describe(name)
    if info is None or not info.wavelengths_nm:
        return None
    try:
        return info.wavelengths_nm.index(wavelength_nm) + 1
    except ValueError:
        return None


def verify(names: list[str] | None = None) -> dict[str, bool]:
    """Check catalogue names against a live list from the instrument.

    Pass the names parsed out of a ``R_GetProtocolNames`` run (see
    :func:`~.script.build_list_protocols_script`). With no argument this cannot
    verify anything and says so, rather than implying the catalogue is live.
    """
    if names is None:
        return {k: False for k in CATALOGUE}
    live = {n.strip().lower() for n in names}
    return {k: (k.lower() in live) for k in CATALOGUE}
