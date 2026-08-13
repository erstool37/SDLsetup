"""Protein assay workflows for the SPECTROstar Nano.

Two layers, deliberately separated:

* **Quantitation maths** (:class:`StandardCurve`, :func:`a280_concentration`) is
  pure Python with no instrument dependency, so it is unit-testable offline and
  usable on absorbance numbers from any source.
* **Assay drivers** (:class:`BCAAssay`, :class:`BradfordAssay`, :class:`A280Assay`)
  bind that maths to a :class:`~.reader.SpectroStarNano` protocol run.

Assay *protocols* live in the control software, not here — this module selects
one by name and interprets the absorbances it returns. The reference wavelengths
below are the standard chemistry, not instrument configuration:

===========  ========  ==========================================
Assay        λ (nm)    Working range (BSA equivalent)
===========  ========  ==========================================
BCA          562       20 – 2000 µg/mL
Bradford     595       125 – 1500 µg/mL (standard), 1 – 25 (micro)
A280         280       0.1 – 3 AU, needs extinction coefficient
===========  ========  ==========================================

Because the reader is a full-spectrum CCD instrument (Hamamatsu S11071, to
1050 nm), a protocol can capture the whole spectrum in one read; a single
wavelength is just one column of that.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

# Standard analytical wavelengths, nm.
BCA_NM = 562
BRADFORD_NM = 595
A280_NM = 280
SCATTER_CORRECTION_NM = 320   # baseline/turbidity reference for A280


class AssayError(RuntimeError):
    """An assay could not be computed or executed."""


# --------------------------------------------------------------------------
# Quantitation maths — no instrument involved
# --------------------------------------------------------------------------


@dataclass
class StandardCurve:
    """Least-squares fit of absorbance against known concentration.

    ``linear`` fits A = m·C + b, appropriate for Bradford and for BCA over a
    narrow range. ``quadratic`` fits A = a·C² + b·C + c, which is what BCA
    actually needs across its full 20–2000 µg/mL range, where the response
    curves noticeably.

    Implemented with plain arithmetic rather than numpy: this package is
    imported on a lab machine whose environment we do not control, and the
    fits are tiny.
    """

    concentrations: Sequence[float]
    absorbances: Sequence[float]
    model: str = "linear"
    coeffs: tuple[float, ...] = field(default_factory=tuple)
    r_squared: float = 0.0

    def __post_init__(self) -> None:
        if len(self.concentrations) != len(self.absorbances):
            raise AssayError("concentrations and absorbances differ in length")
        n = len(self.concentrations)
        need = 3 if self.model == "quadratic" else 2
        if n < need:
            raise AssayError(
                f"{self.model} fit needs at least {need} standards, got {n}"
            )
        if self.model == "linear":
            self.coeffs = self._fit_linear()
        elif self.model == "quadratic":
            self.coeffs = self._fit_quadratic()
        else:
            raise AssayError(f"unknown model {self.model!r}")
        self.r_squared = self._r_squared()

    # -- fitting ----------------------------------------------------------

    def _fit_linear(self) -> tuple[float, float]:
        xs, ys = self.concentrations, self.absorbances
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx == 0:
            raise AssayError("all standards have the same concentration")
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=False))
        slope = sxy / sxx
        return (slope, my - slope * mx)          # (m, b)

    def _fit_quadratic(self) -> tuple[float, float, float]:
        """Solve the 3x3 normal equations for A = a·C² + b·C + c."""
        xs, ys = self.concentrations, self.absorbances
        n = float(len(xs))
        s1 = sum(xs)
        s2 = sum(x**2 for x in xs)
        s3 = sum(x**3 for x in xs)
        s4 = sum(x**4 for x in xs)
        t0 = sum(ys)
        t1 = sum(x*y for x, y in zip(xs, ys, strict=False))
        t2 = sum(x*x*y for x, y in zip(xs, ys, strict=False))
        m = [[s4, s3, s2, t2], [s3, s2, s1, t1], [s2, s1, n, t0]]
        # Gaussian elimination with partial pivoting.
        for col in range(3):
            piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
            if abs(m[piv][col]) < 1e-12:
                raise AssayError("standards are degenerate; cannot fit quadratic")
            m[col], m[piv] = m[piv], m[col]
            for r in range(3):
                if r == col:
                    continue
                f = m[r][col] / m[col][col]
                for c in range(col, 4):
                    m[r][c] -= f * m[col][c]
        return tuple(m[i][3] / m[i][i] for i in range(3))    # (a, b, c)

    def _r_squared(self) -> float:
        my = sum(self.absorbances) / len(self.absorbances)
        ss_tot = sum((y - my) ** 2 for y in self.absorbances)
        ss_res = sum(
            (y - self.predict_absorbance(x)) ** 2
            for x, y in zip(self.concentrations, self.absorbances, strict=False)
        )
        return 1.0 - ss_res / ss_tot if ss_tot else 1.0

    # -- use --------------------------------------------------------------

    def predict_absorbance(self, concentration: float) -> float:
        if self.model == "linear":
            m, b = self.coeffs
            return m * concentration + b
        a, b, c = self.coeffs
        return a * concentration**2 + b * concentration + c

    def concentration(self, absorbance: float) -> float:
        """Invert the curve: absorbance → concentration.

        For the quadratic model this takes the physically meaningful root (the
        ascending branch); an absorbance above the curve's turning point has no
        valid solution and raises.
        """
        if self.model == "linear":
            m, b = self.coeffs
            if m == 0:
                raise AssayError("degenerate curve: zero slope")
            return (absorbance - b) / m
        a, b, c = self.coeffs
        if abs(a) < 1e-15:
            if b == 0:
                raise AssayError("degenerate curve")
            return (absorbance - c) / b
        disc = b**2 - 4 * a * (c - absorbance)
        if disc < 0:
            raise AssayError(
                f"absorbance {absorbance:.4f} lies outside the standard curve"
            )
        root = math.sqrt(disc)
        r1, r2 = (-b + root) / (2 * a), (-b - root) / (2 * a)
        valid = [r for r in (r1, r2) if r >= 0]
        if not valid:
            raise AssayError(
                f"absorbance {absorbance:.4f} gives no non-negative concentration"
            )
        return min(valid)

    def in_range(self, absorbance: float) -> bool:
        lo, hi = min(self.absorbances), max(self.absorbances)
        return lo <= absorbance <= hi


def a280_concentration(
    a280: float,
    extinction_coefficient: float | None = None,
    molecular_weight: float | None = None,
    path_length_cm: float = 1.0,
    a320: float | None = None,
    mass_extinction_coefficient: float | None = None,
) -> float:
    """Direct A280 protein quantitation via Beer–Lambert. Returns mg/mL.

    Three ways to specify the protein's absorptivity, in precedence order:

    1. ``mass_extinction_coefficient`` — the **mass** (specific) coefficient in
       µL/(µg·cm), which is dimensionally L/(g·cm). This is what the BMG
       instrument prompts for, and what an orchestrator will normally supply
       per run::

           c [g/L] = A / (ε_mass · l)

       Since 1 g/L == 1 mg/mL, the result is directly mg/mL. For reference only,
       lysozyme is ≈2.64 L/(g·cm) — verify against your own protein and buffer;
       do not treat that number as authoritative.
    2. ``extinction_coefficient`` (molar, M⁻¹cm⁻¹) **with**
       ``molecular_weight`` (Da)::

           c [g/L] = (A / (ε_molar · l)) · MW

    3. Neither — falls back to the crude 1 AU ≈ 1 mg/mL approximation. Wrong by
       severalfold for proteins with unusual aromatic content, so callers are
       told about it via the ``approximate`` flag rather than it passing silently.

    ``a320`` subtracts a scatter/turbidity baseline — worth using for anything
    that has been through lysis, freeze–thaw, or is near its solubility limit.

    The coefficient is never stored as instrument state: it is passed in per
    call so that whichever value produced a number is recorded alongside it.
    """
    absorbance = a280 - a320 if a320 is not None else a280
    if path_length_cm <= 0:
        raise AssayError("path length must be positive")

    if mass_extinction_coefficient is not None:
        if mass_extinction_coefficient <= 0:
            raise AssayError("mass extinction coefficient must be positive")
        return absorbance / (mass_extinction_coefficient * path_length_cm)

    absorbance /= path_length_cm
    if extinction_coefficient and molecular_weight:
        if extinction_coefficient <= 0:
            raise AssayError("extinction coefficient must be positive")
        molar = absorbance / extinction_coefficient      # mol/L
        return molar * molecular_weight                  # g/L == mg/mL
    return absorbance                                    # 1 AU ~ 1 mg/mL


# --------------------------------------------------------------------------
# Assay drivers — bind the maths to an instrument protocol
# --------------------------------------------------------------------------


@dataclass
class ProteinAssay:
    """Base class binding a control-software protocol to a quantitation model.

    Running an assay is two steps that are deliberately kept separate:

    1. :meth:`measure` drives the reader and returns once the protocol finishes.
    2. :meth:`quantify` turns absorbances into concentrations.

    They are separate because result extraction is not yet solved on this
    instrument: measurements land in the control software's
    ``MeasurementData.abs`` database rather than in per-run export files, so
    absorbances must currently be exported from the UI and handed to
    :meth:`quantify`. Keeping the maths independent means the assay layer is
    usable today regardless.
    """

    reader: object                    # SpectroStarNano; untyped to avoid a cycle
    protocol: str
    wavelength_nm: int = BCA_NM
    model: str = "linear"
    name: str = "protein"

    def measure(self, plate_id: str = "", settle_s: float = 2.0) -> dict:
        """Run the assay protocol on whatever plate is currently loaded."""
        if not self.protocol:
            raise AssayError(
                f"{self.name}: no protocol name set — author one in the "
                "SPECTROstar Nano control software and pass its name here"
            )
        return self.reader.run_protocol(
            self.protocol, plate_id1=plate_id, settle_s=settle_s
        )

    def build_curve(
        self,
        standard_concentrations: Sequence[float],
        standard_absorbances: Sequence[float],
        blank_absorbance: float | None = None,
    ) -> StandardCurve:
        """Fit a standard curve, optionally blank-subtracting first."""
        abs_vals = list(standard_absorbances)
        if blank_absorbance is not None:
            abs_vals = [a - blank_absorbance for a in abs_vals]
        return StandardCurve(
            concentrations=list(standard_concentrations),
            absorbances=abs_vals,
            model=self.model,
        )

    def quantify(
        self,
        curve: StandardCurve,
        sample_absorbances: dict[str, float],
        blank_absorbance: float | None = None,
        dilution_factor: float = 1.0,
    ) -> dict[str, dict]:
        """Map well → concentration, flagging anything outside the curve.

        Returns one record per well with the concentration, whether it fell
        inside the standards' absorbance range, and any error — out-of-range
        wells are reported, never silently extrapolated.
        """
        results: dict[str, dict] = {}
        for well, raw in sample_absorbances.items():
            corrected = raw - blank_absorbance if blank_absorbance is not None else raw
            record = {
                "absorbance_raw": raw,
                "absorbance_corrected": corrected,
                "in_curve_range": curve.in_range(corrected),
                "dilution_factor": dilution_factor,
            }
            try:
                record["concentration"] = curve.concentration(corrected) * dilution_factor
            except AssayError as exc:
                record["concentration"] = None
                record["error"] = str(exc)
            results[well] = record
        return results


@dataclass
class BCAAssay(ProteinAssay):
    """Bicinchoninic acid assay — read at 562 nm.

    Defaults to a quadratic fit: BCA's response is meaningfully curved across
    its 20–2000 µg/mL working range, and forcing a line through it is a common
    source of quiet quantitation error. Detergent-compatible; incompatible with
    reducing agents and chelators.
    """

    wavelength_nm: int = BCA_NM
    model: str = "quadratic"
    name: str = "BCA"


@dataclass
class BradfordAssay(ProteinAssay):
    """Coomassie/Bradford assay — read at 595 nm.

    Linear over its working range, fast, and tolerant of reducing agents, but
    sensitive to detergents and strongly protein-to-protein variable.
    """

    wavelength_nm: int = BRADFORD_NM
    model: str = "linear"
    name: str = "Bradford"


@dataclass
class A280Assay(ProteinAssay):
    """Direct A280 quantitation — no standards, no reagents, non-destructive.

    Needs the protein's extinction coefficient and molecular weight to be
    accurate; without them :func:`a280_concentration` falls back to the crude
    1 AU ≈ 1 mg/mL approximation.
    """

    wavelength_nm: int = A280_NM
    name: str = "A280"
    extinction_coefficient: float | None = None
    molecular_weight: float | None = None

    def quantify_direct(
        self,
        sample_absorbances: dict[str, float],
        path_length_cm: float = 1.0,
        scatter_absorbances: dict[str, float] | None = None,
        dilution_factor: float = 1.0,
    ) -> dict[str, dict]:
        """Quantify without a standard curve, via Beer–Lambert."""
        out: dict[str, dict] = {}
        for well, a280 in sample_absorbances.items():
            a320 = (scatter_absorbances or {}).get(well)
            try:
                mg_ml = a280_concentration(
                    a280,
                    extinction_coefficient=self.extinction_coefficient,
                    molecular_weight=self.molecular_weight,
                    path_length_cm=path_length_cm,
                    a320=a320,
                )
                out[well] = {
                    "absorbance_280": a280,
                    "absorbance_320": a320,
                    "concentration_mg_ml": mg_ml * dilution_factor,
                    "approximate": not (
                        self.extinction_coefficient and self.molecular_weight
                    ),
                }
            except AssayError as exc:
                out[well] = {"absorbance_280": a280, "concentration_mg_ml": None,
                             "error": str(exc)}
        return out


@dataclass
class TurbidityAssay(ProteinAssay):
    """Light-scattering / turbidity read — the particle-onset signal.

    Role in the droplet-crystallization SDL (per the Research Wiki note
    *Nephelometers for Protein Solubility Testing*): this is an **auxiliary**
    scheduling and particle-onset sensor, not the solubility endpoint. It flags
    wells that may contain crystals, aggregates, amorphous precipitate, or
    liquid–liquid phase separation, so the orchestrator knows which wells are
    worth imaging or aspirating next.

    The thermodynamic solubility endpoint remains :class:`A280Assay` on the
    equilibrated supernatant — turbidity answers "are scattering particles
    present?", not "how much protein is dissolved?".

    A dedicated nephelometer is deliberately out of scope for the first build:
    on an absorbance reader, scattering shows up as apparent absorbance at a
    wavelength where the protein does not absorb (350–600 nm; 350 nm is the
    usual aggregation default, 600 nm is less sensitive but flatter).
    """

    wavelength_nm: int = 350
    name: str = "turbidity"
    baseline: float = 0.0
    threshold: float = 0.05

    def classify(
        self,
        absorbances: dict[str, float],
        baseline: float | None = None,
        threshold: float | None = None,
    ) -> dict[str, dict]:
        """Flag wells whose scattering exceeds a baseline by ``threshold``.

        Returns per-well ``{scatter, delta, particles_present}``. This is a
        screening flag, not a quantitation: apparent absorbance from scattering
        depends on particle number, size, morphology, refractive index, plate
        geometry, and meniscus effects, so treat it as ordinal at best.
        """
        base = self.baseline if baseline is None else baseline
        thr = self.threshold if threshold is None else threshold
        out: dict[str, dict] = {}
        for well, a in absorbances.items():
            delta = a - base
            out[well] = {
                "scatter": a,
                "delta": delta,
                "particles_present": delta >= thr,
                "wavelength_nm": self.wavelength_nm,
            }
        return out
