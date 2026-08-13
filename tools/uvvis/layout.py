"""Well-layout construction and BMG ``.lb`` layout-file generation.

``ImportLayout``/``EditLayout`` (fact 2/3) take a layout as either a file
(``.lb``) or an inline string of ``<well>=<content>`` pairs. This module gives
an orchestrator a typed way to build one — assign samples, blanks, standards,
and controls onto a :class:`~.config.PlateSpec`'s wells — then render it to
that string/file, and convert the WSL-visible path to the Windows path
``ImportLayout`` requires.

    from tools.nodes.uv_vis.layout import LayoutPlan
    from tools.nodes.uv_vis.config import PlateSpec

    plan = LayoutPlan.grid(rows=5, columns=3, replicates_along="column",
                           plate=PlateSpec.wells96())
    plan.to_lb_text()      # "EmptyLayout\\nA1=X1 A2=X1 A3=X1 B1=X2 ..."
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from .config import PlateSpec


class WellContent:
    """Layout content-code prefixes (fact 3). Append an index, e.g. ``X1``."""

    SAMPLE = "X"          # unknown sample
    STANDARD = "S"        # standard
    BLANK = "B"
    NEGATIVE = "N"        # negative control
    POSITIVE = "P"        # positive control
    CONTROL = "C"         # generic control
    EMPTY = "Empty"       # clears one well
    CLEAR = "-"           # clears one well (alternate spelling)


@dataclass
class LayoutPlan:
    """An ordered set of well -> content-code assignments for one plate.

    Build with the group methods (:meth:`samples`, :meth:`blanks`,
    :meth:`standards`, :meth:`controls`) or the :meth:`grid` convenience, then
    render with :meth:`to_lb_text`/:meth:`write_lb`. Every method validates
    wells against :attr:`plate` and refuses to assign a well twice — a stale
    or colliding assignment is exactly the class of bug ``EmptyLayout`` +
    explicit codes exists to prevent.
    """

    plate: PlateSpec = field(default_factory=PlateSpec.wells96)
    assignments: dict[str, str] = field(default_factory=dict)
    concentrations: dict[str, float] = field(default_factory=dict)

    # -- internals ----------------------------------------------------------

    def _check_new(self, wells: list[str]) -> list[str]:
        normed = [self.plate.validate(w) for w in wells]
        dupes = [w for w in normed if w in self.assignments]
        if dupes:
            raise ValueError(f"well(s) already assigned in this layout: {dupes}")
        return normed

    # -- builders -------------------------------------------------------------

    def samples(self, wells, replicates: int = 1, group: str | None = None) -> LayoutPlan:
        """Assign ``wells`` as samples, ``X1, X2, ...``.

        A "replicate" is a block of ``replicates`` *consecutive* wells (in the
        order given) sharing one index — e.g. with ``replicates=3``, the first
        three wells all become ``X1``, the next three ``X2``, and so on.
        """
        wells = list(wells)
        normed = self._check_new(wells)
        if replicates < 1 or len(normed) % replicates != 0:
            raise ValueError(
                f"{len(normed)} wells is not a positive multiple of "
                f"replicates={replicates}"
            )
        prefix = f"{WellContent.SAMPLE}{group}" if group else WellContent.SAMPLE
        idx = 1
        for i in range(0, len(normed), replicates):
            for w in normed[i:i + replicates]:
                self.assignments[w] = f"{prefix}{idx}"
            idx += 1
        return self

    def blanks(self, wells, group: str | None = None) -> LayoutPlan:
        """Assign ``wells`` as blanks — all share the code ``B`` (no index)."""
        normed = self._check_new(list(wells))
        code = f"{WellContent.BLANK}{group}" if group else WellContent.BLANK
        for w in normed:
            self.assignments[w] = code
        return self

    def standards(
        self, wells, concentrations: list[float] | None = None,
        replicates: int = 1, group: str | None = None,
    ) -> LayoutPlan:
        """Assign ``wells`` as standards, ``S1, S2, ...``, one level per
        ``replicates``-sized block. ``concentrations``, if given, must have one
        entry per level (``len(wells) // replicates``) and is recorded in
        :attr:`concentrations` per well — it does not appear in the ``.lb`` text,
        which only the control software's own standard-concentration UI/
        ``ImportConcAndVol`` consumes.
        """
        wells = list(wells)
        normed = self._check_new(wells)
        if replicates < 1 or len(normed) % replicates != 0:
            raise ValueError(
                f"{len(normed)} wells is not a positive multiple of "
                f"replicates={replicates}"
            )
        n_levels = len(normed) // replicates
        if concentrations is not None and len(concentrations) != n_levels:
            raise ValueError(
                f"{len(concentrations)} concentrations given for {n_levels} "
                "standard levels"
            )
        prefix = f"{WellContent.STANDARD}{group}" if group else WellContent.STANDARD
        idx = 1
        for i in range(0, len(normed), replicates):
            block = normed[i:i + replicates]
            for w in block:
                self.assignments[w] = f"{prefix}{idx}"
                if concentrations is not None:
                    self.concentrations[w] = concentrations[idx - 1]
            idx += 1
        return self

    def controls(self, wells, kind: str = "C", group: str | None = None) -> LayoutPlan:
        """Assign ``wells`` as controls: ``kind`` is ``"C"`` (generic),
        ``"N"`` (negative), or ``"P"`` (positive) — see fact 3."""
        if kind not in (WellContent.CONTROL, WellContent.NEGATIVE, WellContent.POSITIVE):
            raise ValueError(f"control kind must be 'C', 'N', or 'P', got {kind!r}")
        normed = self._check_new(list(wells))
        prefix = f"{kind}{group}" if group else kind
        idx = 1
        for w in normed:
            self.assignments[w] = f"{prefix}{idx}"
            idx += 1
        return self

    @classmethod
    def grid(
        cls, rows, columns, replicates_along: str = "row",
        plate: PlateSpec | None = None,
    ) -> LayoutPlan:
        """Convenience for the common rectangular sample block.

        ``rows``/``columns`` are either a count (int, taken from the plate's
        own labels/1-based columns) or an explicit list. ``replicates_along``
        selects which axis holds the replicate — ``"column"`` means each row is
        one sample repeated across those columns (e.g. 5 samples x 3
        replicate columns -> A1,A2,A3 all one sample, B1,B2,B3 the next, ...);
        ``"row"`` is the transpose.
        """
        p = plate or PlateSpec.wells96()
        row_labels = list(rows) if not isinstance(rows, int) else p.row_labels[:rows]
        col_nums = list(columns) if not isinstance(columns, int) else list(range(1, columns + 1))
        plan = cls(plate=p)
        if replicates_along == "column":
            wells = [f"{r}{c}" for r in row_labels for c in col_nums]
            plan.samples(wells, replicates=len(col_nums))
        elif replicates_along == "row":
            wells = [f"{r}{c}" for c in col_nums for r in row_labels]
            plan.samples(wells, replicates=len(row_labels))
        else:
            raise ValueError("replicates_along must be 'row' or 'column'")
        return plan

    # -- rendering ------------------------------------------------------------

    def to_lb_text(self, empty_first: bool = True) -> str:
        """The ``.lb`` file body: ``EmptyLayout`` first (default), then one
        line per content-code group (all samples together, all standards
        together, ...), each a space-separated run of ``well=code``."""
        lines: list[str] = []
        if empty_first:
            lines.append("EmptyLayout")
        groups: dict[str, list[str]] = {}
        order: list[str] = []
        for well, code in self.assignments.items():
            prefix = code.rstrip("0123456789")
            if prefix not in groups:
                groups[prefix] = []
                order.append(prefix)
            groups[prefix].append(f"{well}={code}")
        for prefix in order:
            lines.append(" ".join(groups[prefix]))
        return "\n".join(lines)

    def write_lb(self, path: str | Path) -> Path:
        """Write :meth:`to_lb_text` to ``path`` (any extension but ``.lac`` is
        treated as ``.lb`` per fact 3) and return the path."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_lb_text())
        return path

    def measured_wells(self) -> list[str]:
        """The wells this plan will cause to be read, in assignment order —
        i.e. every well with a non-empty/non-cleared code."""
        return [
            w for w, code in self.assignments.items()
            if code not in (WellContent.EMPTY, WellContent.CLEAR)
        ]


# --------------------------------------------------------------------------
# Path conversion — DDE takes Windows paths, everything here lives in WSL
# --------------------------------------------------------------------------

_MNT_RE = re.compile(r"^/mnt/([A-Za-z])(/.*)?$")


def windows_path(posix_path: str | Path) -> str:
    """Convert a WSL ``/mnt/<drive>/...`` path to ``<DRIVE>:\\...``.

    ``ImportLayout``/``EditLayout`` run inside Windows and need a Windows path;
    everything this package writes lives under a WSL path first. Handles any
    drive letter, not just ``c``. A path that is not ``/mnt/<drive>/...``-shaped
    is passed through with forward slashes flipped, best-effort.
    """
    p = str(posix_path)
    m = _MNT_RE.match(p)
    if m:
        drive = m.group(1).upper()
        rest = (m.group(2) or "").lstrip("/")
        win_rest = rest.replace("/", "\\")
        return f"{drive}:\\{win_rest}" if win_rest else f"{drive}:\\"
    return p.replace("/", "\\")
