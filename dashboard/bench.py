"""The bench: where each instrument physically sits, in millimetres.

Operator-measured, 2026-08-04. The bench is **2000 mm wide by 1000 mm deep**,
seen from above with the origin at the back-left corner:

    x=0                       900                              2000
    y=0  +--------------------+--------------------------------+
         |                    |                                |
         |                    |            +-----------+       |
         |     Opentrons      |            | Microscope|       |
         |     900 x 1000     |   [ARM]    |  400x400  |       |
         |   (full left side, |            +-----------+       |
         |    full depth)     |                                |
         |                    |     +--------------------+     |
         |                    |     |  UV-Vis 600 x 300  |     |
    y=1000+--------------------+-----+--------------------+----+

The arm sits between the Opentrons deck and the two instruments on the right,
which is exactly why the arm and the UV-Vis carrier can never move together:
the carrier slides out of the reader into the space the arm swings through.
That constraint is enforced in :mod:`tools.occupancy` and merely *drawn* here.

These are footprints for a status diagram, not a kinematic model -- do not
compute a reach or a clearance from them. The real geometry lives in the taught
workspace under ``~/.sdl_lab/robot_arm/``. What is measured is each instrument's
outline; the arm's box is its base footprint, not its reach.
"""
from __future__ import annotations

#: Bench top, millimetres. Origin back-left, x to the right, y toward the front.
BENCH_W_MM = 2000
BENCH_D_MM = 1000

#: One entry per instrument the panel draws.
#:
#: ``device``    matches the node name and the occupancy key, so a claim lights
#:               the right box up. A mismatch lights the wrong one.
#: ``x/y/w/h``   footprint in millimetres on the bench above.
#: ``verb``      what "busy" means for this one, in the operator's words.
BENCH: list[dict] = [
    {
        "device": "opentrons",
        "label": "Opentrons",
        "sub": "900 x 1000",
        "x": 0, "y": 0, "w": 900, "h": 1000,
        "verb": "pipetting",
        "shape": "box",
    },
    {
        "device": "arm",
        "label": "Arm",
        "sub": "xArm7",
        "x": 1010, "y": 380, "w": 260, "h": 260,
        "verb": "moving",
        "shape": "arm",
    },
    {
        "device": "microscope",
        "label": "Microscope",
        "sub": "400 x 400",
        "x": 1450, "y": 140, "w": 400, "h": 400,
        "verb": "capturing",
        "shape": "box",
    },
    {
        "device": "uv_vis",
        "label": "UV-Vis",
        "sub": "600 x 300",
        "x": 1350, "y": 660, "w": 600, "h": 300,
        "verb": "reading",
        "shape": "box",
    },
]

#: Drawn between two instruments that must never move together, so the reason
#: for a refusal is visible on the diagram and not only in an error string.
BENCH_CONFLICT_PAIRS = [("arm", "uv_vis")]


def bench_spec() -> dict:
    """What the page needs to draw the bench. Pure data; no rendering here."""
    return {
        "width_mm": BENCH_W_MM,
        "depth_mm": BENCH_D_MM,
        "items": BENCH,
        "conflicts": [list(p) for p in BENCH_CONFLICT_PAIRS],
    }


__all__ = [
    "BENCH",
    "BENCH_CONFLICT_PAIRS",
    "BENCH_D_MM",
    "BENCH_W_MM",
    "bench_spec",
]
