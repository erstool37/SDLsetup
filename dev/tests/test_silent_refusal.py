#!/usr/bin/env python3
"""A silently refused move must not be reported as a completed one.

THE FAILURE THIS EXISTS FOR, measured on this rig 2026-08-04: commanding
y=290.625 at x=26.029 stopped the controller dead at y=319.855 with
`state=4, code=-9, error_code=0` -- NO latched fault. The arm simply did not go.

`Arm.move` reads the pose back and checks it against the ENVELOPE. An envelope is
a region, not a destination: the pose the arm stopped at is very often still
inside it -- trivially so when the arm did not move at all, because it started
inside. So the readback check passes and the move reports success.

`MoveResult` carries both `target` and `achieved`. Before this suite, nothing in
the repo compared them (grep `achieved` across tools/ and scripts/). Every
caller that believes "the move returned, so the arm is at the target" was
relying on a check that was never written -- including
`find_target.py:attempt_move`, whose docstring asserts a refusal surfaces as
`SafetyError`, and whose own safety test encoded that same false premise by
making its fake arm RAISE on refusal.

The fake here does the opposite, and it is the whole point: on refusal it
returns an UNCHANGED pose, exactly as the controller does.

No hardware, no motion, no network.

    python dev/tests/test_silent_refusal.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.arm.api import ARRIVAL_TOL_MM, MoveResult  # noqa: E402
from tools.arm.safety import READBACK_SLACK_MM, Envelope, SafetyError  # noqa: E402

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               "  (%s)" % detail if detail else ""))
    if not condition:
        failures += 1


A1 = [26.028557, 590.625244, 185.3177, 179.99515, -0.002807, 89.745359]

# The envelope a sweep at A1 actually uses: 10 mm of rise, a 2.5 mm XY box.
ENV = Envelope.anchored(A1, name="A1", z_max_rise_mm=10.0, xy_max_mm=2.5,
                        orient_tol_deg=0.5)


# -- 1. the premise: a refused move's pose still satisfies the envelope -----
target = ENV.pose_at(x=A1[0], y=A1[1], z=A1[2] + 5.0)
refused = list(A1)                      # the arm never moved

try:
    ENV.check_readback(refused, where="a refused move")
    envelope_accepts_refusal = True
    envelope_error = ""
except SafetyError as exc:
    envelope_accepts_refusal = False
    envelope_error = str(exc)[:70]

ok(envelope_accepts_refusal,
   "the envelope ACCEPTS the pose of a refused move -- which is why an envelope "
   "check alone cannot detect one",
   envelope_error or "accepted, as expected")


# -- 2. what a caller is handed ---------------------------------------------
result = MoveResult("z step", tuple(target), [round(v, 4) for v in refused],
                    True, False)

ok(result.achieved is not None, "the result carries a read-back pose")
gap = max(abs(a - t) for a, t in zip(result.achieved[:3], result.target[:3], strict=False))
ok(gap > 1.0,
   "target and achieved differ by a lot on a refused move",
   "%.3f mm on the worst axis" % gap)


# -- 3. THE CHECK ITSELF ----------------------------------------------------
# This is what must exist. It is written against the API the fix will add.
has_verify = hasattr(result, "verify")
ok(has_verify,
   "MoveResult exposes verify() so a caller can tell a refusal from a move",
   "MoveResult fields: %s" % ", ".join(
       k for k in vars(result)) if not has_verify else "")

if has_verify:
    verdict = result.verify()
    ok(verdict["moved"] is False,
       "verify() reports a refused move as NOT moved", str(verdict.get("reason", ""))[:70])
    ok(verdict["worst_axis"] in ("z", 2),
       "verify() names the axis that did not arrive", str(verdict.get("worst_axis")))

    # A real move that merely settled short must NOT be flagged.
    # Real settling on this rig is 0.6 um worst-case (2026-08-05 transit) and
    # 1.3 um over 1047 mm (2026-08-04). Neither may be flagged.
    for observed_um in (0.6, 1.3, 10.0):
        settled = list(target)
        settled[2] -= observed_um / 1000.0
        v2 = MoveResult("z step", tuple(target), [round(v, 6) for v in settled],
                        True, False).verify()
        ok(v2["moved"] is True,
           "verify() does NOT flag settling of %.1f um (measured on this rig)"
           % observed_um,
           "tolerance %.3f mm" % ARRIVAL_TOL_MM)

    # And it must NOT inherit READBACK_SLACK_MM: a 0.4 mm shortfall is 8x the
    # arrival tolerance and 667x the observed settling. Borrowing the boundary
    # slack here would have called it an arrival.
    coarse = list(target)
    coarse[2] -= 0.4
    v3 = MoveResult("z step", tuple(target), coarse, True, False).verify()
    ok(v3["moved"] is False,
       "a 0.4 mm shortfall is a REFUSAL, not settling -- READBACK_SLACK_MM "
       "(%.1f mm) would have passed it" % READBACK_SLACK_MM,
       "arrival tolerance %.3f mm" % ARRIVAL_TOL_MM)

    # THE MEASURED REFUSAL, 2026-08-04: commanded y=290.625 at x=26.029, the
    # controller stopped at y=319.855. PARTIAL progress -- the arm moved, just
    # not to where it was sent. A fake that only models "did not move at all"
    # never exercises this, which is exactly what the previous safety test did.
    y_cmd, y_stopped = 290.625, 319.855
    partial_target = (A1[0], y_cmd, A1[2], A1[3], A1[4], A1[5])
    partial_actual = [A1[0], y_stopped, A1[2], A1[3], A1[4], A1[5]]
    v5 = MoveResult("Y traverse", partial_target, partial_actual, True, False).verify()
    ok(v5["moved"] is False,
       "the REAL 2026-08-04 refusal (stopped short, not unmoved) is detected",
       "y %.3f short" % (y_stopped - y_cmd))
    ok(v5["worst_axis"] == "y", "and it names y, the axis that stopped short",
       str(v5["worst_axis"]))

    # An orientation-only miss must not be masked by a larger-but-fine mm gap.
    tw = list(target)
    tw[0] += 0.03          # inside the mm tolerance
    tw[5] -= 0.4           # outside the deg tolerance
    v6 = MoveResult("wrist", tuple(target), tw, True, False).verify()
    ok(v6["moved"] is False and v6["worst_axis"] == "yaw",
       "a wrist that did not turn is not hidden by an in-tolerance mm axis",
       "worst=%s" % v6["worst_axis"])

    # Dry-run has no readback and must not claim either way.
    dry = MoveResult("z step", tuple(target), None, False, False)
    v4 = dry.verify(tol_mm=READBACK_SLACK_MM)
    ok(v4["moved"] is None,
       "verify() on a dry-run result is UNKNOWN, not True and not False",
       str(v4.get("reason", ""))[:60])

print()
print("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures)
sys.exit(1 if failures else 0)
