#!/usr/bin/env python3
"""scan_report's new peak-quality facts, judge_scan, and a known-answer replay.

Two ground truths, neither produced by the code under test:

  * a synthetic flat/noisy scan (peak/median must be near 1.0 -- nothing is
    "in focus" when nothing has structure at all);
  * two REAL saved sweeps under dataset/captures/focus_sweep/ -- one with no
    target in frame (a1_z=185.3177, 2026-08-05, 14 coarse frames @ 0.3mm) and
    one with a real focus peak (2026-08-04, 11 coarse + fine frames). These
    are run-generated output, not committed to git, so this SKIPS LOUDLY
    (never silently) when they are absent, same convention as
    test_focus_known_answer.py's fixtures/ check.

    python dev/tests/test_autofocus_judge.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.microscope import autofocus as af  # noqa: E402
from tools.microscope import focus  # noqa: E402

failures = 0


def ok(condition: bool, message: str, detail: str = "") -> None:
    global failures
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        failures += 1


class _S:
    def __init__(self, score, usable=True, reason=""):
        self.score = score
        self.score_normalised = None if score is None else score / 100.0
        self.usable = usable
        self.reason = reason
        self.method = "tenengrad"
        self.masked = True


print("--- scan_report: peak-quality facts on synthetic data ---")

# A dot-free sweep: everything about equally "sharp" -- low prominence.
heights = [0.0, 1.0, 2.0, 3.0, 4.0]
flat = [_S(4.3), _S(5.1), _S(10.4), _S(6.2), _S(4.9)]
flat_report = af.scan_report(flat, heights)
ok(flat_report["ok"], "flat scan is ok (some usable samples)")
ok(abs(flat_report["peak_over_median"] - flat_report["best_score"]
       / flat_report["baseline_median"]) < 1e-3,
   "peak_over_median matches best/median by construction (report rounds it)")
ok(flat_report["peak_over_median"] < 4.0,
   "a flat/noisy scan sits below the measured 4.0 prominence floor",
   f"{flat_report['peak_over_median']} (real barren sweep measured 1.17)")

# A real peak: one sample far above the rest -- high prominence.
peaked = [_S(6.0), _S(7.5), _S(8.0), _S(364.0), _S(9.0)]
peaked_report = af.scan_report(peaked, heights)
ok(peaked_report["peak_over_median"] > 20.0,
   "a real peak has high prominence", f"{peaked_report['peak_over_median']}")
ok(not peaked_report["peak_at_boundary"], "the real peak is interior, not at an edge")

# Boundary peak.
edge = [_S(50.0), _S(10.0), _S(9.0), _S(8.0), _S(7.0)]
edge_report = af.scan_report(edge, heights)
ok(edge_report["peak_at_boundary"], "a peak at the first height is flagged as boundary")

# unusable accounting
mixed = [_S(None, usable=False, reason="scored region too dark (mean 2.0 < 30.0)"),
        _S(None, usable=False, reason="scored region saturated (mean 251.0)"),
        _S(20.0), _S(21.0), _S(22.0)]
mixed_report = af.scan_report(mixed, heights)
ok(mixed_report["n_unusable"] == 2 and abs(mixed_report["unusable_frac"] - 0.4) < 1e-9,
   "n_unusable/unusable_frac count the skipped samples")
ok(mixed_report["unusable_by_reason"] == {"dark": 1, "saturated": 1},
   "unusable_by_reason buckets by category",
   f"{mixed_report['unusable_by_reason']}")

# ok=False shape is preserved.
all_bad = [_S(None, usable=False, reason="x")] * 3
bad_report = af.scan_report(all_bad, [0.0, 1.0, 2.0])
ok(bad_report == {"ok": False, "reason": "no usable sample in the scan",
                  "skipped": bad_report["skipped"], "n_scored": 0},
   "an all-unusable scan keeps the old {ok: False, ...} shape")

print("\n--- judge_scan: caller-supplied thresholds, both required ---")

j_flat = af.judge_scan(flat_report, min_prominence=4.0, min_absolute_score=0.0,
                       max_unusable_frac=1.0, require_interior=False)
ok(not j_flat["usable"], "the flat scan fails judge_scan")
ok(any("prominence" in r for r in j_flat["reasons"]),
   "the flat scan fails on prominence specifically", f"{j_flat['reasons']}")

j_peaked = af.judge_scan(peaked_report, min_prominence=2.0, min_absolute_score=50.0,
                         max_unusable_frac=1.0, require_interior=False)
ok(j_peaked["usable"], "the real peak passes judge_scan")

# Prominence alone is not sufficient: a single spike on an otherwise dark scan
# must still fail on the absolute floor.
spike = [_S(1.0), _S(1.1), _S(50.0), _S(1.2), _S(1.0)]
spike_report = af.scan_report(spike, heights)
j_spike_prom_only = af.judge_scan(spike_report, min_prominence=2.0, min_absolute_score=0.0,
                                  max_unusable_frac=1.0, require_interior=False)
ok(j_spike_prom_only["usable"],
   "sanity: prominence alone WOULD pass a dust-speck spike with no absolute floor")
j_spike_both = af.judge_scan(spike_report, min_prominence=2.0, min_absolute_score=200.0,
                             max_unusable_frac=1.0, require_interior=False)
ok(not j_spike_both["usable"] and any("absolute floor" in r for r in j_spike_both["reasons"]),
   "the absolute-score floor catches what prominence alone would miss",
   f"{j_spike_both['reasons']}")

j_edge_default = af.judge_scan(edge_report, min_prominence=0.0, min_absolute_score=0.0,
                               max_unusable_frac=1.0, require_interior=False)
ok(j_edge_default["usable"], "require_interior=False lets a boundary peak pass")
j_edge_strict = af.judge_scan(edge_report, min_prominence=0.0, min_absolute_score=0.0,
                              max_unusable_frac=1.0, require_interior=True)
ok(not j_edge_strict["usable"] and any("edge" in r for r in j_edge_strict["reasons"]),
   "require_interior=True rejects the same boundary peak",
   f"{j_edge_strict['reasons']}")

j_unusable_cap = af.judge_scan(mixed_report, min_prominence=0.0, min_absolute_score=0.0,
                               max_unusable_frac=0.1, require_interior=False)
ok(not j_unusable_cap["usable"] and any("unusable fraction" in r for r in j_unusable_cap["reasons"]),
   "max_unusable_frac rejects a scan with too many skipped samples")

j_not_ok = af.judge_scan({"ok": False, "reason": "no usable sample in the scan"},
                         min_prominence=1.0, min_absolute_score=1.0,
                         max_unusable_frac=1.0, require_interior=False)
ok(not j_not_ok["usable"], "judge_scan on a not-ok report is never usable")

print("\n--- known-answer replay: dataset/captures/focus_sweep -----------------")

DATASET = REPO / "dataset" / "captures" / "focus_sweep"
NO_TARGET_DIR = DATASET / "20260805-143628-909908"
REAL_PEAK_DIR = DATASET / "20260804-171415-032884"


def _load(directory: Path, glob: str):
    frames = sorted(directory.glob(glob))
    heights = []
    for f in frames:
        # e.g. coarse_11_z+03.000.jpg / fine_05_z+07.900.jpg
        tail = f.stem.split("_z")[-1]
        heights.append(float(tail))
    return frames, heights


if not NO_TARGET_DIR.exists() or not REAL_PEAK_DIR.exists():
    print("[test] SKIP: one or both known-answer directories are absent under %s" % DATASET)
    print("[test] SKIPPED -- run-generated output, not committed to git. Missing:")
    if not NO_TARGET_DIR.exists():
        print("         %s" % NO_TARGET_DIR)
    if not REAL_PEAK_DIR.exists():
        print("         %s" % REAL_PEAK_DIR)
else:
    no_frames, no_heights = _load(NO_TARGET_DIR, "coarse_*_z+*.jpg")
    ok(len(no_frames) == 14, "no-target sweep has 14 coarse frames", f"{len(no_frames)}")
    no_samples = [focus.score_frame(f, method="laplacian", masked=True) for f in no_frames]
    no_report = af.scan_report(no_samples, no_heights)
    print("    no-target: peak_over_median=%s best_score=%s"
         % (no_report.get("peak_over_median"), no_report.get("best_score")))
    ok(no_report["ok"], "no-target sweep still scores (some usable samples)")
    ok(abs(no_report["peak_over_median"] - 1.16) < 0.15,
       "no-target sweep's peak/median is close to the measured ~1.16",
       f"{no_report['peak_over_median']}")
    j_no_target = af.judge_scan(no_report, min_prominence=4.0, min_absolute_score=40.0,
                                max_unusable_frac=1.0, require_interior=False)
    ok(not j_no_target["usable"],
       "judge_scan rejects the no-target sweep at the same thresholds find_target.py defaults to",
       f"{j_no_target['reasons']}")

    peak_frames, peak_heights = _load(REAL_PEAK_DIR, "*_z+*.jpg")
    ok(len(peak_frames) > 0, "real-peak sweep has frames", f"{len(peak_frames)}")
    peak_samples = [focus.score_frame(f, method="laplacian", masked=True) for f in peak_frames]
    peak_report = af.scan_report(peak_samples, peak_heights)
    print("    real peak: peak_over_median=%s best_score=%s best_z_mm=%s"
         % (peak_report.get("peak_over_median"), peak_report.get("best_score"),
            peak_report.get("best_z_mm")))
    ok(peak_report["ok"], "real-peak sweep scores")
    ok(peak_report["peak_over_median"] > 10.0,
       "real-peak sweep's prominence is far above the no-target sweep's",
       f"{peak_report['peak_over_median']} vs {no_report['peak_over_median']}")
    j_peak = af.judge_scan(peak_report, min_prominence=4.0, min_absolute_score=40.0,
                           max_unusable_frac=1.0, require_interior=False)
    ok(j_peak["usable"],
       "judge_scan accepts the real-peak sweep at the same thresholds",
       f"{j_peak['reasons']}")

print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURE(S)" % failures))
sys.exit(1 if failures else 0)
