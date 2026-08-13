# dev/

Everything about checking the code, kept out of the way of the code itself.
Nothing in here runs during an experiment, and nothing in here touches hardware.

```
dev/
  ruff.toml    lint configuration, with the reason for every disabled rule
  lint.sh      dev/lint.sh [--fix]
  test.sh      dev/test.sh -- runs every suite below
  tests/       the suites
```

## Why lint at all

`ruff` reads the code without running it. Python only resolves a name inside a
function when that function is *called*, so a typo in a branch that has not run
yet stays silent — through `--help`, through an import sweep, through the test
suite. Three real crashes in this repo were found exactly that way, the worst of
them in the branch that writes an aligned pose back to `workspace.json`: it
would have surfaced only after the arm had finished a live alignment.

It costs under a second for the whole tree and needs no hardware.

It does **not** check whether a guard is logically right, or whether a focus
metric picks the correct plane. That is what `tests/` and independent
verification are for.

## The suites

| suite | covers |
|---|---|
| `test_arm_safety.py` | the motion guards — 69 assertions. Envelope limits, the readback guard, connect-is-not-arming, retreat under every envelope shape, and the regression pose from the day the rig moved 340 mm |
| `test_focus.py` | focus metrics, the lit-region mask, scan planning and peak selection |
| `test_calibration.py` | the port-based centring routines, driven by fakes |
| `test_microscopy_config.py` | how camera runtime defaults resolve from `configs/config.yaml` |
| `test_microscopy_viewer.py` | the live monitor's save/record endpoints |
| `test_setup_script.py` | `setup.sh` behaviour against a fake pyenv |

`test_arm_safety.py` is the one that matters most: it is the only automated
check that a motion guard still fires. Add to it *before* changing anything
under `tools/arm/`.
