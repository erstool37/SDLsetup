# SDLsetup

Control layer for the WSL self-driving lab: a UFACTORY xArm7 that carries a
96-well plate under a fixed microscope objective, two cameras, and a BMG
SPECTROstar Nano UV-Vis reader.

```bash
pyenv exec python -m pip install -r requirements.txt   # the whole setup
python scripts/microscope/calibration.py --help        # dry-run by default
python -m dashboard                                    # panel on :8770
```

Nothing is installed. `tools/` sits beside `scripts/` and is imported directly.

---

## Layout

**`tools/` holds everything that operates a machine. The scripts decide what to
do with it. `dataset/` holds what came out.**

```
SDLsetup/
  scripts/                  ONE SCRIPT PER RUN -- the layer that DECIDES
    microscope/             operations at the microscopy station
      calibration.py        align well A1 under the objective, end to end
      sharpness_test.py     upward-only Z sweep to find best focus
      xy_center.py          centre the marker by moving X/Y at a locked Z
      pixel_scale.py        measure px/mm and direction at several heights
    tool_building/          the pieces those were assembled from
      plate_imaging.py      the 96-well imaging loop
      pick_place.py         home -> floor -> grab -> microscope
      goto_rise.py          move to a given height above A1, nothing else

  configs/
    config.yaml             runtime controls for every device

  tools/                    EVERYTHING NEEDED TO OPERATE THE MACHINES
    arm/                    UFACTORY xArm7 + BIO Gripper G2
      api.py                MAIN: Arm facade + module-level move()/grip()
      driver.py             the only XArmAPI transport; takes ValidatedMove only
      safety.py             Envelope + ValidatedMove -- guards, enforced by type
      workspace.py          taught locations/routes (the safety anchor)
      geometry.py           the 9 mm well grid, anchored on A1
      approach.py           the 4-leg Y approach into a well under the objective
      teach.py  node.py  cli.py  arm.json  check_sdk.py
    microscope/             Leica K3C + TIS DFK33UX264 + the optics
      api.py                MAIN: Microscope facade -- capture, frames, measure
      capture.py            camera classes, capture batch, capture CLI
      backends/             vendor programs (GenTL, TWAIN, tisgrabber) + PS1
      viewer.py             the live monitor web UI on :8766
      focus.py              sharpness measurement + the lit-region mask
      autofocus.py          scan planning and peak selection
      marker.py             locate the alignment mark, with quality flags
      imaging.py  batch.py  node.py  cameras.json
    uvvis/                  BMG SPECTROstar Nano (reader, assays, analysis)
    calib/                  CROSS-DEVICE pixel <-> arm transforms
    occupancy.py            THE INTERLOCK -- who is moving, and who may not
    config.py               the one runtime-config surface + precedence
    runs.py                 per-run dataset folders + reproducibility manifests
    node.py                 base class for each device's dashboard adapter

  dashboard/                the panel: what is moving, what finished
    __main__.py             python -m dashboard
    bench.py                where each instrument physically sits
    server.py  page.py  app.py  registry.py  bus.py  oplog.py

  dataset/                  MIRRORS scripts/. every run's output. gitignored
    calibration/2026-08-03_143012/{manifest.json,run.log,devices/,results/}

  dev/                      checking the code. never runs an experiment
    ruff.toml  lint.sh  test.sh  tests/
```

## The boundary that matters

**`tools/` reports. `scripts/` decides.** A tool returns measurements, state and
quality flags; choosing the next well, retrying a reading, discarding a sample or
accepting a focus plane happens in the script. Focus is the model:
`tools/microscope/focus.py` measures sharpness and says when a frame is too dark
to score; `tools/microscope/autofocus.py` supplies the planning and peak-picking
*functions*; `scripts/microscope/sharpness_test.py` is what actually decides.

This is the lab's "sensors report, they do not decide" law as a directory
layout — a tool that interprets its own signal cannot be audited afterwards, and
cannot be weighed against the other sensors.

## Two instruments may never move together

The UV-Vis plate carrier slides **out into the arm's workspace**. They are
different instruments, on different interfaces, driven by **different
processes** — so the interlock lives on disk, in `tools/occupancy.py`, and
claiming it is atomic:

```python
from tools import occupancy

with occupancy.claim("arm", doing="A1 focus sweep"):
    ...                      # the UV-Vis carrier is refused for the duration
```

Every arm motion, every gripper actuation and every carrier move takes a claim
before it starts. A claim whose process died is cleared automatically, so a
crashed run cannot lock the rig. `retreat()` deliberately takes no claim: it only
lowers the plate away from the objective, and refusing it would leave the plate
up — the exact state it exists to escape.

The panel draws it. `python -m dashboard` shows a bench diagram with each
instrument's box lit by what it is doing, and a dashed link between the two that
share space.

## Motion safety

Read `Server/lamp-lab/.claude/rules/motion-safety.md` before touching anything
under `tools/arm/`. In short:

- **Dry-run is the default.** `live=False` never imports the SDK and never opens
  a socket, so a dry-run plan *cannot* move the arm.
- **Every commanded pose passes an `Envelope`,** and `driver.send()` accepts only
  a `ValidatedMove`, which only `Envelope.validate()` can build. There is no
  supported path to `set_position` that skips the guards.
- **Z rise ≤ 10 mm above the taught A1** — raising Z drives the plate *toward*
  the fixed objective. Two speed caps: 30 mm/s fine positioning, 100 mm/s
  transit. Approach every Z from below.
- **Controller faults are never cleared automatically.** A latched `error_code`
  may record a previous collision.
- **Connecting is not arming.** `connect(arm=True)` is what enables motion, so
  `sdl-robot controller` and `gripper status` really are read-only.

```python
from tools import arm

with arm.session("configs/config.yaml", live=True) as a:
    a.move_to("microscope"); a.grip()
```

## Checking it

```bash
dev/lint.sh        # ruff; catches undefined names in branches that have not run
dev/test.sh        # every suite; no hardware, no motion, no network
```

`dev/tests/test_arm_safety.py` is the one that matters most — the only automated
check that a motion guard still fires, including the pose from the day the rig
moved 340 mm. Add to it *before* changing anything under `tools/arm/`.

## Adding things

**A machine:** `tools/<name>/` with `api.py`, `node.py`, `__init__.py`,
`README.md`, and an inventory JSON if it has one. Register its node in
`dashboard/app.py`. Add its runtime section to `configs/config.yaml`. If it
occupies bench space, add it to `tools/occupancy.py`'s `CONFLICTS` **and** to
`dashboard/bench.py`.

**A run script:** `scripts/<group>/<name>.py` with `build_parser()` / `run(args)`
/ `main(argv=None) -> int`, importing only from `tools`. Open a `tools.runs.Run`
so its output lands in `dataset/<name>/<stamp>/`.

## Environment

pyenv virtualenv `main` (Python 3.12); `.python-version` pins it. In
non-interactive shells use `~/.pyenv/bin/pyenv exec python ...`.

Agent rules for this codebase live in the lamp-lab control surface on the
operator's Mac: `Server/lamp-lab/.claude/rules/`.

## Repository

https://github.com/erstool37/SDLsetup
