"""Everything needed to operate the machines in this lab.

One sub-package per instrument, plus the cross-device pieces they share. The
**phase scripts at the repo root** (`calibration.py`, `sharpness_test.py`, …)
import from here and are the layer that decides; this package supplies the
functions and reports what it measures.

    from tools import arm, microscope

    arm.move(35, 34, 33, "config.yaml")          # dry-run unless allowed
    with arm.session("config.yaml", live=True) as a:
        a.move_to("microscope"); a.grip()

    sc = microscope.Microscope.from_config("config.yaml")
    frame = sc.grab_frame(Path("shot.jpg"))
    sample = sc.measure_focus(frame.require())

| sub-package | instrument / concern |
|---|---|
| `arm`     | UFACTORY xArm7 + BIO Gripper G2 |
| `microscope`   | Leica K3C + TIS DFK33UX264, and the optics measurements |
| `uvvis`   | BMG SPECTROstar Nano absorbance reader |
| `calib`   | pixel <-> arm transforms; belongs to neither device |
| `lab`     | the node/bus framework and the web dashboard |
| `config`  | the one runtime-config surface (`config.yaml` + precedence) |
| `runs`    | per-run dataset folders under `dataset/` |

`opentrons`, `environment` and `cctv` are vacant stubs awaiting hardware.

Nothing here imports a vendor SDK at module level, so the whole package loads on
a machine with no hardware attached. Motion is dry-run unless explicitly enabled
-- see `Server/lamp-lab/.claude/rules/motion-safety.md`.
"""
from . import arm, calib, config, microscope, runs, uvvis

__all__ = ["arm", "calib", "config", "runs", "microscope", "uvvis", "__version__"]
__version__ = "0.3.0"
