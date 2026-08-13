# arm — UFACTORY xArm7 + BIO Gripper G2

Everything needed to operate the arm. Control box `192.168.1.201`, on a private
switch with no internet uplink; SDK `xarm-python-sdk` 1.18.4.

```python
from sdlsetup.devices import arm

arm.move(35, 34, 33, "config.yaml")          # cached session, dry-run by default
with arm.session("config.yaml", live=True) as a:
    a.move_to("microscope"); a.grip()
```

## Layers

| Module | Holds |
|---|---|
| `api.py` | **main** — `Arm`, `MoveResult`, and the module-level shorthands |
| `driver.py` | the only `XArmAPI` transport; accepts `ValidatedMove` and nothing else |
| `safety.py` | `Envelope` + `ValidatedMove` — the guards, enforced by type |
| `workspace.py` | taught locations/routes JSON store; the safety anchor comes from here |
| `geometry.py` | the 9 mm well grid, anchored on A1 |
| `approach.py` | the 4-leg Y approach route to a well under the objective |
| `teach.py` | capture the current pose into `workspace.json` |
| `node.py` | dashboard adapter — thin, delegates to `api` |
| `cli.py` | `python -m sdlsetup.devices.arm` (also installed as `sdl-robot`) |
| `arm.json` | hardware inventory: host, wiring, gripper model and limits |

## Safety

Read `Server/lamp-lab/.claude/rules/motion-safety.md` (on the operator's Mac)
before changing anything here. In short:

- **Motion is dry-run by default.** `live=False` never imports the SDK and never
  opens a socket, so a dry-run plan cannot move the arm.
- **Every commanded pose passes an `Envelope`**, and `driver.send()` accepts only
  a `ValidatedMove`, which only `Envelope.validate()` can build. There is no
  bypass that does not involve editing `safety.py`.
- **Z rise ≤ 10 mm above the taught A1** — raising Z drives the plate *toward*
  the fixed objective. Speed ≤ 30 mm/s. Approach every Z from below.
- **Controller faults are never cleared automatically.** A latched `error_code`
  may record a previous collision. `--clear-errors` is an operator decision.
- **The gripper is jaw-only** (force-limited ≤20 N, 71–150 mm span) and is
  always allowed. It is still actuation: it is logged, and a fault aborts it.

## Gripper span

The BIO Gripper G2 has two control modes, held in the gripper itself across
power cycles, so `gripper.control_mode` in `arm.json` is configuration rather
than a per-call argument:

| `control_mode` | What the firmware does |
|---|---|
| `0` (default) | open/close only. A commanded position is read as a *threshold*: >90 opens fully, ≤90 closes fully |
| `1` | position mode — any span in 71–150 mm, with speed and force |

> **Mode 1 is not a free switch.** Only mode 0 has ever run on this hardware.
> In mode 1, `grip()`/`release()` take the SDK's legacy register path — expected
> to work, but inferred from the SDK source, not observed — so flipping this key
> moves the live-verified plate cycle onto an untested route. Dry-run the cycle
> and watch the first real grip.

```bash
python -m tools.arm gripper read                        # current span, mm
python -m tools.arm gripper set --mm 96 --control-mode 1
```

`arm.set_opening(96)` is the same thing from a procedure. It returns the
commanded `target_mm` beside the `actual_mm` read back from the gripper. A gap
between them means the jaws stopped early — on an object, but equally on a
stall, too little force, or an obstruction. It is telemetry, not a verdict:
concluding "a plate is held" needs a tolerance and an expected width, and that
is a phase-script decision, not a device one.

The mode is asserted **once per session**, not per grip:
`set_bio_gripper_control_mode` reboots the gripper MCU and rewrites state that
outlives the session. A switch is checked twice — the write's return code, and
a read-back off the device — and a gripper that reports a mode other than the
one asked for refuses to actuate. The read-back is corroboration, not proof:
`XArmAPI` exposes no getter (it is reached one layer down, on the inner
object), and the SDK writes the mode to `0x110A` while reading `0x010A`. Still
worth watching the jaws the first time you switch — a failed switch looks
exactly like a position command that opens or closes fully.

Outside position mode `arm.opening()` still returns a number, flagged
`units_verified: False` — the SDK's setter and getter disagree about units
there, and it has not been checked against a ruler. `set_opening` likewise
returns `settled`, which is False when two readings a moment apart disagree;
the SDK's motion wait can return before the jaws have started, so `actual_mm`
is a measurement only when `settled` is True.

Both jaws move together about the tool axis. The gripper takes one span
register, so **there is no way to drive the left and right jaw separately**;
an off-centre grasp is an arm move, or reversed/asymmetric custom fingers.

> Until 2026-08-11 this package recorded that SDK 1.18.4 had no position
> setpoint. It was wrong — see the `history` field in `arm.json`.

## State

Taught poses and derived calibration live in `~/.sdl_lab/robot_arm/` — machine
specific, never committed. `workspace.json` carries the taught A1 that every
envelope is anchored on, so re-teaching it moves the safety box with it.

---

# Teach / route protocol

_Folded in from the retired `docs/robot-arm-protocol.md`._

The robot arm layer is currently a dry-run protocol store. It does not connect
to or move hardware. Real movement stays disabled until a live hardware adapter,
limits, and operator approval are added.

The code lives in `src/sdlsetup/devices/arm` and installs the `sdl-robot` command
(equivalently `python -m sdlsetup.devices.arm`).
Private taught positions and routes are stored outside the repository by
default:

```bash
~/.sdl_lab/robot_arm/workspace.json
```

## Teach Locations

After manually moving the robot to a safe pose, record the measured coordinates:

```bash
sdl-robot teach-location place_1 \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --description "96 well plate pickup area"

sdl-robot teach-location microscope_stage \
  --x 100 --y 0 --z 20 --roll 0 --pitch 0 --yaw 0 \
  --description "microscope placement area"
```

The numbers above are placeholders. Use real taught values only in the ignored
local store, not in committed docs.

## Save Routes

Routes are ordered moves through taught locations:

```bash
sdl-robot route plate_to_scope \
  --from-location place_1 \
  --to-location microscope_stage \
  --description "Carry 96 well plate from place 1 to microscope"
```

Additional safe waypoints can be inserted:

```bash
sdl-robot route plate_to_scope \
  --from-location place_1 \
  --waypoint lift_clearance \
  --waypoint scope_approach \
  --to-location microscope_stage
```

## 96 Well Plate Imaging Protocol

Create the high-level protocol:

```bash
sdl-robot plate-to-microscope image_plate_001 \
  --plate-location place_1 \
  --microscope-location microscope_stage \
  --transfer-route plate_to_scope
```

Inspect without moving hardware:

```bash
sdl-robot dry-run image_plate_001 --protocol
```

The planned steps are:

1. Move to the plate area or approach route.
2. Grip the 96 well plate.
3. Move through the saved transfer route to the microscope.
4. Release the plate.
5. Wait for settling.
6. Capture both cameras through the microscopy photo package.

Live execution is intentionally blocked for now:

```bash
sdl-robot move image_plate_001 --protocol --execute
```

That command returns an error until a hardware adapter is configured.

---

# xArm SDK setup

_Folded in from the retired `docs/xarm-setup.md`._

This repository uses the official `xarm-python-sdk` package for UFACTORY xArm, Lite6, and 850 robots.

Current pinned package:

```text
xarm-python-sdk==1.18.4
```

## Install

Use the existing pyenv environment for this WSL lab machine:

```bash
pyenv activate main
python -m pip install -r requirements.txt
```

## Verify SDK Import

Run the read-only verification script:

```bash
python scripts/check_xarm_sdk.py
```

Expected behavior:

- Imports `XArmAPI`.
- Prints Python and package version information.
- Does not connect to a robot.
- Does not open sockets.
- Does not enable motion, clear errors, set mode/state, home, or move.

## Hardware Safety Boundary

Do not add live robot commands to setup scripts. Any script that connects to an xArm controller or changes robot state must be reviewed as a separate hardware task.

Before live robot work, document:

- Robot model and controller IP address.
- Workspace bounds and coordinate frame assumptions.
- Emergency stop state and physical clearance.
- Tooling, gripper, payload, and sample clearance.
- Whether the command is read-only, state-changing, or motion-producing.

Use placeholders in committed files. Keep real IPs, private calibration values, credentials, and operator-specific settings in ignored local files such as `.env`.
