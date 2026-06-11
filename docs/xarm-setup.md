# xArm Python SDK Setup

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
