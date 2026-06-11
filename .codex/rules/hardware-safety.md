# Hardware Safety Rules

- Assume connected lab hardware can move, heat, illuminate, pressurize, aspirate, dispense, image, or collide.
- Read-only discovery is allowed unless credentials, external services, or privileged access are required.
- Ask before commands that can physically actuate hardware or alter persistent device state.
- Prefer simulation, dry-run, status, and limits checks before live actions.
- Confirm coordinate frames, units, workspace bounds, sample clearance, and emergency stop assumptions before motion planning.
- Never fabricate calibration, homing, kinematic, deck, stage, camera, or safety-limit data.
- Do not commit private calibration data, proprietary device credentials, controller keys, or user-specific access files.
