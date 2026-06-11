# WSL SDL Lab Instructions

This workspace is for a WSL-based self-driving lab environment. Robotic arms, microscopes, instruments, cameras, and local control services may be connected here.

## Instruction Order

1. User request and current conversation
2. This `AGENTS.md`
3. `.codex/rules/*.md`
4. Tool, skill, and general Codex instructions

## Operating Principles

- Keep changes small, reversible, and source-grounded.
- Do not invent file paths, hardware state, calibration values, device IDs, or experiment results.
- If a file, device, process, port, or service is missing, say so directly.
- Mark inferred relationships as inference.
- Prefer logs, configs, command output, and hardware status checks over guesses.
- Treat this as a multi-user lab setup. Do not store private keys, personal tokens, passwords, user-specific credentials, or machine-specific secrets in the repository.

## Python Environment

- Use the pyenv virtualenv named `main` for this repository.
- The repo-local `.python-version` should stay set to `main`.
- Install Python packages with `python -m pip ...` after confirming `pyenv version` resolves to `main`.
- Do not create or commit project-local virtualenvs, Conda environments, or user-specific interpreter paths unless explicitly requested.

## Hardware Safety

- Treat physical hardware as stateful and potentially hazardous.
- Do not move robotic arms, actuators, stages, pumps, valves, lasers, lights, heaters, or microscopes unless the user explicitly asks.
- Before any command that can cause motion, heat, pressure, illumination, fluid flow, sample contact, or instrument state changes, identify the target device and intended action.
- Prefer dry-run, status, home, idle, simulation, or read-only diagnostics when available.
- Stop and ask before destructive cleanup, calibration overwrite, firmware changes, or persistent device configuration changes.
- For xArm work, SDK import checks are allowed, but controller connections, `motion_enable`, `set_mode`, `set_state`, homing, gripper commands, and movement commands require explicit live-hardware approval.

## Project Layout

- `.codex/rules/` contains local operating rules for this lab.
- Add concrete paths, service names, device maps, runbooks, and experiment structure here as the lab stack becomes real.
- Do not store secrets, tokens, credentials, private keys, or private calibration data in agent rules.
- Shared examples should use templates such as `.env.example`, placeholder device IDs, and documented setup steps instead of real credentials.
- Keep real robot IPs, private calibration values, and operator-specific hardware settings in ignored local files, not committed docs or scripts.

## Troubleshooting

- Preserve exact command lines, paths, timestamps, versions, ports, device IDs, and error text.
- If a failure repeats, inspect logs, process state, generated files, config, and device/service status before retrying.
- For dependency, OS, driver, firmware, API, or hardware behavior that may have changed, verify against current primary sources.
