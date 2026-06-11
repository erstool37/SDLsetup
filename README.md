# SDLsetup

WSL-based setup for a self-driving lab environment.

This repository is intended to hold lightweight setup notes, agent rules, runbooks, and small configuration templates for lab operation. Connected systems may include robotic arms, microscopes, cameras, instruments, and local control services.

Do not commit secrets, runtime state, large datasets, model checkpoints, raw images, experiment output, or private calibration data.

## Codex Rules

- Start with `AGENTS.md` for local operating instructions.
- Local rules live in `.codex/rules/`.
- Hardware commands should default to read-only discovery unless the user explicitly approves live action.
- Use SSH for Git access in multi-user lab setups; see `docs/git-ssh-setup.md`.

## Repository URL

https://github.com/erstool37/SDLsetup
