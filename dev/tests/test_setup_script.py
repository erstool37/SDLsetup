from __future__ import annotations

import os
import subprocess
import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SETUP_SH = PROJECT_ROOT / "setup.sh"


class SetupScriptTests(unittest.TestCase):
    def run_setup_with_fake_pyenv(self, *, running: bool = False) -> list[str]:
        with TemporaryDirectory() as tmp:
            fake_dir = Path(tmp)
            log_path = fake_dir / "pyenv.log"
            pyenv_path = fake_dir / "pyenv"
            # Matches on the WHOLE argument string, not on $1..$4. The capture
            # entry point is now `python -m tools.microscope.capture`, so a
            # positional match would be swallowed by the generic `exec python`
            # branch and `live-session status` would return nothing -- which
            # reads as "not running" and starts a duplicate session on :8766.
            fake_pyenv = """
                #!/usr/bin/env bash
                set -euo pipefail
                printf '%s\n' "$*" >> "$PYENV_LOG"
                args="$*"
                case "$args" in
                  *"live-session status")
                    if [[ "${FAKE_LIVE_RUNNING:-0}" == "1" ]]; then
                      printf '{"running": true}\n'
                    else
                      printf '{"running": false}\n'
                    fi
                    exit 0
                    ;;
                  *"live-session start"*|*"lan-info"*|*"pip install"*)
                    exit 0
                    ;;
                esac
                printf 'unexpected command: %s\n' "$args" >&2
                exit 64
            """
            pyenv_path.write_text(textwrap.dedent(fake_pyenv).lstrip(), encoding="utf-8")
            pyenv_path.chmod(0o755)
            env = os.environ.copy()
            env.update(
                {
                    "PYENV_BIN": str(pyenv_path),
                    "PYENV_LOG": str(log_path),
                    "FAKE_LIVE_RUNNING": "1" if running else "0",
                }
            )

            subprocess.run(
                ["bash", str(SETUP_SH)],
                cwd=PROJECT_ROOT,
                env=env,
                text=True,
                capture_output=True,
                check=True,
                timeout=10,
            )
            return log_path.read_text(encoding="utf-8").splitlines()

    # The repo is no longer installed: pyproject.toml was retired, so there is
    # no `microscope-photo` command. setup.sh installs only the third-party
    # dependencies and calls the module directly. These assertions are the
    # contract -- they caught the change when it was made.
    CAPTURE = "exec python -m tools.microscope.capture"

    def test_setup_installs_deps_and_starts_live_session_when_not_running(self) -> None:
        commands = self.run_setup_with_fake_pyenv(running=False)

        self.assertIn("exec python -m pip install -r requirements.txt", commands)
        self.assertIn(f"{self.CAPTURE} live-session status", commands)
        self.assertIn(f"{self.CAPTURE} live-session start --lan", commands)
        self.assertIn(f"{self.CAPTURE} lan-info --lan --port 8766", commands)

    def test_setup_does_not_start_duplicate_live_session(self) -> None:
        commands = self.run_setup_with_fake_pyenv(running=True)

        self.assertIn("exec python -m pip install -r requirements.txt", commands)
        self.assertIn(f"{self.CAPTURE} live-session status", commands)
        self.assertNotIn(f"{self.CAPTURE} live-session start --lan", commands)
        self.assertIn(f"{self.CAPTURE} lan-info --lan --port 8766", commands)


if __name__ == "__main__":
    unittest.main()
