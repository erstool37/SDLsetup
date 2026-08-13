from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tools.microscope import capture


class RuntimeConfigTests(unittest.TestCase):
    def test_root_yaml_supplies_live_session_defaults_and_cli_overrides(self) -> None:
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            camera_config = root / "cameras.json"
            camera_config.write_text("{}", encoding="utf-8")
            runtime_config = root / "config.yaml"
            runtime_config.write_text(
                "\n".join(
                    [
                        f"camera_config: {camera_config}",
                        "output_dir: /tmp/sdl-captures",
                        "live_session:",
                        "  camera: tis_dfk33ux264",
                        "  stream_interval_s: 2.5",
                        "  save_interval_s: 45",
                        "  refresh_ms: 250",
                        "  port: 8777",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            args = capture.parse_args(
                [
                    "--runtime-config",
                    str(runtime_config),
                    "live-session",
                    "start",
                ]
            )

            self.assertEqual(args.config, camera_config)
            self.assertEqual(args.camera, "tis_dfk33ux264")
            self.assertEqual(args.stream_interval_s, 2.5)
            self.assertEqual(args.save_interval_s, 45.0)
            self.assertEqual(args.refresh_ms, 250)
            self.assertEqual(args.port, 8777)

            override_args = capture.parse_args(
                [
                    "--runtime-config",
                    str(runtime_config),
                    "live-session",
                    "start",
                    "--stream-interval-s",
                    "1.25",
                    "--refresh-ms",
                    "750",
                ]
            )

            self.assertEqual(override_args.stream_interval_s, 1.25)
            self.assertEqual(override_args.refresh_ms, 750)

    def test_root_yaml_can_supply_schedule_frequency(self) -> None:
        with TemporaryDirectory() as tmp:
            runtime_config = Path(tmp) / "config.yaml"
            runtime_config.write_text(
                "\n".join(
                    [
                        "schedule:",
                        "  camera: all",
                        "  interval_s: 12.5",
                        "  start_timeout_s: 7",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            args = capture.parse_args(
                [
                    "--runtime-config",
                    str(runtime_config),
                    "schedule",
                    "--count",
                    "2",
                ]
            )

            self.assertEqual(args.camera, "all")
            self.assertEqual(args.interval_s, 12.5)
            self.assertEqual(args.start_timeout_s, 7.0)

    def test_root_yaml_supplies_live_session_status_paths(self) -> None:
        with TemporaryDirectory() as tmp:
            runtime_config = Path(tmp) / "config.yaml"
            runtime_config.write_text(
                "\n".join(
                    [
                        "live_session:",
                        "  stream_dir: /tmp/custom-sdl-stream",
                        "  pid_file: /tmp/custom-sdl-live.pid",
                        "  log_file: /tmp/custom-sdl-live.log",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            args = capture.parse_args(
                [
                    "--runtime-config",
                    str(runtime_config),
                    "live-session",
                    "status",
                ]
            )

            self.assertEqual(args.stream_dir, Path("/tmp/custom-sdl-stream"))
            self.assertEqual(args.pid_file, Path("/tmp/custom-sdl-live.pid"))
            self.assertEqual(args.log_file, Path("/tmp/custom-sdl-live.log"))


if __name__ == "__main__":
    unittest.main()
