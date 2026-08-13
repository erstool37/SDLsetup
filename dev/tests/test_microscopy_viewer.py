from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import http.server
import json
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from urllib.request import Request, urlopen

from tools.microscope import capture, viewer


class SaveLatestImagesTests(unittest.TestCase):
    def test_saves_latest_displayed_images_and_metadata_to_user_saved_dir(self) -> None:
        with TemporaryDirectory() as tmp:
            root = Path(tmp) / "stream"
            root.mkdir()
            save_dir = Path(tmp) / "user-saved"

            tis = root / "current_tis_dfk33ux264.jpg"
            tis.write_bytes(b"tis-frame")
            tis.with_suffix(tis.suffix + ".json").write_text(
                json.dumps({"camera": "tis_dfk33ux264", "batch_id": "current"}),
                encoding="utf-8",
            )
            leica = root / "vertical_online_image.jpg"
            leica.write_bytes(b"leica-frame")

            result = viewer.save_latest_images(
                root=root,
                save_dir=save_dir,
                timestamp="20260625_140000",
            )

            self.assertEqual(result["save_dir"], str(save_dir))
            self.assertEqual(result["missing"], [])
            saved = {item["camera"]: Path(item["path"]) for item in result["saved"]}
            self.assertEqual(set(saved), {"leica", "tis"})
            self.assertEqual(saved["tis"].name, "20260625_140000_tis.jpg")
            self.assertEqual(saved["leica"].name, "20260625_140000_leica.jpg")
            self.assertEqual(saved["tis"].read_bytes(), b"tis-frame")
            self.assertEqual(saved["leica"].read_bytes(), b"leica-frame")

            metadata_path = saved["tis"].with_suffix(saved["tis"].suffix + ".json")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["camera"], "tis_dfk33ux264")
            self.assertEqual(metadata["saved_camera_slot"], "tis")
            self.assertEqual(metadata["saved_from"], str(tis))

    def test_save_latest_endpoint_uses_monitor_user_saved_dir(self) -> None:
        with TemporaryDirectory() as tmp:
            root = Path(tmp) / "stream"
            root.mkdir()
            save_dir = Path(tmp) / "user-saved"
            (root / "current_tis_dfk33ux264.jpg").write_bytes(b"tis-frame")

            class TestHandler(viewer.MonitorHandler):
                pass

            TestHandler.quiet = True
            TestHandler.refresh_ms = 2000
            TestHandler.user_saved_dir = save_dir
            TestHandler.root = root

            server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), TestHandler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                request = Request(
                    f"http://127.0.0.1:{server.server_port}/api/save-latest",
                    method="POST",
                )
                with urlopen(request, timeout=5) as response:
                    payload = json.loads(response.read().decode("utf-8"))
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=5)

            self.assertEqual(payload["save_dir"], str(save_dir))
            self.assertEqual(payload["saved"][0]["camera"], "tis")
            self.assertTrue(Path(payload["saved"][0]["path"]).exists())

    def test_root_yaml_supplies_user_saved_dir_to_monitor_and_live_session(self) -> None:
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            runtime_config = root / "config.yaml"
            runtime_config.write_text(
                "\n".join(
                    [
                        "monitor:",
                        "  output_dir: captures",
                        "  user_saved_dir: user-saved",
                        "live_session:",
                        "  user_saved_dir: /tmp/sdl-user-saved",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            monitor_args = capture.parse_args(["--runtime-config", str(runtime_config), "monitor"])
            live_args = capture.parse_args(
                ["--runtime-config", str(runtime_config), "live-session", "start"]
            )

            self.assertEqual(monitor_args.output_dir, root / "captures")
            self.assertEqual(monitor_args.user_saved_dir, root / "user-saved")
            self.assertEqual(live_args.user_saved_dir, Path("/tmp/sdl-user-saved"))

            live_command = capture._live_session_run_command(live_args)
            self.assertIn("--user-saved-dir", live_command)
            self.assertIn("/tmp/sdl-user-saved", live_command)


if __name__ == "__main__":
    unittest.main()
