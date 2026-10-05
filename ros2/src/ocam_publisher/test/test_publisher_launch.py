import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


class PublisherLaunchTest(unittest.TestCase):
    def run_publisher(self, scenario, visualization, interrupt=False):
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode="w+") as log:
            events = Path(directory) / "events"
            process = subprocess.Popen(
                [sys.executable, str(Path(__file__).with_name("launch_fixture.py")),
                 visualization],
                env={**os.environ, "OCAM_TEST_SCENARIO": scenario,
                     "OCAM_TEST_EVENTS": str(events),
                     "ROS_LOCALHOST_ONLY": "1", "ROS_DOMAIN_ID": "174"},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            try:
                if interrupt:
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        log.seek(0)
                        output = log.read()
                        helpers = 0 if visualization == "false" else 2
                        if (events.exists() and "frame" in events.read_text().splitlines()
                                and output.count("helper-ready:") == helpers):
                            break
                        time.sleep(0.05)
                    else:
                        self.fail("Publisher and preview did not start")
                    # Terminal Ctrl-C reaches the foreground process group, not just launch.
                    os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
                log.seek(0)
                output = log.read()
                self.assertIn("closed", events.read_text().splitlines())
                self.assertNotIn("escalating", output)
                self.assertEqual(output.count("helper-ready:"), 0 if visualization == "false" else 2)
                if not interrupt:
                    self.assertIn("No complete frame", output)
                with self.assertRaises(ProcessLookupError):
                    os.killpg(process.pid, 0)
            except Exception:
                log.seek(0)
                print(log.read())
                raise
            finally:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)

    def test_camera_failure_without_preview(self):
        self.run_publisher("frames_then_timeout", "false")

    def test_camera_failure_stops_preview_children(self):
        self.run_publisher("frames_then_timeout", "true")

    def test_ctrl_c_stops_camera_and_preview(self):
        self.run_publisher("frames", "true", interrupt=True)
