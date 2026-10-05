import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


class PublisherProcessTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.events = Path(self.directory.name) / "events"
        self.log = tempfile.TemporaryFile(mode="w+")
        self.addCleanup(self.log.close)

    def start(self, scenario, timeout=None):
        command = [os.environ["OCAM_TEST_PUBLISHER"], "--ros-args", "-p", "device_path:=/dev/null"]
        if timeout is not None:
            command += ["-p", f"frame_timeout_sec:={timeout}"]
        self.process = subprocess.Popen(
            command, stdout=self.log, stderr=subprocess.STDOUT,
            env={**os.environ, "OCAM_TEST_SCENARIO": scenario,
                 "OCAM_TEST_EVENTS": str(self.events), "ROS_LOCALHOST_ONLY": "1",
                 "ROS_DOMAIN_ID": "173"},
        )
        self.addCleanup(self.reap, self.process)

    def reap(self, process):
        if process.poll() is None:
            process.kill()
        process.wait(timeout=3)

    def output(self):
        self.log.seek(0)
        return self.log.read()

    def wait_event(self, name):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if self.events.exists() and name in self.events.read_text().splitlines():
                return
            if self.process.poll() is not None:
                self.fail(self.output())
            time.sleep(0.02)
        self.fail(f"Missing event {name}: {self.output()}")

    def test_start_failure_returns_failure_and_releases_camera(self):
        self.start("start_failure")
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertIn("closed", self.events.read_text())

    def test_default_missing_frame_limit_is_five_seconds(self):
        self.start("timeout")
        self.wait_event("read")
        started = time.monotonic()
        self.assertNotEqual(self.process.wait(timeout=6.5), 0)
        self.assertGreater(time.monotonic() - started, 4.5)
        self.assertIn("No complete frame", self.output())
        self.assertIn("closed", self.events.read_text())

    def test_configurable_missing_frame_limit(self):
        for scenario in ("timeout", "partial"):
            with self.subTest(scenario=scenario):
                self.start(scenario, 0.3)
                self.assertNotEqual(self.process.wait(timeout=3), 0)
                self.assertIn("No complete frame", self.output())

    def test_failed_restart_ends_capture(self):
        self.start("restart_failure")
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertIn("Failed to restart", self.output())

    def test_transient_failures_reset_deadline_when_frames_resume(self):
        self.start("transient", 0.5)
        self.wait_event("frame")
        time.sleep(0.8)
        self.assertIsNone(self.process.poll(), self.output())
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=3), 0, self.output())
        self.assertGreaterEqual(self.events.read_text().splitlines().count("frame"), 3)

    def test_sigint_stops_active_capture(self):
        self.start("frames")
        self.wait_event("frame")
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=3), 0, self.output())
        self.assertEqual(self.events.read_text().splitlines()[-2:], ["stop", "closed"])

    def test_interrupted_wait_does_not_restart_camera(self):
        self.start("interrupted", 0.3)
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertEqual(self.events.read_text().splitlines().count("start"), 1)

    def test_failed_stop_returns_failure(self):
        self.start("stop_failure")
        self.wait_event("frame")
        self.process.send_signal(signal.SIGINT)
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertIn("Failed to stop", self.output())

    def test_failed_recovery_stop_does_not_restart(self):
        self.start("recovery_stop_failure")
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertIn("Failed to stop camera for recovery", self.output())
        self.assertEqual(self.events.read_text().splitlines().count("start"), 1)

    def test_capture_exception_stops_and_releases_camera(self):
        self.start("exception")
        self.assertNotEqual(self.process.wait(timeout=3), 0)
        self.assertIn("test frame failure", self.output())
        self.assertEqual(self.events.read_text().splitlines()[-2:], ["stop", "closed"])

    def test_invalid_timeout_is_rejected_before_camera_open(self):
        for value in ("0.0", "-1.0", ".nan", ".inf"):
            with self.subTest(value=value):
                self.start("frames", value)
                self.assertNotEqual(self.process.wait(timeout=3), 0)
                self.assertIn("frame_timeout_sec", self.output())
                self.assertFalse(self.events.exists())
