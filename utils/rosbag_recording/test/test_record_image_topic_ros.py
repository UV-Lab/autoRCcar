import importlib.util
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "record_image_topic.sh"
PUBLISHER = Path(__file__).with_name("publish_test_image.py")
ROS_AVAILABLE = shutil.which("ros2") and importlib.util.find_spec("rclpy")


@unittest.skipUnless(ROS_AVAILABLE, "Requires a sourced ROS 2 Humble environment")
class RecordImageTopicRosTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="image bag acceptance ")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.env = {**os.environ, "ROS_LOCALHOST_ONLY": "1", "ROS_DOMAIN_ID": "175",
                    "PYTHONUNBUFFERED": "1"}
        self.topic = "/independent_camera/image"

    def start(self, args):
        log = tempfile.TemporaryFile(mode="w+")
        self.addCleanup(log.close)
        process = subprocess.Popen(args, cwd=self.root, env=self.env,
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.addCleanup(self.reap, process)
        return process, log

    def reap(self, process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGCONT)
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)

    def wait_log(self, process, log, text):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            log.seek(0)
            output = log.read()
            if text in output:
                return
            self.assertIsNone(process.poll(), output)
            time.sleep(0.05)
        self.fail(f"Timed out waiting for {text}: {output}")

    def read_bag(self, bag):
        import yaml

        metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        self.assertEqual(metadata["storage_identifier"], "sqlite3")
        self.assertFalse(metadata["compression_format"])
        self.assertGreater(metadata["message_count"], 0)
        topics = metadata["topics_with_message_count"]
        self.assertEqual([item["topic_metadata"]["name"] for item in topics], [self.topic])
        self.assertEqual(topics[0]["topic_metadata"]["type"], "sensor_msgs/msg/Image")
        timestamps = []
        for filename in metadata["relative_file_paths"]:
            with sqlite3.connect(bag / filename) as database:
                self.assertEqual(database.execute("pragma integrity_check").fetchone()[0], "ok")
                timestamps.extend(row[0] for row in database.execute("select timestamp from messages"))
        return metadata, sorted(timestamps)

    def test_late_publisher_gap_resume_and_independent_stop(self):
        bag = self.root / "bag with spaces"
        recorder, log = self.start(["/bin/bash", str(SCRIPT), self.topic, "-o", str(bag),
                                    "--max-bag-duration", "1"])
        self.wait_log(recorder, log, "Recording...")
        publisher, publisher_log = self.start([sys.executable, str(PUBLISHER), self.topic])
        self.wait_log(publisher, publisher_log, "subscriber-connected")
        time.sleep(1.5)
        os.killpg(publisher.pid, signal.SIGSTOP)
        time.sleep(6)
        self.assertIsNone(recorder.poll(), "Input loss must not automatically stop recording")
        os.killpg(publisher.pid, signal.SIGCONT)
        time.sleep(1.5)
        os.killpg(recorder.pid, signal.SIGINT)
        self.assertEqual(recorder.wait(timeout=10), 0)
        self.assertIsNone(publisher.poll(), "Stopping the recorder must not stop the publisher")
        metadata, timestamps = self.read_bag(bag)
        self.assertGreaterEqual(len(metadata["relative_file_paths"]), 2)
        self.assertTrue(any(b - a > 5_000_000_000 for a, b in zip(timestamps, timestamps[1:])))

    def test_default_timestamped_output(self):
        publisher, publisher_log = self.start([sys.executable, str(PUBLISHER), self.topic])
        recorder, log = self.start(["/bin/bash", str(SCRIPT), self.topic])
        self.wait_log(recorder, log, "Recording...")
        self.wait_log(publisher, publisher_log, "subscriber-connected")
        time.sleep(0.5)
        os.killpg(recorder.pid, signal.SIGINT)
        self.assertEqual(recorder.wait(timeout=10), 0)
        bags = list(self.root.glob("rosbag2_*"))
        self.assertEqual(len(bags), 1)
        self.read_bag(bags[0])

    def test_existing_output_is_preserved(self):
        bag = self.root / "existing"
        bag.mkdir()
        sentinel = bag / "keep.txt"
        sentinel.write_text("keep this recording")
        result = subprocess.run(["/bin/bash", str(SCRIPT), self.topic, "-o", str(bag)],
                                cwd=self.root, env=self.env, text=True,
                                capture_output=True, timeout=8)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stdout + result.stderr)
        self.assertEqual(sentinel.read_text(), "keep this recording")
        self.assertEqual(list(bag.iterdir()), [sentinel])
