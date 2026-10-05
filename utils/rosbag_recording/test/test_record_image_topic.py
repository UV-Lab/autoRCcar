import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "record_image_topic.sh"


class RecordImageTopicTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="image recording ")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        ros = self.root / "ros2"
        ros.write_text(
            f"#!{sys.executable}\n"
            "import json, os, sys\n"
            "print(json.dumps({'args': sys.argv[1:], 'domain': os.getenv('ROS_DOMAIN_ID'), "
            "'localhost': os.getenv('ROS_LOCALHOST_ONLY')}))\n"
            "sys.exit(int(os.getenv('RECORDER_TEST_EXIT', '0')))\n"
        )
        ros.chmod(0o755)
        self.env = {**os.environ, "PATH": f"{self.root}:{os.environ['PATH']}",
                    "ROS_DOMAIN_ID": "42", "ROS_LOCALHOST_ONLY": "1"}

    def run_script(self, *args, **options):
        return subprocess.run(["/bin/bash", str(SCRIPT), *args], env=self.env,
                              text=True, capture_output=True, timeout=3, **options)

    def test_defaults_arguments_and_ros_environment(self):
        result = self.run_script("/other_camera/image", "-o", "/tmp/run with spaces",
                                 "--max-bag-duration", "60")
        self.assertEqual(result.returncode, 0, result.stderr)
        command = json.loads(result.stdout)
        self.assertEqual(command["args"], [
            "bag", "record", "--storage", "sqlite3", "--compression-mode", "none",
            "--max-bag-duration", "300", "--max-cache-size", "67108864",
            "/other_camera/image", "-o", "/tmp/run with spaces", "--max-bag-duration", "60",
        ])
        self.assertEqual((command["domain"], command["localhost"]), ("42", "1"))

    def test_output_is_left_to_rosbag_when_omitted(self):
        result = self.run_script("/camera/image", cwd=self.root)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("-o", json.loads(result.stdout)["args"])

    def test_help_and_required_topic_without_ros(self):
        self.env["PATH"] = str(self.root / "absent")
        self.assertEqual(self.run_script("--help").returncode, 0)
        for args in ((), ("",), ("-o", "bag")):
            with self.subTest(args=args):
                result = self.run_script(*args)
                self.assertEqual(result.returncode, 2)
                self.assertIn("TOPIC", result.stderr)

    def test_missing_ros_is_actionable(self):
        self.env["PATH"] = str(self.root / "absent")
        result = self.run_script("/camera/image")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("source", result.stderr)

    def test_rosbag_failure_status_is_preserved(self):
        self.env["RECORDER_TEST_EXIT"] = "7"
        self.assertEqual(self.run_script("/camera/image").returncode, 7)
