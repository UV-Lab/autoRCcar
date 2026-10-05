import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1]
DOCKER = shutil.which("docker")


class RvizDockerTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="rviz docker ")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.repo = self.root / "checkout"
        self.helper = self.repo / "utils/rviz_docker"
        self.helper.mkdir(parents=True)
        for name in ("run_rviz_docker.sh", "compose.yaml"):
            if (SOURCE / name).exists():
                shutil.copy2(SOURCE / name, self.helper / name)
        self.script = self.helper / "run_rviz_docker.sh"
        self.cookie = self.root / "session cookie"
        self.cookie.touch()
        self.binary = self.root / "bin"
        self.binary.mkdir()
        docker = self.binary / "docker"
        docker.write_text(
            f"#!{sys.executable}\n"
            "import json, os, subprocess, sys\n"
            "if os.getenv('RVIZ_TEST_RENDER') == '1':\n"
            "    args = sys.argv[1:sys.argv.index('run')]\n"
            "    sys.exit(subprocess.call([os.environ['RVIZ_TEST_DOCKER'], *args, "
            "'config', '--format', 'json']))\n"
            "print(json.dumps({'args': sys.argv[1:], 'cookie': os.getenv('XAUTHORITY')}))\n"
            "sys.exit(int(os.getenv('RVIZ_TEST_EXIT', '0')))\n"
        )
        docker.chmod(0o755)
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith(('RVIZ_', 'COMPOSE_'))}
        self.env.pop("ROS_DOMAIN_ID", None)
        self.env.update(PATH=f"{self.binary}:{os.environ['PATH']}", DISPLAY=":91",
                        XAUTHORITY=str(self.cookie), ROS_LOCALHOST_ONLY="1")

    def run_script(self, *args):
        return subprocess.run(["/bin/bash", str(self.script), *args], env=self.env,
                              cwd=self.root, text=True, capture_output=True, timeout=10)

    def render(self):
        self.env.update(RVIZ_TEST_RENDER="1", RVIZ_TEST_DOCKER=DOCKER)
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)["services"]["rviz"]

    def test_disposable_invocation_and_command_forwarding(self):
        result = self.run_script("rviz2", "-d", "/workspace/profile with spaces.rviz")
        self.assertEqual(result.returncode, 0, result.stderr)
        command = json.loads(result.stdout)
        self.assertEqual(command["args"], [
            "compose", "--project-directory", str(self.repo), "--env-file", "/dev/null",
            "-f", str(self.helper / "compose.yaml"), "run", "--rm", "--no-deps", "rviz",
            "rviz2", "-d", "/workspace/profile with spaces.rviz",
        ])
        self.assertEqual(command["cookie"], str(self.cookie))

    def test_help_and_missing_display_or_cookie(self):
        self.assertEqual(self.run_script("--help").returncode, 0)
        for key, value in (("DISPLAY", ""), ("XAUTHORITY", str(self.root / "missing"))):
            with self.subTest(key=key):
                old = self.env[key]
                self.env[key] = value
                result = self.run_script()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("rviz_docker:", result.stderr)
                self.env[key] = old

    def test_runtime_cookie_fallback(self):
        self.env.pop("XAUTHORITY")
        self.env["HOME"] = str(self.root)
        runtime = self.root / "runtime"
        (runtime / "gdm").mkdir(parents=True)
        cookie = runtime / "gdm/Xauthority"
        cookie.touch()
        self.env["XDG_RUNTIME_DIR"] = str(runtime)
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["cookie"], str(cookie))

    def test_child_failure_is_preserved(self):
        self.env["RVIZ_TEST_EXIT"] = "7"
        self.assertEqual(self.run_script().returncode, 7)

    def test_local_settings_must_be_a_file(self):
        (self.helper / ".env").mkdir()
        result = self.run_script()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Unreadable settings", result.stderr)

    @unittest.skipUnless(DOCKER, "Requires Docker Compose for config rendering only")
    def test_local_values_from_other_working_directory(self):
        workspace = self.root / "my packages"
        preset = self.root / "my preset.rviz"
        (self.helper / ".env").write_text(
            f"ROS_DOMAIN_ID=42\nRVIZ_WORKSPACE_DIR={workspace}\nRVIZ_CONFIG={preset}\n"
        )
        service = self.render()
        self.assertEqual(service["environment"]["ROS_DOMAIN_ID"], "42")
        self.assertEqual(service["environment"]["ROS_LOCALHOST_ONLY"], "0")
        self.assertEqual(service["user"], f"{os.getuid()}:{os.getgid()}")
        self.assertEqual(service["network_mode"], "host")
        self.assertEqual(service["gpus"][0]["count"], -1)
        mounts = {item["target"]: item for item in service["volumes"]}
        self.assertEqual(mounts["/workspace"]["source"], str(workspace))
        self.assertEqual(mounts["/rviz-config/default.rviz"]["source"], str(preset))
        self.assertTrue(mounts["/rviz-config/default.rviz"]["read_only"])
        self.assertFalse(mounts["/workspace"]["bind"]["create_host_path"])

    @unittest.skipUnless(DOCKER, "Requires Docker Compose for config rendering only")
    def test_environment_overrides_file_and_relative_profile_uses_repo_root(self):
        (self.helper / ".env").write_text("ROS_DOMAIN_ID=42\nRVIZ_CONFIG=config/raw.rviz\n")
        self.env["ROS_DOMAIN_ID"] = "43"
        self.env["RVIZ_WORKSPACE_DIR"] = str(self.root)
        service = self.render()
        self.assertEqual(service["environment"]["ROS_DOMAIN_ID"], "43")
        preset = next(item for item in service["volumes"] if item["target"] == "/rviz-config/default.rviz")
        self.assertEqual(preset["source"], str(self.repo / "config/raw.rviz"))

    @unittest.skipUnless(DOCKER, "Requires Docker Compose for config rendering only")
    def test_default_profile_without_local_file(self):
        service = self.render()
        self.assertEqual(service["environment"]["ROS_DOMAIN_ID"], "0")
        preset = next(item for item in service["volumes"] if item["target"] == "/rviz-config/default.rviz")
        self.assertEqual(preset["source"], str(self.repo / "ros2/src/ocam_publisher/config/rviz/image_topic_rviz_15hz.rviz"))

    @unittest.skipUnless(DOCKER, "Requires Docker Compose for config rendering only")
    def test_invalid_local_file_fails_before_running_container(self):
        (self.helper / ".env").write_text('ROS_DOMAIN_ID="unfinished\n')
        self.env.update(RVIZ_TEST_RENDER="1", RVIZ_TEST_DOCKER=DOCKER)
        result = self.run_script()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(".env", result.stderr)
