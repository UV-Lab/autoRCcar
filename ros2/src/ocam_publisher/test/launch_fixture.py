import os
from pathlib import Path
import sys
from unittest.mock import patch

import ament_index_python.packages
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


package = Path(__file__).resolve().parents[1]
real_node = launch_ros.actions.Node
real_share_directory = ament_index_python.packages.get_package_share_directory


def test_node(**options):
    if options["package"] == "ocam_publisher":
        options["package"] = None
        options["executable"] = os.environ["OCAM_TEST_PUBLISHER"]
        options["parameters"] += [{"device_path": "/dev/null", "frame_timeout_sec": 0.3}]
        return real_node(**options)
    name = options["name"]
    return ExecuteProcess(
        cmd=[sys.executable, "-u", "-c",
             "import signal; "
             "signal.signal(signal.SIGINT, lambda *_: exit(0)); "
             f"print('helper-ready:{name}', flush=True); signal.pause()"],
        name=name, condition=options["condition"], output="screen",
    )


def share_directory(name):
    return str(package) if name == "ocam_publisher" else real_share_directory(name)


if __name__ == "__main__":
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / "launch" / "ocam_publish.launch.py")),
            launch_arguments={"publish_rviz": sys.argv[1]}.items(),
        ),
    ]))
    with patch.object(launch_ros.actions, "Node", test_node), patch.object(
        ament_index_python.packages, "get_package_share_directory", share_directory
    ):
        raise SystemExit(service.run())
