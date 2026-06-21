// lib/config/process_definitions.dart
//
// list of processes controlled in the RUN tab.
// referenced by both run_panel.dart (UI) and gcs_controller.dart (LLM action whitelist validation),
// so extracted into a separate config file outside the widget layer.

/// (process id, display name, description)
const List<(String, String, String)> kProcesses = [
  ('gscam', 'GSCAM', 'ros2 run gscam gscam_node'),
  ('livox', 'Livox MID360', 'ros2 launch livox_ros_driver2 msg_MID360_launch.py'),
  ('ublox', 'Ublox F9R', 'ros2 run autorccar_ubloxf9r ubloxf9r'),
  ('lio_sam', 'LIO-SAM', 'ros2 launch lio_sam run.launch.py'),
  ('ins_gnss', 'INS/GNSS Nav', 'ros2 launch autorccar_ins_gnss ins_gnss_nav.launch.py'),
  ('planning_control', 'Planning & Control', 'ros2 launch autorccar_planning_control planning_control.launch.py'),
  ('hardware_control', 'Hardware Control', 'ros2 launch autorccar_hardware_control hardware_control.launch.py'),
  ('costmap', 'Costmap', 'ros2 launch autorccar_costmap costmap.launch.py'),
];

/// rosbag record (separate row, displayed at the bottom)
const (String, String, String) kRosbag =
    ('rosbag', 'Rosbag Record', 'ros2 bag record -a -o ~/bags/rosbag2_<timestamp>');

/// valid process id set for LLM action whitelist validation
final Set<String> kValidProcessIds = {
  ...kProcesses.map((p) => p.$1),
  kRosbag.$1,
};
