# Image-topic recording

Record an already published image topic using ROS 2 Humble's `ros2 bag record`.
The publisher runs separately. This tool does not start cameras, generate
previews, open RViz, invoke Docker or establish SSH connections.

## Usage

Source the recorder host's ROS environment first. From the repository root:

```bash
utils/rosbag_recording/record_image_topic.sh /image_topic -o "$HOME/rosbags/run_01"
```

The first argument is the required topic. Remaining arguments go unchanged to
`ros2 bag record`; its option validation and exit status are preserved. The
tool does not enforce an image message type or rewrite the caller's ROS domain,
network or middleware settings. For a remote camera, recording on the camera
host avoids sending the raw stream over the LAN just for storage.

Defaults are SQLite3, no compression, 300-second bag splitting and
`--max-cache-size 67108864` (64 MiB per cache buffer, not a total RAM limit).
Later options override these defaults, for example:

```bash
utils/rosbag_recording/record_image_topic.sh /camera/image --max-bag-duration 60
```

Without `-o`/`--output`, rosbag creates a timestamped directory in the current
working directory. An existing output directory is rejected, not overwritten.
The `--max-bag-duration` option splits files; it does not stop the recording.

Use Ctrl-C to finalize the bag and wait for the command to exit before closing
the terminal or SSH session. Stopping recording does not stop the publisher.
Input loss does not automatically stop recording; rosbag can discover a publisher
that starts later and continue when messages resume. Check the resulting bag
with `ros2 bag info PATH`.

## Verification

From the repository root:

```bash
python3 -m unittest discover -s utils/rosbag_recording/test -v
```

Unit tests use a fake ROS command. Integration tests require a sourced Humble
environment with `rclpy`, `sensor_msgs`, `ros2bag`, SQLite3 storage and PyYAML;
otherwise they are skipped. They use a tiny synthetic image publisher, temporary
bags, localhost-only DDS and domain 175. They verify late discovery, a six-second
input gap, resumed recording, independent Ctrl-C finalization, output naming,
split overrides and preservation of existing output. No camera or RViz is used.
