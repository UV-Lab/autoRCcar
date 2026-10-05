# ROS 2 wrapper for Withrobot oCam-1CGN-U-T

By default, the publisher captures the camera's native 1280 x 960 `GRBG` Bayer
stream at 30 fps, demosaics it, and downsamples it once with OpenCV
`INTER_AREA`. It publishes the existing `/image_topic` as a 640 x 480 `rgb8`
`sensor_msgs/Image`, so RViz, rosbag, and existing downstream consumers retain
the lower-bandwidth contract. A `hardware_binned` capture mode is also
available for direct 640 x 480 camera acquisition.

The OpenCV viewer in `cpp/` uses the same native-capture and 640 x 480 display
path.  It is intended for local exposure/gain tuning.

# Dependencies

- libv4l       (video for linux Two)
- libudev       (udev, the device manager for the Linux kernel)
- libopencv-dev (development files for OpenCV)

```
sudo apt-get install libv4l-dev libudev-dev libopencv-dev
```

# Configuration

`config/params.yaml` controls the ROS publisher:

- `capture_mode`: chooses the camera acquisition path:
  - `native_downsample` (default): 1280 x 960 `GRBG` capture, demosaic, then
    one `INTER_AREA` downsample to 640 x 480.
  - `hardware_binned`: 640 x 480 `GRBG` capture and demosaic only.
- `OutputWidth` / `OutputHeight`: the fixed published RGB contract; both must
  remain 640 x 480.
- `FPS`: defaults to 30.
- `frame_timeout_sec`: positive finite seconds without a complete frame before
  capture fails (default `5.0`). Checks follow frame reads, which wait up to one
  second; this is not a hard real-time deadline for blocked driver operations.
- `frame_id`: defaults to `ocam_optical_frame`.
- `Exposure`: V4L2 100 microsecond units; the default `128` is 12.8 ms.
- `Brightness`: manual camera gain; the detected camera supports 0 through
  255 and defaults to 128 here.
- `Auto exposure mode`: `1` is manual and `3` is aperture-priority mode.

At startup the publisher fails rather than silently continuing if the camera
does not negotiate the requested `GRBG` capture format or rejects a requested
control.

# Run

Build on the camera host from the ROS workspace after sourcing
`/opt/ros/humble/setup.bash`:

```bash
colcon build --packages-select ocam_publisher --symlink-install
source install/setup.bash
ros2 launch ocam_publisher ocam_publish.launch.py
```

`ocam_publish.launch.py` starts capture and, by default, the preview branch:

| Topic | Message | Default rate |
| --- | --- | --- |
| `/image_topic` | `sensor_msgs/msg/Image`, 640 x 480 `rgb8` | 30 Hz |
| `/image_topic_rviz/compressed` | `sensor_msgs/msg/CompressedImage`, JPEG quality 80 | 15 Hz |

The preview branch drops every other raw frame and compresses the remaining
frames. It does not reduce the raw publication rate. Set `publish_rviz:=false`
to disable that branch, or `params_file:=/absolute/path/to/params.yaml` to use
another camera parameter file.

Normal SIGINT shutdown stops capture and returns success. Missing frames are
retried within the configured limit; failed start/recovery/stop returns failure.
Camera exit shuts down its preview processes.

# Recording

To record `/image_topic`, keep this publisher running and use the separate
[image-topic recorder](../../../utils/rosbag_recording/README.md). See its README
for setup, commands and recording behavior.

# Desktop visualization

Run RViz on the desktop while the camera publisher runs on the camera host.
Both environments need the same `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY=0`,
with LAN connectivity that permits DDS discovery and data traffic.

The package owns two RViz presets for native and container use:

- [Raw image](config/rviz/image_topic.rviz): `/image_topic`, reliable QoS,
  queue depth 5.
- [JPEG preview](config/rviz/image_topic_rviz_15hz.rviz):
  `/image_topic_rviz/compressed`, best-effort QoS, queue depth 1.

On a desktop with ROS 2 Humble, RViz and `compressed_image_transport` available,
source its ROS environment and run from the repository root:

```bash
rviz2 -d ros2/src/ocam_publisher/config/rviz/image_topic_rviz_15hz.rviz
```

Loading the preset does not require building the camera package on the desktop
and does not start capture or recording. Package builds also install the presets
under `share/ocam_publisher/config/rviz/`. The
[Docker RViz helper](../../../utils/rviz_docker/README.md) mounts the selected
package preset read-only; it does not maintain a separate copy.
Both presets retain RViz's 30 FPS rendering setting; the preview publisher
determines the 15 Hz image update rate.

The legacy `ocam_rviz.launch.py` starts RViz on the camera host and is only for local
diagnostics, not the normal NUC workflow.

# Tests

To run tests without a camera, display, or LAN DDS access:

```
ROS_LOCALHOST_ONLY=1 colcon test --packages-select ocam_publisher
colcon test-result --verbose
```

The tests cover image color, capture modes, POSIX frame waits and the actual
publisher process linked to a fake camera API. Publisher-launch tests use that
fake camera and stand-in preview processes on isolated domain 174 to verify
shutdown and cleanup. They do not exercise actual camera drivers, JPEG transport
or hardware thermals. Recorder and Docker-helper verification instructions live
in their respective READMEs linked above.

Package maintainer/email and the package-level license declaration are unresolved.
The Withrobot vendor API retains its own GPL-3-or-later notices.

# Appendix
To configure camera parameters in runtime, you can use the visualizer app in the _cpp_ folder.

Move to the cpp folder and build app

```
mkdir build
cd ./build
cmake .. && make -j
./oCam
```

To adjust parameters,
 - Press the key `]` to increase exposure.
 - Press the key `[` to decrease exposure.
 - Press the key `=` to increase brightness.
 - Press the key `-` to decrease brightness.
 - Press the key `q` to exit
