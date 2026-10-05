# RViz in Docker

Run ROS 2 Humble RViz on an Ubuntu 20.04/22.04 x86_64 desktop with NVIDIA
graphics and an X11 session. Native Humble installation is not required.
Docker Engine, Compose 2.30 or newer, the host NVIDIA driver and
[NVIDIA Container Toolkit configured for Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
must already be available; the launcher does not install them. Native Wayland
and other GPU/platform combinations are outside this helper's verified scope.

## Run

From the repository root:

```bash
utils/rviz_docker/run_rviz_docker.sh
```

The default image is `osrf/ros:humble-desktop-full`. Docker reuses a local image;
the first launch may download it. Each run starts a disposable container with
host networking and NVIDIA GPU access. Closing RViz removes the container, not
its image or host-mounted files. The launcher does not start camera publishing,
recording or an SSH connection.

The default view uses the oCam JPEG preset owned by the
[camera package](../../ros2/src/ocam_publisher/README.md#desktop-visualization).
Its publisher must be started separately. Use the same `ROS_DOMAIN_ID` as the
publisher; the container sets `ROS_LOCALHOST_ONLY=0`. LAN firewalls and DDS
interface selection still need to permit discovery and data traffic.

## Local settings

To customize this desktop, copy `.env.example` to `.env` once in this directory
and edit `.env`. The local file is gitignored and is reused on each launch;
the shared Compose file does not need local edits. Without `.env`, Compose uses
its built-in defaults.

| Setting | Purpose |
| --- | --- |
| `ROS_DOMAIN_ID` | DDS domain shared with the publisher |
| `RVIZ_WORKSPACE_DIR` | Existing host directory mounted read-write at `/workspace` |
| `RVIZ_CONFIG` | Existing host `.rviz` file mounted read-only as the initial view |

Relative paths resolve from the repository root, independent of the directory
from which the launcher is invoked. Shell environment values override `.env`.
For example, select the raw preset for one run:

```bash
RVIZ_CONFIG=ros2/src/ocam_publisher/config/rviz/image_topic.rviz \
  utils/rviz_docker/run_rviz_docker.sh
```

`DISPLAY` and `XAUTHORITY` come from the current desktop session, not the local
settings template. If `XAUTHORITY` is unset, the launcher checks `~/.Xauthority`
then the session's GDM authority file. Export an explicit `XAUTHORITY` path if
your session uses another location. Never commit the cookie itself.

The container runs with the invoking user's UID/GID. Its home is temporary;
save custom RViz settings under `/workspace` if they must survive container
removal. The input preset is read-only. Ignoring `.env` does not back it up.

An optional full container command can replace the default RViz command. For
example, use a saved file already inside the workspace mount:

```bash
utils/rviz_docker/run_rviz_docker.sh rviz2 -d /workspace/my_view.rviz
```

Choosing another preset does not install its custom message packages or RViz
plugins into the image.

## Verification

```bash
python3 -m unittest discover -s utils/rviz_docker/test -v
```

Tests mock container execution. Configuration tests also require Docker Compose,
but do not start containers or access the GPU/display. They check local settings,
environment precedence, paths with spaces, session validation and command/status
forwarding; they do not prove live rendering or LAN DDS connectivity.
