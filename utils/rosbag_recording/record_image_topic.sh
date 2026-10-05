#!/usr/bin/env bash
set -e

usage() {
  printf '%s\n' \
    'Usage: record_image_topic.sh TOPIC [ros2 bag record options ...]' \
    'Source your ROS environment before running this command.' \
    'Defaults: SQLite3, no compression, 300-second splits, 64 MiB cache setting.' \
    'Use -o DIRECTORY for a new output directory; otherwise rosbag names it.' \
    'Stop with Ctrl-C and wait for the bag to finish flushing.'
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  ''|-*) usage >&2; exit 2 ;;
esac

command -v ros2 >/dev/null || {
  printf '%s\n' 'record_image_topic: ros2 is unavailable; source your ROS setup first.' >&2
  exit 1
}

exec ros2 bag record \
  --storage sqlite3 \
  --compression-mode none \
  --max-bag-duration 300 \
  --max-cache-size 67108864 \
  "$@"
