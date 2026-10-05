#!/usr/bin/env bash
set -e

fail() { printf 'rviz_docker: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  -h|--help)
    printf '%s\n' \
      'Usage: run_rviz_docker.sh [container command ...]' \
      'Launch Humble RViz with NVIDIA graphics and host networking.' \
      'Optional local settings: .env next to this script (see .env.example).' \
      'Run from the desktop graphical session; stop with Ctrl-C.'
    exit 0
    ;;
esac

command -v docker >/dev/null || fail 'Docker is not installed'
[[ -n "${DISPLAY:-}" ]] || fail 'Run from the desktop graphical session (DISPLAY is empty)'
if [[ -z "${XAUTHORITY:-}" ]]; then
  if [[ -r "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  else
    export XAUTHORITY="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/gdm/Xauthority"
  fi
fi
[[ -r "$XAUTHORITY" && -f "$XAUTHORITY" ]] || fail "Unreadable X11 cookie: $XAUTHORITY"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/../.." && pwd)
env_file=/dev/null
if [[ -e "$script_dir/.env" ]]; then
  [[ -f "$script_dir/.env" && -r "$script_dir/.env" ]] || fail "Unreadable settings: $script_dir/.env"
  env_file="$script_dir/.env"
fi
export RVIZ_UID=$(id -u) RVIZ_GID=$(id -g)

exec docker compose --project-directory "$repo_dir" --env-file "$env_file" \
  -f "$script_dir/compose.yaml" run --rm --no-deps rviz "$@"
