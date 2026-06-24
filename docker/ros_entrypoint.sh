#!/usr/bin/env bash
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/workspaces/research-seminar}"
export SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}"
export AUDIODEV="${AUDIODEV:-null}"
export PYGAME_HIDE_SUPPORT_PROMPT="${PYGAME_HIDE_SUPPORT_PROMPT:-1}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

exec "$@"
