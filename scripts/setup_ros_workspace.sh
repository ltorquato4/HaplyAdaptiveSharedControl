#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} was not found at /opt/ros/${ROS_DISTRO}."
  echo "Install ROS 2 ${ROS_DISTRO} first, then rerun this script."
  exit 1
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${WORKSPACE_DIR}"

python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
    echo "Initializing rosdep. This may ask for your sudo password."
    sudo rosdep init
  fi
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
else
  echo "rosdep is not installed. Install python3-rosdep for automatic apt dependency setup."
  exit 1
fi

colcon build --symlink-install

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_DIR}/install/setup.bash"
  echo "Workspace built and sourced."
else
  echo "Workspace built."
  echo "Run this to use it in your current shell:"
  echo "  source install/setup.bash"
fi
