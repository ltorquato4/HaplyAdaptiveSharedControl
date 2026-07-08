#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"

if [[ "${UBUNTU_CODENAME}" != "jammy" ]]; then
  echo "Warning: ROS 2 Humble is intended for Ubuntu 22.04 (jammy)." >&2
  echo "Detected Ubuntu codename: ${UBUNTU_CODENAME:-unknown}" >&2
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install system dependencies." >&2
  exit 1
fi

cd "${WORKSPACE_DIR}"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  curl \
  git \
  gnupg \
  lsb-release \
  python3-pip \
  software-properties-common

sudo add-apt-repository -y universe

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-pygame \
  python3-rosdep \
  "ros-${ROS_DISTRO}-builtin-interfaces" \
  "ros-${ROS_DISTRO}-desktop" \
  "ros-${ROS_DISTRO}-geometry-msgs" \
  "ros-${ROS_DISTRO}-rclpy" \
  "ros-${ROS_DISTRO}-rosidl-default-generators" \
  "ros-${ROS_DISTRO}-rosidl-default-runtime" \
  "ros-${ROS_DISTRO}-std-msgs"

python3 -m pip install --user --upgrade \
  casadi \
  mypy \
  orjson \
  ruff \
  "websockets>=10.4,<12"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
colcon build --symlink-install

BASHRC="${HOME}/.bashrc"
LOCAL_BIN_LINE='export PATH="$HOME/.local/bin:$PATH"'
ROS_SOURCE_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SOURCE_LINE="if [ -f ${WORKSPACE_DIR}/install/setup.bash ]; then source ${WORKSPACE_DIR}/install/setup.bash; fi"

grep -qxF "${LOCAL_BIN_LINE}" "${BASHRC}" \
  || echo "${LOCAL_BIN_LINE}" >> "${BASHRC}"
grep -qxF "${ROS_SOURCE_LINE}" "${BASHRC}" \
  || echo "${ROS_SOURCE_LINE}" >> "${BASHRC}"
grep -qxF "${WORKSPACE_SOURCE_LINE}" "${BASHRC}" \
  || echo "${WORKSPACE_SOURCE_LINE}" >> "${BASHRC}"

git config --global --add safe.directory "${WORKSPACE_DIR}"

echo
echo "Setup complete. Open a new WSL shell or run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
