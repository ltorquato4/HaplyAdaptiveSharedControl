#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${WORKSPACE_DIR}/.venv"
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"

if [[ "${UBUNTU_CODENAME}" != "jammy" ]]; then
  echo "Warning: ROS 2 Humble is intended for Ubuntu 22.04 (jammy)." >&2
  echo "Detected Ubuntu codename: ${UBUNTU_CODENAME:-unknown}" >&2
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install system dependencies." >&2
  exit 1
fi

backup_ros_apt_sources() {
  local backup_suffix

  backup_suffix="$(date +%Y%m%d%H%M%S)"

  sudo mkdir -p /etc/apt/sources.list.d

  while IFS= read -r -d "" existing_source; do
    [[ -n "${existing_source}" ]] || continue
    if sudo grep -qi "packages.ros.org/ros2/ubuntu" "${existing_source}"; then
      sudo mv "${existing_source}" "${existing_source}.bak.${backup_suffix}"
    fi
  done < <(sudo find /etc/apt/sources.list.d -maxdepth 1 -type f \( -name "*.list" -o -name "*.sources" \) -print0)
}

write_ros_apt_source() {
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local source_file="/etc/apt/sources.list.d/ros2.list"

  sudo mkdir -p /usr/share/keyrings /etc/apt/sources.list.d

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o "${keyring}"

  echo "deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee "${source_file}" >/dev/null
}

cd "${WORKSPACE_DIR}"

backup_ros_apt_sources

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  curl \
  git \
  gnupg \
  lsb-release \
  python3-pip \
  python3-venv \
  python3-yaml \
  software-properties-common

sudo add-apt-repository -y universe

backup_ros_apt_sources
write_ros_apt_source

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

if [[ ! -d "${VENV_DIR}" ]]; then
  python3 -m venv --system-site-packages "${VENV_DIR}"
fi

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

python3 -m pip install --upgrade pip
python3 -m pip install --upgrade \
  casadi \
  colcon-common-extensions \
  mypy \
  orjson \
  ruff \
  "websockets>=10.4,<12"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update

# shellcheck source=/dev/null
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
"${WORKSPACE_DIR}/build.sh"

BASHRC="${HOME}/.bashrc"
LOCAL_BIN_LINE='export PATH="$HOME/.local/bin:$PATH"'
VENV_SOURCE_LINE="if [ -f ${VENV_DIR}/bin/activate ]; then source ${VENV_DIR}/bin/activate; fi"
ROS_SOURCE_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SOURCE_LINE="if [ -f ${WORKSPACE_DIR}/install/setup.bash ]; then source ${WORKSPACE_DIR}/install/setup.bash; fi"
BUILD_ALIAS_LINE="alias rs-build='${WORKSPACE_DIR}/build.sh'"

grep -qxF "${LOCAL_BIN_LINE}" "${BASHRC}" \
  || echo "${LOCAL_BIN_LINE}" >> "${BASHRC}"
grep -qxF "${VENV_SOURCE_LINE}" "${BASHRC}" \
  || echo "${VENV_SOURCE_LINE}" >> "${BASHRC}"
grep -qxF "${ROS_SOURCE_LINE}" "${BASHRC}" \
  || echo "${ROS_SOURCE_LINE}" >> "${BASHRC}"
grep -qxF "${WORKSPACE_SOURCE_LINE}" "${BASHRC}" \
  || echo "${WORKSPACE_SOURCE_LINE}" >> "${BASHRC}"
grep -qxF "${BUILD_ALIAS_LINE}" "${BASHRC}" \
  || echo "${BUILD_ALIAS_LINE}" >> "${BASHRC}"

git config --global --add safe.directory "${WORKSPACE_DIR}"

echo
echo "Setup complete. Open a new WSL shell or run:"
echo "  source ${VENV_DIR}/bin/activate"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo
echo "For later venv-safe rebuilds, run:"
echo "  ${WORKSPACE_DIR}/build.sh <package_name>"
echo "or, after opening a new shell:"
echo "  rs-build <package_name>"
