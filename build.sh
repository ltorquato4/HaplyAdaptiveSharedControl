#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${WORKSPACE_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"

if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "Missing ${VENV_PYTHON}." >&2
  echo "Run ./setup.sh first to create the direct WSL virtual environment." >&2
  exit 1
fi

cd "${WORKSPACE_DIR}"

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

BUILD_ARGS=(--symlink-install)

if [[ "$#" -gt 0 ]]; then
  if [[ "$1" == --* ]]; then
    BUILD_ARGS+=("$@")
  else
    BUILD_ARGS+=(--packages-select "$@")
  fi
fi

"${VENV_PYTHON}" -m colcon build "${BUILD_ARGS[@]}"

echo
echo "Build complete. Refresh the current shell with:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
