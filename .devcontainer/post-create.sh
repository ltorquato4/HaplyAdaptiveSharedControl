#!/usr/bin/env bash
set -euo pipefail

cd /workspaces/research-seminar

set +u
source /opt/ros/humble/setup.bash
set -u

rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc \
  || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKSPACE_SOURCE="if [ -f /workspaces/research-seminar/install/setup.bash ]; then source /workspaces/research-seminar/install/setup.bash; fi"
grep -qxF "${WORKSPACE_SOURCE}" ~/.bashrc \
  || echo "${WORKSPACE_SOURCE}" >> ~/.bashrc

git config --global --add safe.directory /workspaces/research-seminar
