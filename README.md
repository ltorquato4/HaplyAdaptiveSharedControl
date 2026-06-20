# HaplyAdaptiveSharedControl

## One-command workspace setup

From WSL/Ubuntu with ROS 2 Humble installed:

```bash
source scripts/setup_ros_workspace.sh
```

This installs Python packages from `requirements.txt`, installs ROS package
dependencies with `rosdep`, builds all packages under `src` with `colcon`, and
sources the workspace in the current shell.

After that, the study GUI can be started with:

```bash
ros2 launch haply_study_gui study_gui.launch.py
```
