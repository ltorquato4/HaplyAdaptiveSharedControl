# HaplyAdaptiveSharedControl

## Development Setup

The supported setup for development and hardware testing is direct WSL Ubuntu
22.04. This keeps the Haply USB device, Haply Inverse SDK Service, and ROS 2
nodes in the same Linux environment, matching the copied Haply interface
documentation.

From the repository root in WSL, run:

```bash
./setup.sh
```

The setup script installs ROS 2 Humble dependencies, Python tooling, rosdep
dependencies, builds the workspace, and adds ROS/workspace sourcing to
`~/.bashrc`.

The Docker/devcontainer files remain in the repository as a legacy
non-hardware development path. Do not use Docker for Haply hardware testing:
the hardware workflow already requires forwarding the USB device from Windows
to WSL with `usbipd`, and adding a container creates another device and network
boundary.

Package-specific launch commands are documented in each package README under
[`src/`](src/).

## Haply Hardware Setup

For WSL hardware testing, the Haply device USB connection must be attached to
WSL before starting the SDK service and ROS nodes.

In Administrator PowerShell:

```powershell
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

Then, in WSL:

1. Start the Haply Inverse SDK Service so it listens at
   `ws://localhost:10001`.
2. Verify that ROS can receive the Inverse 3 state:

   ```bash
   ros2 run haply_study_gui test_inverse3_state_topic
   ```

3. Launch the hardware GUI, mapper, and scenario generator:

   ```bash
   ros2 launch haply_study_gui study_gui.launch.py
   ```

## ROS Workspace Commands

After changing a ROS package, rebuild it from the workspace root. Replace
`<package_name>` with the package you changed:

```bash
colcon build --symlink-install --packages-select <package_name>
```

Source the rebuilt workspace before launching nodes from the same terminal:

```bash
source install/setup.bash
```

For a full workspace rebuild, omit `--packages-select`:

```bash
colcon build --symlink-install
```

## Manual Checks

This repository does not install git hooks. Run formatting, linting, and type
checks manually from the repository root:

```bash
ruff format --check --force-exclude .
ruff check --force-exclude .
mypy .
```

To apply Ruff formatting and autofixes:

```bash
ruff format --force-exclude .
ruff check --fix --force-exclude .
```

The copied Haply interface under `src/haply_ros2_interface/` is excluded by
tool configuration and should not be reformatted as project-owned code.
