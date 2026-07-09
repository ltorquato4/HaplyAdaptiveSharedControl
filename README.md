# HaplyAdaptiveSharedControl

## Development Setup

This repository supports two development paths:

1. Direct WSL Ubuntu 22.04 with a project virtual environment.
2. Docker through the VS Code devcontainer for an isolated ROS 2 workspace.

Use the direct WSL path for Haply hardware testing unless you have explicitly
confirmed that the Haply Inverse SDK Service and WebSocket endpoint are reachable
from inside the container. Use the devcontainer for reproducible non-hardware
builds, linting, and ROS package checks.

### Direct WSL / Virtual Environment

From the repository root in WSL, run:

```bash
./setup.sh
```

The setup script installs ROS 2 Humble dependencies, creates `.venv` if needed,
installs Python tooling into that virtual environment, installs rosdep
dependencies, builds the workspace, and adds venv/ROS/workspace sourcing to
`~/.bashrc`.

After opening a new shell, ROS commands should be available directly. To refresh
the current shell manually, run:

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Docker / Devcontainer

With Docker Desktop running, use VS Code's **Dev Containers: Reopen in
Container** command. VS Code builds the image from
[`docker/Dockerfile`](docker/Dockerfile), mounts this repository, keeps
`build/`, `install/`, and `log/` in Docker volumes, builds the ROS workspace,
and sources ROS/workspace setup files for new terminals.

You can also build the Docker image manually:

```bash
docker build -f docker/Dockerfile -t research-seminar:humble .
```

Then run a smoke check without mounting the local workspace:

```bash
docker run --rm research-seminar:humble \
  bash -lc "ruff --version && mypy --version && ros2 pkg list | grep haply_study_gui"
```

The local `.venv` is ignored by Docker and is not used inside the image or
devcontainer.

Package-specific launch commands are documented in each package README under
[`src/`](src/).

## Haply Hardware Setup

The study uses the Haply Inverse3 together with the VerseGrip Stylus. The
VerseGrip Stylus provides orientation tracking and in-hand input buttons; the
study GUI uses the VerseGrip button state published on `/haply_state.buttons`.
Button `a` is the active drawing/start input for hardware runs.

Haply's setup notes for the VerseGrip Stylus are here:
https://docs.haply.co/docs/quick-start-verse-grip-stylus/

The supported hardware path is to attach the Haply USB device to WSL and run
the Linux standalone Haply Inverse Service inside WSL. The USB device can only
be owned by one environment at a time: after `usbipd attach --wsl`, Windows
releases the device, so Windows-side Haply Hub or Windows Inverse Service cannot
see it. ROS should therefore connect to `ws://localhost:10001` from WSL.

Use this WSL-owned hardware path:

1. In Administrator PowerShell, attach the 2 devices USB ports, usually identified as COM3 and COM5,  to WSL:

   ```powershell
   usbipd list
   usbipd bind --busid <busid>
   usbipd attach --wsl --busid <busid>
   ```

   In order to detach, use: 
   ```
   usbipd detach --busid <busid>
   ```


2. In WSL, start the Linux Haply Inverse Service daemon:

   ```bash
   sudo systemctl start haply-inverse-service.service
   ```

   If the service is not installed yet, install the Linux standalone Inverse
   Service from Haply's Inverse Service documentation. Haply also documents
   `restart`, `stop`, and `enable` with the same service name.

3. Verify that ROS can receive the combined Haply state:

   ```bash
   ros2 run haply_study_gui test_haply_state_topic
   ```

4. Launch the study GUI:

   For official study runs (requires pressing Spacebar to start the trial):
   ```bash
   ros2 launch haply_study_gui study_gui.launch.py
   ```

   For hardware testing and parameter tuning (starts automatically):
   ```bash
   ros2 launch haply_study_gui study_gui_haply_test.launch.py
   ```

### Workspace Mapping & Tuning

The study mapping logic converts physical Haply movements into 2D screen coordinates using the following rules:

- **Z-as-Y Mapping**: The Haply's vertical `z` axis is mapped to the screen's `y` axis (`use_z_as_y=True`), meaning vertical physical movement maps to vertical screen movement. The physical depth axis (`y`) is ignored for this 2-DoF study.
- **Scaling**: Physical movements are scaled up by the mapper (e.g. `scale_x=2.0`, `scale_y=2.0`). A 10cm physical movement covers 20cm of task space.
- **Physical Clamping**: The accessible physical workspace is clamped relative to the spot where the arm rested when the script launched (`clamp_raw=True`).
  - To change left/right physical boundaries, edit `raw_x_min` and `raw_x_max`.
  - To change down/up physical boundaries, edit `raw_second_min` and `raw_second_max`.

These parameters can be tuned directly inside `study_gui.launch.py` and `study_gui_haply_test.launch.py` without rebuilding the workspace.


## ROS Workspace Commands

After changing a ROS package, rebuild it from the workspace root. Replace
`haply_study_gui` with the package you changed:

```bash
colcon build --symlink-install --packages-select haply_study_gui
source install/setup.bash
```

For a full workspace rebuild, omit `--packages-select`:

```bash
colcon build --symlink-install
source install/setup.bash
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

The copied Haply interface under `src/haply_ros2_interface/` is excluded by tool
configuration and should not be reformatted as project-owned code.
