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

Build the workspace through the repository helper. It activates `.venv` and
generates ROS Python entrypoints with that interpreter rather than
`/usr/bin/python3`:

```bash
./build.sh
source install/setup.bash
```

If a launch fails with `ModuleNotFoundError` for `websockets` or `casadi`,
rebuild the affected entrypoints and refresh the environment:

```bash
./build.sh haply_interface control_node
source install/setup.bash
```

For other missing project Python modules, rebuild the full workspace with the
first command above. To pass advanced `colcon build` arguments, start with an
option:

```bash
./build.sh --packages-select haply_study_gui --event-handlers console_direct+
```

To verify a Python ROS executable uses `.venv`, inspect its first line:

```bash
head -n 1 install/haply_interface/lib/haply_interface/haply_driver_node
```

It should point at `.venv/bin/python`, not `/usr/bin/python3`.

### Docker / Devcontainer

With Docker Desktop running, use VS Code's **Dev Containers: Reopen in
Container** command. VS Code builds the image from
[`docker/Dockerfile`](docker/Dockerfile), mounts this repository, keeps
`build/`, `install/`, and `log/` in Docker volumes, builds the ROS workspace,
and sources ROS/workspace setup files for new terminals.

To open an interactive Docker shell from the repository root without using the
VS Code devcontainer UI, use the Compose service. This builds or refreshes the
image when needed, mounts your live workspace, and keeps `build/`, `install/`,
and `log/` in Docker volumes:

```bash
docker compose -f docker/compose.yaml run --rm --build research-seminar bash
```

Here `research-seminar` is the Compose service name. The image built for that
service is tagged as `research-seminar:humble` by `docker/compose.yaml`.

If the image is already built and you only want to open a new shell, omit
`--build`:

```bash
docker compose -f docker/compose.yaml run --rm research-seminar bash
```

### Test The Setup

From either a VS Code devcontainer terminal or the Compose container shell, run
the container checks directly in that terminal:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ruff --version
mypy --version
ros2 pkg list | grep haply_study_gui
```

From a host terminal, after building the image, you can also check the image
without mounting the local workspace. Run this from WSL or PowerShell, not from
inside the container:

```bash
docker run --rm research-seminar:humble \
  bash -lc "ruff --version && mypy --version && ros2 pkg list | grep haply_study_gui"
```

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


2. Download and install the Linux standalone Haply Inverse Service in WSL.

   Haply documents two ways to run the Inverse Service: through Haply Hub or as
   a standalone service. For this WSL-owned hardware workflow, use the
   standalone Linux service because the USB device is attached to WSL.

   Download it from Haply's official release page:

   https://develop.haply.co/releases/installer

   On that page, select the latest **Standalone Inverse Service** release, open
   **All downloads**, and download the Linux `.deb` package. From the WSL
   directory containing the downloaded file, install it with:

   ```bash
   sudo apt install ./haply-inverse-service*.deb
   ```

   If the package name differs, replace the filename with the downloaded `.deb`
   file. After installation, the systemd unit should be available as
   `haply-inverse-service.service`.

3. In WSL, start the Linux Haply Inverse Service daemon:

   ```bash
   sudo systemctl start haply-inverse-service.service
   ```

   Haply also documents `restart`, `stop`, and `enable` with the same service
   name:

   ```bash
   sudo systemctl restart haply-inverse-service.service
   sudo systemctl stop haply-inverse-service.service
   sudo systemctl enable haply-inverse-service.service
   ```

4. Verify that ROS can receive the combined Haply state:

   ```bash
   ros2 run haply_study_gui test_haply_state_topic
   ```

5. Launch the study GUI:

   To include a controller (and its required estimator):
   ```bash
   # MPC controller including docking, which is always true here
   ros2 launch haply_study_gui study_gui.launch.py participant_id:=P03

   # Alternative state-feedback controller with no docking enabled
   ros2 launch haply_study_gui study_gui.launch.py controller:=state_feedback participant_id:=P03

   # Alternative state-feedback controller with docking enabled
   ros2 launch haply_study_gui study_gui.launch.py controller:=state_feedback participant_id:=P03 docking_enabled:=true
   ```

   For debugging without Haply device, use the mouse test path instead, can also be tested with controller and estimator:
   ```bash
   # No controller
   ros2 launch haply_study_gui study_gui_mouse.launch.py participant_id:=P03

   # MPC Controller, which always includes docking
   ros2 launch haply_study_gui study_gui_mouse.launch.py controller:=mpc participant_id:=P03

   # State-Feedback with no docking
   ros2 launch haply_study_gui study_gui_mouse.launch.py controller:=state_feedback participant_id:=P03
   
   # State-Feedback with docking
   ros2 launch haply_study_gui study_gui_mouse.launch.py controller:=state_feedback docking_enabled:=true participant_id:=P03
   ```

   The participant sidebar shows only the current trial and run state by
   default. To reveal the controller family and whether the task uses adaptive
   or fixed control while debugging, append `mode:=debug` to either launch:

   ```bash
   ros2 launch haply_study_gui study_gui.launch.py \
     controller:=mpc participant_id:=P03 mode:=debug
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

## Troubleshooting

If `./setup.sh` reports `Conflicting values set for option Signed-By` for
`packages.ros.org/ros2/ubuntu`, the machine has duplicate ROS apt source
definitions. The setup script normalizes this automatically by backing up ROS
source files under `/etc/apt/sources.list.d/` and writing one canonical
`/etc/apt/sources.list.d/ros2.list` entry that uses
`/usr/share/keyrings/ros-archive-keyring.gpg`.
