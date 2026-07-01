# HaplyAdaptiveSharedControl

## Development Setup

The easiest setup is the VS Code devcontainer in
[`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json). Use
VS Code's "Dev Containers: Reopen in Container" command with Docker Desktop
running; VS Code builds the image from [`docker/Dockerfile`](docker/Dockerfile)
if needed, mounts this repository, builds the ROS workspace, and sources
ROS/workspace setup files for new terminals.

Docker Compose is also available if you prefer to build and enter the container
manually. The default image uses the official ROS 2 Humble Desktop base and
installs the project dependencies with apt/rosdep during the image build.

```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm research-seminar
```

Inside the container, the ROS environment and workspace are already sourced, so
you can launch ROS commands directly. For example:

```bash
ros2 pkg list
```

Package-specific launch commands are documented in each package's README under
[`src/`](src/).

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
