# HaplyAdaptiveSharedControl

## Development Setup

The supported development setup is the VS Code devcontainer in
[`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json). Use VS
Code's "Dev Containers: Reopen in Container" command with Docker Desktop
running.

VS Code builds the image from [`docker/Dockerfile`](docker/Dockerfile), mounts
this repository, builds the ROS workspace, sources ROS/workspace setup files for
new terminals, and installs the pre-commit hook.

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

## Pre-Commit Checks

This repository includes pre-commit hooks for Python formatting, linting, type
checking, and basic repository hygiene. The devcontainer image installs
`pre-commit`, `ruff`, and `mypy`, and
[`.devcontainer/post-create.sh`](.devcontainer/post-create.sh) installs the git
hook automatically.

From inside the devcontainer, run the checks manually with:

```bash
pre-commit run
```
