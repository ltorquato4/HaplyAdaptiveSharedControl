# syntax=docker/dockerfile:1

ARG ROS_BASE_IMAGE=osrf/ros:humble-desktop
FROM ${ROS_BASE_IMAGE}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV SDL_AUDIODRIVER=dummy
ENV AUDIODEV=null
ENV PYGAME_HIDE_SUPPORT_PROMPT=1

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /workspaces/research-seminar

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-ament-copyright \
    python3-ament-flake8 \
    python3-ament-pep257 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pygame \
    python3-rosdep \
    python3-yaml \
    software-properties-common \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    casadi \
    colcon-common-extensions \
    mypy \
    orjson \
    ruff \
    "websockets>=10.4,<12"

RUN python3 -m pip install --no-cache-dir --upgrade --ignore-installed "pytest<8"

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then \
        rosdep init; \
    fi

COPY src ./src
COPY pyproject.toml ./
COPY README.md ./

RUN apt-get update \
    && rosdep update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
