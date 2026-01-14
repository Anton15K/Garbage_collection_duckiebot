FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        git \
        openssh-client \
        curl \
        ca-certificates \
        bash-completion \
        build-essential \
        cmake \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-humble-ament-cmake \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        ros-humble-rclcpp \
        ros-humble-rclpy \
        ros-humble-std-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U pip \
    && python3 -m pip install --no-cache-dir \
        ruff \
        black \
        isort \
        pytest

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep init should be done once per image; guard so rebuilds succeed.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

USER ${USERNAME}

# Default to bash (so ROS setup.bash sourcing works nicely)
CMD ["bash"]
