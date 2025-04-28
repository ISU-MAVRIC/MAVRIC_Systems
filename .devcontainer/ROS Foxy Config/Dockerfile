FROM ubuntu:20.04

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install prerequisites
RUN apt-get update && apt-get install -y \
    locales \
    gnupg2 \
    lsb-release \
    curl \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Setup ROS2 apt repository
RUN apt-get update && apt-get install -y software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    && apt-get update

# Install ROS2 Foxy
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=(root) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Default entrypoint
ENTRYPOINT [ "bash", "-lc" ]