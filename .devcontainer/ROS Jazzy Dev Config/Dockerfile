FROM ros:jazzy-ros-base

# The base image already defines `ubuntu` (uid 1000), which matches typical Dev Container
# bind mounts so `colcon` can write build/install/log under /workspace.

# System packages: ROS, bridge stack, and ament/colcon Python helpers (avoid a venv on PATH:
# `#!/usr/bin/env python3` must resolve to the system interpreter so rosbridge/rosapi find
# apt packages like python3-tornado and python3-psutil).
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        sudo \
        iproute2 \
        python3-pip \
        python3-venv \
        python3-catkin-pkg \
        python3-empy \
        python3-lark \
        ros-jazzy-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Project + hardware Python deps for ubuntu (user site); setuptools comes from distro for colcon.
USER ubuntu
RUN pip3 install --no-cache-dir --user --break-system-packages \
        setuptools \
        python-can \
        numpy \
        bson \
        pymongo \
        pyyaml \
        bitstring \
        adafruit-circuitpython-servokit \
        adafruit-circuitpython-pca9685 \
        adafruit-circuitpython-motor \
        adafruit-blinka \
        smbus2

USER root
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu && \
    chmod 0440 /etc/sudoers.d/ubuntu

USER ubuntu
WORKDIR /home/ubuntu

# Default entrypoint: bash login shell (ROS env pre-sourced)
ENTRYPOINT ["bash", "-lc"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> /home/ubuntu/.bashrc
