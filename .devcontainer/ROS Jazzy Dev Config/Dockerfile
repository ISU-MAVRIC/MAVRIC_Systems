FROM ros:jazzy-ros-base

# The base image already defines UID 1000, which matches typical Dev Container
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
        libusb-1.0-0 \
        udev \
        v4l-utils \
        ros-jazzy-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

RUN groupmod --new-name MAVRIC ubuntu && \
    usermod --login MAVRIC --home /home/MAVRIC --move-home ubuntu

ENV PATH="/home/MAVRIC/.local/bin:${PATH}"
ENV BASH_ENV="/ros_env.sh"

# Project + hardware Python deps for MAVRIC (user site); setuptools comes from distro for colcon.
USER MAVRIC
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

RUN pip3 install --no-cache-dir --user --break-system-packages \
        flask \
        flask-socketio \
        python-socketio \
        python-engineio \
        opencv-python-headless \
        pyrealsense2

USER root
RUN echo "MAVRIC ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/MAVRIC && \
    chmod 0440 /etc/sudoers.d/MAVRIC

COPY ros_env.sh /ros_env.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_env.sh /ros_entrypoint.sh

USER MAVRIC
WORKDIR /home/MAVRIC

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

RUN echo "source /ros_env.sh" >> /home/MAVRIC/.bashrc
