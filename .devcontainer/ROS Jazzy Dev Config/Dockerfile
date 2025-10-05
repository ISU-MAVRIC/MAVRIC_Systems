FROM ros:jazzy-ros-base

# Build-time args for non-root user
ARG USERNAME=mavric
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Install necessary utilities
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        sudo \
        iproute2 \
        python3-pip \
        python3-venv \
        ros-jazzy-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*

# Create a virtual environment for Python packages
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies
RUN pip3 install --no-cache-dir \
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
        adafruit-blinkaFFFFFF \
        smbus2

# Create non-root user and enable passwordless sudo
# Create non-root user and enable passwordless sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash -c "$USERNAME" $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Default entrypoint: bash login shell (ROS env pre-sourced)
ENTRYPOINT ["bash", "-lc"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
