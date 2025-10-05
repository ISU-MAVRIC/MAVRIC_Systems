FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ARG USERNAME=mavric
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Enable universe repo and install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update && \
    apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release sudo \
    python3-pip \
    build-essential git \
    net-tools supervisor \
    x11vnc xvfb openbox xauth \
    novnc websockify \
    && rm -rf /var/lib/apt/lists/*

# --- Setup locales ---
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- Install ROS 2 Foxy Desktop ---
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-foxy-desktop \
    ros-foxy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# --- Python dependencies ---
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
    adafruit-blinkaF \
    smbus2

# --- Create non-root user ---
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --- Setup ROS sourcing ---
RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

# --- Copy supervisor config ---
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY entrypoint.sh /usr/local/bin/novnc-entrypoint
RUN chmod +x /usr/local/bin/novnc-entrypoint

# --- Switch to non-root ---
USER $USERNAME
WORKDIR /home/$USERNAME

# --- Expose VNC + noVNC ports ---
EXPOSE 5900 8080

# --- Start entrypoint (prints URL, picks port, starts supervisord) ---
CMD ["/usr/local/bin/novnc-entrypoint"]
