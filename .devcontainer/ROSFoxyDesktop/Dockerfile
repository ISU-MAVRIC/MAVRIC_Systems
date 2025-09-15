FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8


# Enable universe repo and install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release sudo \
    python3-pip python3-colcon-common-extensions \
    build-essential git \
    net-tools supervisor && \
    sed -i 's/^# deb.*universe/deb http:\/\/archive.ubuntu.com\/ubuntu focal universe/' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    x11vnc xvfb openbox xauth && \
    rm -rf /var/lib/apt/lists/*



# --- Setup locales ---
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# --- Add ROS 2 apt repo ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2-latest.list

# --- Install ROS 2 Foxy Desktop ---
RUN apt-get update && apt-get install -y \
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
    novnc \
    websockify

# --- Create non-root user ---
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --- Setup ROS sourcing ---
RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

# --- Copy supervisor config ---
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# --- Switch to non-root ---
USER $USERNAME
WORKDIR /home/$USERNAME

# --- Expose VNC + noVNC ports ---
EXPOSE 5900 8080

# --- Start supervisor (manages Xvfb + VNC + noVNC) ---
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
