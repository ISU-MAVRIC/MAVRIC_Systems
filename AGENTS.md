# AGENTS.md

## Cursor Cloud specific instructions

This is a ROS 2 Jazzy robotics control system that runs inside Docker. The host VM is Ubuntu 24.04, which matches ROS 2 Jazzy’s target platform, so **all ROS 2 commands must run inside the Docker container** (`mavric-ros-jazzy`). The image runs as the `ubuntu` user (UID 1000) so workspace bind mounts remain writable.

### Services overview

The system consists of a single ROS 2 workspace at `ros2_ws_Jazzy/` with 5 packages: `mavric_msg` (custom messages/services), `mavric_launch` (launch files), `managers` (CAN + servo singleton managers), `drive_system` (drive/steer/arm control nodes), and `utils` (shared libraries). See `ros2_ws_Jazzy/README.md` for workspace structure details.

### Running commands inside the container

All build/test/lint/run commands must be executed inside the Docker container. The default path is Docker Compose:

```bash
# Build and launch the full teleop system
docker compose up -d --build

# Exec into the running container
docker exec -it mavric-ros-jazzy bash

# Run a command in the running container
docker exec mavric-ros-jazzy bash -lc "<command>"
```

Use `--build` for Compose rebuilds. `-build` is not a valid Docker Compose flag.

### Build

```bash
docker exec mavric-ros-jazzy bash -lc "cd /workspace/ros2_ws_Jazzy && colcon build --symlink-install"
```

### Test

```bash
docker exec mavric-ros-jazzy bash -lc "cd /workspace/ros2_ws_Jazzy && source install/setup.bash && colcon test --return-code-on-test-failure"
```

**Historical issue:** older runs saw `colcon test` for the `utils` package fail with a collection error because `launch_testing` tried to import `utils/__init__.py` before `mavric_msg` was available. The current Jazzy container has passed the full test command; only use `--packages-skip utils` if that collection error reappears.

### Lint

Black is not pre-installed in the container image; install it first, then run:

```bash
docker exec mavric-ros-jazzy bash -lc "pip3 install --user --break-system-packages black && black --check /workspace/ros2_ws_Jazzy/src/"
```

### Run (launch the full teleop system)

```bash
docker compose up -d --build
```

The system runs in simulation mode without CAN/I2C hardware (CAN bus uses simulation mode, servos use MockServoKit). All 8 nodes (rosbridge_websocket, rosapi, can_manager, servo_manager, drive_control, steer_control, arm_control, scale_tuning) start successfully.

### Gotchas

- The Docker image must be rebuilt (`docker compose build`) whenever `.devcontainer/ROS Jazzy Dev Config/Dockerfile` changes (e.g., new pip dependencies).
- After `colcon build`, you must `source install/setup.bash` before running any ROS 2 nodes or services.
- The `ros:jazzy-ros-base` image uses Python 3.12. The dev image installs `python3-catkin-pkg`, `python3-empy`, and `python3-lark` from apt for `colcon`/`ament_cmake`, and installs project Python deps with `pip install --user --break-system-packages` (PEP 668). The image adds `/home/ubuntu/.local/bin` to `PATH` while keeping `#!/usr/bin/env python3` on the system interpreter.
- The container uses `--network=host` for ROS 2 DDS discovery; rosbridge WebSocket is exposed on port 9090.
- Before starting a fresh teleop container, remove any existing one with `docker compose down`.
