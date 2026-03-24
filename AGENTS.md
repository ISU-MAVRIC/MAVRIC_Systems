# AGENTS.md

## Cursor Cloud specific instructions

This is a ROS 2 Jazzy robotics control system that runs inside Docker devcontainers. The host VM is Ubuntu 24.04, which matches ROS 2 Jazzy’s target platform, so **all ROS 2 commands must run inside the Docker container** (`mavric-jazzy-dev`). The image runs as the `ubuntu` user (UID 1000) so workspace bind mounts remain writable.

### Services overview

The system consists of a single ROS 2 workspace at `ros2_ws_Jazzy/` with 5 packages: `mavric_msg` (custom messages/services), `mavric_launch` (launch files), `managers` (CAN + servo singleton managers), `drive_system` (drive/steer/arm control nodes), and `utils` (shared libraries). See `ros2_ws_Jazzy/README.md` for workspace structure details.

### Running commands inside the container

All build/test/lint/run commands must be executed inside the Docker container. Use one of:

```bash
# One-off command
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Jazzy mavric-jazzy-dev \
  "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && <command>"

# Long-running (detached)
docker run -d --network=host -v /workspace:/workspace -w /workspace/ros2_ws_Jazzy \
  --name mavric-teleop mavric-jazzy-dev \
  "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && <command>"

# Exec into running container
docker exec mavric-teleop bash -lc "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && source /workspace/ros2_ws_Jazzy/install/setup.bash && <command>"
```

### Build

```bash
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Jazzy mavric-jazzy-dev \
  "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
```

### Test

```bash
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Jazzy mavric-jazzy-dev \
  "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --return-code-on-test-failure"
```

**Known issue:** `colcon test` for the `utils` package fails with a collection error because `launch_testing` tries to import `utils/__init__.py` which depends on `mavric_msg`. This is not a real test failure. To skip it: `--packages-skip utils`.

### Lint

Black is not pre-installed in the container image; install it first, then run:

```bash
docker exec <container> bash -lc "pip3 install --user --break-system-packages black && export PATH=\$PATH:/home/ubuntu/.local/bin && black --check /workspace/ros2_ws_Jazzy/src/"
```

### Run (launch the full teleop system)

```bash
docker run -d --network=host -v /workspace:/workspace -w /workspace/ros2_ws_Jazzy \
  --name mavric-teleop mavric-jazzy-dev \
  "export PATH=\"\$HOME/.local/bin:\$PATH\" && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mavric_launch teleop.launch.py"
```

The system runs in simulation mode without CAN/I2C hardware (CAN bus uses simulation mode, servos use MockServoKit). All 8 nodes (rosbridge_websocket, rosapi, can_manager, servo_manager, drive_control, steer_control, arm_control, scale_tuning) start successfully.

### Gotchas

- The Docker image must be rebuilt (`docker build -t mavric-jazzy-dev -f .devcontainer/ROS\ Jazzy\ Dev\ Config/Dockerfile .devcontainer/ROS\ Jazzy\ Dev\ Config/`) whenever `.devcontainer/ROS Jazzy Dev Config/Dockerfile` changes (e.g., new pip dependencies).
- After `colcon build`, you must `source install/setup.bash` before running any ROS 2 nodes or services.
- The `ros:jazzy-ros-base` image uses Python 3.12. The dev image installs `python3-catkin-pkg`, `python3-empy`, and `python3-lark` from apt for `colcon`/`ament_cmake`, and installs project Python deps with `pip install --user --break-system-packages` (PEP 668). Add `$HOME/.local/bin` to `PATH` when running nodes so `#!/usr/bin/env python3` keeps the system interpreter (with apt packages such as `python3-tornado`) while still importing user-site packages.
- The container uses `--network=host` for ROS 2 DDS discovery; rosbridge WebSocket is exposed on port 9090.
- Before starting a new teleop container, remove any existing one: `docker rm -f mavric-teleop`.
