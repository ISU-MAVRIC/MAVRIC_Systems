# AGENTS.md

## Cursor Cloud specific instructions

This is a ROS 2 Foxy robotics control system that runs inside Docker devcontainers. The host VM is Ubuntu 24.04 and ROS 2 Foxy targets Ubuntu 20.04, so **all ROS 2 commands must run inside the Docker container** (`mavric-foxy-dev`).

### Services overview

The system consists of a single ROS 2 workspace at `ros2_ws_Foxy/` with 5 packages: `mavric_msg` (custom messages/services), `mavric_launch` (launch files), `managers` (CAN + servo singleton managers), `drive_system` (drive/steer/arm control nodes), and `utils` (shared libraries). See `ros2_ws_Foxy/README.md` for workspace structure details.

### Running commands inside the container

All build/test/lint/run commands must be executed inside the Docker container. Use one of:

```bash
# One-off command
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Foxy mavric-foxy-dev \
  "source /opt/ros/foxy/setup.bash && <command>"

# Long-running (detached)
docker run -d --network=host -v /workspace:/workspace -w /workspace/ros2_ws_Foxy \
  --name mavric-teleop mavric-foxy-dev \
  "source /opt/ros/foxy/setup.bash && source install/setup.bash && <command>"

# Exec into running container
docker exec mavric-teleop bash -lc "source /opt/ros/foxy/setup.bash && source /workspace/ros2_ws_Foxy/install/setup.bash && <command>"
```

### Build

```bash
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Foxy mavric-foxy-dev \
  "source /opt/ros/foxy/setup.bash && colcon build --symlink-install"
```

### Test

```bash
docker run --rm -v /workspace:/workspace -w /workspace/ros2_ws_Foxy mavric-foxy-dev \
  "source /opt/ros/foxy/setup.bash && source install/setup.bash && colcon test --return-code-on-test-failure"
```

**Known issue:** `colcon test` for the `utils` package fails with a collection error because `launch_testing` tries to import `utils/__init__.py` which depends on `mavric_msg`. This is not a real test failure. To skip it: `--packages-skip utils`.

### Lint

Black is not pre-installed in the container image; install it first, then run:

```bash
docker exec <container> bash -lc "pip3 install black && export PATH=\$PATH:/home/mavric/.local/bin && black --check /workspace/ros2_ws_Foxy/src/"
```

### Run (launch the full teleop system)

```bash
docker run -d --network=host -v /workspace:/workspace -w /workspace/ros2_ws_Foxy \
  --name mavric-teleop mavric-foxy-dev \
  "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 launch mavric_launch teleop.launch.py"
```

The system runs in simulation mode without CAN/I2C hardware (CAN bus uses simulation mode, servos use MockServoKit). All 8 nodes (rosbridge_websocket, rosapi, can_manager, servo_manager, drive_control, steer_control, arm_control, scale_tuning) start successfully.

### Gotchas

- The Docker image must be rebuilt (`docker build -t mavric-foxy-dev -f .devcontainer/Dockerfile .devcontainer/`) whenever `.devcontainer/Dockerfile` changes (e.g., new pip dependencies).
- After `colcon build`, you must `source install/setup.bash` before running any ROS 2 nodes or services.
- The `ros:foxy-ros-base` image uses Python 3.8. Python dependencies must be compatible with Python 3.8.
- The container uses `--network=host` for ROS 2 DDS discovery; rosbridge WebSocket is exposed on port 9090.
- Before starting a new teleop container, remove any existing one: `docker rm -f mavric-teleop`.
