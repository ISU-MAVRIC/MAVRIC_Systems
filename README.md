# MAVRIC_Systems

ROS 2 Jazzy robotics control system for MAVRIC.

## Docker Compose Quickstart

Build and launch the Jazzy runtime container:

```bash
sudo docker compose up -d --build
```

Open an interactive shell in the running container:

```bash
sudo docker exec -it mavric-ros-jazzy bash
```

View runtime logs:

```bash
sudo docker logs -f mavric-ros-jazzy
```

Stop and remove the container:

```bash
sudo docker compose down
```

Force a clean image rebuild:

```bash
sudo docker compose build --no-cache
```

The Compose service uses host networking for ROS 2 DDS discovery, rosbridge on port
9090, and SocketCAN compatibility. The startup command builds
`ros2_ws_Jazzy`, sources the install workspace, and launches
`mavric_launch teleop.launch.py`.
