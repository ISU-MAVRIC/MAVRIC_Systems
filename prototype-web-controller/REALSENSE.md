# RealSense D435 Prototype Notes

This controller uses `pyrealsense2` directly inside the Flask
`prototype-web-controller` app. That is intentionally different from the more
typical ROS 2 integration, where a `realsense2_camera` node publishes color and
depth topics and the web app subscribes through ROS/rosbridge.

For this prototype, direct `pyrealsense2` keeps the live view, center-distance
readout, and dumb obstacle-avoidance toggle close to the existing standalone
Socket.IO controller. A production rover integration should consider moving the
camera into ROS topics so other nodes can consume the same depth data.

## Hardware Setup

- Use an Intel RealSense D435 or compatible D400-series camera.
- Use a known-good USB 3 cable and plug into a USB 3 port. USB 2 often causes
  missing frames, low frame rates, or no depth stream.
- The camera must be visible to the container through `/dev/bus/usb`.
- If Docker cannot see the camera, run the container with USB device access.
  For development, the practical options are:
  - add `privileged: true` to the Compose service temporarily, or
  - add a device mount for `/dev/bus/usb:/dev/bus/usb`.
- Host udev permissions may be needed so the container user can access the
  RealSense device. If access fails, install the librealsense udev rules on the
  host or run the container with elevated device permissions.
- Start the prototype app from the repo mount inside the container:

```bash
cd /workspace/prototype-web-controller
python3 app.py
```

- Rebuild the Docker image after dependency changes:

```bash
docker compose -f compose.yaml -f compose/dev.yaml up -d --build
```

## Runtime Behavior

- The video stream is available at `/camera/stream` as MJPEG.
- Distance telemetry reports the center/crosshair depth in meters.
- If no D435 is attached, or the RealSense Python/OpenCV dependencies are not
  available, the app uses a simulated camera stream and synthetic distance data.
- Obstacle avoidance is deliberately simple: it drives slowly forward when the
  center distance is clear, reverses briefly when blocked, pivots, and retries.
  It is not a safety system.

## Why This Is Not The Typical ROS D435 Setup

The standard ROS 2 approach is to install the RealSense SDK and launch the
`realsense2_camera` wrapper, then consume topics such as color image, aligned
depth image, and camera info. This prototype does not do that yet. It captures
frames in the web-controller process to minimize integration work with the
existing Flask/Socket.IO control loop.

The tradeoff is that other ROS nodes cannot reuse this depth stream. When the
prototype behavior is validated on hardware, the next step should be a ROS node
or `realsense2_camera` launch integration with obstacle avoidance consuming ROS
topics instead of direct Flask-owned camera frames.
