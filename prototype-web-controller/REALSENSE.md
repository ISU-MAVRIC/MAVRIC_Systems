# RealSense D435 Prototype Notes

This controller uses `pyrealsense2` directly inside the FastAPI
`prototype-web-controller` app. That is intentionally different from the more
typical ROS 2 integration, where a `realsense2_camera` node publishes color and
depth topics and the web app subscribes through ROS/rosbridge.

For this prototype, direct `pyrealsense2` keeps the live view, banded depth
readout, and dumb obstacle-avoidance toggle close to the standalone FastAPI/raw
WebSocket controller. A production rover integration should consider moving the
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
- The web UI shows the legacy single-pixel center distance and three corridor
  band distances (`L / C / R`) reflecting the rover's projected footprint at
  the configured look-ahead distance.
- If no D435 is attached, or the RealSense Python/OpenCV dependencies are not
  available, the app uses a simulated camera stream that emits drifting per-band
  distances so the UI can be exercised without hardware. Simulated depth is
  **not** usable for obstacle avoidance; the server blocks avoidance unless a
  real, non-simulated RealSense stream is active.
- Obstacle avoidance is deliberately simple but defensive. It is **not** a
  safety system; it is a low-speed best-effort assist that can fail silently if
  the camera lies (sun glare, glass, low-texture, etc.).

## Corridor Depth Model

The avoider does not look at one center pixel. It consumes a `CorridorReading`
with three `BandReading`s (left/center/right) covering the rover's projected
footprint at `lookahead_m` meters in front of the camera.

```
                 +--------+--------+--------+
   image plane:  |  LEFT  | CENTER |  RIGHT |   <- vertically clipped to skip
                 +--------+--------+--------+      ceiling and bumper-blind floor
                  ^                        ^
                  |    rover footprint     |
                  +----------+-------------+
                             |
                             v
                       look-ahead (default 1.0 m)
```

`CameraGeometry` (in `camera_service.py`) sets the projection. Real-hardware
avoidance is fail-closed until measured geometry is provided through
environment variables. Defaults are used only for simulation and local tests.

| Field | Default | Notes |
| --- | --- | --- |
| `MAVRIC_ROVER_WIDTH_M` | required | Effective chassis width. Pad for overhang. |
| `MAVRIC_CAMERA_HEIGHT_M` | required | Camera optical center above the ground. |
| `MAVRIC_CAMERA_FORWARD_OFFSET_M` | required | Camera optical center ahead of the rover reference point. |
| `MAVRIC_CAMERA_PITCH_DEG` | required | Camera pitch used by the rover-frame projection. |
| `MAVRIC_LOOKAHEAD_M` | required | Distance ahead at which the rover footprint is sized. |
| `MAVRIC_MIN_DEPTH_M` | 0.15 | Optional minimum depth considered by the corridor sampler. |
| `MAVRIC_MAX_DEPTH_M` | 2.0 | Optional maximum depth considered by the corridor sampler. |
| `MAVRIC_WIDTH_MARGIN_M` | 0.08 | Optional extra clearance on each side. |
| `MAVRIC_FLOOR_CLIP_M` | 0.05 | Optional floor clipping threshold. |
| `MAVRIC_CEILING_CLIP_M` | 0.4 | Optional ceiling clipping threshold. |

For each band the service reports `min_m` (10th percentile of valid pixels —
robust to single-pixel noise), `mean_m`, and `valid_ratio` (fraction of pixels
in the band that returned a valid depth). A band with `valid_ratio` below
`AvoidanceConfig.min_valid_ratio` is treated as **blind** and contributes no
distance to the avoider's decision.

## RealSense Settings And Filter Chain

The prototype requests the D435 indoor navigation profile recommended by
RealSense tuning guidance: depth at `848x480 @ 30fps`, auto-exposure on, IR
emitter on, clamped laser power, and the High Accuracy visual preset when the
attached sensor exposes those options.

Raw D435 depth has many invalid pixels. For control, the pipeline applies a
navigation-conservative post-processing chain once per frame, in order:

1. `decimation_filter(2)` — 2x downsample, also reduces noise per pixel.
2. `disparity_transform(True)` — convert depth to disparity for D400 stereo filtering.
3. `spatial_filter()` — edge-preserving smoothing within a frame.
4. `temporal_filter()` — exponential smoothing across frames, with persistence disabled.
5. `disparity_transform(False)` — convert filtered disparity back to depth.

Intrinsics are read from the *post-decimation* depth profile so projection math
matches the actual pixel grid we sample.

The control stream intentionally does **not** use final hole filling. Holes and
low-validity regions remain invalid so the avoider can hold, slow, or scan
instead of trusting fabricated depth.

## Avoidance State Machine

`CorridorAvoider` (in `obstacle_avoidance.py`) is a small state machine:

| State | Throttle | Turn | Triggered when |
| --- | --- | --- | --- |
| `manual` | 0 | 0 | Avoidance disabled. |
| `cruising` | `cruise_throttle` | 0 | All bands clear, hysteresis satisfied. |
| `slowing` | `slow_throttle` | 0 | Worst band below `caution_distance_m`, or center band blind while sides are valid. |
| `searching` | 0 | `±pivot_rate` | Center below the speed-aware stop threshold; pivots toward the side with more clearance. |
| `reversing` | `reverse_throttle` | 0 | Any band below `reverse_distance_m`, or pivot completed without opening the path. |
| `recovering` | 0 (or 0/`±recovery_pivot_rate`) | depends | All bands blind for less than `no_depth_search_s`; rotates slowly after the grace window. |
| `stuck` | 0 | 0 | `max_pivot_attempts` failed. Operator must toggle avoidance off then on to retry. |

Key behaviors:

- **Footprint-aware blocking.** Any band can veto forward motion. The rover no
  longer walks its corners into obstacles the center pixel happened to miss.
- **Speed-aware stop margin.** Effective stop distance is
  `stop_distance_m + reaction_factor * cruise_throttle * max_linear_velocity`,
  so faster top speeds give the rover more room to point-turn in the gap.
- **Hysteresis.** Distinct `caution_distance_m` and `clear_distance_m`
  thresholds keep the rover from flapping between cruising and slowing on noisy
  depth.
- **Informed pivot.** When the path is blocked, the avoider compares the left
  and right band distances and pivots toward whichever has more clearance,
  caching the last-clear side as a tie-breaker.
- **Recoverable lost depth.** A short dropout (`no_depth_grace_s`, default
  1 s) just holds position. A longer dropout starts a slow rotation to
  reacquire view (`recovery_pivot_rate`). Only after `no_depth_search_s`
  (default 3 s) does the rover escalate to a full pivot search, and only after
  `max_pivot_attempts` of those does it give up.
- **Escape escalation.** Each pivot that doesn't open the path is followed by
  a short reverse, and the chosen side is re-evaluated for the next attempt.
  After `max_pivot_attempts` the avoider transitions to `stuck` and broadcasts
  a clear reason rather than silently grinding.

## Tuning on Hardware

`AvoidanceConfig` and `CameraGeometry` are the two knobs. Before real-hardware
avoidance can be enabled, export measured geometry:

```bash
export MAVRIC_ROVER_WIDTH_M=0.82
export MAVRIC_CAMERA_HEIGHT_M=0.46
export MAVRIC_CAMERA_FORWARD_OFFSET_M=0.18
export MAVRIC_CAMERA_PITCH_DEG=-8
export MAVRIC_LOOKAHEAD_M=1.4
```

Once you are on hardware:

1. Measure the **actual rover width** (including any overhang) and update
   `CameraGeometry.rover_width_m`.
2. Measure the **camera mounting height** to the optical center and update
   `MAVRIC_CAMERA_HEIGHT_M`. If the camera is meaningfully tilted, measure
   pitch and set `MAVRIC_CAMERA_PITCH_DEG`.
3. Find the **min effective range** of your D435 in the operating environment
   and set `AvoidanceConfig.reverse_distance_m` slightly above that.
4. Pick a `cruise_throttle` you trust, then set `stop_distance_m` to the
   distance the rover travels in roughly 1 second at that throttle. The
   speed-aware margin will add headroom on top.
5. Bump `min_valid_ratio` up if you see false "all clear" readings on
   low-texture surfaces; bump it down if the avoider is constantly
   "no valid depth" outdoors.

## Why This Is Not The Typical ROS D435 Setup

The standard ROS 2 approach is to install the RealSense SDK and launch the
`realsense2_camera` wrapper, then consume topics such as color image, aligned
depth image, and camera info. This prototype does not do that yet. It captures
frames in the web-controller process to minimize integration work with the
existing FastAPI/raw WebSocket control loop.

The tradeoff is that other ROS nodes cannot reuse this depth stream. When the
prototype behavior is validated on hardware, the next step should be a ROS node
or `realsense2_camera` launch integration with obstacle avoidance consuming ROS
topics instead of direct web-app-owned camera frames.

## Residual D435 Limitations

Even with the corridor model and the filter chain, the D435 will still lie:

- **Minimum range.** Below ~0.105 m the D435 returns invalid depth. Anything
  closer than ~0.2 m is unreliable in practice. Mount the camera so the
  rover's bumper is at least that far ahead of any obstacle the avoider must
  see.
- **Bumper-blind floor.** A forward-facing camera cannot see the patch of
  ground directly under the rover's nose. Low obstacles right at the bumper
  may be invisible.
- **Sun and IR oversaturation.** Direct sunlight saturates the IR projector
  and fills the depth image with holes. The recovery state helps, but in
  bright sun the avoider will spend more time slowing/recovering.
- **Specular and transparent surfaces.** Glass, mirrors, polished metal, and
  water often return zero or wildly incorrect depth. Treat the avoider's
  output as advisory in environments with these surfaces.
- **Negative obstacles.** The avoider does not detect drop-offs, ledges, or
  holes — only positive obstacles within the corridor.
- **No 360° awareness.** Anything behind or beside the rover is invisible.
  Reverse moves are timed; they do not check that the path behind is clear.
  Keep reverse disabled or very short during indoor trials until rear clearance
  is sensed independently.

## Out of Scope

- ROS 2 integration via `realsense2_camera` (still TODO above).
- IMU/odometry-based heading estimation.
- Persistent local cost-map or SLAM-driven planning.
- Detection of negative obstacles (drop-offs).
