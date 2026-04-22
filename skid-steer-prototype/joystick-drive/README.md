# skid_steer_drive

Arcade-style differential (skid-steer) drive math and hardware interface for a
6-wheel robot using NEO550 motors, SparkMAX controllers, and a 20:1 gearbox
driving 10-inch wheels. Ships as a single module, `skid_steer_drive.py`.

## Design overview

The library is split into three classes so the math is testable without any
CAN hardware and so the hardware layer stays thin.

| Class | Role |
| --- | --- |
| `DriveConfig` | Dataclass of physical + tuning parameters. Validates on init. |
| `SkidSteerKinematics` | Pure math. Takes `(throttle, turn)` in `[-1, 1]` and returns per-side motor RPM. No hardware dependencies. |
| `SkidSteerDrive` | Wraps a `SparkBus`, owns six `SparkController` instances (three per side), and provides `arcade()`, `point_turn()`, `pivot_turn()`, `stop()`, and telemetry getters. |

## Quick start

```python
from SparkCANLib import SparkCAN
from skid_steer_drive import DriveConfig, SkidSteerDrive

bus = SparkCAN.SparkBus(channel="can0", bustype="socketcan", bitrate=1000000)

config = DriveConfig(
    max_linear_velocity=3.0,        # m/s — the top speed for throttle=1.0
    left_motor_ids=[11, 12, 13],
    right_motor_ids=[21, 22, 23],
    # Optional overrides:
    # wheel_diameter=0.254,          # 10 in default
    # gear_ratio=20.0,
    # max_motor_rpm=8000.0,
    # deadband=0.05,
    # saturation_mode="scale",       # or "clip"
    # left_inverted=False,
    # right_inverted=True,
)

drive = SkidSteerDrive(bus, config)

drive.arcade(throttle=0.5, turn=0.2)   # forward and slightly right
drive.point_turn(0.4)                  # spin in place
drive.pivot_turn("right", 0.6)         # right side stops, pivot around right wheels
drive.stop()                           # percent_output(0) to all motors
```

## The math

Arcade mixing, applied per command:

```
left_norm  = throttle + turn
right_norm = throttle - turn
```

Both results live in `[-2, 2]` before saturation. The library then:

1. Applies the configured deadband to `throttle` and `turn` independently.
2. Clamps raw inputs to `[-1, 1]`.
3. Applies saturation (see below).
4. Multiplies by the motor RPM that corresponds to `max_linear_velocity`:

   ```
   scale_rpm = (max_linear_velocity / (pi * wheel_diameter)) * 60 * gear_ratio
   ```

5. Hard-caps the result at `max_motor_rpm` as a final safety net.
6. Applies `left_inverted` / `right_inverted` sign flips.
7. Sends the same RPM to all three motors on each side via
   `velocity_output()`.

### Saturation modes

When `throttle + turn` or `throttle - turn` falls outside `[-1, 1]`, two
strategies are available:

- **`"scale"` (default)** — divide both sides by the larger magnitude so the
  throttle/turn ratio is preserved. At `throttle=1, turn=1` you get
  `left=1, right=0` (effectively a right-side pivot). This is how most FRC
  drive code behaves and feels more natural at the stick.
- **`"clip"`** — clamp each side independently to `[-1, 1]`. Simpler, but at
  full throttle turning has no effect because the dominant side is already
  pinned.

### Turn conventions

With default inversion flags (`left_inverted=False`, `right_inverted=True`,
matching a typical chassis where the right-side motors are mirrored):

| Input | Behavior |
| --- | --- |
| `throttle > 0, turn = 0` | Drive forward straight. |
| `throttle > 0, turn > 0` | Arc to the right (right side slower). |
| `throttle > 0, turn < 0` | Arc to the left. |
| `throttle = 0, turn > 0` | Point turn to the right, in place. |
| `throttle = 0, turn < 0` | Point turn to the left, in place. |
| `throttle = rate, turn = rate` | Left side drives at `2*rate`, right side stops → **pivot around the right side**. |
| `throttle = rate, turn = -rate` | Right side drives, left side stops → pivot around the left side. |

If your physical wiring produces the opposite sign on one side, just flip
`left_inverted` or `right_inverted` in the config.

## Special maneuvers

### Point turn (spin in place)

```python
drive.point_turn(0.5)    # positive rate turns in the same direction as arcade(0, +turn)
drive.point_turn(-0.5)   # reverse
```

Equivalent to `arcade(throttle=0, turn=rate)`. Both sides run at equal
magnitude but opposite sign, so the robot rotates around its own center.

### One-side pivot turn

```python
drive.pivot_turn("right", 0.6)   # right side stationary, robot pivots around right wheels
drive.pivot_turn("left", 0.6)    # left side stationary
drive.pivot_turn("right", -0.4)  # backward pivot
```

Internally this sets `throttle = rate/2` and `turn = ±rate/2` so that one
side of the arcade mix lands on exactly zero.

## Telemetry

```python
velocities = drive.get_wheel_velocities()   # {can_id: motor_rpm}
positions  = drive.get_wheel_positions()    # {can_id: position}
left_avg, right_avg = drive.get_average_side_rpm()
```

`get_average_side_rpm()` averages the three motors on each side and undoes
the configured inversion flags, so the returned values are "chassis-frame"
(positive = forward on both sides). Handy for coarse closed-loop checks and
future odometry work.

## Configuration reference

| Field | Unit | Required | Default | Notes |
| --- | --- | --- | --- | --- |
| `max_linear_velocity` | m/s | yes | — | Top speed at `throttle = 1.0`. |
| `left_motor_ids` | list[int] | yes | — | Exactly three CAN IDs. |
| `right_motor_ids` | list[int] | yes | — | Exactly three CAN IDs. |
| `wheel_diameter` | m | no | `0.254` | 10 in. |
| `gear_ratio` | — | no | `20.0` | Motor:wheel reduction. |
| `max_motor_rpm` | RPM | no | `8000.0` | Hard ceiling; NEO550 free speed is ~11k. |
| `deadband` | — | no | `0.0` | Applied to `throttle` and `turn` independently. Must be in `[0, 1)`. |
| `saturation_mode` | str | no | `"scale"` | `"scale"` or `"clip"`. |
| `left_inverted` | bool | no | `False` | Flips left-side RPM sign. |
| `right_inverted` | bool | no | `True` | Flips right-side RPM sign. |
| `track_width` | m | no | `None` | Unused by arcade math; reserved for odometry. |

All validation happens in `__post_init__`: wrong ID counts, duplicate IDs,
non-positive physical constants, and out-of-range deadbands all raise
`ValueError` at construction.

## What this library does *not* do

The following are intentionally out of scope for v1:

- **Slew / acceleration limiting.** The SparkMAX controllers have on-board
  ramping, so we don't duplicate it here.
- **Closed-loop heading control.** No gyro integration.
- **Odometry.** `track_width` is available in the config for when this gets
  added, and `get_average_side_rpm()` is the primitive it would build on.
- **Trajectory following.**

## Testing

A mock `SparkController` and `SparkBus` are in `test_skid_steer_drive.py`.
Run the suite with:

```
python -m unittest test_skid_steer_drive -v
```

The test suite covers config validation, all kinematic edge cases (straight,
reverse, point turn, both pivot directions, saturation in both modes,
deadband, input clamping, max-RPM cap, inversion flags), and the dispatching
behavior of `SkidSteerDrive` (motor fan-out, `stop()`, pivot helpers,
telemetry).

## Files

- `skid_steer_drive.py` — the library
- `test_skid_steer_drive.py` — unit tests with mock CAN bus
