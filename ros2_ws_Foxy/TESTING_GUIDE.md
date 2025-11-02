# CAN Singleton Testing Guide

## Quick Start

### Build
```bash
cd ros2_ws_Foxy
source install/setup.bash

# Build message package
colcon build --packages-select mavric_msg

# Build drive system
colcon build --packages-select drive_system
```

### Run
```bash
# Terminal 1: CAN Manager (Singleton - MUST START FIRST)
source install/setup.bash
ros2 run drive_system can_manager

# Terminal 2: Control Node
source install/setup.bash
ros2 run drive_system can_control

# Terminal 3+: Monitor or publish commands
source install/setup.bash
```

## Monitoring

### List Topics
```bash
ros2 topic list
# Should see:
# /can_commands (mavric_msg/CANCommand)
# /can_status (mavric_msg/CANStatus)
# /drive_train (mavric_msg/DriveTrain)
# /steer_train (mavric_msg/SteerTrain)
# /arm_control (mavric_msg/Arm)
```

### Monitor CAN Commands
```bash
ros2 topic echo /can_commands
# Shows: command_type, controller_id, value, timestamp
```

### Monitor Motor Status
```bash
ros2 topic echo /can_status
# Shows: controller_id, position, velocity, current, voltage, last_percent_output, timestamp
# Published for each motor
```

## Test Commands

### Test Drive Motors
```bash
ros2 topic pub /drive_train mavric_msg/msg/DriveTrain \
  "{front_left: 0.5, front_right: 0.5, back_left: 0.5, back_right: 0.5}"
```
Expected:
- CANCommand messages for controllers 1, 6, 5, 3
- CANStatus feedback from those motors

### Test Steer Motors
```bash
ros2 topic pub /steer_train mavric_msg/msg/SteerTrain \
  "{front_left: 0.3, front_right: 0.3, back_left: 0.3, back_right: 0.3}"
```
Expected:
- CANCommand messages for controllers 7, 10, 9, 2
- CANStatus feedback from those motors

### Test Arm Motors
```bash
ros2 topic pub /arm_control mavric_msg/msg/Arm \
  "{shoulder_pitch: 0.2, shoulder_rot: 0.2, elbow_pitch: 0.2, wrist_pitch: 0.2, wrist_rot: 0.2, claw: 0.5}"
```
Expected:
- CANCommand messages for controllers 11, 12, 13, 14, 15
- CANStatus feedback from those motors
- Servo claw moves (if servo is connected)

## Motor Controller Mapping

### Drive (4 motors)
- FLD (1): Front Left Drive
- FRD (6): Front Right Drive
- BLD (5): Back Left Drive
- BRD (3): Back Right Drive

### Steer (4 motors)
- FLS (7): Front Left Steer
- FRS (10): Front Right Steer
- BLS (9): Back Left Steer
- BRS (2): Back Right Steer

### Arm (5 CAN + 1 PWM)
- shoulder_pitch (11): Shoulder Pitch CAN
- shoulder_rot (12): Shoulder Rotation CAN
- elbow_pitch (13): Elbow Pitch CAN
- wrist_pitch (14): Wrist Pitch CAN
- wrist_rot (15): Wrist Rotation CAN
- claw (PWM): Claw servo (not CAN)

**Total: 13 CAN controllers + 1 PWM servo = 14 motors**

## Expected Status Output

Each motor publishes a CANStatus message with:
- `controller_id`: Motor ID (1-15)
- `position`: Current motor position (from CAN status)
- `velocity`: Current motor velocity (from CAN status)
- `current`: Current draw in amps (from CAN status)
- `voltage`: Bus voltage (from CAN status)
- `last_percent_output`: Last commanded percentage (-1.0 to 1.0)
- `timestamp`: When status was read

Example:
```
controller_id: 1
position: 15.234
velocity: 45.2
current: 2.3
voltage: 12.1
last_percent_output: 0.5
timestamp:
  sec: 1234567890
  nanosec: 123456789
```

## Debugging

### Check CAN Manager Logs
```bash
ros2 run drive_system can_manager --ros-args --log-level debug
```

### Check Controller Initialization
- Watch for "Initialized controller X" messages in CAN manager logs
- Controllers are initialized on-demand when first command received

### Check Message Flow
1. Publish drive command
2. Watch /can_commands topic - should see 4 messages (one per motor)
3. Watch /can_status topic - should see status from those 4 motors

### If Status Not Publishing
- Ensure can_manager has initialized controllers
- Check CAN bus is accessible (or in simulation mode)
- Monitor topic should still publish even in sim mode with dummy data

## Performance Monitoring

### Message Rate
- CANStatus: 50 messages/sec (13 motors × 50Hz)
- Total bandwidth: ~55 KB/s (negligible)

### Latency
- Command to status: ~20-40ms (one publish cycle)
- Controller initialization: ~10-50ms first time

### ROS2 Node Info
```bash
ros2 node info /can_manager
ros2 node info /can_control
```

## Common Issues

### Issue: Can't find CANCommand message
- Did you build mavric_msg? → `colcon build --packages-select mavric_msg`
- Did you source setup.bash? → `source install/setup.bash`

### Issue: CAN Manager crashes on start
- CAN bus not available? → Automatically falls back to simulation mode
- Check logs for detailed errors

### Issue: No motors respond
- CAN Manager must be running first
- Check /can_commands topic is publishing
- Check CAN bus configuration (channel, bitrate)

### Issue: Status not publishing
- Give motors time to initialize (happens on first command)
- Check CAN bus is working
- Try `ros2 topic echo /can_status`

## Next Steps

1. Test all motor groups independently
2. Test combinations (drive + steer simultaneously)
3. Monitor status for all 13 motors
4. Verify motor state tracking is accurate
5. Test parameter changes to status_publish_rate

