# Quick Start - Optimized Drive System

## What Changed?

Your drive system had performance issues at 60Hz due to:
1. **480+ log messages per second** causing I/O bottleneck
2. **Sequential publishing** of 4 separate messages per wheel group
3. **Single-threaded processing** creating callback queuing

## Optimizations Applied

### ✅ 1. Reduced Logging (SparkController.py)
- Changed INFO → DEBUG logging for motor commands
- Eliminates ~480 log messages/second

### ✅ 2. Batch Commands (New Message Type)
- Created `CANCommandBatch.msg` to bundle 4 motor commands
- Reduces 240 msg/sec → 60 msg/sec (75% reduction)

### ✅ 3. Multi-threaded CAN Manager
- Uses 4-thread executor for parallel processing
- Batch command processing in tight loop

### ✅ 4. Updated Control Nodes
- `steer_control.py` - now uses batch mode
- `drive_control.py` - now uses batch mode
- Both have `use_batch_commands` parameter (default: true)

## Build & Run

```bash
cd ~/Documents/code/ROS/MAVRIC_Systems/ros2_ws_Foxy

# Build the updated packages
colcon build --packages-select mavric_msg drive_system

# Source the workspace
source install/setup.bash

# Launch (batch mode is default)
ros2 launch mavric_launch teleop.launch.py
```

## Verify It's Working

```bash
# Check if batch topic exists
ros2 topic list | grep batch
# Should show: /can_commands_batch

# Monitor batch messages
ros2 topic hz /can_commands_batch
# Should show ~60Hz when driving

# Check message content
ros2 topic echo /can_commands_batch --once
# Should show 4 commands in one message
```

## Test Performance (Optional)

```bash
# Run the performance test
python3 test_performance.py --duration 10

# Look for:
# ✅ Rate: 55-65 Hz
# ✅ Avg Batch Size: 4 commands
# ✅ Low jitter (<10ms)
```

## Revert to Legacy Mode (If Needed)

```bash
# Run nodes with batch mode disabled
ros2 run drive_system steer_control --ros-args -p use_batch_commands:=false
ros2 run drive_system drive_control --ros-args -p use_batch_commands:=false
```

## Expected Improvements

- **50-75% lower latency** ⚡
- **Synchronized wheel commands** 🎯
- **Reduced CPU usage** 💻
- **Handles >60Hz smoothly** 📈
- **Less jitter** 📊

## Files Modified

1. `mavric_msg/msg/CANCommandBatch.msg` - NEW
2. `drive_system/can_manager.py` - Enhanced with batch support
3. `drive_system/steer_control.py` - Batch mode added
4. `drive_system/drive_control.py` - Batch mode added
5. `utils/SparkCANLib/SparkController.py` - Logging reduced

All changes are backward compatible! 🎉

---
See `PERFORMANCE_OPTIMIZATION.md` for detailed technical documentation.
