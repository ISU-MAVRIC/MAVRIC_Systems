# Troubleshooting Guide - Drive System Optimization

## Quick Diagnostics

### Check if Batch Mode is Active

```bash
# Should show /can_commands_batch topic
ros2 topic list | grep batch

# Should show messages being published
ros2 topic hz /can_commands_batch

# Check message structure
ros2 topic echo /can_commands_batch --once
```

**Expected Output:**
```
commands:
- command_type: 3
  controller_id: 7
  value: 0.075
- command_type: 3
  controller_id: 10
  value: 0.075
# ... (4 commands total for steer)
```

### Check Node Status

```bash
# List all running nodes
ros2 node list

# Should show:
# /can_manager
# /drive_control
# /steer_control

# Check node info
ros2 node info /can_manager
```

### Check for Errors

```bash
# Run with debug logging
ros2 run drive_system can_manager --ros-args --log-level debug

# Check for warnings in logs
ros2 log view | grep -i warn
```

## Common Issues & Solutions

### Issue 1: Batch Topic Not Appearing

**Symptoms:**
- `ros2 topic list` doesn't show `/can_commands_batch`
- Only `/can_commands` appears

**Diagnosis:**
```bash
# Check if nodes are using batch mode
ros2 param get /steer_control use_batch_commands
ros2 param get /drive_control use_batch_commands
```

**Solution:**
```bash
# Rebuild the mavric_msg package
cd ~/Documents/code/ROS/MAVRIC_Systems/ros2_ws_Foxy
colcon build --packages-select mavric_msg --cmake-clean-cache
source install/setup.bash

# Rebuild drive_system
colcon build --packages-select drive_system
source install/setup.bash

# Restart nodes
ros2 launch mavric_launch teleop.launch.py
```

### Issue 2: Wheels Still Not Synchronized

**Symptoms:**
- Batch topic exists but wheels still lag
- Inconsistent steering response

**Diagnosis:**
```bash
# Check message rate (should be ~60Hz)
ros2 topic hz /can_commands_batch

# Check for high jitter
ros2 topic hz /steer_train
```

**Possible Causes:**
1. **Network latency** (rosbridge/base station)
   ```bash
   # Test network latency
   ping <base_station_ip>
   # Should be <10ms
   ```

2. **CPU throttling**
   ```bash
   # Check CPU usage
   top -p $(pgrep -f can_manager)
   # Should be <30%
   ```

3. **CAN bus issues**
   ```bash
   # Check CAN interface
   ip -details link show can0
   # Should show UP, LOWER_UP
   
   # Check for CAN errors
   ifconfig can0
   # Look for TX errors
   ```

**Solution:**
```bash
# Increase CAN priority
sudo ip link set can0 txqueuelen 1000

# Reduce network latency (if using WiFi)
# Switch to wired Ethernet if possible

# Verify multi-threaded executor is active
# Check can_manager.py line ~147 (should use MultiThreadedExecutor)
```

### Issue 3: Build Errors

**Symptoms:**
```
CMake Error: Could not find package mavric_msg
```

**Solution:**
```bash
# Build messages first
cd ~/Documents/code/ROS/MAVRIC_Systems/ros2_ws_Foxy
colcon build --packages-select mavric_msg
source install/setup.bash

# Then build drive_system
colcon build --packages-select drive_system
source install/setup.bash
```

**Symptoms:**
```
ModuleNotFoundError: No module named 'mavric_msg'
```

**Solution:**
```bash
# Make sure to source the workspace
source ~/Documents/code/ROS/MAVRIC_Systems/ros2_ws_Foxy/install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/Documents/code/ROS/MAVRIC_Systems/ros2_ws_Foxy/install/setup.bash" >> ~/.bashrc
```

### Issue 4: High CPU Usage

**Symptoms:**
- CPU usage >50% for can_manager
- System feels sluggish

**Diagnosis:**
```bash
# Check if debug logging is enabled
ros2 node info /can_manager | grep log

# Monitor log output
ros2 topic hz /rosout
# Should be low (<10Hz)
```

**Solution:**
```bash
# Make sure you're not running with --log-level debug
# Restart without debug flag:
ros2 run drive_system can_manager  # No debug flag

# Verify logging is at INFO level
ros2 param get /can_manager log_level
```

### Issue 5: Messages Not Reaching Motors

**Symptoms:**
- Batch messages published but motors don't respond
- No errors in logs

**Diagnosis:**
```bash
# Check CAN interface
ip link show can0

# Monitor CAN traffic
candump can0

# Check controller initialization
ros2 topic echo /can_status
# Should show position/velocity updates
```

**Solution:**
```bash
# Restart CAN interface
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 up

# Verify bitrate matches
# Check can_manager.py parameter (default: 1000000)
ros2 param get /can_manager can_bitrate
```

### Issue 6: Reverting to Legacy Mode

**When to use:**
- Debugging batch command issues
- Comparing performance
- Testing backward compatibility

**How to:**
```bash
# Stop current launch
Ctrl+C

# Run individual nodes with legacy mode
ros2 run drive_system can_manager &
ros2 run drive_system steer_control --ros-args -p use_batch_commands:=false &
ros2 run drive_system drive_control --ros-args -p use_batch_commands:=false &

# Verify legacy topic
ros2 topic list | grep can_commands
# Should show /can_commands (not /can_commands_batch)
```

## Performance Benchmarks

Use these benchmarks to verify your system is performing correctly:

### Normal Operation (60Hz Input)
```
✅ /can_commands_batch rate:     55-65 Hz
✅ Average batch size:            4 commands
✅ Inter-message jitter:          <10 ms
✅ CPU usage (can_manager):       10-30%
✅ Memory usage (can_manager):    <50 MB
✅ CAN bus load:                  <30%
```

### Stress Test (100Hz Input)
```
✅ /can_commands_batch rate:     95-105 Hz
✅ Average batch size:            4 commands
✅ Inter-message jitter:          <15 ms
✅ CPU usage (can_manager):       30-50%
✅ No dropped messages
```

### Warning Signs
```
⚠️  Batch rate <55Hz or >65Hz (at 60Hz input)
⚠️  Batch size <4 (not batching properly)
⚠️  Jitter >20ms (synchronization issues)
⚠️  CPU usage >60% (potential bottleneck)
⚠️  CAN TX errors increasing (bus overload)
```

## Debugging Commands Cheat Sheet

```bash
# Check everything is running
ros2 node list

# Check topic rates
ros2 topic hz /can_commands_batch
ros2 topic hz /steer_train
ros2 topic hz /drive_train

# Monitor messages
ros2 topic echo /can_commands_batch
ros2 topic echo /can_status

# Check parameters
ros2 param list /can_manager
ros2 param list /steer_control
ros2 param list /drive_control

# View logs
ros2 log view

# Performance test
python3 test_performance.py --duration 10

# CAN diagnostics
candump can0
cansend can0 123#DEADBEEF  # Test message
```

## Getting Help

If issues persist:

1. **Collect diagnostics:**
   ```bash
   # Save system info
   ros2 topic list > topics.txt
   ros2 node list > nodes.txt
   ros2 param list /can_manager > params.txt
   
   # Save logs
   ros2 log view > system_log.txt
   ```

2. **Run performance test:**
   ```bash
   python3 test_performance.py --duration 30 > perf_test.txt
   ```

3. **Check CAN status:**
   ```bash
   candump can0 > can_dump.txt &
   # Run for 10 seconds, then Ctrl+C
   ```

4. **Provide to team:**
   - topics.txt
   - nodes.txt
   - params.txt
   - system_log.txt
   - perf_test.txt
   - can_dump.txt

## Advanced Debugging

### Enable CAN Logging

Edit `utils/SparkCANLib/SparkController.py`:
```python
# Temporarily change DEBUG back to INFO for detailed CAN logging
self.logger.info(f"Sent position output {mod_value} to controller ID {self.id}")
```

Then rebuild:
```bash
colcon build --packages-select utils
```

### Monitor ROS2 Communication

```bash
# Install ros2 debugging tools
sudo apt install ros-foxy-rqt ros-foxy-rqt-common-plugins

# Launch rqt
rqt

# Plugins > Introspection > Node Graph (visualize connections)
# Plugins > Topics > Topic Monitor (monitor rates/throughput)
```

### Profile CPU Usage

```bash
# Install profiling tools
pip3 install py-spy

# Profile can_manager
sudo py-spy record -o profile.svg --pid $(pgrep -f can_manager)

# View profile.svg in browser to find bottlenecks
```

---

**Remember:** The optimized system is backward compatible. You can always
revert to legacy mode if needed while troubleshooting!
