# Refactoring Plan: From Single can_control to Separate Control Nodes

## Current Architecture (What We Have Now)

```
                      ┌──────────────────┐
                      │  CanControl Node │
                      │                  │
         ┌────────────┼─────────────┬────┘
         │            │             │
         ▼            ▼             ▼
    DriveControl  SteerControl  ArmControl
    (utility      (utility      (utility
     class)        class)        class)
         │            │             │
         └────────────┼─────────────┘
                      │
              Publishes CANCommand
                      │
                      ▼
         ┌──────────────────────────┐
         │   CAN Manager Node       │
         │  (Singleton SparkBus)    │
         └──────────────────────────┘
```

**Problem:** 
- Single CanControl node coordinates everything
- All high-level commands come through one node
- Hard to start/stop individual subsystems

## Proposed New Architecture

```
High-Level Inputs        Control Nodes          CAN Manager
    │                        │
    │        DriveTrain      │
    ├──────────────────→ DriveControl Node
    │      (new node)        │
    │                        │
    │       SteerTrain       │
    ├──────────────────→ SteerControl Node
    │      (new node)        │
    │                        │
    │      ArmControl        │
    ├──────────────────→ ArmControl Node
    │      (new node)        │
    │                        │
    └─────────────────────────→ CAN Manager Node
                               (Singleton)
         ┌──────────────────────────┐
         │   CAN Manager Node       │
         │  (Singleton SparkBus)    │
         │  All 13 Controllers      │
         └──────────────────────────┘
```

**Benefits:**
- Each subsystem is independent
- Can start/stop drive without affecting steer or arm
- Cleaner separation of concerns
- Each node owns its own logic and configuration
- Easier to test individually
- More ROS2-idiomatic (nodes are first-class citizens)

---

## Proposed Changes

### DELETE
- `src/drive_system/drive_system/can_control.py` - Will no longer exist

### CONVERT TO NODES
- `drive_control.py` → `drive_control_node.py` (becomes a full ROS2 node)
- `steer_control.py` → `steer_control_node.py` (becomes a full ROS2 node)
- `arm_control.py` → `arm_control_node.py` (becomes a full ROS2 node)

### UPDATE
- `package.xml` - Update entry points for new executables
- `setup.py` - Update entry points if using Python packaging

---

## Architecture Details for Each New Node

### DriveControl Node (`drive_control_node.py`)

**Responsibilities:**
- Subscribe to `/drive_train` messages
- Convert DriveTrain values to CANCommand messages
- Publish to `/can_commands` topic
- Optionally subscribe to `/can_status` for feedback

**Code Structure:**
```python
class DriveControlNode(Node):
    def __init__(self):
        super().__init__('drive_control')
        
        # Publisher to CAN manager
        self.pub_can_commands = self.create_publisher(
            CANCommand, 'can_commands', 10
        )
        
        # Subscriber to high-level commands
        self.sub_drive_train = self.create_subscription(
            DriveTrain, 'drive_train', self.drive_train_callback, 10
        )
        
        # Optional: subscribe to status
        self.sub_can_status = self.create_subscription(
            CANStatus, 'can_status', self.can_status_callback, 10
        )
        
    def drive_train_callback(self, msg: DriveTrain):
        # Convert DriveTrain to 4 CANCommand messages
        # Publish to can_commands
        
    def can_status_callback(self, msg: CANStatus):
        # Optional: process feedback from specific motors
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DriveControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Parameters (optional):**
- `motor_ids` - List of motor IDs for this subsystem (default: [1, 6, 5, 3])
- `invert_motors` - Which motors to invert (default: [6, 3])
- `namespace` - Optional namespace for this node

**Topics:**
- **Subscriptions:**
  - `/drive_train` (DriveTrain) - High-level drive commands
  - `/can_status` (CANStatus) - Optional feedback from motors
  
- **Publications:**
  - `/can_commands` (CANCommand) - Low-level motor commands to manager

---

### SteerControl Node (`steer_control_node.py`)

**Responsibilities:**
- Subscribe to `/steer_train` messages
- Convert SteerTrain values to CANCommand messages
- Publish to `/can_commands` topic
- Optionally subscribe to `/can_status` for feedback

**Code Structure:** (Same pattern as DriveControlNode)

**Parameters (optional):**
- `motor_ids` - List of motor IDs (default: [7, 10, 9, 2])
- `invert_motors` - Which motors to invert (default: [9, 2])

**Topics:**
- **Subscriptions:**
  - `/steer_train` (SteerTrain)
  - `/can_status` (CANStatus) - Optional
  
- **Publications:**
  - `/can_commands` (CANCommand)

---

### ArmControl Node (`arm_control_node.py`)

**Responsibilities:**
- Subscribe to `/arm_control` messages
- Convert Arm values to CANCommand messages (for CAN motors)
- Handle PWM servo control (claw) separately
- Publish CANCommand to manager
- Optionally subscribe to `/can_status` for feedback

**Code Structure:** (Same pattern, but with servo kit integration)

**Parameters (optional):**
- `can_motor_ids` - List of CAN motor IDs (default: [11, 12, 13, 14, 15])
- `invert_motors` - Which motors to invert (default: [15])
- `servo_channel` - Servo kit PWM channel for claw (default: 1)

**Topics:**
- **Subscriptions:**
  - `/arm_control` (Arm)
  - `/can_status` (CANStatus) - Optional
  
- **Publications:**
  - `/can_commands` (CANCommand)

---

## Node Startup Order

**Required Order:**
1. **CAN Manager** (must start first - singleton)
   ```bash
   ros2 run drive_system can_manager
   ```

2. **Control Nodes** (can start in any order, independently)
   ```bash
   ros2 run drive_system drive_control_node
   ros2 run drive_system steer_control_node
   ros2 run drive_system arm_control_node
   ```

**Advantage:** Each can be stopped/restarted independently without affecting others.

---

## Communication Flow

### Example: Drive Commands

```
User publishes DriveTrain message
         │
         ▼
    /drive_train topic
         │
         ▼
DriveControlNode receives
         │
    ┌────┴────┐
    │ Converts │
    │ 4 motors│
    └────┬────┘
         │
    Creates 4 CANCommand messages:
    - Controller 1: front_left
    - Controller 6: front_right (inverted)
    - Controller 5: back_left
    - Controller 3: back_right (inverted)
         │
         ▼
    /can_commands topic
         │
         ▼
CAN Manager receives all 4
         │
    ┌────┴──────┐
    │  Executes  │
    │ on motors  │
    └────┬──────┘
         │
    CAN Bus
         │
    Publishes CANStatus for each motor at 50Hz
         │
         ▼
    /can_status topic
         │
    Available to subscribers
```

---

## File Changes Summary

### Deleted
- ❌ `src/drive_system/drive_system/can_control.py` (86 lines)

### Created (Convert utility classes to full nodes)
- ✨ `src/drive_system/drive_system/drive_control_node.py` (~60 lines)
  - Adds Node boilerplate
  - Adds parameter management
  - Simplifies main/entry point
  
- ✨ `src/drive_system/drive_system/steer_control_node.py` (~55 lines)
  - Similar to drive_control_node
  
- ✨ `src/drive_system/drive_system/arm_control_node.py` (~70 lines)
  - Includes ServoKit initialization
  - Handles PWM servo separately

### Updated
- ✏️  `setup.py` or `package.xml` - Add entry points for 3 new executables
- ✏️  `drive_system/setup.cfg` or equivalent - Add console_scripts entries

### No Changes
- ✓ `can_manager.py` - Stays the same
- ✓ All message definitions - Stay the same
- ✓ SparkCANLib - Stays the same

---

## Entry Points Configuration

### In `setup.py` (if using setuptools)

**Before:**
```python
entry_points={
    'console_scripts': [
        'can_control=drive_system.can_control:main',
    ],
},
```

**After:**
```python
entry_points={
    'console_scripts': [
        'can_manager=drive_system.can_manager:main',
        'drive_control_node=drive_system.drive_control_node:main',
        'steer_control_node=drive_system.steer_control_node:main',
        'arm_control_node=drive_system.arm_control_node:main',
    ],
},
```

### Or in `setup.cfg`

**Add:**
```
[options.entry_points]
console_scripts =
    can_manager = drive_system.can_manager:main
    drive_control_node = drive_system.drive_control_node:main
    steer_control_node = drive_system.steer_control_node:main
    arm_control_node = drive_system.arm_control_node:main
```

---

## ROS2-Idiomatic Patterns

### Pattern 1: Control Nodes are First-Class Citizens
- Each node can be launched independently
- Each node can be restarted without affecting others
- Aligns with ROS2 philosophy

### Pattern 2: Configuration via Parameters
Each node can have parameters for customization:
```yaml
# config/drive_control.yaml
drive_control_node:
  ros__parameters:
    motor_ids: [1, 6, 5, 3]
    invert_motors: [6, 3]
    publish_rate: 50
```

### Pattern 3: Composition
Can easily compose nodes for different robot configurations:
```bash
# Basic robot
ros2 launch drive_system basic.launch.py

# Advanced robot with arm
ros2 launch drive_system full.launch.py
```

---

## Testing Strategy

### Unit Testing (Easy with separate nodes)

**DriveControlNode:**
```python
def test_drive_forward():
    node = DriveControlNode()
    
    # Mock publisher
    mock_pub = Mock()
    node.pub_can_commands = mock_pub
    
    # Send command
    msg = DriveTrain(front_left=0.5, front_right=0.5, back_left=0.5, back_right=0.5)
    node.drive_train_callback(msg)
    
    # Verify 4 commands published
    assert mock_pub.publish.call_count == 4
```

### Integration Testing (Each node independently)

```bash
# Terminal 1: Start CAN Manager
ros2 run drive_system can_manager

# Terminal 2: Start only drive control
ros2 run drive_system drive_control_node

# Terminal 3: Send commands
ros2 topic pub /drive_train mavric_msg/msg/DriveTrain "{...}"

# Terminal 4: Monitor
ros2 topic echo /can_commands  # Should see only drive commands
```

---

## Migration Path

### Step 1: Implement New Nodes
Create three new node files with full Node boilerplate

### Step 2: Update Setup
Add entry points for new nodes in setup.py/setup.cfg

### Step 3: Test Individually
Test each node independently before removing old code

### Step 4: Delete Old Code
Remove `can_control.py` once all nodes working

### Step 5: Update Documentation
- Update launch files
- Update testing guide
- Update architecture documentation

---

## Advantages of This Approach

✅ **Independent Control** - Each subsystem operates independently  
✅ **Better Fault Isolation** - One subsystem crash doesn't affect others  
✅ **Easier Scaling** - Add new subsystems as new nodes  
✅ **ROS2 Idiomatic** - Aligns with ROS2 design patterns  
✅ **Better Testing** - Test each node in isolation  
✅ **Flexible Deployment** - Start only needed subsystems  
✅ **Cleaner Code** - No coordinator node needed  
✅ **Future-Proof** - Easy to add features per subsystem

---

## Disadvantages (Minor)

⚠️ **More Nodes** - More processes to manage (trivial overhead)  
⚠️ **More Memory** - Three separate ROS2 nodes vs one (negligible)  
⚠️ **Launch Complexity** - Need to launch 4 nodes instead of 1 (mitigated with launch files)

---

## Comparison: Before vs After

### Before (Coordinator Pattern)
```
User Apps
    │
    ├─→ /drive_train ──→ ┌─────────────────────┐
    ├─→ /steer_train ──→ │   CanControl Node   │
    └─→ /arm_control  ──→ │  (Coordinator)      │
                          │  - Creates all      │
                          │  - Manages all      │
                          │  - Single point     │
                          │    of failure       │
                          └──────────┬──────────┘
                                     │
                              /can_commands
                                     │
                         ┌───────────▼──────────────┐
                         │    CAN Manager Node      │
                         │     (Singleton Bus)      │
                         └──────────────────────────┘
```

**Issues:**
- Single point of failure
- All coordination in one node
- Hard to start/stop individual subsystems

### After (Distributed Pattern)
```
User Apps
    │
    ├─→ /drive_train  ──→ ┌──────────────────┐
    │                      │ DriveControl Node│ ──┐
    │                      └──────────────────┘   │
    │                                              │
    ├─→ /steer_train  ──→ ┌──────────────────┐  │
    │                      │ SteerControl Node│  ├─→ /can_commands
    │                      └──────────────────┘  │
    │                                              │
    └─→ /arm_control  ──→ ┌──────────────────┐  │
                          │  ArmControl Node │ ──┘
                          └──────────────────┘
                                    │
                         ┌──────────▼──────────────┐
                         │    CAN Manager Node      │
                         │     (Singleton Bus)      │
                         └──────────────────────────┘
```

**Benefits:**
- Independent control nodes
- Each can fail/restart independently
- Cleaner separation
- More ROS2-idiomatic

---

## Implementation Checklist

- [ ] Create `drive_control_node.py` as full ROS2 Node
- [ ] Create `steer_control_node.py` as full ROS2 Node
- [ ] Create `arm_control_node.py` as full ROS2 Node
- [ ] Test each node individually
- [ ] Test all nodes together with can_manager
- [ ] Update `setup.py` or `setup.cfg` with entry points
- [ ] Test with: `ros2 run drive_system drive_control_node` etc.
- [ ] Delete `can_control.py`
- [ ] Update documentation:
  - Update QUICK_REFERENCE.md
  - Update TESTING_GUIDE.md
  - Update IMPLEMENTATION_SUMMARY.md
  - Update architecture diagrams
- [ ] Create new launch file for all nodes
- [ ] Update ARCHITECTURE_CHANGES.md

---

## Questions to Consider

1. **Launch Files** - Should we create a launch file to start all three nodes at once?
   - YES - Recommended for convenience
   - Could have `basic.launch.py` (drive+steer only) and `full.launch.py` (all)

2. **Namespacing** - Should nodes use namespaces?
   - NO - Keep it simple for now (topics are already clear)
   - MAYBE - Later if needed for multiple robots

3. **Shared Configuration** - Should there be a shared config file?
   - NO - Each node can have its own config
   - MAYBE - Later if patterns emerge

4. **Status Aggregation** - Should any node aggregate status from all motors?
   - NO - Keep nodes independent
   - Each can subscribe to /can_status independently if needed

---

## Next Steps

If this plan looks good:

1. **Confirm approach** - Any concerns with this architecture?
2. **Review code patterns** - Review proposed structure for each node
3. **Plan implementation** - Decide on timeline/order of changes
4. **Execute** - Implement the three new nodes
5. **Test thoroughly** - Test each node independently and together
6. **Document** - Update all documentation
7. **Clean up** - Remove old `can_control.py` and update entry points

