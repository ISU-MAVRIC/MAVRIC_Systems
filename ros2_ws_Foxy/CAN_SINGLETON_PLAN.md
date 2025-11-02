# CAN Bus Singleton Architecture Plan

## Current Architecture Issues

**Tight Coupling:** The `CanControl` node currently instantiates:
- `SparkBus` (CAN interface) 
- `DriveControl`, `SteerControl`, `ArmControl` (business logic classes)
- All subscriptions and callbacks

All dependent nodes must know about and import `can_control.py` directly if they need CAN functionality.

**Why External Library Singleton Won't Work:**
Each ROS node importing the SparkCAN library creates its own Python module namespace, so a singleton at library level would be duplicated across nodes.

---

## Proposed Solution: CAN Manager Singleton Node

### Architecture Overview

```
CAN Manager Node (Singleton)
├── SparkBus (single instance)
├── Controller Management
└── Custom Message Interface

         ↓ (publishes commands via ROS topics)
    
[DriveControl Node] [SteerControl Node] [ArmControl Node] ...
(independent, communicate via ROS messages)
```

### Key Design Decisions

1. **Single CAN Manager Node**: One dedicated ROS node manages the SparkBus and all controllers
   - Runs once per system
   - All CAN I/O goes through this node
   - Other nodes communicate via ROS messages

2. **Custom Message Interface**: Create a generic CAN command message to decouple control from CAN
   - Allows asynchronous, message-based communication
   - Supports future extensions easily
   - Removes direct bus dependency from control classes

3. **Decoupled Control Nodes**: DriveControl, SteerControl, ArmControl become independent
   - Subscribe to their respective high-level commands (DriveTrain, SteerTrain, Arm)
   - Publish CAN command requests to the manager
   - Can optionally consume status feedback from manager

---

## Current SparkCAN Library APIs

### SparkBus Class

**Constructor:**
```python
SparkBus(channel="can0", bustype="socketcan", bitrate=1000000, suppress_errors=True)
```

**Key Methods:**
- `init_controller(canID: int) → Controller` - Create/get controller object
- `send_msg(msg: Message) → None` - Send CAN message to bus
- `enable_heartbeat() → None` - Start heartbeat thread
- `disable_heartbeat() → None` - Stop heartbeat thread

**Internal Threads:**
- `_heartbeat_runnable()` - Sends enable array periodically (20ms interval)
- `bus_monitor()` - Receives and decodes incoming CAN messages

**State:**
- `self.controllers: dict[int, Controller]` - Active controllers by ID
- `self.can_ids: list[int]` - List of initialized controller IDs
- `self.bus` - Underlying python-can Bus (or _DummyBus in simulation)
- `self.simulated: bool` - Whether running in simulation mode

---

### Controller Class

**Constructor:**
```python
Controller(bus: SparkBus, id: int)
```

**Command Methods:**
- `percent_output(value: float) → None` - Set motor to percentage (-1 to 1)
- `velocity_output(value: float) → None` - Set motor velocity with count conversion
- `position_output(value: float) → None` - Set motor position with count conversion
- `enable() → None` - TODO: Not implemented
- `disable() → None` - TODO: Not implemented

**Status Properties:**
- `@property velocity → float` - Get current velocity from status
- `@property position → float` - Get current position from status

**State:**
- `self.bus: SparkBus` - Reference to parent bus
- `self.id: int` - CAN ID of this controller (0-15)
- `self.statuses: dict[int, Status]` - Status message decoders by API type
  - 0x60: None (reserved)
  - 0x61: Velocity status (float, uint, uint, uint)
  - 0x62: Position status (float)
  - 0x63-0x64: None (reserved)
- `self.percentProps, velocityProps, positionProps` - Scaling parameters (dir, countConversion)

**CAN Message IDs:**
- Percent output: `0x02050080 + controller_id`
- Velocity output: `0x02050480 + controller_id`
- Position output: `0x02050C80 + controller_id`

---

### Status Class

**Constructor:**
```python
Status(id: int, datasizes: tuple[int], datatypes: tuple[str])
```

**Methods:**
- `decode(msg_data: bytes) → None` - Parse incoming CAN data
- `get_value(index: int) → any` - Get decoded value by index

**State:**
- `self.data: list[any]` - Decoded values from latest message
- `self.id: int` - Full CAN arbitration ID for this status message
- `self.datasizes: tuple[int]` - Bit sizes for each field
- `self.datatypes: tuple[str]` - Type conversions ("float", "int", "uint")

**Supported Datatypes:**
- "float" - IEEE 754 float
- "int" - Signed integer
- "uint" - Unsigned integer

---

## Current Control Classes

### DriveControl

**Constructor:** `DriveControl(bus: SparkBus)`

**Motors (CAN IDs):**
- FLD = 1 (Front Left Drive)
- FRD = 6 (Front Right Drive) 
- BLD = 5 (Back Left Drive)
- BRD = 3 (Back Right Drive)

**Methods:**
- `set_velocity(msg: DriveTrain) → None` - Calls `percent_output()` on each motor
  - Applies inversion to right motors (multiplied by -1)

### SteerControl

**Constructor:** `SteerControl(bus: SparkBus)`

**Motors (CAN IDs):**
- FLS = 7 (Front Left Steer)
- FRS = 10 (Front Right Steer)
- BLS = 9 (Back Left Steer)
- BRS = 2 (Back Right Steer)

**Methods:**
- `set_velocity(msg: SteerTrain) → None` - Calls `percent_output()` on each motor
  - Applies inversion to back-left motor

### ArmControl

**Constructor:** `ArmControl(bus: SparkBus)`

**Motors (CAN IDs):**
- shoulder_pitch = 11
- shoulder_rot = 12
- elbow_pitch = 13
- wrist_pitch = 14
- wrist_rot = 15
- claw = 1 (PWM, uses ServoKit not CAN)

**Methods:**
- `set_velocity(msg: Arm) → None` - Calls `percent_output()` on CAN motors, `throttle` on servo

---

## Proposed Custom Messages

### CANCommand (Outbound: Control → Manager)

**File:** `src/mavric_msg/msg/CANCommand.msg`

**Design:** Generic, extensible command interface

```
# Command operation types
uint8 PERCENT_OUTPUT = 1
uint8 VELOCITY_OUTPUT = 2
uint8 POSITION_OUTPUT = 3
uint8 REQUEST_STATUS = 4

uint8 command_type          # Operation to perform
uint8 controller_id         # Target motor controller (0-15)
float32 value               # Command value (0 for REQUEST_STATUS)
builtin_interfaces/Time timestamp  # When issued
```

**Rationale:**
- Single message handles all motor control operations
- Extensible for future command types (enable, disable, configure)
- Decouples control logic from CAN implementation details
- Allows queuing and async processing

---

### CANStatus (Inbound: Manager → Control nodes)

**File:** `src/mavric_msg/msg/CANStatus.msg`

**Design:** Motor status feedback from controller

```
uint8 controller_id         # Which motor controller (0-15)
float32 position            # Current position (from status 0x62)
float32 velocity            # Current velocity (from status 0x61)
float32 current             # Current draw (from status 0x61, index 1)
float32 voltage             # Bus voltage (from status 0x61, index 2)
float32 last_percent_output # Last set percent output
builtin_interfaces/Time timestamp  # When status was updated
```

**Rationale:**
- Provides read-back of all important motor parameters
- Includes last commanded value for diagnostics
- Status published continuously (or on-demand if bandwidth is concern)
- Allows control nodes to monitor motor state
- Can detect stuck/failed motors

---

### Motor State Tracking

The manager must maintain state for each controller:

```python
class MotorState:
    def __init__(self, controller_id: int):
        self.controller_id = controller_id
        self.last_percent_output = 0.0
        self.last_velocity_output = 0.0
        self.last_position_output = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0
        self.voltage = 0.0
        self.last_update = None
```

Manager publishes CANStatus for each motor periodically (e.g., 50Hz)

---

## New CAN Manager Node Architecture

### Node: `can_manager.py`

**Responsibilities:**
1. Initialize SparkBus once at system startup
2. Maintain dictionary of controllers with motor state tracking
3. Subscribe to CANCommand topic from control nodes
4. Execute commands by dispatching to appropriate controller
5. Publish periodic CANStatus feedback for all initialized controllers
6. Provide internal methods for status queries

**Subscriptions:**
- `can_commands` (CANCommand) - Commands from control nodes

**Publications:**
- `can_status` (CANStatus) - Motor status published at ~50Hz
  - Published for each initialized controller
  - Updates include current position, velocity, and last command

**Parameters (configurable):**
- `can_channel` - CAN interface (default: "can0")
- `can_bustype` - Bus type (default: "socketcan")
- `can_bitrate` - Bit rate (default: 1000000)
- `status_publish_rate` - Hz to publish status (default: 50)

**Internal State:**
```python
self.motor_states = {}  # dict[int, MotorState] - state for each controller
self.controllers = {}   # dict[int, Controller] - SparkBus controllers
self.bus = SparkBus()   # Singleton instance
```

**Command Processing:**
```python
def can_command_callback(self, msg: CANCommand):
    if msg.command_type == CANCommand.PERCENT_OUTPUT:
        controller = self.get_or_init_controller(msg.controller_id)
        controller.percent_output(msg.value)
        self.motor_states[msg.controller_id].last_percent_output = msg.value
    elif msg.command_type == CANCommand.VELOCITY_OUTPUT:
        controller = self.get_or_init_controller(msg.controller_id)
        controller.velocity_output(msg.value)
        self.motor_states[msg.controller_id].last_velocity_output = msg.value
    # ... etc
```

**Status Publishing:**
```python
def status_publish_timer_callback(self):
    # Called periodically (default 50Hz = 20ms)
    for controller_id, state in self.motor_states.items():
        controller = self.controllers[controller_id]
        status = CANStatus(
            controller_id=controller_id,
            position=controller.position,
            velocity=controller.velocity,
            current=controller.statuses[0x61].data[1],
            voltage=controller.statuses[0x61].data[2],
            last_percent_output=state.last_percent_output,
            timestamp=self.get_clock().now().to_msg()
        )
        self.pub_can_status.publish(status)
```

**Internal Query Methods:**
```python
def get_motor_position(self, controller_id: int) -> float:
    """Get current position of a motor"""
    if controller_id in self.controllers:
        return self.controllers[controller_id].position
    return None

def get_motor_velocity(self, controller_id: int) -> float:
    """Get current velocity of a motor"""
    if controller_id in self.controllers:
        return self.controllers[controller_id].velocity
    return None

def get_motor_last_percent_output(self, controller_id: int) -> float:
    """Get last commanded percent output"""
    if controller_id in self.motor_states:
        return self.motor_states[controller_id].last_percent_output
    return None

def get_all_motor_states(self) -> dict[int, MotorState]:
    """Get state dict for all motors"""
    return self.motor_states.copy()
```

**Usage Pattern for Control Nodes:**
```python
# Option 1: Subscribe to status topic (recommended - decoupled)
self.sub_can_status = self.create_subscription(
    CANStatus, 'can_status', self.on_motor_status, 10
)

# Option 2: Call manager directly via service (if needed - tighter coupling)
# future work: could add service for direct queries
```

---

## Refactored Control Classes

### Current Pattern (Tightly Coupled):
```python
class DriveControl:
    def __init__(self, bus: SparkBus):  # Takes bus reference
        self.bus = bus
        self.FLMotor = self.bus.init_controller(FLD)  # Direct initialization
        
    def set_velocity(self, msg: DriveTrain):
        self.FLMotor.percent_output(msg.front_left)  # Direct method call
```

### New Pattern (Decoupled):

**Option A - Utility Class (Simpler):**
```python
class DriveControl:
    def __init__(self, can_commands_publisher):  # Takes publisher instead
        self.pub = can_commands_publisher
        
    def set_velocity(self, msg: DriveTrain):
        cmd = CANCommand(
            command_type=CANCommand.PERCENT_OUTPUT,
            controller_id=FLD,
            value=msg.front_left
        )
        self.pub.publish(cmd)  # Publish command, not execute directly
```

**Option B - Node-Based (More Idiomatic ROS):**
```python
class DriveControlNode(Node):
    def __init__(self):
        super().__init__('drive_control')
        self.pub_can_commands = self.create_publisher(
            CANCommand, 'can_commands', 10
        )
        self.sub_drive_train = self.create_subscription(
            DriveTrain, 'drive_train', self.drive_train_callback, 10
        )
        
    def drive_train_callback(self, msg: DriveTrain):
        motor_commands = [
            (FLD, msg.front_left),
            (FRD, INVERTED * msg.front_right),
            # ...
        ]
        for motor_id, value in motor_commands:
            cmd = CANCommand(
                command_type=CANCommand.PERCENT_OUTPUT,
                controller_id=motor_id,
                value=value
            )
            self.pub_can_commands.publish(cmd)
```

**Recommendation:** Option A initially (less refactoring), can evolve to Option B if needed.

---

## Implementation Roadmap

### Phase 1: Create CAN Messages
- Create `src/mavric_msg/msg/CANCommand.msg` (outbound commands)
- Create `src/mavric_msg/msg/CANStatus.msg` (inbound status)
- Update `CMakeLists.txt` and `package.xml` to build messages
- Build and test message generation

### Phase 2: Implement CAN Manager Node
- Create `src/drive_system/drive_system/can_manager.py`
- Initialize SparkBus and controller dictionary
- Track motor state for each initialized controller
- Implement CANCommand subscriber with command dispatcher
- Handle all command types (percent, velocity, position, request_status)
- Implement status publishing timer (~50Hz)
- Implement internal query methods (get_motor_position, get_motor_velocity, etc.)
- Add error handling and logging

### Phase 3: Refactor Control Classes
- Update `drive_control.py` to use publisher instead of bus
- Update `steer_control.py` to use publisher instead of bus
- Update `arm_control.py` to use publisher instead of bus
- Add status subscribers to control nodes if they need feedback

### Phase 4: Update CanControl Node
- Integrate can_manager spawn
- Remove direct SparkBus initialization
- Remove dependency injection to control classes
- Have control classes receive publisher reference

### Phase 5: Testing & Integration
- Verify single CAN bus instance across all nodes
- Test all control paths (drive, steer, arm)
- Test status publishing and reading
- Check for timing issues or race conditions
- Verify heartbeat and status decoding still works
- Test motor state tracking across multiple controllers

---

## Benefits of This Architecture

✅ **Single CAN Bus Instance** - True singleton via dedicated node
✅ **Loose Coupling** - Control nodes don't know about CAN implementation
✅ **Full State Visibility** - Can read position, velocity, current for each motor
✅ **Extensible** - Easy to add new command types, controllers, or diagnostics
✅ **ROS Best Practices** - Message-based async communication
✅ **Testable** - Can mock CANCommand publisher for unit tests
✅ **Observable** - Can monitor all CAN activity + motor state via topics
✅ **Independent Nodes** - Control classes can be developed/tested separately
✅ **Future-Proof** - Easy to add status feedback, logging, rate limiting
✅ **Bidirectional** - Publish commands, receive status - full duplex communication

---

## Potential Considerations

⚠️ **Message Latency** - Each command becomes a ROS message (typically <1ms, negligible)
⚠️ **Async Processing** - Commands are no longer synchronous (usually desired for robustness)
⚠️ **Heartbeat Coordination** - Must ensure heartbeat thread still runs in manager
⚠️ **Status Feedback** - May want separate status topic for telemetry/diagnostics
⚠️ **Error Handling** - Manager should gracefully handle invalid controller IDs

---

## Current Code Structure Summary

**SparkCANLib Location:** `src/utils/utils/SparkCANLib/`
- `SparkCAN.py` - SparkBus class (212 lines)
- `SparkController.py` - Controller class (73 lines)
- `Statuses.py` - Status decoder class (67 lines)

**Control Classes Location:** `src/drive_system/drive_system/`
- `can_control.py` - Current CanControl node (86 lines) - **TO BE REFACTORED**
- `drive_control.py` - DriveControl class (49 lines) - **TO BE REFACTORED**
- `steer_control.py` - SteerControl class (43 lines) - **TO BE REFACTORED**
- `arm_control.py` - ArmControl class (51 lines) - **TO BE REFACTORED**

**Messages Location:** `src/mavric_msg/msg/`
- `DriveTrain.msg` - Drive command
- `SteerTrain.msg` - Steer command
- `Arm.msg` - Arm command
- `CANCommand.msg` - **NEW MESSAGE TO CREATE** (outbound control)
- `CANStatus.msg` - **NEW MESSAGE TO CREATE** (inbound feedback)


---

## Performance Analysis: 13 Motors @ 50Hz

### Current Publishing Plan: Baseline
- **13 controllers** each publishing CANStatus
- **50Hz** publish rate = **20ms interval** 
- **Total: 13 messages every 20ms**

### Message Size Calculation
CANStatus message fields:
- `uint8 controller_id` = 1 byte
- `float32 position` = 4 bytes
- `float32 velocity` = 4 bytes
- `float32 current` = 4 bytes
- `float32 voltage` = 4 bytes
- `float32 last_percent_output` = 4 bytes
- `builtin_interfaces/Time timestamp` = 8 bytes (2x uint32)

**Per-message overhead** (ROS2 DDS):
- ROS headers/serialization: ~50-100 bytes (varies by middleware)
- Total per message: ~85 bytes

**Bandwidth Calculation:**
- 13 messages × 85 bytes × 50 Hz = **55.25 KB/s**
- Peak burst (13 messages in quick succession): ~1.1 KB

### CPU Usage Estimate
- **Message creation**: Minimal (~0.1ms per message)
- **13 messages × 0.1ms** = ~1.3ms total per publish cycle
- **Every 20ms**: ~6.5% CPU for message publishing alone
- **Reasonable** (well under 50ms ROS cycle time)

### Latency
- **Status latency**: 0-20ms (depends on when in cycle status is read)
- **Command-to-status round-trip**: ~40-60ms (two publish cycles)
- **Acceptable** for non-real-time robot control

### Network/Middleware Considerations

**Local DDS (same machine):**
- ✅ No network I/O, just shared memory
- ✅ Can handle many more messages easily
- ✅ Negligible overhead

**Network DDS (distributed ROS2):**
- ⚠️ 50Hz × 13 = 650 messages/sec across network
- ⚠️ Bandwidth: ~56 KB/s (modest but notable)
- ⚠️ May need QoS tuning (RELIABLE vs BEST_EFFORT)

### Verdict: Current Plan ✅ PERFORMS WELL

**For 13 motors:**
- 55 KB/s bandwidth is trivial
- 6.5% CPU usage is acceptable
- Latency is reasonable for robot control
- **No optimization needed initially**

---

## Performance Optimization Options (If Needed)

### Option 1: Reduce Publish Rate (Conservative)
Change from 50Hz to 20Hz or 30Hz:
- **20Hz**: 13 × 85B × 20 = 22.1 KB/s, 2.6% CPU
- **30Hz**: 13 × 85B × 30 = 33.15 KB/s, 3.9% CPU
- **Trade-off**: More stale telemetry (50-100ms old)
- **Recommendation**: Only if network is severely constrained

### Option 2: Selective Publishing (Moderate)
Only publish when motor state *changes significantly*:
- Uses hysteresis thresholds (e.g., velocity changes >5%)
- Reduces messages on idle motors
- Estimated: 10-30 KB/s average (depends on activity)
- **Trade-off**: More complex logic, variable latency
- **Recommendation**: Good for telemetry, not control feedback

### Option 3: Aggregate Messages (Moderate)
Bundle multiple motor statuses into one message:
- `CANStatusArray` with 2-3 motors per message
- Reduces from 13 messages to ~5 messages per cycle
- Estimated: 20-25 KB/s
- **Trade-off**: Slightly more complex unpacking on receiver end
- **Recommendation**: Good compromise if network is tight

### Option 4: On-Demand Queries (Advanced)
Replace periodic publishing with ROS2 Services:
- Control nodes request status when needed
- Service call: `request_motor_status(controller_id) → CANStatus`
- Only publishes when requested
- Estimated: <1 KB/s baseline, higher during active queries
- **Trade-off**: No continuous telemetry stream, service call latency ~5-10ms
- **Recommendation**: If you don't need real-time feedback, adds complexity

### Option 5: Hybrid Approach (Best)
**Default behavior:**
- Publish at 10-20Hz for continuous monitoring
- Control nodes can also make service calls for immediate feedback
- Best of both worlds

---

## Recommended Configuration

For your use case (robot with 13 motors, local robot computer):

```python
# In can_manager.py launch config
can_manager_node = Node(
    package='drive_system',
    executable='can_manager',
    parameters=[
        ('can_channel', 'can0'),
        ('can_bustype', 'socketcan'),
        ('can_bitrate', 1000000),
        ('status_publish_rate', 50),  # 50Hz - good default
        ('status_hysteresis_enabled', False),  # Disable for now
    ]
)
```

**Stay with 50Hz because:**
1. ✅ Bandwidth (55 KB/s) is negligible for local robot
2. ✅ CPU (6.5%) is well within budget
3. ✅ Latency (0-20ms) is acceptable for control
4. ✅ Simpler code, no hysteresis logic needed
5. ✅ Excellent observability for debugging

---

## Network Deployment Considerations

If you ever need to **network multiple robots or stream telemetry over limited bandwidth**, then consider:
- Option 3 (Aggregate) for modest bandwidth savings
- Option 5 (Hybrid) for flexible adaptation
- Adjustable rate via parameters for runtime tuning

But **for now, 50Hz with 13 motors is perfectly fine** ✅

