# ServoKit Singleton Architecture - Planning Document

**Date:** November 2, 2025  
**Status:** Planning Phase (No Code Written)  
**Objective:** Refactor servo control to use singleton pattern like CAN manager

---

## Current Problem

### arm_control.py Issues
```python
# Currently does this:
try:
    from adafruit_servokit import ServoKit
    self.kit = ServoKit(channels=16)  # Creates NEW instance in every node
    self.servo_channel = servo_channel
except Exception as e:
    self.kit = None  # Silent failure
```

**Problems:**
1. ❌ Each node creates its own ServoKit instance
2. ❌ Only arm_control.py can control servos (tightly coupled)
3. ❌ No logging or state tracking
4. ❌ Hard to mock for testing
5. ❌ No centralized servo state management
6. ❌ Shared I2C bus conflicts (multiple PCA9685 access)

---

## Proposed Solution: Singleton Pattern

### Architecture Overview
```
┌─────────────────────────────────────────────────────┐
│                  ROS2 Application                   │
├─────────────────────────────────────────────────────┤
│                                                     │
│  drive_control  steer_control  arm_control  ...     │
│       │               │              │               │
│       └───────────────┼──────────────┘               │
│                       │ (publish to /servo_commands) │
│                       ↓                              │
│            ┌──────────────────────┐                 │
│            │  servo_manager_node  │ ← NEW SINGLETON │
│            │  (owns ServoKit)     │   NODE          │
│            └──────────────────────┘                 │
│                       │                              │
│                       ↓                              │
│            ┌──────────────────────┐                 │
│            │  ServoKit Singleton  │                 │
│            │  (wrapper + mock)    │                 │
│            └──────────────────────┘                 │
│                       │                              │
│                       ↓                              │
│            ┌──────────────────────┐                 │
│            │  adafruit_servokit   │                 │
│            │  (actual hardware)   │                 │
│            └──────────────────────┘                 │
│                       │                              │
│                       ↓                              │
│            Hardware (PCA9685 I2C)                   │
└─────────────────────────────────────────────────────┘
```

---

## Implementation Plan

### Phase 1: Create Mock Object in utils/

#### File: `src/utils/utils/ServoKitMock.py`

**Purpose:** Mock ServoKit for testing/development without hardware

**Structure:**
```python
class ServoKitMock:
    """Mock adafruit_servokit.ServoKit for testing"""
    
    def __init__(self, channels=16):
        self.channels = channels
        self.continuous_servo = {}
        self._servo_values = {}
        self._enabled = True
        self._logger = None  # Optional ROS logger
        # Initialize all servo objects
    
    def set_logger(self, logger):
        """Allow optional ROS logger for debugging"""
        self._logger = logger
        
    def log_servo_command(self, channel, throttle):
        """Log servo command (for debugging)"""
        if self._logger:
            self._logger.debug(f"ServoKit[{channel}].throttle = {throttle}")
        else:
            print(f"[ServoKit Mock] Channel {channel}: throttle = {throttle}")
    
    # Implement continuous_servo interface
    # Track all servo values
    # Provide telemetry/status
```

**Key Features:**
- ✅ Drop-in replacement for ServoKit
- ✅ Logs all servo commands
- ✅ Can be used in development/testing
- ✅ Track servo state
- ✅ Optional ROS logger integration

---

### Phase 2: Create ServoKit Singleton Wrapper

#### File: `src/utils/utils/ServoKitSingleton.py`

**Purpose:** Centralized ServoKit access with singleton pattern

**Structure:**
```python
class ServoKitSingleton:
    """Singleton wrapper for adafruit_servokit.ServoKit"""
    
    _instance = None
    _use_mock = False  # Can be set to True for testing
    
    def __new__(cls, channels=16, use_mock=False):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize(channels, use_mock)
        return cls._instance
    
    def _initialize(self, channels, use_mock):
        """Initialize the servo kit (real or mock)"""
        if use_mock or (check if hardware available):
            self.kit = ServoKitMock(channels)
        else:
            self.kit = ServoKit(channels)
        
        self.channels = channels
        self.servo_states = {}
        self._logger = None
    
    def set_logger(self, logger):
        """Set ROS logger for debugging"""
        self._logger = logger
        if hasattr(self.kit, 'set_logger'):
            self.kit.set_logger(logger)
    
    def get_servo(self, channel):
        """Get servo object for channel"""
        # Delegate to kit
    
    def set_throttle(self, channel, throttle):
        """Set servo throttle with logging"""
        # Log command
        # Set throttle
        # Track state
    
    def get_status(self):
        """Return all servo statuses"""
```

**Key Features:**
- ✅ True singleton (only one instance ever)
- ✅ Automatic fallback to mock if hardware unavailable
- ✅ Comprehensive logging
- ✅ State tracking for all servos
- ✅ Thread-safe access

---

### Phase 3: Create servo_manager Node

#### File: `src/drive_system/drive_system/servo_manager.py`

**Purpose:** Singleton node that owns servo control

**Structure:**
```python
class ServoManager(Node):
    """
    Singleton servo manager node.
    
    - Owns the single ServoKit instance
    - Subscribes to ServoCommand messages
    - Publishes ServoStatus messages
    - Handles servo state and telemetry
    """
    
    def __init__(self):
        super().__init__("servo_manager")
        
        # Declare parameters
        self.declare_parameter("servo_channels", 16)
        self.declare_parameter("use_mock", False)  # For testing
        self.declare_parameter("status_publish_rate", 10)  # Hz
        
        # Get parameters
        servo_channels = self.get_parameter("servo_channels").value
        use_mock = self.get_parameter("use_mock").value
        
        # Initialize ServoKit singleton
        self.servo_kit = ServoKitSingleton(
            channels=servo_channels,
            use_mock=use_mock
        )
        self.servo_kit.set_logger(self.get_logger())
        
        # Create subscriber for servo commands
        self.sub_servo_commands = self.create_subscription(
            ServoCommand, "servo_commands", self.servo_command_callback, 10
        )
        
        # Create publisher for servo status
        self.pub_servo_status = self.create_publisher(
            ServoStatus, "servo_status", 10
        )
        
        # Create timer for status publishing
        status_interval = 1.0 / status_publish_rate
        self.status_timer = self.create_timer(
            status_interval, self.status_publish_timer_callback
        )
```

**Key Features:**
- ✅ Owns ServoKit singleton
- ✅ ROS node interface
- ✅ Message-based servo commands
- ✅ Status publishing
- ✅ Can run standalone or with other nodes

---

### Phase 4: Create Custom ROS Messages

#### File: `src/mavric_msg/msg/ServoCommand.msg`

```
uint8 servo_channel      # 0-15
float32 throttle         # -1.0 to 1.0 (for continuous servo)
builtin_interfaces/Time timestamp
```

#### File: `src/mavric_msg/msg/ServoStatus.msg`

```
uint8 servo_channel      # 0-15
float32 throttle         # Current throttle value
bool is_active           # Is servo responding
string error_msg         # Error message if any
builtin_interfaces/Time timestamp
```

---

### Phase 5: Refactor arm_control.py

**Changes:**
1. ❌ Remove ServoKit initialization
2. ❌ Remove try/except for servo setup
3. ✅ Create publisher to `servo_commands` topic
4. ✅ Publish ServoCommand when claw command received
5. ✅ Optional: subscribe to `servo_status` for feedback

**Before:**
```python
try:
    from adafruit_servokit import ServoKit
    self.kit = ServoKit(channels=16)
    self.servo_channel = servo_channel
except Exception as e:
    self.kit = None

# Later...
if self.kit is not None:
    try:
        self.kit.continuous_servo[self.servo_channel].throttle = msg.claw
    except Exception as e:
        self.get_logger().warn(...)
```

**After:**
```python
# Just publish servo command
servo_cmd = ServoCommand()
servo_cmd.servo_channel = CLAW_SERVO_CHANNEL
servo_cmd.throttle = msg.claw
servo_cmd.timestamp = self.get_clock().now().to_msg()
self.pub_servo_commands.publish(servo_cmd)
```

---

## File Structure After Refactoring

```
ros2_ws_Foxy/
├── src/
│   ├── utils/
│   │   └── utils/
│   │       ├── __init__.py
│   │       ├── SparkCANLib/
│   │       │   └── ...
│   │       ├── ServoKitMock.py          ← NEW (mock object)
│   │       ├── ServoKitSingleton.py     ← NEW (singleton wrapper)
│   │       └── servo_utils.py            ← NEW (utility functions)
│   │
│   ├── drive_system/
│   │   ├── drive_system/
│   │   │   ├── arm_control.py           ← REFACTORED
│   │   │   ├── servo_manager.py         ← NEW (singleton node)
│   │   │   ├── can_manager.py           ← UNCHANGED
│   │   │   ├── drive_control.py         ← UNCHANGED
│   │   │   └── steer_control.py         ← UNCHANGED
│   │   └── setup.py                     ← UPDATED (add servo_manager entry point)
│   │
│   ├── mavric_msg/
│   │   ├── msg/
│   │   │   ├── ServoCommand.msg         ← NEW
│   │   │   ├── ServoStatus.msg          ← NEW
│   │   │   └── ...existing
│   │   └── CMakeLists.txt               ← UPDATED
│   │
│   └── mavric_launch/
│       └── launch/
│           ├── servo.launch.py          ← NEW (just servo_manager)
│           └── teleop.launch.py         ← UPDATED (add servo_manager)
```

---

## Key Design Decisions

### 1. Why Singleton Pattern?
- ✅ Single I2C connection to PCA9685
- ✅ Prevents resource conflicts
- ✅ Centralized state tracking
- ✅ Matches CAN manager pattern

### 2. Why Mock Object?
- ✅ Test code without hardware
- ✅ Development/simulation mode
- ✅ Easy debugging via logging
- ✅ Can disable/enable real hardware

### 3. Why separate servo_manager node?
- ✅ Decoupled from arm_control
- ✅ Other subsystems can control servos too (future)
- ✅ Clean separation of concerns
- ✅ Can run independently
- ✅ Matches CAN manager architecture

### 4. Why ROS messages?
- ✅ Clean interface
- ✅ Can republish for debugging
- ✅ Easy to log/record
- ✅ Future flexibility (add actuators without code changes)

---

## Integration Points

### arm_control.py will:
```python
# Subscribe to Arm message (unchanged)
self.sub_arm = self.create_subscription(Arm, "arm_control", ...)

# NEW: Publish to servo_commands
self.pub_servo_commands = self.create_publisher(
    ServoCommand, "servo_commands", 10
)

# In callback:
servo_cmd = ServoCommand()
servo_cmd.servo_channel = CLAW_SERVO_CHANNEL
servo_cmd.throttle = msg.claw
servo_cmd.timestamp = self.get_clock().now().to_msg()
self.pub_servo_commands.publish(servo_cmd)
```

### servo_manager.py will:
```python
# Subscribe to servo commands
self.sub_servo_commands = self.create_subscription(
    ServoCommand, "servo_commands", self.servo_command_callback, 10
)

# Own the ServoKit singleton
self.servo_kit = ServoKitSingleton(...)

# Process commands and control hardware
def servo_command_callback(self, msg: ServoCommand):
    self.servo_kit.set_throttle(msg.servo_channel, msg.throttle)
```

---

## Benefits of This Architecture

### Decoupling
- ✅ arm_control doesn't know about servo hardware
- ✅ servo_manager owns servo hardware
- ✅ Easy to add new servo-controlled subsystems
- ✅ Easy to swap implementations (mock ↔ real)

### Testing
- ✅ Mock ServoKit for unit tests
- ✅ No hardware required for development
- ✅ Complete logging of all servo commands
- ✅ Reproducible test scenarios

### Maintainability
- ✅ Single point of servo management
- ✅ Clear responsibilities
- ✅ Follows established patterns (CAN manager)
- ✅ Easy to add features (PID, failsafe, etc.)

### Debugging
- ✅ Log all servo commands
- ✅ Monitor servo status
- ✅ Track servo states over time
- ✅ Easy to replay scenarios

---

## Implementation Order

1. **Create ServoKitMock.py**
   - Implement mock servo objects
   - Add logging support
   - Test with logging enabled

2. **Create ServoKitSingleton.py**
   - Implement singleton pattern
   - Hardware detection/fallback
   - State tracking

3. **Create ServoCommand.msg and ServoStatus.msg**
   - Define message formats
   - Update CMakeLists.txt
   - Build messages

4. **Create servo_manager.py**
   - Node initialization
   - Subscription handling
   - Command processing
   - Status publishing

5. **Refactor arm_control.py**
   - Remove ServoKit init
   - Add servo_commands publisher
   - Update callback to publish commands
   - Test integration

6. **Update launch files**
   - Add servo_manager to teleop.launch.py
   - Create standalone servo.launch.py
   - Update setup.py entry points

---

## Testing Strategy

### Unit Tests (For Later)
- Mock ServoKit functionality
- Singleton thread safety
- Message generation

### Integration Tests
- servo_manager starts without errors
- Receives servo commands
- Publishes status
- arm_control publishes servo commands correctly

### Hardware Tests
- Real PCA9685 works with singleton
- Multiple nodes can request servo control
- Servo commands execute correctly

---

## Potential Issues and Mitigation

### Issue 1: Multiple servo command sources
**Solution:** servo_manager processes all commands on same I2C bus (no conflicts)

### Issue 2: Servo command latency
**Solution:** Direct I2C write (minimal latency), status publishing optional

### Issue 3: Servo feedback/position tracking
**Solution:** Can add position feedback message later if needed

### Issue 4: Node ordering (servo_manager must start first)
**Solution:** Launch file ensures order, or lazy initialization if needed

---

## Summary

**Architecture:** Singleton + Mock + Manager Node (like CAN)  
**Components:** 4 new files + refactored arm_control + new messages + updated launch  
**Benefits:** Decoupled, testable, maintainable, extensible  
**Complexity:** Medium (straightforward singleton pattern)  
**Timeline:** ~2 hours of implementation  

**Status:** ✅ Plan Complete - Ready for Implementation

