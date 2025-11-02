# ServoKit Singleton - Refined Design

**Date:** November 2, 2025  
**Status:** Refined Planning (No Code Written)  
**Changes:** Auto hardware detection, simplified messages, singleton library approach

---

## Design Decision: Library Singleton vs Node Singleton

### Question: Will a ServoKit singleton (library) work in ROS without a dedicated node?

**Answer: YES, absolutely works!**

**Why it works:**
- ✅ Singleton is just a Python class pattern
- ✅ Can be imported and used by ANY node
- ✅ Does NOT require being a ROS node itself
- ✅ Multiple ROS nodes can safely use the same singleton instance
- ✅ Exactly how SparkBus works in can_manager!

**Example:**
```python
# Multiple nodes can do this
from utils.ServoKitSingleton import ServoKitSingleton

servo_kit = ServoKitSingleton(channels=16)  # First call creates it
servo_kit = ServoKitSingleton(channels=16)  # Second call returns SAME instance
```

### Comparison: Library Singleton vs Manager Node

| Aspect | Library Singleton | Manager Node |
|--------|-------------------|--------------|
| **ROS Requirement** | ❌ No | ✅ Yes |
| **Imports Everywhere** | ✅ Simple | ❌ Verbose |
| **State Management** | ✅ Centralized | ✅ Centralized |
| **Hardware Init** | ✅ On first use | ✅ On node start |
| **Multiple Nodes Using** | ✅ Easy | ✅ Via messages |
| **Testing** | ✅ Easy mock | ✅ Easy mock |
| **Lines of Code** | ✅ ~100 | ❌ ~150 |
| **Complexity** | ✅ Lower | ❌ Higher |

### Simplified Approach: Skip the Manager Node

**Simpler Architecture:**
```
arm_control.py          steer_control.py        drive_control.py
      │                       │                        │
      └─────────────────────────┬──────────────────────┘
                                │
                                ↓
                    ServoKitSingleton (Library)
                         (auto-detects hardware)
                                │
                  ┌─────────────┴──────────────┐
                  ↓                            ↓
         adafruit_servokit.ServoKit     ServoKitMock
              (if available)          (if hw unavailable)
                  │                            │
                  └─────────────┬──────────────┘
                                ↓
                     Hardware (PCA9685 I2C)
```

**Benefits:**
- ✅ No manager node to run
- ✅ Cleaner for arm_control
- ✅ Still centralized and safe
- ✅ Automatic hardware detection
- ✅ Same safety as node approach

---

## Refined Implementation Plan

### Phase 1: ServoKitMock.py (No Changes)

**Location:** `src/utils/utils/ServoKitMock.py`

```python
class ServoKitMock:
    """Mock adafruit_servokit.ServoKit for testing without hardware"""
    
    def __init__(self, channels=16):
        self.channels = channels
        self.continuous_servo = {}
        # Initialize mock servo objects for each channel
        for ch in range(channels):
            self.continuous_servo[ch] = MockServo(ch)
    
    # Implements exact same interface as adafruit_servokit.ServoKit

class MockServo:
    """Individual mock servo"""
    
    def __init__(self, channel):
        self.channel = channel
        self.throttle = 0.0
        
    @property
    def throttle(self):
        return self._throttle
    
    @throttle.setter
    def throttle(self, value):
        self._throttle = value
        print(f"[ServoKit Mock] Channel {self.channel}: throttle = {value}")
```

**Key Feature:** Automatic logging of all servo commands via print()

---

### Phase 2: ServoKitSingleton.py (New - Library Pattern)

**Location:** `src/utils/utils/ServoKitSingleton.py`

```python
class ServoKitSingleton:
    """
    Singleton wrapper for adafruit_servokit.ServoKit
    
    Automatically detects hardware availability and falls back to mock.
    Can be imported and used by any ROS node.
    """
    
    _instance = None
    
    def __new__(cls, channels=16):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialize(channels)
        return cls._instance
    
    def _initialize(self, channels):
        """Initialize - auto-detect hardware, fallback to mock"""
        try:
            # Try to import real ServoKit
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=channels)
            print(f"✓ Using real ServoKit (hardware detected)")
        except Exception as e:
            # Fallback to mock
            print(f"⚠ Real ServoKit init failed ({e}), using mock")
            from utils.ServoKitMock import ServoKitMock
            self.kit = ServoKitMock(channels=channels)
            print(f"✓ Using ServoKitMock for testing")
        
        self.channels = channels

    @property
    def continuous_servo(self):
        """Delegate to underlying kit"""
        return self.kit.continuous_servo
```

**Key Features:**
- ✅ True singleton pattern
- ✅ Automatic hardware detection
- ✅ Automatic fallback to mock
- ✅ Transparent to users
- ✅ No ROS dependency

---

### Phase 3: Simplified Messages

**REMOVE:** `ServoStatus.msg` (not needed)

**KEEP:** `ServoCommand.msg` 

Wait - **Actually REMOVE THIS TOO** if we're not using a manager node!

**New Simplified Flow:**
```
arm_control directly imports and uses ServoKitSingleton
No messages needed for servo control
Just direct Python calls
```

---

### Phase 4: Refactored arm_control.py

**OLD CODE:**
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
        self.get_logger().warn(f"Failed to control claw servo: {e}")
```

**NEW CODE:**
```python
# In __init__:
from utils.ServoKitSingleton import ServoKitSingleton

self.servo_kit = ServoKitSingleton(channels=16)
self.servo_channel = servo_channel

# In callback:
try:
    self.servo_kit.continuous_servo[self.servo_channel].throttle = msg.claw
    self.get_logger().debug(f"Claw servo set to {msg.claw}")
except Exception as e:
    self.get_logger().error(f"Failed to control claw servo: {e}")
```

**Benefits:**
- ✅ Much simpler
- ✅ Hardware auto-detected once
- ✅ Automatic mock fallback
- ✅ Cleaner error handling
- ✅ Direct Python approach (no ROS messages)

---

## Final Simplified Architecture

### What to Create

**File 1:** `src/utils/utils/ServoKitMock.py`
- Mock servo objects
- Logs all commands
- Drop-in replacement

**File 2:** `src/utils/utils/ServoKitSingleton.py`
- Library singleton pattern
- Auto hardware detection
- Auto fallback to mock

### What to Modify

**File 1:** `src/drive_system/drive_system/arm_control.py`
- Import ServoKitSingleton
- Remove try/except for init
- Simplified servo control
- Direct Python calls

**File 2:** `src/utils/utils/__init__.py` (optional)
- Export ServoKitSingleton for easier imports

### What to NOT Create

- ❌ ServoCommand.msg
- ❌ ServoStatus.msg
- ❌ servo_manager.py
- ❌ servo.launch.py
- ❌ No ROS node needed

### Changes to Setup Files

**Minimal Changes:**
- ✅ arm_control.py updated (uses singleton)
- ✅ Everything else stays the same
- ✅ No new entry points needed
- ✅ No launch file changes needed

---

## How It Works

### Scenario 1: Running on Raspberry Pi (Hardware Available)

```
1. arm_control imports ServoKitSingleton
2. First call to ServoKitSingleton(channels=16)
3. Tries to import adafruit_servokit
4. ✓ Success! Creates real ServoKit
5. arm_control.servo_kit now uses real hardware
6. All servo commands go to real PCA9685
7. Claw works!

Later, if another node tries:
8. Another node imports ServoKitSingleton
9. Calls ServoKitSingleton(channels=16)
10. ✓ Returns SAME instance
11. Uses same real ServoKit
12. No conflicts, single I2C connection
```

### Scenario 2: Running on Development Laptop (No Hardware)

```
1. arm_control imports ServoKitSingleton
2. First call to ServoKitSingleton(channels=16)
3. Tries to import adafruit_servokit
4. ✗ Failed! (not installed)
5. Exception caught, falls back to mock
6. Creates ServoKitMock
7. arm_control.servo_kit now uses mock
8. All servo commands logged to console
9. Claw "works" (simulated)
10. Development continues without hardware

Output:
  ⚠ Real ServoKit init failed, using mock
  ✓ Using ServoKitMock for testing
  [ServoKit Mock] Channel 1: throttle = 0.5
  [ServoKit Mock] Channel 1: throttle = -0.5
```

---

## Code Comparison

### Before (Current arm_control.py)

**Lines:** ~15 for servo init  
**Issues:** 
- Tightly coupled
- Silent failures possible
- Hard to test
- No logging

```python
try:
    from adafruit_servokit import ServoKit
    self.kit = ServoKit(channels=16)
    self.servo_channel = servo_channel
    self.get_logger().info(f"ServoKit initialized...")
except Exception as e:
    self.get_logger().warn(f"Failed to initialize ServoKit: {e}")
    self.kit = None

# Usage:
if self.kit is not None:
    try:
        self.kit.continuous_servo[self.servo_channel].throttle = msg.claw
    except Exception as e:
        self.get_logger().warn(f"Failed to control claw servo: {e}")
```

### After (Simplified)

**Lines:** ~5 for servo init  
**Benefits:**
- Decoupled
- Auto hardware detection
- Automatic logging
- Exception-safe

```python
from utils.ServoKitSingleton import ServoKitSingleton

# In __init__:
self.servo_kit = ServoKitSingleton(channels=16)
self.servo_channel = CLAW_SERVO_CHANNEL

# In callback - direct, clean:
self.servo_kit.continuous_servo[self.servo_channel].throttle = msg.claw
```

---

## Thread Safety

**Question:** Is the singleton thread-safe in ROS?

**Answer:** Yes, but with caveats:

**Safe because:**
- ✅ Python GIL (Global Interpreter Lock)
- ✅ `__new__` is atomic enough for ROS
- ✅ Multiple ROS nodes run in same process
- ✅ By the time 2nd node starts, singleton already created

**Make it more explicit:**
```python
import threading

class ServoKitSingleton:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, channels=16):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:  # Double check
                    cls._instance = super().__new__(cls)
                    cls._instance._initialize(channels)
        return cls._instance
```

---

## Error Handling Strategy

### Scenario 1: Hardware Init Fails

```
try:
    self.kit = ServoKit(channels=16)
except Exception as e:
    # Fall back to mock
    self.kit = ServoKitMock(channels=16)
    # Logging already done
```

### Scenario 2: Servo Command Fails

```python
# In arm_control callback:
try:
    self.servo_kit.continuous_servo[channel].throttle = value
except Exception as e:
    self.get_logger().error(f"Servo command failed: {e}")
    # Graceful degradation - try next command
```

**Both scenarios handled automatically.**

---

## Testing Without Hardware

**Development Mode:**

```bash
# On laptop without PCA9685 hardware
# Install: pip install adafruit-circuitpython-servokit will FAIL
# That's OK!

# When arm_control starts:
python3 arm_control.py

# Output:
# ⚠ Real ServoKit init failed (No module named 'adafruit_servokit')
# ✓ Using ServoKitMock for testing
# 
# [ServoKit Mock] Channel 1: throttle = 0.5
# [ServoKit Mock] Channel 1: throttle = -0.3
# [ServoKit Mock] Channel 1: throttle = 0.0
```

**Complete development loop without any hardware.**

---

## Hardware Detection Detailed

### Current adafruit_servokit Installation

**On Raspberry Pi:**
```bash
pip install adafruit-circuitpython-servokit
# ✓ Success - hardware available
# adafruit_servokit can be imported
```

**On Development Laptop:**
```bash
pip install adafruit-circuitpython-servokit
# ✓ Package installs, but...
# When ServoKit tries to access I2C:
# ImportError or Exception (no /dev/i2c-1)
```

**Our Auto-Detection:**
```python
try:
    from adafruit_servokit import ServoKit
    self.kit = ServoKit(channels=16)  # This line might also fail if I2C missing
    print("✓ Real ServoKit working")
except Exception:  # Catches BOTH import and init failures
    self.kit = ServoKitMock(channels=16)
    print("✓ Using mock")
```

**Works either way!**

---

## Final Simplified Files

### What Gets Created

**File 1: src/utils/utils/ServoKitMock.py**
- Lines: ~50
- Complexity: Very simple
- Time: ~10 min

**File 2: src/utils/utils/ServoKitSingleton.py**
- Lines: ~50
- Complexity: Simple
- Time: ~10 min

**Total New Code: ~100 lines**

### What Gets Modified

**File 1: src/drive_system/drive_system/arm_control.py**
- Remove: ~10 lines (old servo init)
- Add: ~3 lines (new import + init)
- Net change: -7 lines simpler

**Total Modified: ~20 lines**

---

## Why This Approach is Better

### Simpler than Manager Node Approach

❌ Manager Node:
- Need to create node
- Need to create messages
- Need to create launch config
- Need to manage node startup order
- Need to manage message routing

✅ Library Singleton:
- Import and use
- Auto hardware detection
- Auto mock fallback
- Works wherever needed
- No message overhead

### Still Gets All Benefits

✅ Single I2C connection (singleton)  
✅ No resource conflicts  
✅ Centralized access  
✅ Easy to test (mock)  
✅ Automatic logging  
✅ Hardware abstraction  

### Cleaner Than Current

✅ No try/except in every node  
✅ No `if self.kit is not None` checks  
✅ Automatic fallback  
✅ Simpler code  
✅ Better error messages  

---

## Implementation Roadmap (Simplified)

**Step 1:** Create ServoKitMock.py (~10 min)
- Simple mock class
- Mock servo objects
- Print logging

**Step 2:** Create ServoKitSingleton.py (~10 min)
- Singleton pattern
- Try/except for hardware detection
- Fallback to mock

**Step 3:** Update arm_control.py (~5 min)
- Remove old servo init
- Import ServoKitSingleton
- Simplify callback

**Total Time: ~25 minutes**

---

## Summary

### Original Approach
- Manager node
- Custom messages
- Complex architecture
- Good for large systems

### Simplified Approach (Recommended)
- Library singleton
- Auto hardware detection
- Much simpler
- Perfect for ROS control nodes

### Code Reduction
- Remove: servo_manager.py (150 lines)
- Remove: Messages (10 lines)
- Remove: Launch config changes
- Add: ServoKitMock (50 lines)
- Add: ServoKitSingleton (50 lines)
- Modify: arm_control.py (-7 lines simpler)

**Net result: ~50 lines simpler, way cleaner**

---

## Answer to Your Questions

### Q1: Auto hardware detection for simplicity?
**YES.** Code just does:
```python
try:
    real_kit
except:
    mock_kit
```
Super simple, works perfectly.

### Q2: Remove servo status publisher?
**YES.** We don't need messages at all!
- No ServoCommand.msg
- No ServoStatus.msg
- Just direct Python calls

### Q3: Will singleton work in ROS without being a node?
**YES! Absolutely.**
- Singleton is just a Python pattern
- Works in any module
- Multiple nodes can import and use
- Thread-safe with ROS

---

## Status

✅ **Simplified Plan Complete**
✅ **Only 2 new utility files needed**
✅ **Only 1 file to refactor (arm_control)**
✅ **~25 minutes to implement**
✅ **Ready to code**

