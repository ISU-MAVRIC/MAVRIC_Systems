# CAN Command Publisher Refactoring Summary

## Overview
Refactored `drive_control.py`, `arm_control.py`, and `steer_control.py` to use a new `CANCommandPublisher` helper class from the utils package. This eliminates code duplication across all three subsystem controllers.

## Files Created
- **`utils/can_publisher.py`** - New helper class with:
  - `publish_batch()` - Main method for publishing batched CAN commands
  - `publish_single()` - Alternative for single motor commands
  - `create_motor_commands()` - Static utility for building command tuples

## Files Refactored
1. `drive_control.py` - Drive subsystem control
2. `arm_control.py` - Arm subsystem control  
3. `steer_control.py` - Steer subsystem control

## Key Improvements

### Code Reduction
- **drive_control**: ~30 lines → ~15 lines per callback (50% reduction)
- **arm_control**: ~30 lines → ~15 lines per callback (50% reduction)
- **steer_control**: ~25 lines → ~15 lines per callback (40% reduction)

### Before: Common Pattern (Duplicated across all 3 nodes)
```python
# Create batch message with all commands
batch = CANCommandBatch()
for motor_id, value in motor_commands:
    # Apply inversion if configured
    if motor_id in self.invert_motors:
        value = value * INVERTED

    final_value = value * c_Scale

    # Only add to batch if value changed significantly
    if self.deduplicator.should_send(motor_id, final_value):
        cmd = CANCommand(
            command_type=CANCommand.VELOCITY_OUTPUT,
            controller_id=motor_id,
            value=final_value
        )
        batch.commands.append(cmd)

# Only publish if batch has commands
if batch.commands:
    self.pub_can_batch.publish(batch)
```

### After: Using CANCommandPublisher
```python
# Define motor-to-value mapping with scaling
motor_commands = [
    (self.motor_ids[0], msg.front_left * c_Scale),    # FLD
    (self.motor_ids[1], msg.front_right * c_Scale),   # FRD
    (self.motor_ids[2], msg.back_left * c_Scale),     # BLD
    (self.motor_ids[3], msg.back_right * c_Scale),    # BRD
]

# Publish batch of commands via helper
self.can_publisher.publish_batch(motor_commands)
```

## Changes per Node

### DriveControlNode
- Imports: Added `CANCommandPublisher`
- `__init__`: Initialize `self.can_publisher` with VELOCITY_OUTPUT
- `drive_train_callback`: Simplified to create motor list and call `publish_batch()`

### ArmControlNode
- Imports: Added `CANCommandPublisher`
- `__init__`: Initialize `self.can_publisher` with PERCENT_OUTPUT
- `arm_callback`: Simplified to create motor list and call `publish_batch()`
- PWM servo control for claw remains unchanged

### SteerControlNode
- Imports: Added `CANCommandPublisher`
- `__init__`: Initialize `self.can_publisher` with POSITION_OUTPUT (no motor inversions)
- `steer_train_callback`: Simplified to create motor list with directional scaling and call `publish_batch()`

## CANCommandPublisher Features

### Encapsulated Logic
- Motor inversion handling
- Deduplicator filtering
- Batch command creation
- Conditional publishing (only if batch has commands)

### Flexibility
- Configurable command types (VELOCITY_OUTPUT, PERCENT_OUTPUT, POSITION_OUTPUT)
- Override command type per-call if needed
- Support for single command publishing
- Static utility for building command tuples

### Maintainability
- Changes to CAN publishing logic only need to be made in one place
- Subsystem-specific scaling remains in individual control nodes
- Clear separation of concerns

## Testing
All three nodes maintain the same behavior as before:
- Command deduplication still works
- Motor inversion still applied correctly
- Batch publishing still conditional
- All command types preserved (VELOCITY, PERCENT, POSITION)

## Migration Notes
- `self.deduplicator` removed from individual nodes (passed to helper)
- `self.pub_can_batch` removed from individual nodes (passed to helper)
- `self.can_publisher` added to each node for public interface
- No changes to external subscriptions/publishers
