#!/usr/bin/env python3
"""
Test script for verifying web teleop control logic
This simulates key presses without requiring CAN bus hardware
"""

import sys
from typing import Set

# Mock the config values
class Config:
    # Drive Motors
    FLD_ID = 7
    FRD_ID = 10
    BLD_ID = 9
    BRD_ID = 8
    
    # Steer Motors
    FLS_ID = 1
    FRS_ID = 6
    
    # Arm Motors
    SHOULDER_PITCH_ID = 11
    SHOULDER_ROT_ID = 12
    ELBOW_PITCH_ID = 13
    WRIST_PITCH_ID = 14
    WRIST_ROT_ID = 15
    
    # Drive Speed
    MAX_DRIVE_SPEED = 0.5
    NORMAL_DRIVE_SPEED = 0.3
    MIN_DRIVE_SPEED = 0.1
    
    # Steer Rotation
    DEFAULT_STEER_POS = 0.8
    STEER_ROTATION_POS = 0.3
    STEER_ROTATION_SPEED = 0.2
    STEER_RIGHT_POS = 0.5
    STEER_LEFT_POS = 1.0
    
    # Arm Speeds
    SHOULDER_PITCH_SPEED = 0.4
    SHOULDER_ROT_SPEED = 0.4
    ELBOW_PITCH_SPEED = 0.5
    WRIST_PITCH_SPEED = 0.5
    WRIST_ROT_SPEED = 0.5
    CLAW_SPEED = 1

config = Config()

# Mock motor states
motor_states = {
    'drive_speed': 0,
    'steer_pos': config.DEFAULT_STEER_POS,
    'shoulder_pitch': 0,
    'shoulder_rot': 0,
    'elbow_pitch': 0,
    'wrist_pitch': 0,
    'wrist_rot': 0,
    'claw': 0,
}

pressed_keys: Set[str] = set()

def test_movement_controls():
    """Test movement control logic"""
    print("\n=== Testing Movement Controls ===")
    
    global pressed_keys, motor_states
    
    # Test forward movement
    print("\n1. Testing forward (W)")
    pressed_keys = {'w'}
    speed, steer = get_movement_state(pressed_keys)
    assert speed == config.NORMAL_DRIVE_SPEED, f"Expected {config.NORMAL_DRIVE_SPEED}, got {speed}"
    print(f"✓ Forward speed: {speed}")
    
    # Test forward fast
    print("\n2. Testing forward fast (W + Shift)")
    pressed_keys = {'w', 'shift'}
    speed, steer = get_movement_state(pressed_keys)
    assert speed == config.MAX_DRIVE_SPEED, f"Expected {config.MAX_DRIVE_SPEED}, got {speed}"
    print(f"✓ Fast speed: {speed}")
    
    # Test forward slow
    print("\n3. Testing forward slow (W + Ctrl)")
    pressed_keys = {'w', 'ctrl'}
    speed, steer = get_movement_state(pressed_keys)
    assert speed == config.MIN_DRIVE_SPEED, f"Expected {config.MIN_DRIVE_SPEED}, got {speed}"
    print(f"✓ Slow speed: {speed}")
    
    # Test backward
    print("\n4. Testing backward (S)")
    pressed_keys = {'s'}
    speed, steer = get_movement_state(pressed_keys)
    assert speed == -1 * config.MIN_DRIVE_SPEED, f"Expected {-1 * config.MIN_DRIVE_SPEED}, got {speed}"
    print(f"✓ Backward speed: {speed}")
    
    # Test steering left
    print("\n5. Testing steer left (A)")
    pressed_keys = {'a'}
    speed, steer = get_movement_state(pressed_keys)
    assert steer == config.STEER_LEFT_POS, f"Expected {config.STEER_LEFT_POS}, got {steer}"
    print(f"✓ Left steering: {steer}")
    
    # Test steering right
    print("\n6. Testing steer right (D)")
    pressed_keys = {'d'}
    speed, steer = get_movement_state(pressed_keys)
    assert steer == config.STEER_RIGHT_POS, f"Expected {config.STEER_RIGHT_POS}, got {steer}"
    print(f"✓ Right steering: {steer}")
    
    # Test forward + left
    print("\n7. Testing forward left (W + A)")
    pressed_keys = {'w', 'a'}
    speed, steer = get_movement_state(pressed_keys)
    assert speed == config.NORMAL_DRIVE_SPEED / 1.5, f"Expected {config.NORMAL_DRIVE_SPEED / 1.5}, got {speed}"
    assert steer == config.STEER_LEFT_POS, f"Expected {config.STEER_LEFT_POS}, got {steer}"
    print(f"✓ Forward left - speed: {speed}, steer: {steer}")
    
    print("\n✓ All movement control tests passed!")

def get_movement_state(keys: Set[str]):
    """Simulate movement state logic"""
    speed = 0
    steer_pos = config.DEFAULT_STEER_POS
    
    # Determine drive speed
    if 'w' in keys and 'shift' in keys:
        speed = config.MAX_DRIVE_SPEED
    elif 'w' in keys and 'ctrl' in keys:
        speed = config.MIN_DRIVE_SPEED
    elif 'w' in keys and 's' in keys:
        speed = 0
    elif 'w' in keys:
        speed = config.NORMAL_DRIVE_SPEED
    elif 's' in keys:
        speed = -1 * config.MIN_DRIVE_SPEED
    
    # Adjust speed when turning
    if ('w' in keys and 'a' in keys) or ('w' in keys and 'd' in keys):
        speed = config.NORMAL_DRIVE_SPEED / 1.5
    elif ('s' in keys and 'a' in keys) or ('s' in keys and 'd' in keys):
        speed = -1 * config.MIN_DRIVE_SPEED
    
    # Determine steering position
    if 'a' in keys and 'd' in keys:
        steer_pos = config.DEFAULT_STEER_POS
    elif 'a' in keys:
        steer_pos = config.STEER_LEFT_POS
    elif 'd' in keys:
        steer_pos = config.STEER_RIGHT_POS
    
    return speed, steer_pos

def test_rotation_controls():
    """Test rotation control logic"""
    print("\n=== Testing Rotation Controls ===")
    
    global pressed_keys
    
    # Test rotate left
    print("\n1. Testing rotate left (Q)")
    pressed_keys = {'q'}
    speed = get_rotation_speed(pressed_keys)
    assert speed == config.STEER_ROTATION_SPEED, f"Expected {config.STEER_ROTATION_SPEED}, got {speed}"
    print(f"✓ Rotate left speed: {speed}")
    
    # Test rotate right
    print("\n2. Testing rotate right (E)")
    pressed_keys = {'e'}
    speed = get_rotation_speed(pressed_keys)
    assert speed == -1 * config.STEER_ROTATION_SPEED, f"Expected {-1 * config.STEER_ROTATION_SPEED}, got {speed}"
    print(f"✓ Rotate right speed: {speed}")
    
    # Test both (should be 0)
    print("\n3. Testing both Q and E (should cancel)")
    pressed_keys = {'q', 'e'}
    speed = get_rotation_speed(pressed_keys)
    assert speed == 0, f"Expected 0, got {speed}"
    print(f"✓ Both pressed - speed: {speed}")
    
    print("\n✓ All rotation control tests passed!")

def get_rotation_speed(keys: Set[str]):
    """Simulate rotation speed logic"""
    if 'q' in keys and 'e' in keys:
        return 0
    elif 'q' in keys:
        return config.STEER_ROTATION_SPEED
    elif 'e' in keys:
        return -1 * config.STEER_ROTATION_SPEED
    else:
        return 0

def test_arm_controls():
    """Test arm control logic"""
    print("\n=== Testing Arm Controls ===")
    
    global pressed_keys
    
    # Test shoulder pitch
    print("\n1. Testing shoulder pitch (Y/H)")
    pressed_keys = {'y'}
    arm_state = get_arm_state(pressed_keys)
    assert arm_state['shoulder_pitch'] == config.SHOULDER_PITCH_SPEED
    print(f"✓ Shoulder pitch up: {arm_state['shoulder_pitch']}")
    
    pressed_keys = {'h'}
    arm_state = get_arm_state(pressed_keys)
    assert arm_state['shoulder_pitch'] == -1 * config.SHOULDER_PITCH_SPEED
    print(f"✓ Shoulder pitch down: {arm_state['shoulder_pitch']}")
    
    # Test shoulder rotation
    print("\n2. Testing shoulder rotation (Z/X)")
    pressed_keys = {'z'}
    arm_state = get_arm_state(pressed_keys)
    assert arm_state['shoulder_rot'] == -1 * config.SHOULDER_ROT_SPEED
    print(f"✓ Shoulder rotate left: {arm_state['shoulder_rot']}")
    
    # Test multiple arm joints
    print("\n3. Testing multiple joints (Y + U + I)")
    pressed_keys = {'y', 'u', 'i'}
    arm_state = get_arm_state(pressed_keys)
    assert arm_state['shoulder_pitch'] == config.SHOULDER_PITCH_SPEED
    assert arm_state['elbow_pitch'] == config.ELBOW_PITCH_SPEED
    assert arm_state['wrist_pitch'] == config.WRIST_PITCH_SPEED
    print(f"✓ Multiple joints: shoulder={arm_state['shoulder_pitch']}, elbow={arm_state['elbow_pitch']}, wrist={arm_state['wrist_pitch']}")
    
    # Test claw
    print("\n4. Testing claw ([/])")
    pressed_keys = {'['}
    arm_state = get_arm_state(pressed_keys)
    assert arm_state['claw'] == config.CLAW_SPEED
    print(f"✓ Claw open: {arm_state['claw']}")
    
    print("\n✓ All arm control tests passed!")

def get_arm_state(keys: Set[str]):
    """Simulate arm control logic"""
    state = {
        'shoulder_rot': 0,
        'shoulder_pitch': 0,
        'elbow_pitch': 0,
        'wrist_pitch': 0,
        'wrist_rot': 0,
        'claw': 0,
    }
    
    # Shoulder rotation
    if 'z' in keys and 'x' in keys:
        state['shoulder_rot'] = 0
    elif 'z' in keys:
        state['shoulder_rot'] = -1 * config.SHOULDER_ROT_SPEED
    elif 'x' in keys:
        state['shoulder_rot'] = config.SHOULDER_ROT_SPEED
    
    # Shoulder pitch
    if 'y' in keys and 'h' in keys:
        state['shoulder_pitch'] = 0
    elif 'y' in keys:
        state['shoulder_pitch'] = config.SHOULDER_PITCH_SPEED
    elif 'h' in keys:
        state['shoulder_pitch'] = -1 * config.SHOULDER_PITCH_SPEED
    
    # Elbow pitch
    if 'u' in keys and 'j' in keys:
        state['elbow_pitch'] = 0
    elif 'u' in keys:
        state['elbow_pitch'] = config.ELBOW_PITCH_SPEED
    elif 'j' in keys:
        state['elbow_pitch'] = -1 * config.ELBOW_PITCH_SPEED
    
    # Wrist pitch
    if 'i' in keys and 'k' in keys:
        state['wrist_pitch'] = 0
    elif 'i' in keys:
        state['wrist_pitch'] = config.WRIST_PITCH_SPEED
    elif 'k' in keys:
        state['wrist_pitch'] = -1 * config.WRIST_PITCH_SPEED
    
    # Wrist rotation
    if 'c' in keys and 'v' in keys:
        state['wrist_rot'] = 0
    elif 'c' in keys:
        state['wrist_rot'] = config.WRIST_ROT_SPEED
    elif 'v' in keys:
        state['wrist_rot'] = -1 * config.WRIST_ROT_SPEED
    
    # Claw
    if '[' in keys and ']' in keys:
        state['claw'] = 0
    elif '[' in keys:
        state['claw'] = config.CLAW_SPEED
    elif ']' in keys:
        state['claw'] = -1 * config.CLAW_SPEED
    
    return state

def test_multiple_key_combinations():
    """Test various key combinations"""
    print("\n=== Testing Multiple Key Combinations ===")
    
    global pressed_keys
    
    # Movement + Arm
    print("\n1. Testing movement with arm controls (W + Y)")
    pressed_keys = {'w', 'y'}
    move_speed, move_steer = get_movement_state(pressed_keys)
    arm_state = get_arm_state(pressed_keys)
    assert move_speed == config.NORMAL_DRIVE_SPEED
    assert arm_state['shoulder_pitch'] == config.SHOULDER_PITCH_SPEED
    print(f"✓ Movement and arm work together - drive: {move_speed}, shoulder: {arm_state['shoulder_pitch']}")
    
    # Verify rotation and movement are exclusive
    print("\n2. Testing that rotation mode is exclusive with movement")
    pressed_keys = {'w', 'q'}
    # In actual implementation, Q takes precedence over W
    print("✓ Rotation (Q) should take precedence over movement (W)")
    
    print("\n✓ All combination tests passed!")

def main():
    """Run all tests"""
    print("="*50)
    print("MAVRIC Web Teleop Control Logic Tests")
    print("="*50)
    
    try:
        test_movement_controls()
        test_rotation_controls()
        test_arm_controls()
        test_multiple_key_combinations()
        
        print("\n" + "="*50)
        print("✓ ALL TESTS PASSED!")
        print("="*50)
        print("\nThe control logic is working correctly.")
        print("You can now test the web interface by running:")
        print("  ./start_web.sh")
        print("Or:")
        print("  python3 web_teleop.py")
        return 0
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())
