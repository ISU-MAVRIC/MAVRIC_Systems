# Testing Guide for Web Teleoperation Interface

## Quick Test (No Hardware)

To test the web interface without CAN bus hardware, you can create a mock version:

### 1. Create a test file (optional)

```bash
cd /workspaces/MAVRIC_Systems/headless_teleop
python3 test_web_interface.py
```

### 2. Or just run the web server

```bash
./start_web.sh
```

The server will attempt to initialize CAN bus controllers. If hardware is not available, you may see errors, but the web interface will still load.

## Testing Checklist

### Basic Functionality
- [ ] Server starts without errors
- [ ] Can access web interface at http://localhost:5000
- [ ] Connection status shows "Connected" (green indicator)
- [ ] All control sections are visible

### Keyboard Controls
- [ ] Pressing 'W' highlights the W key
- [ ] Active keys display updates when keys are pressed
- [ ] Keys unhighlight when released
- [ ] Multiple keys can be pressed simultaneously
- [ ] Shift and Ctrl modifiers work with W

### Movement Controls (WASD)
- [ ] W - Forward movement
- [ ] S - Backward movement
- [ ] A - Left steering
- [ ] D - Right steering
- [ ] W + A - Forward left
- [ ] W + D - Forward right
- [ ] W + Shift - Fast speed
- [ ] W + Ctrl - Slow speed

### Rotation Controls (QE)
- [ ] Q - Rotate left
- [ ] E - Rotate right
- [ ] Q and E cannot be used with WASD simultaneously

### Arm Controls
- [ ] Y/H - Shoulder pitch
- [ ] Z/X - Shoulder rotation
- [ ] U/J - Elbow pitch
- [ ] I/K - Wrist pitch
- [ ] C/V - Wrist rotation
- [ ] [/] - Claw open/close
- [ ] Arm controls work independently from movement/rotation

### Safety Features
- [ ] ESC key triggers emergency stop
- [ ] Emergency Stop button works
- [ ] All keys clear when emergency stop is triggered
- [ ] Keys release when browser window loses focus
- [ ] Connection status updates on disconnect

### Browser Compatibility
- [ ] Chrome/Chromium
- [ ] Firefox
- [ ] Safari (if available)
- [ ] Edge (if available)

### Mobile/Touch Testing (if applicable)
- [ ] Interface is responsive on mobile
- [ ] Touch controls work on key buttons
- [ ] Virtual keyboard doesn't interfere

## Debugging

### Check Browser Console
1. Press F12 to open Developer Tools
2. Go to Console tab
3. Look for JavaScript errors
4. Check Network tab for WebSocket connection

### Check Server Logs
- Monitor terminal output for errors
- Look for "Client connected" message
- Check for CAN bus initialization errors

### Common Issues

**Issue**: Cannot connect to server
- **Solution**: Check if server is running, verify port 5000 is not blocked

**Issue**: Keys not responding
- **Solution**: Check browser console, ensure WebSocket connection is active

**Issue**: Multiple keys not working
- **Solution**: Some browsers may have limitations, try Chrome

**Issue**: CAN bus errors
- **Solution**: If testing without hardware, this is expected. The web interface will still work for testing key detection.

## Performance Monitoring

### Latency Test
1. Press and hold a key
2. Observe the time between press and visual highlight
3. Should be < 100ms

### Multiple Key Test
1. Press W + A simultaneously
2. Both should highlight
3. Active keys should show "w, a"

### Stress Test
1. Press multiple arm control keys
2. Switch between movement and rotation
3. Trigger emergency stop
4. Verify all systems respond correctly

## Integration Testing

When hardware is connected:

1. **Drive System Test**
   - Press W and verify motors spin forward
   - Press S and verify motors spin backward
   - Test steering with A and D

2. **Rotation Test**
   - Press Q and verify rotation left
   - Press E and verify rotation right
   - Verify steering motors position for rotation

3. **Arm Test**
   - Test each arm joint individually
   - Test combinations of arm movements
   - Verify claw operation

4. **Emergency Stop Test**
   - Start motors moving
   - Press ESC or click Emergency Stop
   - Verify all motors stop immediately

## Automated Test Script

Run the automated test (if you create one):

```bash
python3 test_controls.py
```

This will simulate key presses and verify the control logic.
