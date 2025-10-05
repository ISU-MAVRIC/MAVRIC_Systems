# MAVRIC Web Teleoperation Interface

A browser-based GUI for controlling the MAVRIC rover with real-time keyboard input support.

## Features

- **Real-time Control**: WebSocket-based communication for instant response
- **Multiple Key Support**: Handle multiple simultaneous key presses
- **Visual Feedback**: Interactive UI with highlighted active controls
- **Separate Control Modes**: 
  - Movement mode (WASD) for driving
  - Rotation mode (QE) for in-place rotation
  - Arm controls (always active)
- **Emergency Stop**: Quick shutdown of all motors
- **Cross-platform**: Works on any device with a modern web browser

## Installation

1. Install the required Python packages:
```bash
pip install -r requirements_web.txt
```

## Usage

1. Start the web server:
```bash
python3 web_teleop.py
```

2. Open your web browser and navigate to:
```
http://localhost:5000
```

Or from another device on the same network:
```
http://<rover-ip-address>:5000
```

3. Use your keyboard to control the rover!

## Controls

### Movement Controls (WASD)
- **W** - Drive forward (normal speed)
  - **W + Shift** - Fast speed
  - **W + Ctrl** - Slow speed
- **S** - Drive backward
- **A** - Steer left
- **D** - Steer right

### Rotation Controls (QE)
- **Q** - Rotate left (in-place)
- **E** - Rotate right (in-place)

**Note**: Rotation mode is exclusive with movement mode - you cannot use WASD while using QE.

### Arm Controls (Always Available)

#### Shoulder
- **Y** - Pitch up
- **H** - Pitch down
- **Z** - Rotate left
- **X** - Rotate right

#### Elbow
- **U** - Pitch up
- **J** - Pitch down

#### Wrist
- **I** - Pitch up
- **K** - Pitch down
- **C** - Rotate left
- **V** - Rotate right

#### Claw
- **[** - Open claw
- **]** - Close claw

### Emergency Controls
- **ESC** - Emergency stop (stops all motors)
- **Emergency Stop Button** - Click to stop all motors

## Features

### Multiple Key Press Support
The interface supports pressing multiple keys simultaneously:
- Drive forward while steering (W + A or W + D)
- Control multiple arm joints at once
- Modifier keys (Shift, Ctrl) with movement

### Visual Feedback
- Active keys are highlighted in green
- Connection status indicator shows server connectivity
- Active keys display shows all currently pressed keys
- Interactive key buttons provide click/touch support

### Safety Features
- Automatic key release when window loses focus
- Emergency stop functionality
- All motors stop when client disconnects
- Exclusive movement/rotation modes prevent conflicts

## Architecture

### Backend (web_teleop.py)
- Flask web server for serving the interface
- Flask-SocketIO for WebSocket communication
- Maintains pressed key state
- Controls CAN bus motor controllers
- Implements control logic from `initializer.py`

### Frontend
- **index.html** - Main interface structure
- **style.css** - Modern, responsive styling
- **script.js** - Client-side keyboard handling and WebSocket communication

### Communication Protocol
WebSocket events:
- `key_down` - Client sends when key is pressed
- `key_up` - Client sends when key is released
- `emergency_stop` - Client sends for emergency stop
- `key_state` - Server broadcasts current key state
- `status` - Server sends connection status

## Troubleshooting

### Cannot connect to web interface
- Ensure the server is running: `python3 web_teleop.py`
- Check firewall settings allow port 5000
- Verify you're using the correct IP address

### Keys not responding
- Check the browser console for errors (F12)
- Ensure the connection status shows "Connected"
- Try refreshing the page
- Check that no other application is capturing keyboard events

### Motors not moving
- Verify CAN bus connection is working
- Check motor IDs in `config.py`
- Ensure controllers are initialized properly
- Check for error messages in the server terminal

## Development

The web interface is built with:
- **Flask** - Python web framework
- **Flask-SocketIO** - WebSocket support for real-time communication
- **Socket.IO** - Client-side WebSocket library
- **Vanilla JavaScript** - No frameworks, lightweight and fast
- **Modern CSS** - Responsive design with CSS Grid and Flexbox

## Compatibility

- **Browsers**: Chrome, Firefox, Safari, Edge (latest versions)
- **Devices**: Desktop, laptop, tablet, smartphone
- **Operating Systems**: Windows, macOS, Linux, iOS, Android

## License

Part of the MAVRIC Systems project.
