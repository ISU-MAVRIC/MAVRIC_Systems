# MAVRIC Web Teleop - Quick Reference Card

## 🚀 Start the Server
```bash
cd /workspaces/MAVRIC_Systems/headless_teleop
./start_web.sh
```

## 🌐 Access the Interface
```
http://localhost:5000
```

## ⌨️ Keyboard Controls

### Movement (Mutually Exclusive with Rotation)
| Key | Action |
|-----|--------|
| W | Forward (0.3) |
| W+Shift | Fast (0.5) |
| W+Ctrl | Slow (0.1) |
| S | Backward |
| A | Left |
| D | Right |

### Rotation (Mutually Exclusive with Movement)
| Key | Action |
|-----|--------|
| Q | Rotate Left |
| E | Rotate Right |

### Arm - Shoulder (Always Available)
| Key | Action |
|-----|--------|
| Y | Pitch Up |
| H | Pitch Down |
| Z | Rotate Left |
| X | Rotate Right |

### Arm - Elbow (Always Available)
| Key | Action |
|-----|--------|
| U | Pitch Up |
| J | Pitch Down |

### Arm - Wrist (Always Available)
| Key | Action |
|-----|--------|
| I | Pitch Up |
| K | Pitch Down |
| C | Rotate Left |
| V | Rotate Right |

### Arm - Claw (Always Available)
| Key | Action |
|-----|--------|
| [ | Open |
| ] | Close |

### Emergency
| Key | Action |
|-----|--------|
| ESC | Emergency Stop |

## 📋 Key Features
✅ Multiple simultaneous key presses
✅ Real-time WebSocket communication
✅ Visual feedback for active keys
✅ Auto-stop on disconnect
✅ Touch/click support for testing

## 🔒 Safety
- ESC stops everything
- Window blur releases all keys
- Disconnect stops all motors
- Emergency stop button available

## 🧪 Testing
```bash
python3 test_controls.py
```

## 📖 Documentation
- `WEB_README.md` - Full user guide
- `TESTING.md` - Test procedures
- `CONTROL_LAYOUT.md` - Visual layout
- `WEB_INTERFACE_SUMMARY.md` - Complete summary

## 🐛 Troubleshooting
1. Check connection status (green = connected)
2. Open browser console (F12)
3. Check server terminal for errors
4. Verify port 5000 is available

## 💡 Tips
- Hold multiple keys for combined actions
- Arm controls work in any mode
- Can't use rotation (Q/E) with movement (WASD)
- Click on virtual keys for testing

---
For full documentation, see WEB_README.md
