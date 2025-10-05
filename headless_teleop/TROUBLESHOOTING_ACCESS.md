# Troubleshooting: Access to Localhost Denied

## Problem: "Access to localhost denied" or Port Issues

### Solution 1: Kill Existing Process on Port 5000

If you see "Address already in use" or similar errors:

```bash
# Find the process using port 5000
lsof -ti :5000

# Kill it
lsof -ti :5000 | xargs kill

# Or manually
kill <PID>
```

### Solution 2: Use a Different Port

```bash
# Start on port 8080 instead
python3 web_teleop.py --port 8080

# Then access at http://localhost:8080
```

### Solution 3: Use the Startup Script

The startup script automatically handles port conflicts:

```bash
./start_web.sh
```

### Solution 4: Check Network Binding

The server is configured to bind to `0.0.0.0` which means it accepts connections from any network interface. If you're in a dev container or remote environment, you may need to:

1. **Access via the container's exposed port**
   - Check Docker/dev container port mappings
   - The port might be forwarded to a different number

2. **Use the network IP address**
   ```bash
   # Find your IP
   hostname -I
   
   # Access at http://<IP>:5000
   ```

3. **Check VS Code port forwarding**
   - In VS Code, go to "PORTS" panel (bottom)
   - Port 5000 should be listed and forwarded
   - Click on the "Local Address" to open in browser

### Solution 5: Firewall Issues

If running on a remote machine or in a container:

```bash
# Check if the port is listening
netstat -tlnp | grep :5000

# Should show something like:
# tcp  0  0 0.0.0.0:5000  0.0.0.0:*  LISTEN  <PID>/python3
```

### Solution 6: Permission Issues

If you get permission denied for ports < 1024:

```bash
# Use a port > 1024 (recommended)
python3 web_teleop.py --port 5000  # Usually OK
python3 web_teleop.py --port 8080  # Alternative
```

## Quick Diagnosis

Run these commands to diagnose the issue:

```bash
# 1. Check if anything is on port 5000
lsof -i :5000

# 2. Try to start the server
python3 web_teleop.py

# 3. Check the output for specific errors

# 4. In another terminal, verify it's running
curl http://localhost:5000
```

## Common Error Messages and Solutions

### "Address already in use"
**Cause**: Another process is using port 5000
**Solution**: Kill the process or use a different port

```bash
# Kill existing
lsof -ti :5000 | xargs kill

# Or use different port
python3 web_teleop.py --port 8080
```

### "Permission denied"
**Cause**: No permission to bind to the port
**Solution**: Use a port > 1024 or run with appropriate permissions

```bash
python3 web_teleop.py --port 8080
```

### "Cannot connect to localhost"
**Cause**: Server not running or firewall blocking
**Solution**: 
1. Verify server is running: `netstat -tlnp | grep :5000`
2. Check VS Code port forwarding (PORTS panel)
3. Try accessing via IP address instead of localhost

### "Connection refused"
**Cause**: Server not started or crashed
**Solution**: Check terminal for error messages, restart server

## Dev Container / Remote Development

If you're using VS Code Remote Development or dev containers:

1. **Check the PORTS tab** in VS Code (usually at the bottom)
2. Port 5000 should appear when the server starts
3. VS Code will show a "Local Address" - click it to open
4. The local address might be something like `http://localhost:5000` or forwarded to another port

## Testing the Server

Once running, test with:

```bash
# Test from command line
curl http://localhost:5000

# Should return HTML of the web page

# Test WebSocket (if needed)
python3 -c "
import socketio
sio = socketio.Client()
sio.connect('http://localhost:5000')
print('Connected!')
sio.disconnect()
"
```

## Still Having Issues?

1. **Check the full error message** in the terminal
2. **Look for Python exceptions** or stack traces
3. **Verify all dependencies** are installed: `pip install -r requirements_web.txt`
4. **Check CAN bus warnings** (these are OK in simulation mode)
5. **Try the test script** first: `python3 test_controls.py`

## Working Configuration

When everything is working, you should see:

```
============================================================
Starting MAVRIC Web Teleop Interface...
============================================================

Access the interface at:
  - Local:    http://localhost:5000
  - Network:  http://0.0.0.0:5000

Press Ctrl+C to stop the server
============================================================

 * Serving Flask app 'web_teleop'
 * Debug mode: off
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:5000
 * Running on http://192.168.x.x:5000
Press CTRL+C to quit
```

Then you can open your browser to any of those addresses!
