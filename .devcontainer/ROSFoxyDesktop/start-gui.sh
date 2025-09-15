#!/bin/bash
# Start virtual X server
Xvfb :1 -screen 0 1280x800x24 &
export DISPLAY=:1

# Start minimal window manager
openbox &

# Start VNC server
x11vnc -display :1 -forever -nopw -rfbport 5900 &

# Start noVNC
websockify --web /usr/share/novnc/ 6080 localhost:5900 &

# Keep container alive
tail -f /dev/null