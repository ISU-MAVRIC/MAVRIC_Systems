#!/bin/bash
# Quick start script for MAVRIC Web Teleop Interface

echo "=========================================="
echo "  MAVRIC Web Teleoperation Interface"
echo "=========================================="
echo ""

# Check if running in correct directory
if [ ! -f "web_teleop.py" ]; then
    echo "Error: Please run this script from the headless_teleop directory"
    exit 1
fi

# Kill any existing instances on port 5000
echo "Checking for existing instances..."
EXISTING_PID=$(lsof -ti :5000 2>/dev/null)
if [ ! -z "$EXISTING_PID" ]; then
    echo "Killing existing process on port 5000 (PID: $EXISTING_PID)..."
    kill $EXISTING_PID 2>/dev/null
    sleep 1
fi

# Check Python version
echo "Checking Python version..."
python3 --version

# Check if required packages are installed
echo "Checking dependencies..."
python3 -c "import flask, flask_socketio" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip install -r requirements_web.txt
fi

echo ""
echo "Starting web server..."
echo "=========================================="
echo ""
echo "  Access the interface at:"
echo "  http://localhost:5000"
echo ""
echo "  Or from another device:"
echo "  http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Start the web server
python3 web_teleop.py
