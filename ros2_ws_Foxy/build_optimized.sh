#!/bin/bash
# Quick build script for drive system optimizations

set -e  # Exit on error

echo "🔧 Building optimized drive system..."
echo ""

cd "$(dirname "$0")"

# Build the messages first (dependency)
echo "📦 Building mavric_msg package..."
colcon build --packages-select mavric_msg

# Source the messages
source install/setup.bash

# Build drive system
echo "📦 Building drive_system package..."
colcon build --packages-select drive_system

echo ""
echo "✅ Build complete!"
echo ""
echo "To use the optimized system:"
echo "  source install/setup.bash"
echo "  ros2 launch mavric_launch teleop.launch.py"
echo ""
echo "To verify batch mode is active:"
echo "  ros2 topic list | grep batch"
echo "  ros2 topic hz /can_commands_batch"
echo ""
