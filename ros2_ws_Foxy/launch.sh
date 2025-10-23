source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
ros2 launch mavric_launch teleop.launch.py