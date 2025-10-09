from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}]
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi'
        ),
        Node(
            package='drive_system',
            executable='teleop.py',
            name='teleop',
            parameters=[{'max_speed': 1.0}]
        ),
        Node(
            package='drive_system',
            executable='can_control.py',
            name='can_control'
        )
    ])