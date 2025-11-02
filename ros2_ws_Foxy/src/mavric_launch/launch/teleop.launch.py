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
            executable='can_composer.py',
            name='can_composer'
        ),
        # Node(
        #     package='drive_system',
        #     executable='drive_control.py',
        #     name='drive_control'
        # ),
        # Node(
        #     package='drive_system',
        #     executable='steer_control.py',
        #     name='steer_control'
        # ),
        # Node(
        #     package='drive_system',
        #     executable='arm_control.py',
        #     name='arm_control'
        # ),
    ])