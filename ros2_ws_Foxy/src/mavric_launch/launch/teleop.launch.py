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
            package='managers',
            executable='can_manager',
            name='can_manager',
            parameters=[{
                'status_mode': 'service',  # 'service', 'publish', or 'both'
                # 'status_publish_rate': 20  # Only used if mode is 'publish' or 'both'
            }]
        ),
        Node(
            package='managers',
            executable='servo_manager',
            name='servo_manager',
            parameters=[{
                # I2C Address for the PCA9685 Board
                'address': 0x40,
                
                # Channel 1 - Claw (HiTec HS-785HB)
                'channel_1.min_pulse': 900,
                'channel_1.max_pulse': 2100,
                'channel_1.actuation_range': 1890,
                'channel_1.start_angle': 120
            }]
        ),
        Node(
            package='drive_system',
            executable='drive_control',
            name='drive_control'
        ),
        Node(
            package='drive_system',
            executable='steer_control',
            name='steer_control'
        ),
        Node(
            package='drive_system',
            executable='arm_control',
            name='arm_control'
        ),
        Node(
            package='drive_system',
            executable='scale_tuning.py',
            name='scale_tuning'
        )
    ])