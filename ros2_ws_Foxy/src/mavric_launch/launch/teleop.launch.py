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
                
                # List which channels we are configuring custom specs for
                'configured_channels': [1], 
                
                # --- CLAW CONFIGURATION (Channel 1) ---
                # Datasheet Specs for HiTec HS-755HB
                # Pulse Width: 900us to 2100us 
                'servo_config.channel_1.min_pulse': 900,
                'servo_config.channel_1.max_pulse': 2100,
                
                # Operating Travel: 60 degrees (implied +/- 60, so 120 total) 
                'servo_config.channel_1.actuation_range': 1890,
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