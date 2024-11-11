from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    science_mode_arg = DeclareLaunchArgument('scienceMode', default_value='false')
    drive_mode_arg = DeclareLaunchArgument('driveMode', default_value='true')
    auto_mode_arg = DeclareLaunchArgument('autoMode', default_value='false')
    lora_on_arg = DeclareLaunchArgument('LoRa_On', default_value='true')

    # Include rosbridge_websocket.launch.py
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.py'
            )
        ),
        launch_arguments={'port': '9090'}.items()
    )

    # Include Drive.launch.py with arguments
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavric'),
                'launch',
                'Drive.launch.py'
            )
        ),
        launch_arguments={
            'scienceMode': LaunchConfiguration('scienceMode'),
            'driveMode': LaunchConfiguration('driveMode'),
            'autoMode': LaunchConfiguration('autoMode'),
            'LoRa_On': LaunchConfiguration('LoRa_On')
        }.items()
    )

    # Include Sensors.launch.py
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavric'),
                'launch',
                'Sensors.launch.py'
            )
        )
    )

    # Include Cameras.launch.py
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavric'),
                'launch',
                'Cameras.launch.py'
            )
        )
    )

    # Include Servos.launch.py with arguments
    servos_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavric'),
                'launch',
                'Servos.launch.py'
            )
        ),
        launch_arguments={
            'scienceMode': LaunchConfiguration('scienceMode'),
            'driveMode': LaunchConfiguration('driveMode'),
            'autoMode': LaunchConfiguration('autoMode'),
            'LoRa_On': LaunchConfiguration('LoRa_On')
        }.items()
    )

    # Define LoRa_Interface node with condition and remappings
    lora_interface_node = Node(
        package='mavric',
        executable='900MHz_Interface.py',
        name='LoRa_Interface',
        remappings=[
            ('Drive_Train', 'Drive/Drive_Command'),
            ('Steer_Train', 'Drive/Steer_Command')
        ],
        condition=IfCondition(LaunchConfiguration('LoRa_On'))
    )

    return LaunchDescription([
        science_mode_arg,
        drive_mode_arg,
        auto_mode_arg,
        lora_on_arg,
        rosbridge_launch,
        drive_launch,
        sensors_launch,
        cameras_launch,
        servos_launch,
        lora_interface_node,
    ])