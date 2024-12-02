from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                Node(
                    package='spark_can',
                    executable='BNO055_IMU.py',
                    name='BNO055',
                    output='screen',
                    respawn=True
                ),
                Node(
                    package='spark_can',
                    executable='GPS_Neo-M9N.py',
                    name='GPS',
                    respawn=True
                ),
                Node(
                    package='spark_can',
                    executable='Battery_Voltage.py',
                    name='ADC',
                    respawn=True
                ),
            ],
            namespace='HW'
        )
    ])
