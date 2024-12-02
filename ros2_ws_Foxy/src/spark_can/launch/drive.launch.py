#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    science_mode_arg = DeclareLaunchArgument(
        'scienceMode',
        default_value='false',
        description='Enable science mode'
    )
    
    drive_mode_arg = DeclareLaunchArgument(
        'driveMode',
        default_value='true',
        description='Enable drive mode'
    )
    
    auto_mode_arg = DeclareLaunchArgument(
        'autoMode',
        default_value='false',
        description='Enable auto mode'
    )

    return LaunchDescription([
        science_mode_arg,
        drive_mode_arg,
        auto_mode_arg,
        
        # Drivetrain Control Node (non-science mode)
        Node(
            package='spark_can',
            # executable='SparkCAN_Drive_Train',
            executable='spark_can_drive_train',
            name='Drivetrain_Control',
            remappings=[
                ('Drive_Train', 'Drive/Drive_Command'),
                ('Steer_Train', 'Drive/Steer_Command')
            ],
            condition=UnlessCondition(LaunchConfiguration('scienceMode')),
        ),
        
        # Scale Startups Drive Node
        Node(
            package='spark_can',
            executable='Scale_Startups',
            name='Scale_Startups_Drive',
            parameters=[{
                'Drive_Sens': 1.0,
                'ShoulderRot': 0.25,
                'ShoulderPitch': 0.75,
                'ElbowPitch': 0.75,
                'WristPitch': 0.3,
                'WristRot': 0.75
            }],
            condition=IfCondition(LaunchConfiguration('driveMode')),
        ),
        
        # Drivetrain Control Node (science mode)
        Node(
            package='spark_can',
            # executable='SparkCAN_Drive_Train',
            executable='spark_can_drive_train',
            name='Drivetrain_Control',
            remappings=[
                ('Drive_Train', 'Drive/Drive_Command'),
                ('Steer_Train', 'Drive/Steer_Command')
            ],
            condition=IfCondition(LaunchConfiguration('scienceMode')),
        ),
        
        # Scale Startups Science Node
        Node(
            package='spark_can',
            executable='Scale_Startups',
            name='Scale_Startups_Science',
            parameters=[{
                'ShoulderRot': 0.25,
                'ShoulderPitch': 0.50,
                'ElbowPitch': 0.0,
                'WristPitch': 0.75,
                'WristRot': 0.05
            }],
            condition=IfCondition(LaunchConfiguration('scienceMode')),
        ),
        
        # Scale Startups Auto Node
        Node(
            package='spark_can',
            executable='Scale_Startups',
            name='Scale_Startups_Auto',
            parameters=[{
                'Drive_Sens': 1.0,
                'ShoulderRot': 0.0,
                'ShoulderPitch': 0.0,
                'ElbowPitch': 0.0,
                'WristPitch': 0.0,
                'WristRot': 0.0
            }],
            condition=IfCondition(LaunchConfiguration('autoMode')),
        ),
    ])