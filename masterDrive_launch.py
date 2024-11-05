from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch Arguments
    science_mode = DeclareLaunchArgument(
        'scienceMode',
        default_value='false',
        description='Enable science mode'
    )
    
    auto_mode = DeclareLaunchArgument(
        'autoMode',
        default_value='false',
        description='Enable auto mode'
    )

    # Nodes
    drivetrain_science = Node(
        package='SparkCan',
        executable='sparkcan_drive_train',  # Remove .py extension
        name='Drivetrain_Control',
        output='screen',
        respawn=True,
        remappings=[
            ('Drive_Train', 'Drive/Drive_Command'),
            ('Steer_Train', 'Drive/Steer_Command')
        ],
        condition=IfCondition(LaunchConfiguration('scienceMode'))
    )

    scale_startups_science = Node(
        package='SparkCan',
        executable='scale_startups',  # Remove .py extension
        name='Scale_Startups_Science',
        output='screen',
        parameters=[{
            'ShoulderRot': 0.25,
            'ShoulderPitch': 0.50,
            'ElbowPitch': 0.0,
            'WristPitch': 0.75,
            'WristRot': 0.05
        }],
        condition=IfCondition(LaunchConfiguration('scienceMode'))
    )

    scale_startups_auto = Node(
        package='SparkCan',
        executable='scale_startups',  # Remove .py extension
        name='Scale_Startups_Auto',
        output='screen',
        parameters=[{
            'Drive_Sens': 1.0,
            'ShoulderRot': 0.0,
            'ShoulderPitch': 0.0,
            'ElbowPitch': 0.0,
            'WristPitch': 0.0,
            'WristRot': 0.0
        }],
        condition=IfCondition(LaunchConfiguration('autoMode'))
    )

    # Return LaunchDescription with all components
    return LaunchDescription([
        science_mode,
        auto_mode,
        drivetrain_science,
        scale_startups_science,
        scale_startups_auto
    ])