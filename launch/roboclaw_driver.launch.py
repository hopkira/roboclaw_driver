import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('roboclaw_driver'),
            'config',
            'motor_driver.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # RoboClaw driver node
    roboclaw_driver_node = Node(
        package='roboclaw_driver',
        executable='roboclaw_driver_node',
        name='roboclaw_driver',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        roboclaw_driver_node
    ])
