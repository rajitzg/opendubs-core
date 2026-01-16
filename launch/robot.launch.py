import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    if not os.getenv('ROS_WORKSPACE'):
        # Fallback for when running in an environment without ROS_WORKSPACE set 
        workspace_root = os.path.join(str(Path.home()), 'opendubs-core')
        print(f"ROS_WORKSPACE not set, defaulting to: {workspace_root}")
    else:
        workspace_root = os.getenv('ROS_WORKSPACE')

    default_config_path = os.path.join(
        workspace_root,
        'config',
        'robot_params.yaml'
    )

    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the config file'
    )

    return LaunchDescription([
        config_file,
        Node(
            package='ll_control',
            executable='ll_control_node',
            name='ll_control_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
