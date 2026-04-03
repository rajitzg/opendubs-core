"""Launch teleop input interface and REST API bridge nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch teleop controller interface and API nodes.

    Returns:
        LaunchDescription: Launch definition for teleop subsystem.
    """

    config_launch_arg = DeclareLaunchArgument('config_file')

    return LaunchDescription([
        config_launch_arg,
        Node(
            package='teleop_interface',
            executable='controller_interface_node',
            namespace='teleop_interface',
            name='controller_interface_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
        Node(
            package='teleop_interface',
            executable='api_node',
            namespace='teleop_interface',
            name='api_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        )
    ])
