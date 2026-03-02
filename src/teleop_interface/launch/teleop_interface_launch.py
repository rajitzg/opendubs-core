from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_launch_arg = DeclareLaunchArgument('config_file')

    return LaunchDescription([
        config_launch_arg,
        Node(
            package='teleop_interface',
            executable='teleop_interface_node',
            namespace='teleop_interface',
            name='teleop_interface_node',
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
