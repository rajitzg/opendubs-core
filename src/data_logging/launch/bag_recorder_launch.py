from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config path
    config_launch_arg = DeclareLaunchArgument('config_file')

    bag_recorder_node = Node(
        package='data_logging',
        executable='bag_recorder',
        name='bag_recorder',
        namespace='data_logging',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_launch_arg,
        bag_recorder_node
    ])
