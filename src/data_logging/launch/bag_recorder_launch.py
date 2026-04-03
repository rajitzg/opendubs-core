"""Launch the data logging node for rosbag recording services."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
import yaml

def generate_launch_description():
    """Launch the bag recorder node using an external config file.

    Returns:
        LaunchDescription: Launch definition for data logging recorder service.
    """
    # Get config path
    config_launch_arg = DeclareLaunchArgument("config_file")

    bag_recorder_node = Node(
        package="data_logging",
        executable="bag_recorder",
        name="bag_recorder",
        namespace="data_logging",
        output="screen",
        parameters=[LaunchConfiguration("config_file")]
    )

    return LaunchDescription([
        config_launch_arg,
        bag_recorder_node
    ])
