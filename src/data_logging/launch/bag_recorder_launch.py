from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    # Get config path
    pkg_share = get_package_share_directory('data_logging')
    config = os.path.join(pkg_share, 'config', 'data_logging.yaml')

    bag_recorder_node = Node(
        package="data_logging",
        executable="bag_recorder",
        name="bag_recorder",
        output="screen",
        parameters=[config]
    )

    return LaunchDescription([
        bag_recorder_node
    ])
