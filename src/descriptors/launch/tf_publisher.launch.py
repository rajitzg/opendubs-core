from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='descriptors').find('descriptors')
    default_model_path = os.path.join(pkg_share, 'urdf', 'open_dubs_description.urdf')

    model_file = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot model file'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    return LaunchDescription([
       model_file,
        robot_state_publisher_node,
    ])