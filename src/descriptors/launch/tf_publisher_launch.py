from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_model_path = PathJoinSubstitution([
        FindPackageShare("descriptors"), "urdf", "open_dubs_descriptoin.urdf"
    ])

    launch_arguments = [
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        )
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        )
    ]

    return LaunchDescription(launch_arguments + nodes)