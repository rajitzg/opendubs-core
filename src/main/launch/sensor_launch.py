from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare('main'), 'config', 'robot_params.yaml'
    ])

    launch_arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config_path),
    ]
    
    nodes = [
        Node(
            package="main",
            executable="ekf_initializer",
            name="ekf_initializer",
            namespace="main",
            output="screen",
            parameters=[LaunchConfiguration("config_file")]
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_front_node",
            namespace="sensors",
            output="screen",
            parameters=[{
                'serial_port': '/dev/rplidar1',
                'serial_baudrate': 115200,
                'frame_id': 'front_lidar_link',
            }],
            remappings=[('scan', 'scan_front')]
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_back_node",
            namespace="sensors",
            output="screen",
            parameters=[{
                'serial_port': '/dev/rplidar0',
                'serial_baudrate': 115200,
                'frame_id': 'back_lidar_link',
            }],
            remappings=[('scan', 'scan_back')]
        )
    ]

    return LaunchDescription(launch_arguments + nodes)