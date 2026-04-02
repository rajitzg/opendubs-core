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
    
    launches =[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a1_launch.py'
                ])
            ),
            launch_arguments={
                "serial_port": "/dev/ttyUSB0",
                "frame_id": "front_lidar_link",
                "scan_topic": "scan_front",
            }.items()
        ),

        '''IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a1_launch.py'
                ])
            ),
            launch_arguments={
                "serial_port": "/dev/ttyUSB1",
                "frame_id": "back_lidar_link",
                "scan_topic": "scan_back",
            }.items()
        ),'''
        LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'
                    ])
                ]),
                launch_arguments={
                    "serial_port": "/dev/ttyUSB0",
                    "frame_id": "back_lidar_link"
                }.items()
            )
        ]),
    ]
    nodes = [
        Node(
            package="main",
            executable="ekf_initializer",
            name="ekf_initializer",
            namespace="main",
            output="screen",
            parameters=[LaunchConfiguration("config_file")]
        )
    ]

    return LaunchDescription(launch_arguments + launches + nodes)