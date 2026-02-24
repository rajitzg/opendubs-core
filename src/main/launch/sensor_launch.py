import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'
            ])
        ]),
        launch_arguments={
            "serial_port": "/dev/rplidar"
            }.items()
    )
    ])