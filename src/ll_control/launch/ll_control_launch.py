from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    DeclareLaunchArgument("config_file")
    
    return LaunchDescription([
        Node(
            package='ll_control',
            executable='ll_control_node',
            name='ll_control_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])