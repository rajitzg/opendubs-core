from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    config_launch_arg = DeclareLaunchArgument("config_file")
    debug_mode_launch_arg = DeclareLaunchArgument("debug_mode", default_value="false")
    
    return LaunchDescription([
        config_launch_arg,
        debug_mode_launch_arg,
        Node(
            package='ll_control',
            executable='ll_control_node',
            namespace='ll_control',
            name='ll_control_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'debug_mode': LaunchConfiguration('debug_mode')}
            ]
        )
    ])