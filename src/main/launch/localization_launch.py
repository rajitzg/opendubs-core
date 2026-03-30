from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    config_launch_arg = DeclareLaunchArgument("config_file")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")]
    )
    
    return LaunchDescription([
        config_launch_arg,
        ekf_node
    ])