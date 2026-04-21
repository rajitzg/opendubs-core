from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare('main'), 'config', 'robot_params.yaml'
    ])

    launch_arguments = [
        DeclareLaunchArgument('config_file', default_value=default_config_path),
    ]

    nodes = [
        Node(
            package='main',
            executable='ekf_initializer',
            name='ekf_initializer',
            namespace='sensors',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_front_node',
            namespace='sensors',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('scan', 'scan_front')]
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_back_node',
            namespace='sensors',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[('scan', 'scan_back')]
        ),
        Node(
            package='main',
            executable='lidar_merger',
            name='lidar_merger_node',
            namespace='sensors',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('scan_input1', 'scan_front'),
                ('scan_input2', 'scan_back')
            ]
        )
    ]

    return LaunchDescription(launch_arguments + nodes)
