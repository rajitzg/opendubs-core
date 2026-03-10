from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_model_path = PathJoinSubstitution([
        FindPackageShare("descriptors"), "urdf", "open_dubs_description.urdf"
    ])

    launch_arguments = [
        DeclareLaunchArgument(
            name='config_file',
            description='Path to the SSOT configuration file',
        ),
        DeclareLaunchArgument(
            name='odom_input_topic',
            default_value='/mavros/local_position/odom',
            description='Topic name for the input odometry data'
        ),
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
            namespace='descriptors',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='descriptors',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            namespace='descriptors',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ]

    return LaunchDescription(launch_arguments + nodes)