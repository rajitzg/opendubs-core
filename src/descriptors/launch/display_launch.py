from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_model_path = PathJoinSubstitution([
        FindPackageShare("descriptors"), "sdf", "open_dubs_description.sdf"
    ])
    default_rviz_path = PathJoinSubstitution([
        FindPackageShare("descriptors"), "rviz", "config.rviz"
    ])

    launch_arguments = [
        DeclareLaunchArgument(
            name='gui', 
            default_value='True', 
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path, 
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_path, 
            description='Absolute path to rviz config file'
        ),
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'open_dubs_urdf': Command(['xacro ', default_model_path])}],
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )
    ]

    return LaunchDescription(launch_arguments + nodes)