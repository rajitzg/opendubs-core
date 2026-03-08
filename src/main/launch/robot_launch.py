import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    os.system('usbreset "ChibiOS/RT Virtual COM Port"')
    
    default_config_path = PathJoinSubstitution([
        FindPackageShare("main"), "config", "robot_params.yaml"
    ])
    
    launch_arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config_path),
        DeclareLaunchArgument("debug_mode", default_value="false") # Debug mode will skip launching MAVROS and disable interactions with it in ll_control
    ]
    
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('teleop_interface'), 'launch', 'teleop_interface_launch.py'
                ])
            ]),
            condition=UnlessCondition(LaunchConfiguration("debug_mode")),
            launch_arguments={
                'config_file': LaunchConfiguration("config_file")
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ll_control'), 'launch', 'll_control_launch.py'
                ])
            ]),
            condition=UnlessCondition(LaunchConfiguration("debug_mode")),
            launch_arguments={
                'config_file': LaunchConfiguration("config_file")
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('main'), 'launch', 'mavros_launch.py'
                ])
            ]),
            condition=UnlessCondition(LaunchConfiguration("debug_mode")),
            launch_arguments={'config_file': LaunchConfiguration("config_file")}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('main'), 'launch', 'sensor_launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('diagnostics'), 'launch', 'diagnostics_launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('descriptors'), 'launch', 'tf_publisher_launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('data_logging'), 'launch', 'bag_recorder_launch.py'
                ])
            ]),
            launch_arguments={'config_file': LaunchConfiguration("config_file")}.items()
        )
    ]
    
    return LaunchDescription(launch_arguments + launches)