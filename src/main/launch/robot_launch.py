import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    os.system('usbreset "ChibiOS/RT Virtual COM Port"')
    
    default_config_path = PathJoinSubstitution([
        FindPackageShare("main"), "config", "robot_params.yaml"
    ])
    
    launch_arguments = [
        DeclareLaunchArgument("config_file", default_value=default_config_path)
    ]
    
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ll_control'), 'launch', 'll_control_launch.py'
                ])
            ]),
            launch_arguments={'config_file': LaunchConfiguration("config_file")}.items()
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('main'), 'launch', 'apm.launch'
                ])
            ]),
        ),
    ]
    
    return LaunchDescription(launch_arguments + launches)