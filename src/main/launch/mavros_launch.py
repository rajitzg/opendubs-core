import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def load_mavros_params(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_file").perform(context)
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
          
        params = config.get("mavros_params_loader", {})
        
        if not params:
            print("[WARN] No '/mavros/params' found in config file. Skipping servo setup.")
            return []
        
        params = {"/**": params}
        
        temp_servo_config = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.dump(params, temp_servo_config)
        temp_servo_config.close()
        
        load_params = ExecuteProcess(
            cmd=[
                'ros2', 'param', 'load',
                '/mavros/param', 
                temp_servo_config.name
            ],
            output='screen'
        )
        
        return [TimerAction(period=15.0, actions=[load_params])]
            
    except Exception as e:
        print(f"[ERROR] Failed to extract MAVROS params: {e}")
        return []

def generate_launch_description():
    
    config_launch_arg = DeclareLaunchArgument("config_file")
    fcu_launch_arg = DeclareLaunchArgument("fcu_url", default_value="/dev/serial/by-id/usb-ArduPilot_fmuv2_1C0027000D51373337333031-if00:57600")
    gcs_launch_arg = DeclareLaunchArgument("gcs_url", default_value="")
    
    launch_args = [config_launch_arg, fcu_launch_arg, gcs_launch_arg]
    
    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'), 'launch', 'apm.launch'
            ])
        ]),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            }.items()
    )
    
    return LaunchDescription(
        launch_args + [
            mavros_launch,
            OpaqueFunction(function=load_mavros_params)
        ]
    )