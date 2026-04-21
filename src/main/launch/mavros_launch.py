import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import yaml


def load_mavros_params(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)
    script_path = PathJoinSubstitution([
        FindPackageShare('main'),
        'scripts',
        'wait_and_load_mavros_params.sh'
    ]).perform(context)

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        params = config.get('mavros_params_loader', {})

        if not params:
            print(
                "[WARN] No 'mavros_params_loader' section found in config file. "
                'Skipping MAVROS param load.'
            )
            return []

        load_actions = []

        for (node_name, node_params) in params.items():
            expected_params = list(node_params.get('ros__parameters', {}).keys())
            node_params = {'/**': node_params}

            temp_servo_config = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
            yaml.dump(node_params, temp_servo_config)
            temp_servo_config.close()

            load_params = ExecuteProcess(
                cmd=[
                    'bash',
                    script_path,
                    '/mavros/' + node_name,
                    temp_servo_config.name,
                    *expected_params,
                ],
                output='screen'
            )

            load_actions.append(load_params)

        return load_actions

    except Exception as e:
        print(f'[ERROR] Failed to extract MAVROS params: {e}')
        return []


def generate_launch_description():
    config_launch_arg = DeclareLaunchArgument('config_file')
    fcu_launch_arg = DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:57600')
    gcs_launch_arg = DeclareLaunchArgument('gcs_url', default_value='udp://@10.18.153.129')

    launch_args = [config_launch_arg, fcu_launch_arg, gcs_launch_arg]

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'), 'launch', 'apm.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'params_file': LaunchConfiguration('config_file'),
        }.items()
    )

    return LaunchDescription(
        launch_args + [
            mavros_launch,
            OpaqueFunction(function=load_mavros_params)
        ]
    )
