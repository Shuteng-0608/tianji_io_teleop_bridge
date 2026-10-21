from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_bridge(context):
    overrides = {
        'connect_on_start': ParameterValue(
            LaunchConfiguration('connect_on_start'), value_type=bool
        ),
        'allow_motion': ParameterValue(
            LaunchConfiguration('allow_motion'), value_type=bool
        ),
    }
    calibration_file = LaunchConfiguration('right_ft_calibration_file').perform(context).strip()
    if calibration_file:
        overrides['right_ft_calibration_file'] = calibration_file
    # An omitted launch override preserves an explicit YAML path. An empty path
    # in both places is rejected by the node before opening the SDK connection.
    return [Node(
        package='tianji_io_teleop_bridge',
        executable='marvin_teleop_bridge_node',
        name='tianji_marvin_teleop_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            overrides,
        ],
    )]


def generate_launch_description():
    package_share = Path(get_package_share_directory('tianji_io_teleop_bridge'))
    default_config = str(package_share / 'config' / 'bridge.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file', default_value=default_config,
                description='Absolute path to the Marvin bridge ROS parameter file',
            ),
            DeclareLaunchArgument(
                'connect_on_start', default_value='true',
                description='Whether to open the exclusive Marvin SDK connection',
            ),
            DeclareLaunchArgument(
                'allow_motion', default_value='false',
                description='Master motion gate; false guarantees monitor-only operation',
            ),
            DeclareLaunchArgument(
                'right_ft_calibration_file', default_value='',
                description='This startup calibration.yaml absolute path; empty uses explicit YAML setting, no historical default',
            ),
            OpaqueFunction(function=launch_bridge),
        ]
    )
