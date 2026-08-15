from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('tianji_io_teleop_bridge'))
    default_config = str(package_share / 'config' / 'bridge.yaml')

    config_argument = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path to the Marvin bridge ROS parameter file',
    )
    connect_argument = DeclareLaunchArgument(
        'connect_on_start',
        default_value='true',
        description='Whether to open the exclusive Marvin SDK connection',
    )
    allow_motion_argument = DeclareLaunchArgument(
        'allow_motion',
        default_value='false',
        description='Master motion capability gate; false guarantees monitor-only operation',
    )

    bridge_node = Node(
        package='tianji_io_teleop_bridge',
        executable='marvin_teleop_bridge_node',
        name='tianji_marvin_teleop_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'connect_on_start': ParameterValue(
                    LaunchConfiguration('connect_on_start'), value_type=bool
                ),
                'allow_motion': ParameterValue(
                    LaunchConfiguration('allow_motion'), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [config_argument, connect_argument, allow_motion_argument, bridge_node]
    )
