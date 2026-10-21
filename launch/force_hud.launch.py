"""Launch only read-only telemetry consumers. Does not start the robot bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("tianji_io_teleop_bridge"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config/force_hud.yaml"),
                              description="Read-only estimator and HUD ROS parameter YAML"),
        DeclareLaunchArgument("input_source", default_value="right_ft_sensor",
                              choices=["right_ft_sensor", "joint_estimate"],
                              description="HUD source; right FT is the same compensated/tared recording topic"),
        Node(package="tianji_io_teleop_bridge", executable="force_hud_estimator_node",
             name="tianji_force_estimator", output="screen",
             condition=LaunchConfigurationEquals("input_source", "joint_estimate"),
             parameters=[LaunchConfiguration("config_file")]),
        Node(package="tianji_io_teleop_bridge", executable="force_hud_node",
             name="tianji_force_hud", output="screen",
             parameters=[LaunchConfiguration("config_file"),
                         {"input_source": LaunchConfiguration("input_source")}]),
    ])
