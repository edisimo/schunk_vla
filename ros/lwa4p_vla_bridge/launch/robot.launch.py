"""LWA 4P (real or mock) with MoveIt, RViz and the VLA bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit = get_package_share_directory("lwa4p_moveit_config")
    bridge = get_package_share_directory("lwa4p_vla_bridge")

    def include(name, mode, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit, "launch", name)),
            condition=LaunchConfigurationEquals("mode", mode),
            launch_arguments=args.items(),
        )

    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="real", choices=["real", "mock"]),
            DeclareLaunchArgument("rviz", default_value="true"),
            # comma-decimal locales (e.g. sk_SK) make ROS 2 read YAML doubles as strings
            SetEnvironmentVariable("LC_NUMERIC", "en_US.UTF-8"),
            include("real.launch.py", "real", rviz=rviz),
            include("demo.launch.py", "mock", use_rviz=rviz),
            Node(
                package="lwa4p_vla_bridge",
                executable="bridge",
                name="lwa4p_vla_bridge",
                output="screen",
                parameters=[os.path.join(bridge, "config", "bridge.yaml")],
            ),
        ]
    )
