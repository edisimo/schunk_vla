"""LWA 4P (real or mock) with MoveIt, RViz and the VLA bridge.

mode:=real driver:=ip runs the arm through the lwa4p_hardware ros2_control plugin (CiA 402
interpolated position, 125 Hz); driver:=python keeps the original profile-position driver.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit = get_package_share_directory("lwa4p_moveit_config")
    bridge = get_package_share_directory("lwa4p_vla_bridge")
    hardware = get_package_share_directory("lwa4p_hardware")
    driver = get_package_share_directory("lwa4p_driver")
    mode, impl, rviz = (LaunchConfiguration(n) for n in ("mode", "driver", "rviz"))
    ip = IfCondition(PythonExpression(["'", mode, "' == 'real' and '", impl, "' == 'ip'"]))
    python = IfCondition(PythonExpression(["'", mode, "' == 'real' and '", impl, "' == 'python'"]))

    def include(path, condition, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            condition=condition,
            launch_arguments=args.items(),
        )

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(hardware, "urdf", "lwa4p_ip.urdf.xacro")]), value_type=str
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager", *extra],
            condition=ip,
        )

    ip_stack = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            condition=ip,
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                os.path.join(hardware, "config", "controllers.yaml"),
            ],
            condition=ip,
        ),
        spawner("joint_state_broadcaster"),
        # inactive until ~/enable: with the drives off it rejects goals, like the old driver
        spawner("arm_controller", "--inactive"),
        Node(
            package="lwa4p_driver",
            executable="gripper",
            name="gripper",
            output="screen",
            parameters=[os.path.join(driver, "config", "gripper.yaml")],
            condition=ip,
        ),
        Node(
            package="lwa4p_driver",
            executable="ftm",
            name="ftm",
            output="screen",
            parameters=[os.path.join(driver, "config", "ftm.yaml")],
            condition=ip,
        ),
        include(os.path.join(moveit, "launch", "move_group.launch.py"), ip),
        include(
            os.path.join(moveit, "launch", "moveit_rviz.launch.py"),
            IfCondition(
                PythonExpression(
                    ["'", mode, "' == 'real' and '", impl, "' == 'ip' and '", rviz, "' == 'true'"]
                )
            ),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="real", choices=["real", "mock"]),
            DeclareLaunchArgument("driver", default_value="python", choices=["python", "ip"]),
            DeclareLaunchArgument("rviz", default_value="true"),
            # comma-decimal locales (e.g. sk_SK) make ROS 2 read YAML doubles as strings
            SetEnvironmentVariable("LC_NUMERIC", "en_US.UTF-8"),
            include(os.path.join(moveit, "launch", "real.launch.py"), python, rviz=rviz),
            include(
                os.path.join(moveit, "launch", "demo.launch.py"),
                LaunchConfigurationEquals("mode", "mock"),
                use_rviz=rviz,
            ),
            *ip_stack,
            Node(
                package="lwa4p_vla_bridge",
                executable="bridge",
                name="lwa4p_vla_bridge",
                output="screen",
                parameters=[os.path.join(bridge, "config", "bridge.yaml")],
            ),
        ]
    )
