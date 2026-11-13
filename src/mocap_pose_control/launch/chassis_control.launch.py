from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from robokit_control.limits import DEFAULT_HOST


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("mocap_pose_control")) / "config" / "control.yaml")
    config = LaunchConfiguration("config")
    controller = Node(
        package="mocap_pose_control", executable="pose_controller", output="screen",
        parameters=[config, {
            "rigid_body_name": ParameterValue(LaunchConfiguration("rigid_body_name"), value_type=str),
        }],
    )
    bridge = Node(
        package="robokit_control", executable="robokit_cmd_vel", output="screen",
        parameters=[config, {
            "hostname": ParameterValue(LaunchConfiguration("hostname"), value_type=str),
            "robot_ip": ParameterValue(LaunchConfiguration("robot_ip"), value_type=str),
            "protocol_version": ParameterValue(LaunchConfiguration("protocol_version"), value_type=int),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("hostname", default_value=DEFAULT_HOST, description="Robokit IP or hostname"),
        DeclareLaunchArgument("robot_ip", default_value="", description="Legacy address override; takes precedence when nonempty"),
        DeclareLaunchArgument("rigid_body_name", description="Exact chassis rigid_body_name in /rigid_bodies"),
        DeclareLaunchArgument("protocol_version", default_value="1", description="RBK3.4=1, RBK3.5=2"),
        DeclareLaunchArgument("config", default_value=default_config),
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason="Pose controller exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="TCP bridge exited"))],
        )),
        controller,
        bridge,
    ])
