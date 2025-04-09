"""Launch file to launch the moveit rviz demo as the control UI while controlling the actual servos."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_arm_arduino_bridge = get_package_share_path("arm_arduino_bridge")
    pkg_arm_moveit_config = get_package_share_path("arm_moveit_config")

    node_bridge = Node(package="arm_arduino_bridge", executable="bridge")
    launch_moveit = IncludeLaunchDescription(
        str(pkg_arm_moveit_config / "launch" / "demo.launch.py")
    )

    ld.add_action(launch_moveit)
    ld.add_action(TimerAction(period=3.0, actions=[node_bridge]))

    return ld
