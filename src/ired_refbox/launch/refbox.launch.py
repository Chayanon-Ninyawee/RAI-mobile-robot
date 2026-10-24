import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    refbox_param = LaunchConfiguration("refbox_param")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "refbox_param",
                default_value=os.path.join(
                    get_package_share_directory("ired_bringup"),
                    "param",
                    "refbox.yaml",
                ),
                description="Path to the RefBox configuration file",
            ),
            Node(
                package="ired_refbox",
                executable="ired_refbox_node",
                name="ired_refbox",
                output="log",
                parameters=[refbox_param],
            ),
        ]
    )
