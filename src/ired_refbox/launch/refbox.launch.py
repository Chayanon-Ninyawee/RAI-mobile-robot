import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("ired_refbox")

    return LaunchDescription(
        [
            Node(
                package="ired_refbox",
                executable="ired_refbox_node",
                name="ired_refbox",
                output="screen",
                parameters=[
                    os.path.join(package_dir, "config", "refbox.yaml"),
                ],
            ),
        ]
    )
