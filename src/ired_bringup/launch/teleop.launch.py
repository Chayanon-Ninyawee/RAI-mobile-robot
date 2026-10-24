import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("ired_bringup")
    lidar_dir = get_package_share_directory("rplidar_ros")

    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(bringup_dir, "param", "ired.yaml"),
    )

    urdf_file = os.path.join(bringup_dir, "urdf", "ired.urdf.xacro")
    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription(
        [
            LogInfo(msg="Starting iRED Teleoperation (no Refbox or navigation)..."),
            DeclareLaunchArgument(
                "param_dir",
                default_value=os.path.join(bringup_dir, "param", "ired.yaml"),
                description="Path to the robot parameter file",
            ),
            # Robot model and TF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 10.0,
                    }
                ],
            ),
            # Motor control and odometry
            Node(
                package="ired_bringup",
                executable="forward_kinematics",
                parameters=[param_dir],
                output="screen",
            ),
            Node(
                package="ired_bringup",
                executable="inverse_kinematics",
                parameters=[param_dir],
                output="screen",
            ),
            Node(
                package="ired_bringup",
                executable="imu_node",
                parameters=[param_dir],
                output="screen",
            ),
            Node(
                package="ired_bringup",
                executable="odom_node",
                parameters=[param_dir],
                output="screen",
            ),
            Node(
                package="ired_bringup",
                executable="modbus_serial",
                parameters=[param_dir],
                output="screen",
            ),
            # LiDAR
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lidar_dir, "launch", "rplidar_a1_launch.py")
                ),
                launch_arguments={
                    "serial_port": "/dev/ttyLiDAR",
                    "serial_baudrate": "115200",
                    "frame_id": "base_scan",
                }.items(),
            ),
        ]
    )
