import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("ired_bringup"), "param", "ired.yaml"
        ),
    )
    laser_param_dir = LaunchConfiguration(
        "laser_param_dir",
        default=os.path.join(
            get_package_share_directory("ired_bringup"), "param", "laser_config.yaml"
        ),
    )
    refbox_param_dir = LaunchConfiguration(
        "refbox_param_dir",
        default=os.path.join(
            get_package_share_directory("ired_bringup"), "param", "refbox.yaml"
        ),
    )
    map_generator_param_dir = LaunchConfiguration(
        "map_generator_param_dir",
        default=os.path.join(
            get_package_share_directory("ired_bringup"),
            "param",
            "map_generator.yaml",
        ),
    )
    nav_param_dir = LaunchConfiguration(
        "nav_param_dir",
        default=os.path.join(
            get_package_share_directory("ired_bringup"),
            "param",
            "navigation.yaml",
        ),
    )
    urdf_file = os.path.join(
        get_package_share_directory("ired_bringup"), "urdf", "ired.urdf.xacro"
    )
    robot_description = Command(["xacro ", str(urdf_file)])
    rplidar_launch = LaunchConfiguration(
        "rplidar_launch",
        default=os.path.join(get_package_share_directory("rplidar_ros"), "launch"),
    )

    map_file = LaunchConfiguration(
        "map_file",
        default="/tmp/ired_map/map.yaml",
    )
    map_generator = Node(
        package="ired_bringup",
        executable="map_generator",
        name="map_generator",
        output="screen",
        parameters=[map_generator_param_dir],
    )
    navigation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ired_navigation"),
                "launch",
                "navigation.launch.xml",
            )
        ),
        launch_arguments={
            "nav_param_dir": nav_param_dir,
            "map_file": map_file,
        }.items(),
    )

    def start_navigation_if_successful(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg="Map generated successfully. Starting navigation..."),
                navigation,
            ]

        return [
            LogInfo(
                msg=(
                    f"Map generation failed with exit code {event.returncode}. "
                    "Navigation will not start."
                )
            ),
        ]

    start_navigation_after_map = RegisterEventHandler(
        OnProcessExit(
            target_action=map_generator,
            on_exit=lambda event, context: start_navigation_if_successful(
                event, context
            ),
        )
    )
    return LaunchDescription(
        [
            LogInfo(msg=["Starting iRED Bringup..."]),
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
                description="Specifying parameter direction",
            ),
            DeclareLaunchArgument(
                "laser_param_dir",
                default_value=laser_param_dir,
                description="Laser filter parameter file",
            ),
            DeclareLaunchArgument(
                "refbox_param_dir",
                default_value=refbox_param_dir,
                description="RefBox parameter file",
            ),
            DeclareLaunchArgument(
                "map_generator_param_dir",
                default_value=map_generator_param_dir,
                description="Path to the map generator parameters",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=map_file,
                description="Path to the generated map YAML",
            ),
            DeclareLaunchArgument(
                "nav_param_dir",
                default_value=nav_param_dir,
                description="Path to the Nav2 parameter file",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "publish_frequency": 10.0}
                ],
            ),
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
            Node(
                package="ired_bringup",
                executable="aruco_node",
                parameters=[param_dir],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [rplidar_launch, "/rplidar_a1_launch.py"]
                ),
                launch_arguments={
                    "serial_port": "/dev/ttyLiDAR",
                    "serial_baudrate": "115200",
                    "frame_id": "base_scan",
                }.items(),
            ),
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[param_dir, laser_param_dir],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ired_refbox"),
                        "launch",
                        "refbox.launch.py",
                    )
                ),
                launch_arguments={
                    "refbox_param": refbox_param_dir,
                }.items(),
            ),
            Node(
                package="ired_bringup",
                executable="refbox_parser",
                name="refbox_parser",
                output="screen",
            ),
            map_generator,
            start_navigation_after_map,
        ]
    )
