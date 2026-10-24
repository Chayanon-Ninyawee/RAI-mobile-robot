import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("ired_bringup")
    navigation_share = get_package_share_directory("ired_navigation")
    refbox_share = get_package_share_directory("ired_refbox")
    lidar_share = get_package_share_directory("rplidar_ros")

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    param_dir = LaunchConfiguration("param_dir")
    laser_param_dir = LaunchConfiguration("laser_param_dir")
    refbox_param_dir = LaunchConfiguration("refbox_param_dir")
    map_generator_param_dir = LaunchConfiguration("map_generator_param_dir")
    nav_param_dir = LaunchConfiguration("nav_param_dir")
    map_file = LaunchConfiguration("map_file")

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    urdf_file = os.path.join(
        bringup_share,
        "urdf",
        "ired.urdf.xacro",
    )

    robot_description = Command(["xacro ", urdf_file])

    robot_state_publisher = Node(
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
    )

    # ---------------------------------------------------------
    # Hardware and base nodes
    # ---------------------------------------------------------

    forward_kinematics = Node(
        package="ired_bringup",
        executable="forward_kinematics",
        parameters=[param_dir],
        output="screen",
    )

    inverse_kinematics = Node(
        package="ired_bringup",
        executable="inverse_kinematics",
        parameters=[param_dir],
        output="screen",
    )

    imu_node = Node(
        package="ired_bringup",
        executable="imu_node",
        parameters=[param_dir],
        output="screen",
    )

    odom_node = Node(
        package="ired_bringup",
        executable="odom_node",
        parameters=[param_dir],
        output="screen",
    )

    modbus_serial = Node(
        package="ired_bringup",
        executable="modbus_serial",
        parameters=[param_dir],
        output="screen",
    )

    aruco_node = Node(
        package="ired_bringup",
        executable="aruco_node",
        parameters=[param_dir],
        output="screen",
    )

    # ---------------------------------------------------------
    # LiDAR and scan filtering
    # ---------------------------------------------------------

    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lidar_share,
                "launch",
                "rplidar_a1_launch.py",
            )
        ),
        launch_arguments={
            "serial_port": "/dev/ttyLiDAR",
            "serial_baudrate": "115200",
            "frame_id": "base_scan",
        }.items(),
    )

    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[param_dir, laser_param_dir],
        output="screen",
    )

    # ---------------------------------------------------------
    # RefBox
    # ---------------------------------------------------------

    refbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                refbox_share,
                "launch",
                "refbox.launch.py",
            )
        ),
        launch_arguments={
            "refbox_param": refbox_param_dir,
        }.items(),
    )

    refbox_parser = Node(
        package="ired_bringup",
        executable="refbox_parser",
        name="refbox_parser",
        parameters=[refbox_param_dir],
        output="screen",
    )

    # ---------------------------------------------------------
    # Map generator
    # ---------------------------------------------------------

    map_generator = Node(
        package="ired_bringup",
        executable="map_generator",
        name="map_generator",
        parameters=[map_generator_param_dir],
        output="screen",
    )

    # ---------------------------------------------------------
    # Navigation
    # ---------------------------------------------------------

    navigation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "navigation.launch.xml",
            )
        ),
        launch_arguments={
            "nav_param_dir": nav_param_dir,
            "map_file": map_file,
        }.items(),
    )

    # ---------------------------------------------------------
    # Localization and challenge manager
    # ---------------------------------------------------------

    localization_initializer = Node(
        package="ired_bringup",
        executable="localization_initializer",
        name="localization_initializer",
        output="screen",
    )

    challenge_manager = Node(
        package="ired_bringup",
        executable="challenge_manager",
        name="challenge_manager",
        output="screen",
    )

    # ---------------------------------------------------------
    # Startup event handler
    # ---------------------------------------------------------

    map_output = ""
    navigation_started = False

    def start_navigation_after_map(event):
        nonlocal map_output, navigation_started

        chunk = event.text
        if isinstance(chunk, bytes):
            chunk = chunk.decode(errors="replace")

        map_output = (map_output + chunk)[-4096:]

        if (
            not navigation_started
            and "Map generation completed successfully" in map_output
        ):
            navigation_started = True

            return [
                LogInfo(
                    msg=(
                        "Map generation completed. "
                        "Starting Nav2, localization, and challenge manager..."
                    )
                ),
                navigation,
                localization_initializer,
                challenge_manager,
            ]

        return []

    start_navigation_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=map_generator,
            on_stdout=start_navigation_after_map,
            on_stderr=start_navigation_after_map,
        )
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription(
        [
            LogInfo(msg="Starting iRED bringup..."),
            DeclareLaunchArgument(
                "param_dir",
                default_value=os.path.join(
                    bringup_share,
                    "param",
                    "ired.yaml",
                ),
                description="Robot parameter file",
            ),
            DeclareLaunchArgument(
                "laser_param_dir",
                default_value=os.path.join(
                    bringup_share,
                    "param",
                    "laser_config.yaml",
                ),
                description="Laser filter parameter file",
            ),
            DeclareLaunchArgument(
                "refbox_param_dir",
                default_value=os.path.join(
                    bringup_share,
                    "param",
                    "refbox.yaml",
                ),
                description="RefBox parameter file",
            ),
            DeclareLaunchArgument(
                "map_generator_param_dir",
                default_value=os.path.join(
                    bringup_share,
                    "param",
                    "map_generator.yaml",
                ),
                description="Map generator parameter file",
            ),
            DeclareLaunchArgument(
                "nav_param_dir",
                default_value=os.path.join(
                    bringup_share,
                    "param",
                    "navigation.yaml",
                ),
                description="Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="/tmp/ired_map/map.yaml",
                description="Generated map YAML file",
            ),
            # Register the handler before starting map_generator.
            start_navigation_handler,
            robot_state_publisher,
            forward_kinematics,
            inverse_kinematics,
            imu_node,
            odom_node,
            modbus_serial,
            aruco_node,
            rplidar_node,
            scan_filter,
            refbox,
            refbox_parser,
            map_generator,
        ]
    )
