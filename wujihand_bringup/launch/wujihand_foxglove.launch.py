import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wujihand_bringup_dir = get_package_share_directory("wujihand_bringup")
    wujihand_description_dir = get_package_share_directory("wujihand_description")

    # URDF file (default to right hand)
    urdf_file = os.path.join(wujihand_description_dir, "urdf", "right-ros.urdf")

    # Launch arguments
    serial_number_arg = DeclareLaunchArgument(
        "serial_number",
        default_value="",
        description="Serial number of the WujiHand device",
    )

    # Read URDF file
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
        emulate_tty=True,
    )

    # WujiHand driver launch
    wujihand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wujihand_bringup_dir, "launch", "wujihand.launch.py")
        ),
        launch_arguments={
            "serial_number": LaunchConfiguration("serial_number"),
        }.items(),
    )

    # Foxglove Bridge node with asset URI allowlist for mesh loading
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "send_buffer_limit": 50000000,
                "use_sim_time": False,
                "max_qos_depth": 10,
                # Allow loading mesh files from ROS packages
                "asset_uri_allowlist": [
                    "^package://.*\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro|STL)$"
                ],
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            serial_number_arg,
            robot_state_publisher_node,
            wujihand_launch,
            foxglove_bridge_node,
        ]
    )
