import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch both hands by including the single-hand launch twice.

    Left hand connects via hand_side='left', right via 'right'. A serial_number
    override (when provided) takes precedence over hand_side in the driver.
    """
    single_launch = os.path.join(
        get_package_share_directory("wujihand_bringup"), "launch", "wujihand.launch.py"
    )

    args = [
        DeclareLaunchArgument(
            "left_hand_name", default_value="left_hand", description="Left hand namespace"
        ),
        DeclareLaunchArgument(
            "right_hand_name", default_value="right_hand", description="Right hand namespace"
        ),
        DeclareLaunchArgument(
            "left_serial_number",
            default_value="",
            description="Left hand serial number (overrides hand_side when set)",
        ),
        DeclareLaunchArgument(
            "right_serial_number",
            default_value="",
            description="Right hand serial number (overrides hand_side when set)",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false", description="Launch RViz for both hands"
        ),
        DeclareLaunchArgument(
            "foxglove", default_value="false", description="Launch Foxglove Bridge for both hands"
        ),
    ]

    left_hand = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            "hand_name": LaunchConfiguration("left_hand_name"),
            "hand_side": "left",
            "serial_number": LaunchConfiguration("left_serial_number"),
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": LaunchConfiguration("foxglove"),
        }.items(),
    )

    right_hand = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            "hand_name": LaunchConfiguration("right_hand_name"),
            "hand_side": "right",
            "serial_number": LaunchConfiguration("right_serial_number"),
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": LaunchConfiguration("foxglove"),
        }.items(),
    )

    return LaunchDescription(args + [left_hand, right_hand])
