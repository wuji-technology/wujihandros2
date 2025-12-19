"""Launch WujiHand driver with Foxglove Bridge for visualization."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Add launch directory to path for local imports
sys.path.insert(0, os.path.dirname(__file__))
from common import get_common_launch_arguments, spawn_robot_state_publisher


def generate_launch_description():
    wujihand_bringup_dir = get_package_share_directory("wujihand_bringup")

    hand_name = LaunchConfiguration("hand_name")

    # WujiHand driver launch
    wujihand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wujihand_bringup_dir, "launch", "wujihand.launch.py")
        ),
        launch_arguments={
            "hand_name": hand_name,
            "serial_number": LaunchConfiguration("serial_number"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "filter_cutoff_freq": LaunchConfiguration("filter_cutoff_freq"),
            "diagnostics_rate": LaunchConfiguration("diagnostics_rate"),
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
                "asset_uri_allowlist": [
                    "^package://.*\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro|STL)$"
                ],
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    # Auto-detect handedness and spawn robot_state_publisher after driver starts
    auto_detect_action = TimerAction(
        period=1.0,  # Wait 1 second for driver to start
        actions=[OpaqueFunction(function=spawn_robot_state_publisher)],
    )

    return LaunchDescription(
        get_common_launch_arguments()
        + [
            # Always launch driver first
            wujihand_launch,
            # Auto-detect handedness and spawn robot_state_publisher
            auto_detect_action,
            # Foxglove Bridge
            foxglove_bridge_node,
        ]
    )
