"""Common launch utilities for WujiHand."""

import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def spawn_robot_state_publisher(context):
    """Spawn robot_state_publisher after detecting handedness from driver.

    This function polls the driver node for the handedness parameter,
    then creates a robot_state_publisher with the appropriate URDF.

    Args:
        context: Launch context

    Returns:
        List containing robot_state_publisher Node, or empty list on failure
    """
    hand_name = LaunchConfiguration("hand_name").perform(context)
    driver_node_name = f"/{hand_name}/wujihand_driver"
    wujihand_description_dir = get_package_share_directory("wujihand_description")

    # Poll for handedness parameter (retry up to 30 times with 0.5s interval)
    hand_type = None
    for _ in range(30):
        try:
            result = subprocess.run(
                ["ros2", "param", "get", driver_node_name, "handedness"],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            if result.returncode == 0:
                output = result.stdout.strip().lower()
                if "left" in output:
                    hand_type = "left"
                    break
                elif "right" in output:
                    hand_type = "right"
                    break
        except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError):
            # Expected during driver startup, continue polling
            pass
        time.sleep(0.5)

    if hand_type is None:
        print("[WARN] Could not detect handedness, defaulting to 'right'")
        hand_type = "right"

    print(f"[INFO] Detected handedness: {hand_type}")

    # Use xacro to process the URDF with prefix
    xacro_file = os.path.join(
        wujihand_description_dir, "urdf", f"{hand_type}.urdf.xacro"
    )
    prefix = f"{hand_name}/"
    try:
        result = subprocess.run(
            ["xacro", xacro_file, f"prefix:={prefix}"],
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        if result.returncode != 0:
            print(f"[ERROR] xacro failed: {result.stderr}")
            return []
        robot_description = result.stdout
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, OSError) as e:
        print(f"[ERROR] Failed to process xacro: {e}")
        return []

    # Return robot_state_publisher node with URDF string as parameter
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=hand_name,
            parameters=[
                {"robot_description": ParameterValue(robot_description, value_type=str)}
            ],
            output="screen",
        )
    ]


def get_common_launch_arguments():
    """Return common launch argument declarations.

    Returns:
        List of DeclareLaunchArgument actions
    """
    return [
        DeclareLaunchArgument(
            "hand_name",
            default_value="hand_0",
            description="Hand name used as namespace and URDF prefix",
        ),
        DeclareLaunchArgument(
            "serial_number",
            default_value="",
            description="Serial number of the WujiHand device",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="1000.0",
            description="Joint state publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "filter_cutoff_freq",
            default_value="10.0",
            description="Low-pass filter cutoff frequency in Hz",
        ),
        DeclareLaunchArgument(
            "diagnostics_rate",
            default_value="10.0",
            description="Diagnostics publish rate in Hz",
        ),
    ]
