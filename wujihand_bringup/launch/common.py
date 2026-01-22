"""Common launch utilities for WujiHand."""

import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import logging
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.node import Node as RclpyNode

# Get launch logger
_logger = logging.get_logger(__name__)


def detect_handedness(hand_name, timeout_sec=15):
    """Detect handedness from driver node.

    Args:
        hand_name: Hand name (namespace)
        timeout_sec: Timeout in seconds (default 15s)

    Returns:
        "left" or "right" if detected, None otherwise
    """
    driver_node_name = f"/{hand_name}/wujihand_driver"
    _logger.info(f"Attempting to detect handedness from {driver_node_name}")

    hand_type = None
    try:
        if not rclpy.ok():
            rclpy.init()
        temp_node = RclpyNode("_handedness_detector_temp")

        # Poll for handedness parameter
        max_attempts = int(timeout_sec / 0.5)
        for attempt in range(max_attempts):
            try:
                from rcl_interfaces.srv import GetParameters

                client = temp_node.create_client(
                    GetParameters, f"{driver_node_name}/get_parameters"
                )
                if client.wait_for_service(timeout_sec=1.0):
                    request = GetParameters.Request()
                    request.names = ["handedness"]
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(temp_node, future, timeout_sec=2.0)
                    if future.result() is not None:
                        values = future.result().values
                        if values and values[0].string_value:
                            hand_type = values[0].string_value.lower()
                            _logger.info(f"Detected handedness: {hand_type}")
                            break
                temp_node.destroy_client(client)
            except Exception as e:
                _logger.debug(f"Attempt {attempt + 1}: {e}")
            time.sleep(0.5)

        temp_node.destroy_node()
    except Exception as e:
        _logger.error(f"Error detecting handedness: {e}")

    if hand_type is None or hand_type not in ["left", "right"]:
        _logger.error(
            f"Could not detect handedness from {driver_node_name} after {timeout_sec} seconds."
        )
        return None

    return hand_type


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
    wuji_hand_description_dir = get_package_share_directory("wuji_hand_description")

    # Detect handedness from driver node
    hand_type = detect_handedness(hand_name)
    if hand_type is None:
        _logger.error(
            "Please ensure the driver node is running and the device is connected."
        )
        return []

    _logger.info(f"Using handedness: {hand_type}")

    # Read the pre-generated ROS URDF file
    urdf_file = os.path.join(
        wuji_hand_description_dir, "urdf", f"{hand_type}-ros.urdf"
    )
    try:
        with open(urdf_file, "r") as f:
            robot_description = f.read()
    except OSError as e:
        _logger.error(f"Failed to read URDF file: {e}")
        return []

    # Return robot_state_publisher node with URDF string as parameter
    # Note: robot_state_publisher subscribes to joint_states with SensorDataQoS by default
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=hand_name,
            parameters=[
                {
                    "robot_description": ParameterValue(
                        robot_description, value_type=str
                    )
                },
                {"publish_frequency": 100.0},
            ],
            remappings=[
                ("joint_states", "joint_states"),
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
