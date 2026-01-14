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


def spawn_robot_state_publisher(context):
    """Spawn robot_state_publisher after detecting handedness from driver.

    This function first checks if hand_type is explicitly provided via launch argument.
    If not, it polls the driver node for the handedness parameter.

    Args:
        context: Launch context

    Returns:
        List containing robot_state_publisher Node, or empty list on failure
    """
    hand_name = LaunchConfiguration("hand_name").perform(context)
    driver_node_name = f"/{hand_name}/wujihand_driver"
    wuji_hand_description_dir = get_package_share_directory("wuji_hand_description")

    # Check if hand_type is explicitly provided via launch argument
    hand_type_arg = LaunchConfiguration("hand_type").perform(context)

    hand_type = None
    if hand_type_arg and hand_type_arg.lower() in ["left", "right"]:
        hand_type = hand_type_arg.lower()
        _logger.info(f"Using explicitly provided hand_type: {hand_type}")
    else:
        _logger.info(f"Attempting to detect handedness from {driver_node_name}")

        # Use ROS 2 Python API to get parameter (more reliable than subprocess)
        try:
            if not rclpy.ok():
                rclpy.init()
            temp_node = RclpyNode("_handedness_detector_temp")

            # Poll for handedness parameter (retry up to 30 times with 0.5s interval)
            for attempt in range(30):
                try:
                    # Get parameter client for the driver node
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
            f"Could not detect handedness from {driver_node_name} after 15 seconds. "
            "Please ensure the driver node is running and the device is connected, "
            "or specify hand_type:=left or hand_type:=right explicitly."
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
