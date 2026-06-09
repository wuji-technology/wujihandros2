import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Add launch directory to path for local imports
sys.path.insert(0, os.path.dirname(__file__))
from common import detect_handedness, list_serial_numbers, make_robot_state_publisher

_logger = logging.get_logger("wujihand_dual")


def _spawn_description(node_hand_name, want_rviz):
    """Build an OpaqueFunction that, once the driver is up, detects its handedness
    and spawns robot_state_publisher (and optionally RViz) under /hand_<handedness>,
    matching the driver's handedness-named topics."""

    def _fn(context):
        hand_type = detect_handedness(node_hand_name)
        if hand_type is None:
            _logger.error(f"Could not detect handedness for {node_hand_name}; skipping RViz/URDF.")
            return []

        namespace = f"hand_{hand_type}"
        actions = []
        rsp = make_robot_state_publisher(hand_type, namespace)
        if rsp is not None:
            actions.append(rsp)

        if want_rviz:
            rviz_config = os.path.join(
                get_package_share_directory("wuji_description"), "rviz", f"{hand_type}.rviz"
            )
            actions.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    namespace=namespace,
                    arguments=["-d", rviz_config],
                    output="screen",
                )
            )
        return actions

    return _fn


def spawn_dual_hands(context):
    """Discover connected devices by serial number and start one driver per device.

    Discovery (wujihand_list) only reads USB serial descriptors, and each driver
    connects by its own serial number, so two drivers never race on the USB claim.
    Each driver self-detects its handedness and publishes topics under
    /hand_<handedness>; the node itself stays under a generic hand_<index> namespace.
    """
    serial_numbers = list_serial_numbers()
    if not serial_numbers:
        _logger.error("No WujiHand devices found. Is the hardware connected?")
        return []

    _logger.info(f"Discovered {len(serial_numbers)} device(s): {', '.join(serial_numbers)}")
    want_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")

    actions = []
    for idx, serial_number in enumerate(serial_numbers):
        node_hand_name = f"hand_{idx}"
        actions.append(
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace=node_hand_name,
                parameters=[{"serial_number": serial_number, "name_by_handedness": True}],
                output="screen",
                emulate_tty=True,
            )
        )
        # Wait for the driver to connect and publish its handedness, then spawn
        # robot_state_publisher (and optional RViz) under /hand_<handedness>.
        actions.append(
            TimerAction(
                period=2.0,
                actions=[OpaqueFunction(function=_spawn_description(node_hand_name, want_rviz))],
            )
        )
    return actions


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Whether to launch RViz (one window per hand)",
    )

    foxglove_arg = DeclareLaunchArgument(
        "foxglove",
        default_value="false",
        description="Whether to launch a single Foxglove Bridge for web visualization",
    )

    # One Foxglove Bridge bridges all topics for both hands (avoids a port clash
    # that two per-hand bridges would hit on the default 8765).
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    return LaunchDescription(
        [
            rviz_arg,
            foxglove_arg,
            OpaqueFunction(function=spawn_dual_hands),
            foxglove_bridge_node,
        ]
    )
