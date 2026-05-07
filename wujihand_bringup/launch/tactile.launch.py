"""Launch the tactile sensor driver and the static TF that anchors its frame.

Composable: wujihand_full.launch.py IncludeLaunchDescription's this file
with `parent_frame` and `namespace` overridden to slot the tactile sensor
under a hand-namespaced TF tree.

Auto-discovery (empty serial_number) succeeds only when exactly one
device with PID 0x5700 is on the bus. If two or more tactile boards are
connected, the SDK refuses to pick one and the driver throws on connect
listing the found serials — pin serial_number explicitly when running
in a multi-tactile rig.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import discover_usb_devices  # noqa: E402


def _warn_if_ambiguous_serial(context):
    """Warn when >1 tactile board is on USB without an explicit serial pin.

    The SDK refuses to pick when multiple gloves match; the driver will
    throw on connect. Surface this as a launch-time WARN so the operator
    sees the issue before the node crashes.
    """
    if LaunchConfiguration("serial_number").perform(context):
        return []
    _, tactiles = discover_usb_devices()
    if len(tactiles) <= 1:
        return []
    return [LogInfo(msg=(
        f"[tactile.launch.py] WARN: {len(tactiles)} tactile boards on USB and "
        "no serial_number set — the driver will refuse to start. "
        "Pass serial_number:=<SN> to pin one."
    ))]


def generate_launch_description():
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for the tactile driver node and "
                        "the static TF (empty = global namespace).",
        ),
        DeclareLaunchArgument(
            "serial_number",
            default_value="",
            description="Tactile board USB serial number (empty = auto-"
                        "discover; non-deterministic if >1 board attached).",
        ),

        # When the operator left serial_number unset and >1 tactile board
        # is on the bus, log a warning before the driver starts. We don't
        # hard-fail because single-board setups shouldn't need to know
        # their serial number.
        OpaqueFunction(function=_warn_if_ambiguous_serial),
        DeclareLaunchArgument(
            "image_rate",
            default_value="30.0",
            description="Heatmap image publish rate (Hz).",
        ),
        DeclareLaunchArgument(
            "sample_rate_hz",
            default_value="120",
            description="Tactile data-frame rate (1..120, applied at startup).",
        ),
        DeclareLaunchArgument(
            "streaming_at_startup",
            default_value="true",
            description="Whether to enable tactile streaming when the driver starts.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="tactile_sensor_link",
            description="TF frame ID published by the tactile driver.",
        ),
        DeclareLaunchArgument(
            "joint_driver_node",
            default_value="",
            description="Sibling joint-controller node name to cross-check "
                        "handedness against. Empty (default) skips the check; "
                        "wujihand_full sets this to 'wujihand_driver' so a "
                        "left-controller + right-glove (or vice versa) "
                        "mis-pairing is rejected at startup.",
        ),
        DeclareLaunchArgument(
            "parent_frame",
            default_value="palm_link",
            description="TF parent frame to which the tactile sensor frame is "
                        "attached. Standalone runs default to 'palm_link'; "
                        "wujihand_full passes '<handedness>_palm_link' so the "
                        "tactile frame anchors under the joint URDF tree.",
        ),

        # --- Tactile driver node ---
        Node(
            package="wujihand_tactile_driver",
            executable="tactile_driver_node",
            name="tactile_driver_node",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "serial_number": ParameterValue(
                    LaunchConfiguration("serial_number"), value_type=str),
                "image_rate": ParameterValue(
                    LaunchConfiguration("image_rate"), value_type=float),
                "sample_rate_hz": ParameterValue(
                    LaunchConfiguration("sample_rate_hz"), value_type=int),
                "streaming_at_startup": ParameterValue(
                    LaunchConfiguration("streaming_at_startup"), value_type=bool),
                "frame_id": LaunchConfiguration("frame_id"),
                "joint_driver_node": LaunchConfiguration("joint_driver_node"),
            }],
            output="screen",
            emulate_tty=True,
        ),

        # --- Static TF: parent_frame → frame_id ---
        # Identity offset by default. Update with measured calibration once
        # the sensor is mounted.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tactile_tf",
            namespace=LaunchConfiguration("namespace"),
            arguments=["0", "0", "0", "0", "0", "0",
                       LaunchConfiguration("parent_frame"),
                       LaunchConfiguration("frame_id")],
        ),
    ])
