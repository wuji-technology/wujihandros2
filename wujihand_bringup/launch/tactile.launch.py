"""Launch the tactile sensor driver and the static TF that anchors its frame.

Composable: wujihand_full.launch.py IncludeLaunchDescription's this file
with `parent_frame` and `namespace` overridden to slot the tactile sensor
under a hand-namespaced TF tree.

Auto-discovery (empty serial_number) picks the first device with PID 0x5700
on the bus. If two tactile boards are connected at the same time, the
chosen one is non-deterministic across reboots/USB re-enumeration. Pin
serial_number explicitly when running in a multi-tactile rig.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import TACTILE_VID_PID  # noqa: E402


def _warn_if_ambiguous_serial(context):
    """Warn (don't fail) when multiple tactile boards are present and the
    user didn't pin serial_number. The driver's auto-discover picks the
    first match, but `/sys` enumeration order is not stable across
    reboots — operators should pin to get a deterministic launch.
    """
    if LaunchConfiguration("serial_number").perform(context):
        return []  # explicit pin: nothing to warn about
    try:
        from pathlib import Path
        count = 0
        for entry in os.listdir("/sys/bus/usb/devices"):
            dev = f"/sys/bus/usb/devices/{entry}"
            if not os.path.isfile(f"{dev}/idVendor"):
                continue
            try:
                vid = Path(f"{dev}/idVendor").read_text().strip()
                pid = Path(f"{dev}/idProduct").read_text().strip()
            except (OSError, PermissionError):
                continue
            if f"{vid}:{pid}" == TACTILE_VID_PID:
                count += 1
        if count > 1:
            return [LogInfo(
                msg=f"[tactile.launch.py] WARN: {count} tactile boards "
                    f"on USB; auto-discover will pick the first one "
                    f"non-deterministically. Pass serial_number:=<SN> "
                    f"to pin."
            )]
    except OSError:
        pass  # /sys unreadable — nothing we can warn about
    return []


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
