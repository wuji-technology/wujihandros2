"""One-click launch for WujiHand: hand driver + tactile sensor + visualization.

Usage:
    # Auto-discover all hardware, launch everything:
    ros2 launch wujihand_bringup wujihand_full.launch.py

    # With RViz (requires display):
    ros2 launch wujihand_bringup wujihand_full.launch.py viz:=rviz

    # With Foxglove (web-based, no display needed):
    ros2 launch wujihand_bringup wujihand_full.launch.py viz:=foxglove

    # Disable tactile:
    ros2 launch wujihand_bringup wujihand_full.launch.py tactile:=false

    # Override serial numbers (advanced):
    ros2 launch wujihand_bringup wujihand_full.launch.py hand_serial:=347838683433 tactile_serial:=12345678
"""

import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import detect_handedness, spawn_robot_state_publisher

_logger = logging.get_logger("wujihand_full")

# USB VID:PID for device identification
WUJIHAND_VID_PID = "0483:2000"  # Sboard (hand controller)
TACTILE_VID_PID = "0483:5700"   # G-Board (tactile sensor)


def discover_usb_devices():
    """Scan USB for WujiHand devices. Returns (hand_serials, tactile_serials)."""
    hands = []
    tactiles = []
    try:
        # Use /sys/bus/usb/devices to get serial numbers
        for entry in os.listdir("/sys/bus/usb/devices"):
            dev_path = f"/sys/bus/usb/devices/{entry}"
            vid_path = f"{dev_path}/idVendor"
            pid_path = f"{dev_path}/idProduct"
            serial_path = f"{dev_path}/serial"
            product_path = f"{dev_path}/product"
            if not os.path.isfile(vid_path):
                continue
            try:
                from pathlib import Path
                vid = Path(vid_path).read_text().strip()
                pid = Path(pid_path).read_text().strip()
                serial = Path(serial_path).read_text().strip() if os.path.isfile(serial_path) else ""
                product = Path(product_path).read_text().strip() if os.path.isfile(product_path) else ""
            except (OSError, PermissionError):
                continue

            vid_pid = f"{vid}:{pid}"
            if vid_pid == WUJIHAND_VID_PID:
                hands.append(serial)
                _logger.info(f"Found hand: {product} (SN: {serial})")
            elif vid_pid == TACTILE_VID_PID:
                tactiles.append(serial)
                _logger.info(f"Found tactile: {product} (SN: {serial})")
    except OSError as e:
        _logger.warning(f"USB scan failed: {e}")
    return hands, tactiles


def setup_full_launch(context):
    """OpaqueFunction: discover devices and spawn all nodes."""
    hand_name = LaunchConfiguration("hand_name").perform(context)
    viz = LaunchConfiguration("viz").perform(context)
    tactile_enabled = LaunchConfiguration("tactile").perform(context).lower() == "true"

    # Manual overrides
    hand_serial_override = LaunchConfiguration("hand_serial").perform(context)
    tactile_serial_override = LaunchConfiguration("tactile_serial").perform(context)

    # Auto-discover (scan once, use results for both)
    discovered_hands, discovered_tactiles = discover_usb_devices()
    hand_serials = [hand_serial_override] if hand_serial_override else discovered_hands
    tactile_serials = [tactile_serial_override] if tactile_serial_override else discovered_tactiles

    if not hand_serials:
        _logger.error("No WujiHand device found! Check USB connection.")
        return []

    _logger.info(f"Launching with hand SN={hand_serials[0]}, "
                 f"tactile={'SN=' + tactile_serials[0] if tactile_serials else 'none'}")

    nodes = [SetLaunchConfiguration("tactile_active", "false")]

    # --- Hand driver node ---
    nodes.append(Node(
        package="wujihand_driver",
        executable="wujihand_driver_node",
        name="wujihand_driver",
        namespace=hand_name,
        parameters=[{
            "serial_number": hand_serials[0],
            "publish_rate": float(LaunchConfiguration("publish_rate").perform(context)),
            "filter_cutoff_freq": float(LaunchConfiguration("filter_cutoff_freq").perform(context)),
            "diagnostics_rate": float(LaunchConfiguration("diagnostics_rate").perform(context)),
        }],
        output="screen",
        emulate_tty=True,
    ))

    # --- Tactile driver node (under hand namespace) ---
    if tactile_enabled and tactile_serials:
        nodes.append(SetLaunchConfiguration("tactile_active", "true"))
        nodes.append(Node(
            package="wujihand_driver",
            executable="tactile_driver_node",
            name="tactile_driver_node",
            namespace=hand_name,
            parameters=[{
                "serial_number": tactile_serials[0],
                "image_rate": float(LaunchConfiguration("image_rate").perform(context)),
                "pressure_max": int(LaunchConfiguration("pressure_max").perform(context)),
                "frame_id": "tactile_sensor_link",
            }],
            output="screen",
            emulate_tty=True,
        ))
    elif tactile_enabled:
        _logger.warning("Tactile enabled but no tactile board found. Skipping.")

    return nodes


def setup_viz_and_urdf(context):
    """Spawn robot_state_publisher and visualization after driver connects."""
    hand_name = LaunchConfiguration("hand_name").perform(context)
    viz = LaunchConfiguration("viz").perform(context)
    tactile_active = (
        LaunchConfiguration("tactile_active").perform(context).lower() == "true"
    )

    nodes = []

    # Detect handedness ONCE, reuse for both URDF and RViz config
    hand_type = detect_handedness(hand_name) or "right"
    _logger.info(f"Using handedness: {hand_type}")

    # Robot state publisher with URDF
    wuji_hand_description_dir = get_package_share_directory("wuji_hand_description")
    urdf_file = os.path.join(wuji_hand_description_dir, "urdf", f"{hand_type}-ros.urdf")
    try:
        with open(urdf_file, "r") as f:
            robot_description = f.read()
    except OSError as e:
        _logger.error(f"Failed to read URDF: {e}")
        return []

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=hand_name,
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"publish_frequency": 100.0},
        ],
        output="screen",
    ))

    if tactile_active:
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tactile_tf",
            namespace=hand_name,
            arguments=["0", "0", "0", "0", "0", "0",
                        f"{hand_type}_palm_link", "tactile_sensor_link"],
        ))

    # Visualization
    use_rviz = viz == "rviz" or (viz == "auto" and os.environ.get("DISPLAY"))
    use_foxglove = viz == "foxglove" or (viz == "auto" and not os.environ.get("DISPLAY"))

    if use_rviz:
        # Use tactile-enabled rviz config from wujihand_bringup (includes tactile Image panel)
        bringup_dir = get_package_share_directory("wujihand_bringup")
        rviz_config = os.path.join(bringup_dir, "rviz", f"{hand_type}_tactile.rviz")
        try:
            with open(rviz_config, "r") as f:
                rviz_text = f.read()
            hand_namespace = hand_name.strip("/")
            rviz_text = rviz_text.replace("/hand_0/", f"/{hand_namespace}/")
            with tempfile.NamedTemporaryFile(
                mode="w",
                prefix=f"wujihand_{hand_namespace.replace('/', '_') or 'hand'}_",
                suffix=".rviz",
                delete=False,
            ) as f:
                f.write(rviz_text)
                rviz_config = f.name
        except OSError as e:
            _logger.error(f"Failed to rewrite RViz config: {e}")
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=hand_name,
            arguments=["-d", rviz_config],
            output="screen",
        ))
        _logger.info("Launching RViz")
    elif use_foxglove:
        nodes.append(Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            namespace=hand_name,
            output="screen",
        ))
        _logger.info("Launching Foxglove Bridge (ws://localhost:8765)")

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("hand_name", default_value="hand_0",
                              description="Hand namespace"),
        DeclareLaunchArgument("viz", default_value="auto",
                              description="Visualization: auto/rviz/foxglove/none"),
        DeclareLaunchArgument("tactile", default_value="true",
                              description="Enable tactile sensor"),
        DeclareLaunchArgument("hand_serial", default_value="",
                              description="Hand serial number (empty=auto-discover)"),
        DeclareLaunchArgument("tactile_serial", default_value="",
                              description="Tactile serial number (empty=auto-discover)"),
        DeclareLaunchArgument("publish_rate", default_value="1000.0"),
        DeclareLaunchArgument("filter_cutoff_freq", default_value="10.0"),
        DeclareLaunchArgument("diagnostics_rate", default_value="10.0"),
        DeclareLaunchArgument("image_rate", default_value="30.0",
                              description="Tactile heatmap publish rate (Hz)"),
        DeclareLaunchArgument("pressure_max", default_value="2135",
                              description="Tactile pressure normalization max"),

        # --- Phase 1: Discover devices and spawn driver nodes ---
        OpaqueFunction(function=setup_full_launch),

        # --- Phase 2: After driver connects (~3s), spawn URDF + viz ---
        TimerAction(
            period=3.5,
            actions=[OpaqueFunction(function=setup_viz_and_urdf)],
        ),
    ])
