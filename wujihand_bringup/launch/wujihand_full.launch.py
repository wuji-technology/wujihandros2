"""One-click launch: joint driver + tactile driver + visualization.

Composes the standalone wujihand.launch.py and tactile.launch.py via
IncludeLaunchDescription. Adds USB auto-discovery, handedness-aware
URDF + tactile-aware RViz on top.

Usage:
    # Auto-discover all hardware, launch everything (RViz if DISPLAY set,
    # otherwise Foxglove):
    ros2 launch wujihand_bringup wujihand_full.launch.py

    # Force RViz / Foxglove / no viz:
    ros2 launch wujihand_bringup wujihand_full.launch.py viz:=rviz
    ros2 launch wujihand_bringup wujihand_full.launch.py viz:=foxglove
    ros2 launch wujihand_bringup wujihand_full.launch.py viz:=none

    # Disable tactile (joint driver only):
    ros2 launch wujihand_bringup wujihand_full.launch.py tactile:=false

    # Manually pin serial numbers:
    ros2 launch wujihand_bringup wujihand_full.launch.py \\
        hand_serial:=347838683433 tactile_serial:=12345678
"""

import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import detect_handedness, discover_usb_devices  # noqa: E402

_logger = logging.get_logger("wujihand_full")


def _bringup_launch_dir():
    return os.path.join(
        get_package_share_directory("wujihand_bringup"), "launch"
    )


def setup_drivers(context):
    """OpaqueFunction: discover devices and Include the per-driver launches."""
    hand_name = LaunchConfiguration("hand_name").perform(context)
    tactile_enabled = (
        LaunchConfiguration("tactile").perform(context).lower() == "true"
    )

    # Manual serial overrides win; otherwise scan USB.
    hand_override = LaunchConfiguration("hand_serial").perform(context)
    tactile_override = LaunchConfiguration("tactile_serial").perform(context)
    hands, tactiles = discover_usb_devices()
    hand_serials = [hand_override] if hand_override else hands
    tactile_serials = [tactile_override] if tactile_override else tactiles

    if not hand_serials:
        _logger.error("No WujiHand device found! Check USB connection.")
        return []

    _logger.info(
        f"Launching with hand SN={hand_serials[0]}, "
        f"tactile={'SN=' + tactile_serials[0] if tactile_serials else 'none'}"
    )

    actions = [
        # Default tactile_active to false; flipped to true below if we
        # actually launch a tactile driver. setup_viz_and_urdf reads it
        # to decide whether to include the tactile RViz panel.
        SetLaunchConfiguration("tactile_active", "false"),
        # Joint driver via the standalone wujihand.launch.py. Suppress
        # its own RViz/Foxglove because wujihand_full owns the composite
        # tactile-aware viz at the end.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_bringup_launch_dir(), "wujihand.launch.py")
            ),
            launch_arguments={
                "hand_name": hand_name,
                "serial_number": hand_serials[0],
                "publish_rate":
                    LaunchConfiguration("publish_rate").perform(context),
                "filter_cutoff_freq":
                    LaunchConfiguration("filter_cutoff_freq").perform(context),
                "diagnostics_rate":
                    LaunchConfiguration("diagnostics_rate").perform(context),
                "rviz": "false",
                "foxglove": "false",
            }.items(),
        ),
    ]

    if tactile_enabled and tactile_serials:
        actions.append(SetLaunchConfiguration("tactile_active", "true"))
        # Tactile driver via the standalone tactile.launch.py. parent_frame
        # gets bound to <handedness>_palm_link inside setup_viz_and_urdf —
        # at this point we don't know handedness yet, so use a placeholder
        # palm_link here and let the static_transform_publisher in
        # setup_viz_and_urdf override the TF (see tactile_active branch).
        #
        # NOTE: tactile.launch.py's own static_transform_publisher will
        # publish identity TF to "palm_link" which won't exist in the
        # joint URDF tree. setup_viz_and_urdf re-publishes the same TF
        # under the correct parent <handedness>_palm_link. The duplicate
        # publisher to a non-existent frame is harmless (TF tools just
        # warn).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_bringup_launch_dir(), "tactile.launch.py")
            ),
            launch_arguments={
                "namespace": hand_name,
                "serial_number": tactile_serials[0],
                "image_rate":
                    LaunchConfiguration("image_rate").perform(context),
                "sample_rate_hz":
                    LaunchConfiguration("sample_rate_hz").perform(context),
                "streaming_at_startup":
                    LaunchConfiguration("streaming_at_startup").perform(context),
                # parent_frame intentionally LEFT at default (palm_link);
                # setup_viz_and_urdf publishes the real per-handedness TF.
            }.items(),
        ))
    elif tactile_enabled:
        _logger.warning(
            "Tactile enabled but no tactile board found. Skipping."
        )

    return actions


def setup_viz_and_urdf(context):
    """Spawn URDF + tactile-aware RViz/Foxglove after drivers are up."""
    hand_name = LaunchConfiguration("hand_name").perform(context)
    viz = LaunchConfiguration("viz").perform(context)
    tactile_active = (
        LaunchConfiguration("tactile_active").perform(context).lower() == "true"
    )

    nodes = []

    # Detect handedness once; reuse for URDF + RViz config selection.
    hand_type = detect_handedness(hand_name) or "right"
    _logger.info(f"Using handedness: {hand_type}")

    wuji_hand_description_dir = get_package_share_directory("wuji_hand_description")
    urdf_file = os.path.join(
        wuji_hand_description_dir, "urdf", f"{hand_type}-ros.urdf"
    )
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

    # Re-publish the tactile static TF with the correct per-handedness
    # parent. tactile.launch.py also publishes one, parented to
    # "palm_link" — that one anchors to a non-existent frame, so this one
    # is what TF resolves against at runtime.
    if tactile_active:
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tactile_tf_anchor",
            namespace=hand_name,
            arguments=["0", "0", "0", "0", "0", "0",
                       f"{hand_type}_palm_link", "tactile_sensor_link"],
        ))

    # Visualization selection.
    use_rviz = viz == "rviz" or (viz == "auto" and os.environ.get("DISPLAY"))
    use_foxglove = (
        viz == "foxglove" or (viz == "auto" and not os.environ.get("DISPLAY"))
    )

    if use_rviz:
        # Tactile-aware RViz config from wujihand_bringup. Phase 8 will
        # collapse these into a per-handedness base + tactile overlay
        # composed at launch time.
        bringup_dir = get_package_share_directory("wujihand_bringup")
        rviz_config = os.path.join(
            bringup_dir, "rviz", f"{hand_type}_tactile.rviz"
        )
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
        DeclareLaunchArgument("sample_rate_hz", default_value="120",
                              description="Tactile data-frame rate (1..120, applied at startup)"),
        DeclareLaunchArgument("streaming_at_startup", default_value="true",
                              description="Whether to enable tactile streaming when the driver starts"),

        # --- Phase 1: Discover devices and include per-driver launches ---
        OpaqueFunction(function=setup_drivers),

        # --- Phase 2: After drivers connect (~3.5 s), spawn URDF + viz ---
        TimerAction(
            period=3.5,
            actions=[OpaqueFunction(function=setup_viz_and_urdf)],
        ),
    ])
