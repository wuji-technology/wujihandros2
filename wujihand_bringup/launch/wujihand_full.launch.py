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

import atexit
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


def _compose_rviz_config(base_rviz_path, overlay_text_path,
                         hand_namespace, include_tactile):
    """Return base RViz config or a temp config with the tactile overlay."""
    if not include_tactile:
        return base_rviz_path

    with open(base_rviz_path, "r") as f:
        rviz_text = f.read()
    with open(overlay_text_path, "r") as f:
        overlay = f.read()

    topic = f"/{hand_namespace}/tactile/image" if hand_namespace else "/tactile/image"
    overlay = overlay.replace("{IMAGE_TOPIC}", topic)

    # Inject before the top-level Enabled anchor that follows Displays.
    marker = "\n  Enabled: true\n"
    idx = rviz_text.find(marker)
    if idx == -1:
        _logger.warning(
            "tactile overlay anchor not found in base rviz; "
            "skipping tactile panel injection"
        )
        return base_rviz_path
    rviz_text = rviz_text[:idx + 1] + overlay + rviz_text[idx + 1:]

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"wujihand_{hand_namespace.replace('/', '_') or 'hand'}_",
        suffix=".rviz",
        delete=False,
    ) as tmp:
        tmp_path = tmp.name
        atexit.register(lambda p=tmp_path: os.path.exists(p) and os.unlink(p))
        tmp.write(rviz_text)
    return tmp_path


def _bringup_launch_dir():
    return os.path.join(
        get_package_share_directory("wujihand_bringup"), "launch"
    )


def setup_drivers(context):
    """OpaqueFunction: discover devices, launch the joint driver.

    The tactile driver is included later, in setup_viz_and_urdf, because
    its `parent_frame` arg needs the handedness (left/right), which is
    only available after the joint driver is up and detect_handedness
    succeeds.
    """
    hand_name = LaunchConfiguration("hand_name").perform(context)
    tactile_enabled = (
        LaunchConfiguration("tactile").perform(context).lower() == "true"
    )

    # Manual serial overrides win; otherwise scan USB.
    hand_override = LaunchConfiguration("hand_serial").perform(context)
    tactile_override = LaunchConfiguration("tactile_serial").perform(context)
    hands, tactiles = discover_usb_devices()

    # Fail fast on typoed serial overrides; otherwise the deferred URDF +
    # tactile + RViz bringup waits 15 s on a driver that cannot start.
    if hand_override and hand_override not in hands:
        _logger.error(
            f"hand_serial:={hand_override} does not match any WujiHand on USB "
            f"(discovered: {hands or 'none'}). Check the SN or unset the "
            f"override to auto-discover."
        )
        return [
            SetLaunchConfiguration("hand_active", "false"),
            SetLaunchConfiguration("tactile_active", "false"),
            SetLaunchConfiguration("tactile_serial_resolved", ""),
        ]
    if tactile_override and tactile_override not in tactiles:
        _logger.warning(
            f"tactile_serial:={tactile_override} does not match any tactile "
            f"board on USB (discovered: {tactiles or 'none'}). Tactile will "
            f"be skipped."
        )
        # Soft-skip tactile, don't kill the whole launch — joint driver may
        # still be the operator's primary target.
        tactile_override = ""

    hand_serials = [hand_override] if hand_override else hands
    tactile_serials = [tactile_override] if tactile_override else tactiles

    if not hand_serials:
        _logger.error("No WujiHand device found! Check USB connection.")
        # Populate all deferred launch configs before skipping later setup.
        return [
            SetLaunchConfiguration("hand_active", "false"),
            SetLaunchConfiguration("tactile_active", "false"),
            SetLaunchConfiguration("tactile_serial_resolved", ""),
        ]

    # Multi-device topology guard. /sys USB enumeration order is the
    # filesystem traversal order; it is NOT stable across reboots, USB
    # re-enumeration, or hub reseating. Picking [0] without warning silently
    # binds this launch to whichever board the kernel happened to bring up
    # first. For a deterministic launch, the operator must pin
    # `hand_serial:=` / `tactile_serial:=`.
    if not hand_override and len(hands) > 1:
        _logger.warning(
            f"Found {len(hands)} hand boards on USB ({hands}); picking the "
            f"first one nondeterministically. Pass hand_serial:=<SN> to pin."
        )
    if not tactile_override and len(tactiles) > 1:
        _logger.warning(
            f"Found {len(tactiles)} tactile boards on USB ({tactiles}); "
            f"picking the first one nondeterministically. Pass "
            f"tactile_serial:=<SN> to pin."
        )

    _logger.info(
        f"Launching with hand SN={hand_serials[0]}, "
        f"tactile={'SN=' + tactile_serials[0] if tactile_serials else 'none'}"
    )

    # Empty serial means "driver auto-picks"; tactile_active tracks device presence.
    has_tactile = tactile_enabled and bool(tactile_serials)
    tactile_serial = tactile_serials[0] if has_tactile else ""
    tactile_active = "true" if has_tactile else "false"
    if tactile_enabled and not tactile_serials:
        _logger.warning(
            "Tactile enabled but no tactile board found. Skipping."
        )

    return [
        SetLaunchConfiguration("hand_active", "true"),
        SetLaunchConfiguration("tactile_active", tactile_active),
        SetLaunchConfiguration("tactile_serial_resolved", tactile_serial),
        # Joint driver via the standalone wujihand.launch.py. Suppress its
        # own RViz/Foxglove + RSP because wujihand_full owns the composite
        # tactile-aware viz AND the URDF/RSP (we need to pick the URDF
        # variant per handedness, which the standalone launch doesn't
        # know about). Without spawn_robot_state_publisher:=false this
        # launches two RSPs in the same namespace, producing duplicate
        # parameter services + duplicate TF for the same frames.
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
                "spawn_robot_state_publisher": "false",
            }.items(),
        ),
    ]


def setup_viz_and_urdf(context):
    """Spawn URDF + tactile + tactile-aware RViz/Foxglove.

    Runs after the joint driver has had time to come up (TimerAction
    period below). Detects handedness from the joint driver, then
    Includes tactile.launch.py with the correct per-handedness
    parent_frame so the tactile TF is published exactly once and
    anchors under the joint URDF tree.
    """
    hand_name = LaunchConfiguration("hand_name").perform(context)
    viz = LaunchConfiguration("viz").perform(context)
    hand_active = (
        LaunchConfiguration("hand_active").perform(context).lower() == "true"
    )
    tactile_active = (
        LaunchConfiguration("tactile_active").perform(context).lower() == "true"
    )
    tactile_serial = LaunchConfiguration("tactile_serial_resolved").perform(context)

    # Skip deferred bringup when setup_drivers found no hand.
    if not hand_active:
        _logger.error(
            "Skipping URDF + tactile + viz spawn because no hand board was "
            "discovered. Plug in the WujiHand or pass hand_serial:=<SN> and "
            "re-launch."
        )
        return []

    actions = []

    # Detect handedness once; reuse for URDF, tactile parent_frame, and
    # RViz config selection.
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

    actions.append(Node(
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

    # Parent tactile under the handedness-specific palm link, namespace its
    # leaf frame for multi-launch use, and strip slashes for a valid TF frame.
    sanitized_hand = hand_name.strip("/")
    tactile_frame_id = (
        f"{sanitized_hand}_tactile_sensor_link" if sanitized_hand
        else "tactile_sensor_link"
    )
    if tactile_active:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_bringup_launch_dir(), "tactile.launch.py")
            ),
            launch_arguments={
                "namespace": hand_name,
                "serial_number": tactile_serial,
                "image_rate":
                    LaunchConfiguration("image_rate").perform(context),
                "sample_rate_hz":
                    LaunchConfiguration("sample_rate_hz").perform(context),
                "streaming_at_startup":
                    LaunchConfiguration("streaming_at_startup").perform(context),
                "parent_frame": f"{hand_type}_palm_link",
                "frame_id": tactile_frame_id,
            }.items(),
        ))

    # Visualization selection.
    use_rviz = viz == "rviz" or (viz == "auto" and os.environ.get("DISPLAY"))
    use_foxglove = (
        viz == "foxglove" or (viz == "auto" and not os.environ.get("DISPLAY"))
    )

    if use_rviz:
        # Compose: joint-only base from wuji_hand_description + optional
        # tactile Image-display overlay from wujihand_bringup, written to
        # a temp file. The temp file is unlinked at process exit via
        # atexit (registered inside _compose_rviz_config).
        bringup_dir = get_package_share_directory("wujihand_bringup")
        base_rviz = os.path.join(
            wuji_hand_description_dir, "rviz", f"{hand_type}.rviz"
        )
        overlay_text = os.path.join(
            bringup_dir, "rviz", "tactile_overlay.txt"
        )
        try:
            rviz_config = _compose_rviz_config(
                base_rviz_path=base_rviz,
                overlay_text_path=overlay_text,
                hand_namespace=hand_name.strip("/"),
                include_tactile=tactile_active,
            )
        except OSError as e:
            _logger.error(f"Failed to compose RViz config: {e}")
            rviz_config = base_rviz  # fall back to base joint-only config
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=hand_name,
            arguments=["-d", rviz_config],
            output="screen",
        ))
        _logger.info("Launching RViz")
    elif use_foxglove:
        actions.append(Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            namespace=hand_name,
            output="screen",
        ))
        _logger.info("Launching Foxglove Bridge (ws://localhost:8765)")

    return actions


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
        # Internal-use launch configurations: set by setup_drivers and read
        # by setup_viz_and_urdf. Declared with empty/false defaults so a
        # bug or early-exit in setup_drivers can't leave them unset and
        # produce an opaque SubstitutionFailure 3.5 s later when the
        # deferred function tries to .perform() them.
        DeclareLaunchArgument("hand_active", default_value="false",
                              description="(internal) populated by setup_drivers"),
        DeclareLaunchArgument("tactile_active", default_value="false",
                              description="(internal) populated by setup_drivers"),
        DeclareLaunchArgument("tactile_serial_resolved", default_value="",
                              description="(internal) populated by setup_drivers"),
        DeclareLaunchArgument("publish_rate", default_value="1000.0"),
        DeclareLaunchArgument("filter_cutoff_freq", default_value="10.0"),
        DeclareLaunchArgument("diagnostics_rate", default_value="10.0"),
        DeclareLaunchArgument("image_rate", default_value="30.0",
                              description="Tactile heatmap publish rate (Hz)"),
        DeclareLaunchArgument("sample_rate_hz", default_value="120",
                              description="Tactile data-frame rate (1..120, applied at startup)"),
        DeclareLaunchArgument("streaming_at_startup", default_value="true",
                              description="Whether to enable tactile streaming when the driver starts"),

        # Discover USB devices and include the joint driver launch.
        OpaqueFunction(function=setup_drivers),

        # ~3.5 s later (after the joint driver is up so detect_handedness
        # can succeed): spawn URDF, include the tactile driver launch
        # with the correct per-handedness parent_frame, and bring up
        # RViz/Foxglove if requested.
        TimerAction(
            period=3.5,
            actions=[OpaqueFunction(function=setup_viz_and_urdf)],
        ),
    ])
