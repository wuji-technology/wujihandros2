import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Add launch directory to path for local imports
sys.path.insert(0, os.path.dirname(__file__))
from common import spawn_robot_state_publisher, detect_handedness


def spawn_rviz(context):
    """Spawn RViz node with proper namespace and handedness-specific config."""
    hand_name = LaunchConfiguration("hand_name").perform(context)
    wuji_hand_description_dir = get_package_share_directory("wuji_hand_description")

    # Detect handedness from driver node
    hand_type = detect_handedness(hand_name)
    if hand_type is None:
        # Fallback to left.rviz if detection fails
        hand_type = "left"

    rviz_config = os.path.join(wuji_hand_description_dir, "rviz", f"{hand_type}.rviz")

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=hand_name,
            arguments=["-d", rviz_config],
            output="screen",
        )
    ]


def generate_launch_description():
    hand_name_arg = DeclareLaunchArgument(
        "hand_name",
        default_value="hand_0",
        description="Hand name used as namespace and TF prefix",
    )

    serial_number_arg = DeclareLaunchArgument(
        "serial_number",
        default_value="",
        description="Serial number of the WujiHand device",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="1000.0", description="State publish rate in Hz"
    )

    filter_cutoff_freq_arg = DeclareLaunchArgument(
        "filter_cutoff_freq",
        default_value="10.0",
        description="Low-pass filter cutoff frequency in Hz",
    )

    diagnostics_rate_arg = DeclareLaunchArgument(
        "diagnostics_rate",
        default_value="10.0",
        description="Diagnostics publish rate in Hz",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Whether to launch RViz for visualization",
    )

    foxglove_arg = DeclareLaunchArgument(
        "foxglove",
        default_value="false",
        description="Whether to launch Foxglove Bridge for web visualization",
    )

    # Force serial_number to string type (workaround for ROS2 Kilted type inference)
    serial_number_str = ParameterValue(
        LaunchConfiguration("serial_number"), value_type=str
    )

    wujihand_driver_node = Node(
        package="wujihand_driver",
        executable="wujihand_driver_node",
        name="wujihand_driver",
        namespace=LaunchConfiguration("hand_name"),
        parameters=[
            {
                "serial_number": serial_number_str,
                "publish_rate": LaunchConfiguration("publish_rate"),
                "filter_cutoff_freq": LaunchConfiguration("filter_cutoff_freq"),
                "diagnostics_rate": LaunchConfiguration("diagnostics_rate"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    # Auto-detect handedness and spawn robot_state_publisher after driver starts
    auto_detect_action = TimerAction(
        period=2.0,  # Wait 2 seconds for driver to fully initialize
        actions=[OpaqueFunction(function=spawn_robot_state_publisher)],
    )

    # Conditionally spawn RViz after handedness detection completes
    # Wait 2.5s (after robot_state_publisher spawns at 2.0s)
    rviz_action = TimerAction(
        period=2.5,
        actions=[OpaqueFunction(function=spawn_rviz)],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Conditionally spawn Foxglove Bridge for web visualization
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        namespace=LaunchConfiguration("hand_name"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    return LaunchDescription(
        [
            hand_name_arg,
            serial_number_arg,
            publish_rate_arg,
            filter_cutoff_freq_arg,
            diagnostics_rate_arg,
            rviz_arg,
            foxglove_arg,
            wujihand_driver_node,
            auto_detect_action,
            rviz_action,
            foxglove_bridge_node,
        ]
    )
