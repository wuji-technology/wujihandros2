from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parse_hand_names(raw):
    """Split a comma-separated hand_names string into a clean, non-empty list."""
    return [name.strip() for name in raw.split(",") if name.strip()]


def spawn_home_nodes(context):
    """Spawn one home.py node per hand name in hand_names."""
    raw = LaunchConfiguration("hand_names").perform(context)
    duration = float(LaunchConfiguration("duration").perform(context))
    rate = float(LaunchConfiguration("rate").perform(context))

    nodes = []
    for name in parse_hand_names(raw):
        nodes.append(
            Node(
                package="wujihand_bringup",
                executable="home.py",
                name=f"home_{name}",
                parameters=[{"hand_name": name, "duration": duration, "rate": rate}],
                output="screen",
                emulate_tty=True,
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_names",
                default_value="hand_0",
                description="Comma-separated hand namespaces, e.g. left_hand,right_hand",
            ),
            DeclareLaunchArgument(
                "duration", default_value="2.0", description="Homing duration in seconds"
            ),
            DeclareLaunchArgument(
                "rate", default_value="100.0", description="Command publish rate in Hz"
            ),
            OpaqueFunction(function=spawn_home_nodes),
        ]
    )
