"""Launch file for the tactile sensor driver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            "serial_number",
            default_value="",
            description="Tactile board USB serial number (empty = auto-discover)",
        ),
        DeclareLaunchArgument(
            "image_rate",
            default_value="30.0",
            description="Heatmap image publish rate (Hz)",
        ),
        DeclareLaunchArgument(
            "sample_rate_hz",
            default_value="120",
            description="Tactile data-frame rate (1..120, applied at startup)",
        ),
        DeclareLaunchArgument(
            "streaming_at_startup",
            default_value="true",
            description="Whether to enable tactile streaming when the driver starts",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="tactile_sensor_link",
            description="TF frame ID for tactile data",
        ),

        # --- Tactile driver node ---
        Node(
            package="wujihand_driver",
            executable="tactile_driver_node",
            name="tactile_driver_node",
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

        # --- Static TF: palm_link → tactile_sensor_link ---
        # Default identity transform. Update with actual calibration offsets
        # once the sensor is mounted on the hand.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tactile_tf",
            arguments=["0", "0", "0", "0", "0", "0",
                        "palm_link", LaunchConfiguration("frame_id")],
        ),
    ])
