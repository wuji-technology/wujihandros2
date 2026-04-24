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
            "pressure_max",
            default_value="2135",
            description="Pressure normalization max value (ADC open-circuit)",
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
                "pressure_max": ParameterValue(
                    LaunchConfiguration("pressure_max"), value_type=int),
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
