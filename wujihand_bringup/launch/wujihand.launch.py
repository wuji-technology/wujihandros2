from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    # Build joint_prefix as "hand_name/" to match XACRO-generated URDF
    joint_prefix = PythonExpression(["'", LaunchConfiguration("hand_name"), "/'"])

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
                "joint_prefix": joint_prefix,
                "publish_rate": LaunchConfiguration("publish_rate"),
                "filter_cutoff_freq": LaunchConfiguration("filter_cutoff_freq"),
                "diagnostics_rate": LaunchConfiguration("diagnostics_rate"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            hand_name_arg,
            serial_number_arg,
            publish_rate_arg,
            filter_cutoff_freq_arg,
            diagnostics_rate_arg,
            wujihand_driver_node,
        ]
    )
