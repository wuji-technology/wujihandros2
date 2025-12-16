from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    handedness_arg = DeclareLaunchArgument(
        "handedness",
        default_value="right",
        description="Hand type: 'left' or 'right'",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="1000.0", description="State publish rate in Hz"
    )

    diagnostics_rate_arg = DeclareLaunchArgument(
        "diagnostics_rate",
        default_value="10.0",
        description="Diagnostics publish rate in Hz",
    )

    mock_driver_node = Node(
        package="wujihand_bringup",
        executable="mock_driver.py",
        name="wujihand_driver",
        parameters=[
            {
                "handedness": LaunchConfiguration("handedness"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "diagnostics_rate": LaunchConfiguration("diagnostics_rate"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            handedness_arg,
            publish_rate_arg,
            diagnostics_rate_arg,
            mock_driver_node,
        ]
    )
