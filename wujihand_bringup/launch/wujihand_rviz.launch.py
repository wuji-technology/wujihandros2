import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wujihand_bringup_dir = get_package_share_directory("wujihand_bringup")
    wujihand_description_dir = get_package_share_directory("wujihand_description")

    # Default to right hand URDF
    urdf_file = os.path.join(wujihand_description_dir, "urdf", "right-ros.urdf")

    serial_number_arg = DeclareLaunchArgument(
        "serial_number",
        default_value="",
        description="Serial number of the WujiHand device",
    )

    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="false",
        description="Use mock driver instead of real hardware",
    )

    # Read URDF file directly (not xacro)
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
        emulate_tty=True,
    )

    # Real hardware driver
    wujihand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wujihand_bringup_dir, "launch", "wujihand.launch.py")
        ),
        launch_arguments={
            "serial_number": LaunchConfiguration("serial_number"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_mock")),
    )

    # Mock driver
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wujihand_bringup_dir, "launch", "wujihand_mock.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_mock")),
    )

    rviz_config = os.path.join(wujihand_description_dir, "rviz", "robot_display.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            serial_number_arg,
            use_mock_arg,
            robot_state_publisher_node,
            wujihand_launch,
            mock_launch,
            rviz_node,
        ]
    )
