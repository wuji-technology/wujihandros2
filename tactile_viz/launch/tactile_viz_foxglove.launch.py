"""
Launch file for tactile visualization with Foxglove

Starts:
- tactile_viz_node: Reads tactile data and publishes MarkerArray
- foxglove_bridge: WebSocket bridge for Foxglove/Lichtblick (ws://localhost:8765)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tactile_viz')

    # Paths to config files
    config_file = os.path.join(pkg_share, 'config', 'point_positions.yaml')

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Foxglove bridge WebSocket port'
    )

    # Tactile visualization node
    tactile_viz_node = Node(
        package='tactile_viz',
        executable='tactile_viz_node',
        name='tactile_viz_node',
        output='screen',
        parameters=[config_file],
    )

    # Foxglove Bridge node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'address': '0.0.0.0',
                'send_buffer_limit': 50000000,
                'use_sim_time': False,
                'max_qos_depth': 10,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        tactile_viz_node,
        foxglove_bridge_node,
    ])
