"""
Launch file for tactile visualization with Foxglove

Starts:
- tactile_viz_node: Reads tactile data and publishes MarkerArray (raw & filtered)
- robot_state_publisher (x2): Publishes URDF for both raw and filtered visualization
- foxglove_bridge: WebSocket bridge for Foxglove/Lichtblick (ws://localhost:8765)

In Foxglove, add two 3D Panels:
- Panel 1 (Filtered):
  - URDF topic: /filtered/robot_description
  - Markers topic: /tactile_markers_filtered
- Panel 2 (Raw):
  - URDF topic: /raw/robot_description
  - Markers topic: /tactile_markers_raw
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_raw_urdf(urdf_content: str) -> str:
    """Create raw version of URDF by prefixing all link/joint names with 'raw_'"""
    raw_urdf = urdf_content

    # Update robot name
    raw_urdf = raw_urdf.replace('name="36_points_robot"', 'name="36_points_robot_raw"')

    # Replace link names with quotes to avoid partial matches
    # e.g., "point_link_1" won't match in "point_link_10"
    for i in range(1, 37):
        raw_urdf = raw_urdf.replace(f'"point_link_{i}"', f'"raw_point_link_{i}"')

    # Replace joint names with quotes
    for i in range(1, 37):
        raw_urdf = raw_urdf.replace(f'"joint_{i}"', f'"raw_joint_{i}"')

    # Replace other link/joint names (these have unique names, no partial match risk)
    raw_urdf = raw_urdf.replace('"base_link"', '"raw_base_link"')
    raw_urdf = raw_urdf.replace('"surface_link"', '"raw_surface_link"')
    raw_urdf = raw_urdf.replace('"surface_joint"', '"raw_surface_joint"')

    return raw_urdf


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tactile_viz')

    # Paths to config files
    config_file = os.path.join(pkg_share, 'config', 'point_positions.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', '36_points_fixed.urdf')

    # Read URDF file content
    with open(urdf_file, 'r') as f:
        robot_description_filtered = f.read()

    # Create raw version with all links/joints prefixed with 'raw_'
    robot_description_raw = create_raw_urdf(robot_description_filtered)

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Foxglove bridge WebSocket port'
    )

    # Robot state publisher for FILTERED visualization
    # Also publishes to /robot_description for default Foxglove compatibility
    robot_state_publisher_filtered = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='filtered',
        output='screen',
        parameters=[{
            'robot_description': robot_description_filtered,
            'publish_frequency': 10.0,
        }],
        remappings=[
            ('robot_description', '/robot_description'),  # Also publish to global topic
        ],
    )

    # Robot state publisher for RAW visualization
    robot_state_publisher_raw = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='raw',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 10.0,
        }],
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
                'capabilities': ['clientPublish', 'services', 'connectionGraph', 'assets'],
                'asset_uri_allowlist': ['package://.*', 'file://.*'],
                'include_hidden': False,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        robot_state_publisher_filtered,
        robot_state_publisher_raw,
        tactile_viz_node,
        foxglove_bridge_node,
    ])
