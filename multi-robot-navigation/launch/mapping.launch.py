#!/usr/bin/env python3
"""
Launch file to start:
1. The asynchronous SLAM Toolbox node
2. RViz2 with a preconfigured layout
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate a LaunchDescription with SLAM Toolbox and RViz nodes."""

    # --- Paths to configuration files ---
    slam_config_file = os.path.join(
        get_package_share_directory("multi-robot-navigation"),
        "config",
        "slam_toolbox_async_mapping.yaml"
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("multi-robot-navigation"),
        "rviz",
        "slam_toolbox.rviz"  # your RViz config file
    )

    # --- SLAM Toolbox Node ---
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config_file,
            {"use_sim_time": True}
        ]
    )

    # --- RViz Node ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    # --- LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld
