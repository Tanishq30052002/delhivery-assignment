#!/usr/bin/env python3
"""
Launch file to bring up:
1. The Gazebo small warehouse world
2. The multi-robot fleet spawn
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate a LaunchDescription for the warehouse world and fleet spawn."""

    # Launch Gazebo Small Warehouse World
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"),
                "launch",
                "small_warehouse_launch.py",
            ])
        ])
    )

    # Launch Robot Fleet (multiple robots)
    spawn_fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("multi-robot-navigation"),
                "launch",
                "spawn_robots.launch.py",
            ])
        ])
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('multi-robot-navigation'),
        'rviz',
        'navigation.rviz'
    )

    # --- RViz Node (starts immediately) ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    localization_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("multi-robot-navigation"),
                "launch",
                "localization_navigation.launch.py",
            ])
        ])
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("multi-robot-navigation"),
                "launch",
                "navigation.launch.py",
            ])
        ])
    )


    # Combine launches
    return LaunchDescription([
        world_launch,
        spawn_fleet_launch,
        rviz_node,
        localization_navigation_launch,
        navigation_launch
    ])
