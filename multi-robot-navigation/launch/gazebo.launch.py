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

    # Combine launches
    return LaunchDescription([
        world_launch,
        spawn_fleet_launch,
    ])
