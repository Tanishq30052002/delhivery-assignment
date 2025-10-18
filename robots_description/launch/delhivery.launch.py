from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 1️⃣ Path to your world launch file
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"),
                "launch",
                "small_warehouse_launch.py"
            ])
        ])
    )

    # 2️⃣ Path to your robot spawn launch file
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robots_description"),
                "launch",
                "spawn_fleet.launch.py"
            ])
        ])
    )

    # 3️⃣ Launch RViz with preconfigured config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare("robots_description"),
                "rviz",
                "fleet_config.rviz"  # your RViz config file
            ])
        ],
        output="screen"
    )

    # Return combined LaunchDescription
    return LaunchDescription([
        world_launch,
        spawn_launch,
        rviz_node
    ])
