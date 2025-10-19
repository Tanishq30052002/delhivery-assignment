#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()


    # Fleet: (robot, namespace, x, y, z, yaw)
    robots = [
        ("tug", "tug_1", 0.0, 0.0, 0.0, 0.0),
        ("tug", "tug_2", 2.0, 0.0, 0.0, 1.57),
        ("picker", "pick_1", 2.0, 5.0, 0.0, 1.57),
        ("picker", "pick_2", 2.0, -5.0, 0.0, -1.57),
    ]

    for robot, entity_name, x, y, z, yaw in robots:
        xacro_file = f"{robot}.core.xacro"
        robot_desc_path = os.path.join(get_package_share_directory("robots_description"), "urdf", xacro_file)

        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=entity_name,
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name]),

            }],
            output="screen",
        )

        spawn_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=entity_name,
            arguments=[
                "-entity", entity_name,
                "-topic", f"/{entity_name}/robot_description",
                "-x", str(x),
                "-y", str(y),
                "-z", str(z),
                "-Y", str(yaw)
            ],
            output="screen"
        )

        ld.add_action(rsp_node)
        ld.add_action(spawn_node)

    return ld
