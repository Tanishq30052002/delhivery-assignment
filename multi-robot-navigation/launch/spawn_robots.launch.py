#!/usr/bin/env python3
from math import pi
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch multiple robots in Gazebo with namespaced state publishers."""

    ld = LaunchDescription()

    # Define fleet: (model_type, namespace, x, y, z, yaw)
    fleet = [
        ("tug",    "tug_1",    0.0,  0.0,  0.0,  0.0),
        ("tug",    "tug_2",    2.0,  0.0,  0.0,  pi/2),
        ("picker", "picker_1", 2.0,  5.0,  0.0,  pi/2),
        ("picker", "picker_2", 2.0, -5.0,  0.0, -pi/2),
    ]

    # Common path to robot description package
    urdf_dir = os.path.join(get_package_share_directory("multi-robot-navigation"), "urdf")

    # Helper to create robot nodes
    def create_robot_launch_entities(model, ns, x, y, z, yaw):
        """Create robot_state_publisher and spawn_entity nodes for one robot."""

        xacro_path = os.path.join(urdf_dir, f"{model}.core.xacro")

        # Robot State Publisher
        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command(["xacro", " ", xacro_path, " ", "ns:=", ns]),
            }],
            output="screen",
        )

        # Gazebo Spawn Entity
        spawn_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=ns,
            arguments=[
                "-entity", ns,
                "-topic", f"/{ns}/robot_description",
                "-x", str(x),
                "-y", str(y),
                "-z", str(z),
                "-Y", str(yaw),
            ],
            output="screen",
        )

        return [rsp_node, spawn_node]

    # Create and add all robot nodes
    for robot in fleet:
        for node in create_robot_launch_entities(*robot):
            ld.add_action(node)

    return ld
