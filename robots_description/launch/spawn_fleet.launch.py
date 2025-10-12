from pty import spawn
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robots_description")

    amr1_urdf = Command(["xacro ", PathJoinSubstitution([pkg_share, "urdf", "amr1.xacro"])])
    amr2_urdf = Command(["xacro ", PathJoinSubstitution([pkg_share, "urdf", "amr2.xacro"])])

    spawn_robots = []

    spawn_robots.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string", amr1_urdf, "-name", "amr1_1", "-x", "0", "-y", "0", "-z", "0.1"],
            output="screen",
        )
    )

    spawn_robots.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string", amr1_urdf, "-name", "amr1_2", "-x", "1", "-y", "0", "-z", "0.1"],
            output="screen",
         )
    )

    spawn_robots.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string", amr2_urdf, "-name", "amr2_1", "-x", "2", "-y", "0", "-z", "0.1"],
            output="screen",
        )
    )

    spawn_robots.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-string", amr2_urdf, "-name", "amr2_2", "-x", "3", "-y", "0", "-z", "0.1"],
            output="screen",
        )
    )

    return LaunchDescription(spawn_robots)
