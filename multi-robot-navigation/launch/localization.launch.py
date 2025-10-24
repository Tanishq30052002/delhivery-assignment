import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('multi-robot-navigation'),
        'rviz',
        'localization.rviz'
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


    return LaunchDescription(
        [rviz_node] + [localization_navigation_launch]
    )
