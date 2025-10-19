import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    params_tug_1 = os.path.join(get_package_share_directory("robots_description"),
                                       'config', 'slam_toolbox_async_mapping.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[
          params_tug_1,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
        )

    ld = LaunchDescription()

    ld.add_action(start_async_slam_toolbox_node)

    return ld
