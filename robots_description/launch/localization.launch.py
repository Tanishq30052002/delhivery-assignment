import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    amcl_config_tug_1 = os.path.join(get_package_share_directory(
        'robots_description'), 'config', 'amcl_config_tug_1.yaml')
    amcl_config_tug_2 = os.path.join(get_package_share_directory(
       'robots_description'), 'config', 'amcl_config_tug_2.yaml')
    amcl_config_picker_2 = os.path.join(get_package_share_directory(
       'robots_description'), 'config', 'amcl_config_picker_2.yaml')
    amcl_config_picker_1 = os.path.join(get_package_share_directory(
        'robots_description'), 'config', 'amcl_config_picker_1.yaml')
    map_file = os.path.join(get_package_share_directory(
        'robots_description'), 'maps', 'warehouse_map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename':map_file}]
        ),

        Node(
            namespace='tug_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_tug_1]
        ),

        Node(
            namespace='tug_2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_tug_2]
        ),

        Node(
            namespace='picker_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_picker_1]
        ),

        Node(
            namespace='picker_2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_picker_2]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'tug_1/amcl', 'tug_2/amcl', 'picker_1/amcl', 'picker_2/amcl']}]
        )
    ])
