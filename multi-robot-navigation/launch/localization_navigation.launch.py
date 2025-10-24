import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Map file
    map_file = os.path.join(
        get_package_share_directory('multi-robot-navigation'),
        'maps',
        'warehouse_map.yaml'
    )

    # Robot namespaces and their respective AMCL config files
    robots_amcl = {
        'tug_1': 'amcl_config_tug_1.yaml',
        'tug_2': 'amcl_config_tug_2.yaml',
        'picker_1': 'amcl_config_picker_1.yaml',
        'picker_2': 'amcl_config_picker_2.yaml',
    }



    # --- Map Server Node (delayed) ---
    map_server_node = TimerAction(
        period=2.0,  # delay launching map_server by 2 seconds
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'yaml_filename': map_file}
                ]
            )
        ]
    )

    # --- AMCL Nodes (delayed) ---
    amcl_nodes = []
    for ns, config_file in robots_amcl.items():
        amcl_config_path = os.path.join(
            get_package_share_directory('multi-robot-navigation'),
            'config',
            config_file
        )
        amcl_nodes.append(
                    Node(
                        namespace=ns,
                        package='nav2_amcl',
                        executable='amcl',
                        name='amcl',
                        output='screen',
                        parameters=[amcl_config_path]
                    )
        )

    # --- Lifecycle Manager Node (delayed) ---
    lifecycle_manager_node= Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'bond_timeout': 0.0},
            {'node_names': ['map_server'] + [f"{ns}/amcl" for ns in robots_amcl.keys()]}
        ]
    )


    return LaunchDescription(
        [map_server_node] + amcl_nodes + [lifecycle_manager_node]
    )
