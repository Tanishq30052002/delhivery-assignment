# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robots_nav = {
        'tug_1': 'nav2_params_tug_1.yaml',
        'tug_2': 'nav2_params_tug_2.yaml',
        'picker_1': 'nav2_params_picker_1.yaml',
        'picker_2': 'nav2_params_picker_2.yaml',
    }

    lifecycle_nodes = ['controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                        'waypoint_follower'
                        ]


    default_bt_xml_filename=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': 'true',
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': 'true',
        'map_subscribe_transient_local': 'false'
    }

    nav_nodes = []
    lifecycle_manager_nodes = []
    for ns, config_file in robots_nav.items():
        config_file_path = os.path.join(
            get_package_share_directory('multi-robot-navigation'),
            'config',
            config_file
        )
        configured_params = RewrittenYaml(
            source_file=config_file_path,
            root_key=ns,
            param_rewrites=param_substitutions,
            convert_types=True)
        nav_nodes.append(
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                namespace=ns,
                parameters=[configured_params]
            )
        )
        nav_nodes.append(
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                namespace=ns,
                parameters=[configured_params]
            )
        )
        nav_nodes.append(
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                namespace=ns,
                parameters=[configured_params]
            )
        )
        nav_nodes.append(
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                namespace=ns,
                parameters=[configured_params]
            )
        )
        nav_nodes.append(
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                namespace=ns,
                parameters=[configured_params]
            )
        )

        lifecycle_manager_nodes.append(Node(
                                    package='nav2_lifecycle_manager',
                                    executable='lifecycle_manager',
                                    name='lifecycle_manager_navigation',
                                    output='screen',
                                    namespace=ns,
                                    parameters=[{'use_sim_time': True},
                                                {'autostart': True},
                                                {'node_names': lifecycle_nodes}]
                                ))

    return LaunchDescription(
        nav_nodes + lifecycle_manager_nodes
    )
