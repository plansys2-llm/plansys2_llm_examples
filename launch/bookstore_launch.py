# Copyright 2026 Álvaro Valencia
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = get_package_share_directory('plan_bookstore')
    namespace = LaunchConfiguration('namespace')
    displaced_book = LaunchConfiguration('displaced_book')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_displaced_book_cmd = DeclareLaunchArgument(
        'displaced_book',
        default_value='blue_book',
        description='Which book is displaced from its shelf (simulates missing book)')

    declare_fake_navigation_cmd = DeclareLaunchArgument(
        'fake_navigation',
        default_value='true',
        description='Skip EasyNav, fake the move action with a tick counter')

    declare_fake_check_cmd = DeclareLaunchArgument(
        'fake_check',
        default_value='true',
        description='Skip /perception_events, fake CheckBookPresent against displaced_book')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace,
          'params_file': example_dir + '/params/planner_param.yaml',
          }.items())

    # PlanSys2-only testing — no Gazebo, no EasyNav.
    # Move runs in fake mode (just counts ticks and returns SUCCESS).

    perception_sim_cmd = Node(
        package='plan_bookstore',
        executable='perception_sim_node',
        name='perception_sim',
        namespace=namespace,
        output='screen',
        parameters=[{
            'detected_object': displaced_book,
            'detected_location': 'middle_path',
            'observed_x': -1.5,
            'observed_y': -3.0,
        }])

    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/bt_xml/move.xml',
            'fake_navigation': LaunchConfiguration('fake_navigation'),
          }
        ])

    pick_book_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_book',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick_book',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/bt_xml/pick_book.xml',
            'displaced_book': displaced_book,
            'fake_check': LaunchConfiguration('fake_check'),
          }
        ])

    place_book_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='place_book',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'place_book',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/bt_xml/place_book.xml',
          }
        ])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_displaced_book_cmd)
    ld.add_action(declare_fake_navigation_cmd)
    ld.add_action(declare_fake_check_cmd)
    ld.add_action(perception_sim_cmd)
    ld.add_action(move_cmd)
    ld.add_action(pick_book_cmd)
    ld.add_action(place_book_cmd)
    ld.add_action(plansys2_cmd)

    return ld
