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

"""Full integration launch: Kobuki in AWS bookstore with EasyNav + PlanSys2.

Launches:
  1. Gazebo with AWS bookstore world + Kobuki robot (lidar + camera)
  2. ROS-Gazebo bridges (clock, odom, tf, cmd_vel, scan, camera, joints)
  3. EasyNavigation system (controller, planner, localizer, maps, sensors)
  4. PlanSys2 (domain expert, problem expert, planner, executor)
  5. BT action nodes (move, pick_book, place_book)
  6. Perception simulator
"""

import os
from os.path import join

import yaml

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


# (model_dir, x, y, z, roll, pitch, yaw)
BOOK_SPECS = {
    'red_book':    ('colored_book_red',    -0.7720, -2.2646, -0.0113, 0.1718, 0.0183, -0.0869),
    'green_book':  ('colored_book_green',   1.0574, -5.1190, 0.2000, 0.2768, 0.0,  1.4530),
    'yellow_book': ('colored_book_yellow',  1.0314, -1.7404, 0.1866, 0.3304, 0.0,  1.5137),
    'blue_book':   ('colored_book_blue',   -2.1531, -1.5000, 0.1633, -0.4021, 0.0898, 1.8254),
}
FLOOR_BOOK_Z = 0.10


def _has_nvidia_gpu():
    return (os.path.exists('/proc/driver/nvidia/version')
            or os.path.exists('/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0'))


def spawn_books(context):
    displaced = LaunchConfiguration('displaced_book').perform(context)
    bookstore_dir = get_package_share_directory('plan_bookstore')
    with open(join(bookstore_dir, 'config', 'params.yaml')) as f:
        mx, my, _ = yaml.safe_load(f)['move']['ros__parameters']['waypoint_coords']['middle_path']
    floor_pose = (mx, my, FLOOR_BOOK_Z, 0.0, 0.0, 0.0)

    actions = []
    for book_name, (model_dir, *shelf_pose) in BOOK_SPECS.items():
        x, y, z, roll, pitch, yaw = floor_pose if book_name == displaced else shelf_pose
        sdf = join(bookstore_dir, 'models', model_dir, 'model.sdf')
        actions.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{book_name}',
            arguments=['-name', book_name, '-file', sdf,
                       '-x', str(x), '-y', str(y), '-z', str(z),
                       '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)],
            output='screen',
        ))
    return actions


def generate_launch_description():
    bookstore_dir = get_package_share_directory('plan_bookstore')
    kobuki_desc_dir = get_package_share_directory('kobuki_description')
    aws_bookstore_dir = get_package_share_directory('aws_robomaker_bookstore_world')

    easynav_available = True
    try:
        get_package_share_directory('easynav_system')
    except PackageNotFoundError:
        easynav_available = False

    yolo_available = True
    yolo_bringup_dir = None
    try:
        yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    except PackageNotFoundError:
        yolo_available = False

    namespace = LaunchConfiguration('namespace')
    displaced_book = LaunchConfiguration('displaced_book')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_displaced_book = DeclareLaunchArgument(
        'displaced_book', default_value='blue_book',
        description='Which book is displaced from its shelf')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI')

    declare_perception_mode = DeclareLaunchArgument(
        'perception_mode', default_value='sim',
        description='Perception backend: "sim" (scripted events) or "real" (YOLO+HSV)')

    declare_fake_navigation = DeclareLaunchArgument(
        'fake_navigation',
        default_value='false' if easynav_available else 'true',
        description='Skip EasyNav, fake the move action with a tick counter')

    declare_fake_check = DeclareLaunchArgument(
        'fake_check',
        default_value='false' if yolo_available else 'true',
        description='Skip /perception_events, fake CheckBookPresent against displaced_book')

    # -- Environment -----------------------------------------------------------
    model_path = os.pathsep.join([
        os.path.join(aws_bookstore_dir, 'models'),
        os.path.join(bookstore_dir, 'models'),
    ])
    resource_path = model_path
    if 'GZ_SIM_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    # -- 1. Gazebo -------------------------------------------------------------
    world_file = join(aws_bookstore_dir, 'worlds', 'bookstore.world')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_file]}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # -- 2. Kobuki -------------------------------------------------------------
    # Robot description + bridges + image bridges (everything except the create).
    # We declare the `create` Node ourselves below so we can chain the book
    # spawn to its OnProcessExit (it only exits when the world is ready).
    kobuki_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(kobuki_desc_dir, 'launch', 'kobuki_description.launch.py')),
        launch_arguments={
            'gazebo': 'true',
            'camera': 'true',
            'lidar': 'true',
            'use_sim_time': 'true',
        }.items(),
    )

    spawn_kobuki = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_kobuki',
        output='screen',
        arguments=[
            '-name', 'kobuki',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0',
        ],
    )

    # -- 4. EasyNavigation -----------------------------------------------------
    easynav_config = join(bookstore_dir, 'config', 'easynav_kobuki_bookstore.yaml')
    easynav_system = Node(
        package='easynav_system',
        executable='system_main',
        output='screen',
        parameters=[{'use_sim_time': True}],
        ros_arguments=['--params-file', easynav_config],
    )

    # -- 5. PlanSys2 -----------------------------------------------------------
    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': join(bookstore_dir, 'pddl', 'domain.pddl'),
            'namespace': namespace,
            'params_file': join(bookstore_dir, 'params', 'planner_param.yaml'),
        }.items(),
    )

    # -- 6. Perception ---------------------------------------------------------
    perception_sim = Node(
        package='plan_bookstore',
        executable='perception_sim_node',
        name='perception_sim',
        namespace=namespace,
        output='screen',
        parameters=[{
            'detected_object': displaced_book,
            'detected_location': 'middle_path',
            'observed_x': 0.0,
            'observed_y': -3.0,
            'use_sim_time': True,
        }],
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('perception_mode'), 'sim')),
    )

    perception_yolo = Node(
        package='plan_bookstore',
        executable='perception_yolo_node',
        name='perception_yolo',
        namespace=namespace,
        output='screen',
        parameters=[{
            'displaced_book': displaced_book,
            'displaced_location': 'middle_path',
            'use_sim_time': True,
        }],
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('perception_mode'), 'real')),
    )

    yolo_v8 = None
    if yolo_available:
        yolo_v8 = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(join(
                        yolo_bringup_dir,
                        'launch', 'yolov8.launch.py')),
                    launch_arguments={
                        'model': 'yolov8m.pt',
                        'input_image_topic': '/rgbd_camera/image',
                        'threshold': '0.25',
                        'device': 'cuda:0',
                        'namespace': 'yolo',
                        'use_sim_time': 'true',
                    }.items(),
                ),
            ],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('perception_mode'), 'real')),
            scoped=True,
        )

    # -- 7. BT Action Nodes ---------------------------------------------------
    move_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'move',
                'publisher_port': 1668,
                'server_port': 1669,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'move.xml'),
                'fake_navigation': LaunchConfiguration('fake_navigation'),
                'use_sim_time': True,
            },
        ],
    )

    pick_book_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_book',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'pick_book',
                'publisher_port': 1670,
                'server_port': 1671,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'pick_book.xml'),
                'displaced_book': displaced_book,
                'fake_check': LaunchConfiguration('fake_check'),
                'use_sim_time': True,
            },
        ],
    )

    place_book_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='place_book',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'place_book',
                'publisher_port': 1672,
                'server_port': 1673,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'place_book.xml'),
                'use_sim_time': True,
            },
        ],
    )

    # -- 8. Waypoint TF Visualizer ---------------------------------------------
    waypoint_tf = ExecuteProcess(
        cmd=['python3', join(bookstore_dir, 'scripts', 'visualize_waypoints.py')],
        output='screen',
    )

    # -- Build Launch Description ----------------------------------------------
    ld = LaunchDescription()

    # GPU acceleration only on hosts that actually have NVIDIA. On Intel/AMD-only
    # systems forcing __GLX_VENDOR_LIBRARY_NAME=nvidia breaks libGLX.
    if _has_nvidia_gpu():
        ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
        ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    ld.add_action(declare_namespace)
    ld.add_action(declare_displaced_book)
    ld.add_action(declare_gui)
    ld.add_action(declare_perception_mode)
    ld.add_action(declare_fake_navigation)
    ld.add_action(declare_fake_check)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(kobuki_description_launch)
    ld.add_action(spawn_kobuki)

    if easynav_available:
        ld.add_action(easynav_system)
    ld.add_action(plansys2)
    ld.add_action(perception_sim)
    if yolo_available:
        ld.add_action(perception_yolo)
        ld.add_action(yolo_v8)

    ld.add_action(move_node)
    ld.add_action(pick_book_node)
    ld.add_action(place_book_node)
    ld.add_action(waypoint_tf)

    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=spawn_kobuki,
        on_exit=[OpaqueFunction(function=spawn_books)],
    )))

    return ld
