# Copyright 2026 Manuel Beschi
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

"""
Perception and motion services of the pick and place example.

    source ~/venvs/yolo/bin/activate      # Ultralytics, for the detector
    ros2 launch picking_example services.launch.py

Starts, after sim.launch.py (move_group must be running):
- yolo_rgbd_detector (~/detect);
- scene_db, scene_updater_wrist (wrist camera: objects and grasp views) and,
  with fixed_camera:=zed|rs1|rs2, scene_updater_fixed;
- ik_solver (ur_ik_solver), collision_checker_server, parallel_multigoal_planner;
- time_parametrization (compute_joint_trajectory).
vgn_grasp_server runs in its own environment (see README.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _setup(context):
    share = get_package_share_directory('picking_example')
    gz_share = get_package_share_directory('sharework_gazebo')
    moveit_share = get_package_share_directory('sharework_cembre_moveit_config')
    config = os.path.join(share, 'config')
    sim_time = {'use_sim_time': True}

    # Same description as move_group (sim.launch.py): the collision checker and
    # the time parametrization build their own RobotModel.
    moveit_config = (
        MoveItConfigsBuilder('sharework_cell', package_name='sharework_cembre_moveit_config')
        .robot_description(
            file_path=LaunchConfiguration('description_file').perform(context),
            mappings={'simulation_controllers': os.path.join(gz_share, 'config',
                                                             'gz_controllers.yaml')})
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, 'config', 'sharework_cell.srdf'))
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    model = [moveit_config.robot_description, moveit_config.robot_description_semantic,
             moveit_config.robot_description_kinematics]
    with open(os.path.join(moveit_share, 'config', 'joint_limits.yaml')) as f:
        joint_limits = {'robot_description_planning': yaml.safe_load(f)}

    # --- Motion: ik_solver -> collision_checker_server -> parallel_multigoal_planner
    ik_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ik_solver'), 'launch', 'ik_solver.launch.py')),
        launch_arguments={'config': os.path.join(config, 'ik_solver.yaml')}.items(),
    )
    collision_checker = Node(
        package='collision_checker_server',
        executable='ik_collision_checker',
        name='ik_collision_check',
        output='screen',
        parameters=model + [os.path.join(config, 'collision_checker.yaml')],
    )
    planner = Node(
        package='parallel_multigoal_planner',
        executable='parallel_multigoal_planner_node',
        name='parallel_multigoal_planner',
        output='screen',
        parameters=[os.path.join(config, 'mirrt_star.yaml'),
                    {'ik_solver_ns': 'ur_ik_solver',
                     'collision_check_service': '/collision_check',
                     'service_timeout_s': 20.0}],
    )
    time_parametrization = Node(
        package='time_parametrization',
        executable='apply_time_parametrization_server',
        name='compute_joint_trajectory',
        output='screen',
        parameters=model + [joint_limits, os.path.join(config, 'time_parametrization.yaml')],
    )

    # --- Perception
    detector = Node(
        package='yolo_rgbd_detector',
        executable='detector_node',
        name='yolo_rgbd_detector',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(config, 'detector.yaml'), sim_time],
        condition=IfCondition(LaunchConfiguration('detector')),
    )
    scene_params = os.path.join(config, 'scene.yaml')
    scene_db = Node(
        package='scene_db',
        executable='scene_db_node',
        name='scene_db',
        output='screen',
        parameters=[scene_params, sim_time,
                    {'database_file': LaunchConfiguration('database_file'),
                     'reset_on_start': True}],
    )
    wrist_updater = Node(
        package='scene_db',
        executable='scene_updater',
        name='scene_updater_wrist',
        output='screen',
        parameters=[scene_params, sim_time],
        remappings=[('color/image_raw', '/wrist/color/image_raw'),
                    ('depth/image_raw', '/wrist/aligned_depth_to_color/image_raw'),
                    ('color/camera_info', '/wrist/color/camera_info')],
    )
    nodes = [detector, scene_db, wrist_updater, time_parametrization, ik_solver, collision_checker, planner]

    fixed = LaunchConfiguration('fixed_camera').perform(context)
    if fixed:
        # Gazebo renders color and depth with the same intrinsics: aligned.
        nodes.append(Node(
            package='scene_db',
            executable='scene_updater',
            name='scene_updater_fixed',
            output='screen',
            parameters=[scene_params, sim_time],
            remappings=[('color/image_raw', f'/{fixed}/color/image_raw'),
                        ('depth/image_raw', f'/{fixed}/depth/image_rect_raw'),
                        ('color/camera_info', f'/{fixed}/color/camera_info')],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('picking_example')
    return LaunchDescription([
        DeclareLaunchArgument('description_file', default_value=os.path.join(
            share, 'urdf', 'sharework_picking.urdf.xacro')),
        DeclareLaunchArgument('detector', default_value='true',
                              description='Start yolo_rgbd_detector (needs Ultralytics)'),
        DeclareLaunchArgument('fixed_camera', default_value='',
                              description='zed, rs1 or rs2: also an updater on that camera'),
        DeclareLaunchArgument('database_file', default_value='~/.ros/scene_db_picking.sqlite'),
        OpaqueFunction(function=_setup),
    ])
