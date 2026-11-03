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
Gazebo side of the pick and place example.

    ros2 launch picking_example sim.launch.py

The Sharework cell of sharework_gazebo (MoveIt and RViz included) with
- urdf/sharework_picking.urdf.xacro: wrist RealSense + DetachableJoints,
- worlds/picking.sdf: objects on the rail and a basket,
- the bridge of the wrist camera,
- sim_grasp_manager: holds the object while the gripper is closed on it,
- RViz with rviz/picking.rviz (MoveIt scene, scene_db objects, VGN grasps,
  wrist camera, YOLO detections).
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


def _rviz(context):
    if LaunchConfiguration('rviz').perform(context).lower() not in ('true', '1'):
        return []
    moveit_share = get_package_share_directory('sharework_cembre_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder('sharework_cell', package_name='sharework_cembre_moveit_config')
        .robot_description(
            file_path=LaunchConfiguration('description_file').perform(context),
            mappings={'sim_cameras': LaunchConfiguration('sim_cameras').perform(context)})
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, 'config', 'sharework_cell.srdf'))
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    return [Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, {'use_sim_time': True}],
    )]


def generate_launch_description():
    share = get_package_share_directory('picking_example')
    sim_time = {'use_sim_time': True}
    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz (rviz_config)'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'picking.rviz')),
        DeclareLaunchArgument('sim_cameras', default_value='true',
                              description='Render the cameras (the wrist one is needed)'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'picking.sdf')),
        DeclareLaunchArgument('description_file', default_value=os.path.join(
            share, 'urdf', 'sharework_picking.urdf.xacro')),
    ]

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sharework_gazebo'), 'launch',
            'sharework_gazebo.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'launch_rviz': 'false',     # RViz of this example below
            'sim_cameras': LaunchConfiguration('sim_cameras'),
            'world': LaunchConfiguration('world'),
            'description_file': LaunchConfiguration('description_file'),
        }.items(),
    )

    wrist_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wrist_camera_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge_wrist.yaml')},
                    sim_time],
        condition=IfCondition(LaunchConfiguration('sim_cameras')),
    )

    grasp_manager = Node(
        package='picking_example',
        executable='sim_grasp_manager',
        output='screen',
        parameters=[{'objects_file': os.path.join(share, 'config', 'objects.yaml')}, sim_time],
    )

    return LaunchDescription(args + [cell, wrist_bridge, grasp_manager,
                                     OpaqueFunction(function=_rviz)])
