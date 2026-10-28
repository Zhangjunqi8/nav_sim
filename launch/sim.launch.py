# nav_sim entrypoint.
#
# Launch order:
#   1. Gazebo with the chosen 2D world (dynamic obstacles come along)
#   2. robot_state_publisher (expands the AGV xacro into robot_description)
#   3. Spawn the AGV from robot_description
#   4. (optional) ground-truth publisher for evaluation
#
# This launch file does NOT start any navigation stack. Consume /scan, /odom,
# /tf, /camera/* from whatever stack you want.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    nav_sim_share = FindPackageShare('nav_sim')
    agv_xacro = PathJoinSubstitution([nav_sim_share, 'urdf', 'agv', 'agv.urdf.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='square_dynamic.world',
                              description='World file under worlds/'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_ground_truth', default_value='false',
                              description='Start ground_truth_publisher node'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open rviz with nav_sim config'),

        # ---- 1. Gazebo with the world (dynamic obstacles spawn with it) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                ])
            ),
            launch_arguments={
                'world': PathJoinSubstitution([nav_sim_share, 'worlds', LaunchConfiguration('world')]),
                'verbose': 'true',
            }.items()
        ),

        # ---- 2. Expand xacro -> robot_description, publish /tf (static) ----
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', agv_xacro]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # ---- 3. Spawn the AGV from robot_description ----
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'agv',
                '-x', '0.0', '-y', '0.0', '-z', '0.05',
            ],
            output='screen'
        ),

        # ---- 4. Optional ground truth ----
        Node(
            package='nav_sim',
            executable='ground_truth_publisher',
            name='ground_truth_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_ground_truth'))
        ),
    ])