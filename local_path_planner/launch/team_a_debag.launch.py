#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team_a_obstacle_detector',
            executable='team_a_obstacle_detector_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        
        Node(
            package='team_a_localizer',
            executable='team_a_localizer_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_a_global_path_planner',
            executable='team_a_global_path_planner_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_a_local_goal_creator',
            executable='team_a_local_goal_creator_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_a_local_path_planner',
            executable='team_a_local_path_planner_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d','./src/chibi25_a/local_path_planner/launch/Rviz_config.rviz'],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/user/ws/src/chibi25_a/local_path_planner/bagfiles/map_bag','--clock'],
            output='screen'
        )
    ])