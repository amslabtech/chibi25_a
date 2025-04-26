from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team_a_obstacle_detector',
            executable='team_a_obstacle_detector_node',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='team_a_localizer',
            executable='team_a_localizer_node',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='team_a_global_path_planner',
            executable='team_a_global_path_planner_node',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='team_a_local_goal_creator',
            executable='team_a_local_goal_creator_node',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='team_a_local_map_creator',
            executable='team_a_local_map_creator_node',
            parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='team_a_local_path_planner',
            executable='team_a_local_path_planner_node',
            output="screen",
            parameters=[{'use_sim_time':True}],
        ),
    ])