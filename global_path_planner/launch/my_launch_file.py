from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    map_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        '../../map/bagfiles/a_bagfile4.yaml'
    ))

    # ノードたち
    global_planner_node = Node(
        package='team_a_global_path_planner',
        executable='team_a_global_path_planner_node',
        name='global_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_path}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    return LaunchDescription([
        # ステップ1: グローバルプランナー起動
        global_planner_node,

        # ステップ2: 少し遅れてRViz起動（2秒後）
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),

        # ステップ3: さらに遅れて map_server（4秒後）
        TimerAction(
            period=4.0,
            actions=[map_server_node]
        ),

        # ステップ4: 最後に lifecycle manager（5秒後）
        TimerAction(
            period=5.0,
            actions=[lifecycle_manager]
        )
    ])
    