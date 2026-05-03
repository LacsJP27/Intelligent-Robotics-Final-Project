import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sec_tour_guide')
    nav_share = get_package_share_directory('turtlebot4_navigation')
    viz_share = get_package_share_directory('turtlebot4_viz')

    waypoints_file = os.path.join(pkg, 'config', 'tour_waypoints.yaml')
    slam_params_file = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    map_file = os.path.join(pkg, 'config', 'cardboard_real_map.yaml')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'map': map_file, 'params_file': nav2_params_file}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params_file}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_share, 'launch', 'view_navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    safety_monitor = Node(
        package='sec_tour_guide',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
    )

    tour_fsm = Node(
        package='sec_tour_guide',
        executable='tour_state_machine',
        name='tour_state_machine',
        output='screen',
        parameters=[{
            'waypoints_file': waypoints_file,
            'use_stamped_cmd_vel': True,
        }],
    )

    group_tracker = Node(
        package='sec_tour_guide',
        executable='group_tracker',
        name='group_tracker',
        output='screen',
    )

    return LaunchDescription([
        localization,
        nav2,
        rviz,
        safety_monitor,
        group_tracker,
        TimerAction(period=30.0, actions=[tour_fsm]),
    ])
