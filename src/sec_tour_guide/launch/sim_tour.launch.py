import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sec_tour_guide')
    nav2_share = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg, 'worlds', 'world.sdf')
    waypoints_file = os.path.join(pkg, 'config', 'tour_waypoints.yaml')

    # ----------------------------------------------------------------
    # Gazebo + TurtleBot 4 + Nav2 + SLAM  (reused from Project 2)
    # ----------------------------------------------------------------
    tb4_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'tb4_simulation_launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'slam':    'True',
            'headless': 'False',
            # Robot spawn position — center of the test world
            'x_pose': '2.286',
            'y_pose': '3.048',
            'z_pose': '0.01',
            'yaw':    '0.0',
        }.items(),
    )

    # ----------------------------------------------------------------
    # Our custom nodes
    # ----------------------------------------------------------------
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
        parameters=[{'waypoints_file': waypoints_file}],
    )

    group_tracker = Node(
        package='sec_tour_guide',
        executable='group_tracker',
        name='group_tracker',
        output='screen', 
    )

    return LaunchDescription([
        tb4_sim,
        safety_monitor,
        tour_fsm,
        group_tracker,
    ])
