import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sec_tour_guide = get_package_share_directory('sec_tour_guide')
    pkg_turtlebot4_gazebo = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Config files
    twist_mux_config = os.path.join(pkg_sec_tour_guide, 'config', 'twist_mux.yaml')
    nav2_params_file = os.path.join(pkg_sec_tour_guide, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_sec_tour_guide, 'config', 'slam_toolbox_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ------------------------------------------------------------------ #
    # TODO (Member A): Gazebo Harmonic world + TurtleBot 4 spawn          #
    # ------------------------------------------------------------------ #
    # Pass path without .sdf — turtlebot4_gz -> sim.launch.py appends '.sdf' automatically
    world_path = os.path.join(pkg_sec_tour_guide, 'worlds', 'test_world')

    # Gazebo launch 
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_turtlebot4_gazebo,
                'launch',
                'turtlebot4_gz.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items(),
    )

    # SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # Nav2 bringup
    # use_composition must be 'False' (capital F) — nav2 evaluates it via PythonExpression('not ' + value)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_composition': 'False',
        }.items(),
    )


    # twist_mux — velocity priority arbitration                           #
    #   P0: /safety/cmd_vel  (emergency stop, highest priority)           #
    #   P1: /cmd_vel_key     (keyboard teleop override)                   #
    #   P2: /nav_vel         (Nav2 autonomous output, lowest priority)    #
    #   output → /cmd_vel    (TurtleBot 4 hardware driver)                #
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ------------------------------------------------------------------ #
    # TODO (Member C): safety_monitor node                                #
    # ------------------------------------------------------------------ #
    # safety_monitor_node = Node(
    #     package='sec_tour_guide',
    #     executable='safety_monitor',
    #     name='safety_monitor',
    # )

    # ------------------------------------------------------------------ #
    # TODO (Member B): tour_state_machine node                            #
    # ------------------------------------------------------------------ #
    # tour_state_machine_node = Node(
    #     package='sec_tour_guide',
    #     executable='tour_state_machine',
    #     name='tour_state_machine',
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo_launch,
        slam_launch,
        nav2_launch,
        twist_mux_node,
        # add custome nodes here
        # safety_monitor_node,
        # tour_state_machine_node,
    ])
