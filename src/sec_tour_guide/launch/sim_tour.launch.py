import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sec_tour_guide')
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    # ------------------------------------------------------------------ #
    # twist_mux — velocity priority arbitration                           #
    #   P0: /safety/cmd_vel  (emergency stop, highest priority)           #
    #   P1: /cmd_vel_key     (keyboard teleop override)                   #
    #   P2: /nav_vel         (Nav2 autonomous output, lowest priority)    #
    #   output → /cmd_vel    (TurtleBot 4 hardware driver)                #
    # ------------------------------------------------------------------ #
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # ------------------------------------------------------------------ #
    # TODO (Member A): Gazebo Harmonic world + TurtleBot 4 spawn          #
    # ------------------------------------------------------------------ #
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([...]),
    #     launch_arguments={...}.items(),
    # )

    # ------------------------------------------------------------------ #
    # TODO (Member A): Nav2 bringup                                       #
    #   Remember to remap Nav2's cmd_vel output → /nav_vel so             #
    #   twist_mux can arbitrate it:                                        #
    #     remappings=[('/cmd_vel', '/nav_vel')]                           #
    # ------------------------------------------------------------------ #
    # nav2 = IncludeLaunchDescription(...)

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
        twist_mux_node,
        # ... other nodes
    ])
