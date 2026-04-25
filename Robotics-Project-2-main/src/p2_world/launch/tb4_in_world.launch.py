from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('p2_world')
    default_world = os.path.join(pkg_share, 'worlds', 'world.sdf')

    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    nav2_share = get_package_share_directory('nav2_bringup')
    tb4_sim_launch = os.path.join(nav2_share, 'launch', 'tb4_simulation_launch.py')
    
    controller = Node(
    package='project2_control',
    executable='controller',
    name='project2_controller',
    output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('headless', default_value='False'),
        
        # Set spawn location of the robot in the world
        DeclareLaunchArgument('x_pose', default_value='2.286'),
        DeclareLaunchArgument('y_pose', default_value='3.048'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb4_sim_launch),
            launch_arguments={
                'world': world,
                'slam': slam,
                'headless': headless,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
                'yaw': yaw,
            }.items()
        ),
        
        controller,
    ])

