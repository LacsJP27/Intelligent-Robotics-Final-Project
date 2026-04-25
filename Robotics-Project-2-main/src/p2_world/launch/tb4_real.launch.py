from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('turtlebot4_navigation')
    viz_share = get_package_share_directory('turtlebot4_viz')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_share, 'launch', 'view_navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    controller = Node(
        package='project2_control',
        executable='controller',
        name='project2_controller',
        output='screen',
    )

    return LaunchDescription([
        slam,
        rviz,
        controller,
    ])
