from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sec_tour_guide',
            executable='group_spawner',
            name='group_spawner',
            output='screen'
        ),
    ])