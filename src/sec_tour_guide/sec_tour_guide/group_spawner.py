import os
import subprocess

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


WORLD = 'default'

# Robot spawns at (2.286, 3.048) facing +x, so "behind" is -x.
# Cylinder positions are robot_spawn + offset.
PEOPLE = [
    ('person_1', 1.286, 3.548, 0.85),   # 1.0 m behind, 0.5 m left
    ('person_2', 1.286, 2.548, 0.85),   # 1.0 m behind, 0.5 m right
    ('person_3', 0.786, 3.048, 0.85),   # 1.5 m behind, centre
]

# x velocity (m/s) — applied every MOVE_PERIOD seconds
X_VEL     = 0.3
MOVE_PERIOD = 1.0


class GroupSpawner(Node):
    def __init__(self):
        super().__init__('group_spawner')

        pkg = get_package_share_directory('sec_tour_guide')
        self._sdf = os.path.join(
            pkg, 'worlds', 'models', 'person_cylinder', 'model.sdf')

        if not os.path.getsize(self._sdf):
            self.get_logger().error(f'SDF file is empty: {self._sdf}')
            return

        # positions as mutable list so move_people can update x
        self._poses = [[x, y, z] for _, x, y, z in PEOPLE]

        for i, (name, _, _, _) in enumerate(PEOPLE):
            self._spawn(name, *self._poses[i])

        self.get_logger().info('Spawned 3 person cylinders.')
        self._timer = self.create_timer(MOVE_PERIOD, self._move_people)


    def _spawn(self, name, x, y, z):
        subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', WORLD,
            '-name',  name,
            '-file',  self._sdf,
            '-x', str(x), '-y', str(y), '-z', str(z),
        ])

    def _move_people(self):
        for i, (name, _, _, _) in enumerate(PEOPLE):
            self._poses[i][0] += X_VEL * MOVE_PERIOD
            x, y, z = self._poses[i]
            subprocess.Popen([
                'ros2', 'run', 'ros_gz_sim', 'set_entity_pose',
                '--name', name,
                '--pos', str(x), str(y), str(z),
            ])


def main(args=None):
    rclpy.init(args=args)
    node = GroupSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
