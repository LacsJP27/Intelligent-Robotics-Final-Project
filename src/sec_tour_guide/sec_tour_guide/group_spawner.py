import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity, SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import os
from ament_index_python.packages import get_package_share_directory

class GroupSpawner(Node):
    def __init__(self):
        super().__init__('group_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityPose, '/world/default/set_pose')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set state service not available, waiting again...')

        self.person_sdf = self.get_person_sdf()
        self.person_names = ['person_1', 'person_2', 'person_3']
        self.initial_poses = [
            Pose(position=Point(x=-1.0, y=0.5, z=0.85)),
            Pose(position=Point(x=-1.0, y=-0.5, z=0.85)),
            Pose(position=Point(x=-1.5, y=0.0, z=0.85))
        ]
        self.velocities = [
            Twist(linear=Point(x=0.5, y=0.0, z=0.0)),
            Twist(linear=Point(x=0.5, y=0.0, z=0.0)),
            Twist(linear=Point(x=0.5, y=0.0, z=0.0))
        ]

        for name, pose in zip(self.person_names, self.initial_poses):
            self.spawn_person(name, pose)
        
        self.timer = self.create_timer(0.1, self.move_people)
        self.get_logger().info('Spawned and now moving 3 people.')

    def get_person_sdf(self):
        package_share_directory = get_package_share_directory('sec_tour_guide')
        sdf_file_path = os.path.join(package_share_directory, 'worlds', 'models', 'person_cylinder', 'model.sdf')
        try:
            with open(sdf_file_path, 'r') as f:
                return f.read()
        except FileNotFoundError:
            self.get_logger().error(f"SDF file not found at {sdf_file_path}")
            return None

    def spawn_person(self, name, pose):
        if self.person_sdf is None:
            return
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = self.person_sdf
        req.entity_factory.pose = pose
        self.spawn_client.call_async(req)

    def move_people(self):
        for i, name in enumerate(self.person_names):
            req = SetEntityPose.Request()
            req.entity.name = name
            req.pose = self.initial_poses[i] # This needs to be updated
            
            # Update position for next iteration
            self.initial_poses[i].position.x += self.velocities[i].linear.x * 0.1 # 0.1 is timer period

            self.set_state_client.call_async(req)

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