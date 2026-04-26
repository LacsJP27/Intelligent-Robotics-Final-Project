import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from sec_tour_guide.utils.clustering import (
    filter_rear_arc,
    polar_to_cartesian,
    cluster,
    closest_distance,
)

TOPIC_SCAN = '/scan'
TOPIC_GROUP_DETECTED = '/group/detected'
TOPIC_GROUP_DISTANCE = '/group/closest_distance'


# TODO — Next steps (in order):
# 1. Add group_tracker to sim_tour.launch.py (same pattern as safety_monitor node)
# 2. Add person cylinder SDF under worlds/models/person_cylinder/model.sdf
#    so you have something to detect in Gazebo
# 3. Wire /group/detected and /group/closest_distance into tour_state_machine.py:
#    - subscribe both topics
#    - in NAVIGATING: if not detected for 0.5s → cancel goal, enter WAITING_FOR_GROUP
#    - in WAITING_FOR_GROUP: when detected again → re-send same waypoint goal
# 4. Sanity test: stationary robot, cylinder at 1m / 2m / 3m / no cylinder
#    run:  ros2 topic echo /group/detected
#          ros2 topic echo /group/closest_distance
# 5. If walls trigger false positives, add a cluster-width filter in clustering.py

class GroupTracker(Node):

    def __init__(self):
        super().__init__('group_tracker')

        self._detected_pub = self.create_publisher(Bool, TOPIC_GROUP_DETECTED, 10)
        self._distance_pub = self.create_publisher(Float32, TOPIC_GROUP_DISTANCE, 10)

        self.create_subscription(LaserScan, TOPIC_SCAN, self._scan_cb, 10)

        self.get_logger().info('Group tracker ready.')

    def _scan_cb(self, msg: LaserScan):
        # Steps 2–3: rear arc + range filter
        angles, ranges = filter_rear_arc(msg)

        # Step 4: polar → cartesian
        if len(ranges) == 0:
            self._publish(detected=False, distance=0.0)
            return
        points = polar_to_cartesian(angles, ranges)

        # Step 5: DBSCAN
        centroids = cluster(points)

        # Steps 6–7: compute distance and publish
        dist = closest_distance(centroids)
        if dist is None:
            self._publish(detected=False, distance=0.0)
        else:
            self._publish(detected=True, distance=dist)

    def _publish(self, detected: bool, distance: float):
        self._detected_pub.publish(Bool(data=detected))
        self._distance_pub.publish(Float32(data=distance))


def main(args=None):
    rclpy.init(args=args)
    node = GroupTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
