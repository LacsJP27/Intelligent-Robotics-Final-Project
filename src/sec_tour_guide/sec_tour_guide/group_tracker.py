import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from sec_tour_guide.utils.clustering import (
    filter_rear_arc,
    polar_to_cartesian,
    cluster,
    closest_distance,
)
from sec_tour_guide.utils.constants import (
    TOPIC_SCAN,
    TOPIC_GROUP_DETECTED,
    GROUP_THRESHOLD_DISTANCE,
)



class GroupTracker(Node):

    def __init__(self):
        super().__init__('group_tracker')

        self._detected_pub = self.create_publisher(Bool, TOPIC_GROUP_DETECTED, 10)

        self.create_subscription(LaserScan, TOPIC_SCAN, self._scan_cb, 10)

        self.get_logger().info('Group tracker ready.')

    def _scan_cb(self, msg: LaserScan):
        # Steps 2–3: rear arc + range filter
        angles, ranges = filter_rear_arc(msg)

        # Step 4: polar → cartesian
        if len(ranges) == 0:
            self._detected_pub.publish(Bool(data=False))
            return
        points = polar_to_cartesian(angles, ranges)

        # Step 5: DBSCAN
        centroids = cluster(points)

        # Steps 6–7: compute distance and publish
        dist = closest_distance(centroids)
        detected = dist is not None and dist <= GROUP_THRESHOLD_DISTANCE
        self._detected_pub.publish(Bool(data=detected))


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
