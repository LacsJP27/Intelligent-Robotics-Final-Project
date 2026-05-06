import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from sec_tour_guide.utils.constants import (
    EMERGENCY_STOP_DISTANCE,
    FORWARD_ARC_HALF_DEG,
    TOPIC_SCAN,
    TOPIC_EMERGENCY,
)


# Rear exclusion: ignore this half-arc centred at 180° (camera mount blind spot)
_REAR_EXCLUSION_HALF_RAD = math.radians(40.0)
# Forward arc: LiDAR angle that corresponds to the robot's forward direction
_FORWARD_CENTER_RAD = math.radians(-93.6)


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self._forward_half_rad = math.radians(FORWARD_ARC_HALF_DEG)

        self._scan_sub = self.create_subscription(
            LaserScan, TOPIC_SCAN, self._scan_callback, 10)
        self._emstop_pub = self.create_publisher(Bool, TOPIC_EMERGENCY, 10)

        self.get_logger().info('Safety monitor ready.')

    def _scan_callback(self, msg: LaserScan):
        emergency = False
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and r >= msg.range_min:
                # Normalise angle to [-pi, pi]
                a = angle % (2.0 * math.pi)
                if a > math.pi:
                    a -= 2.0 * math.pi
                # Skip the rear blind spot where the camera mount sits
                if abs(abs(a) - math.pi) <= _REAR_EXCLUSION_HALF_RAD:
                    angle += msg.angle_increment
                    continue
                if abs(a - _FORWARD_CENTER_RAD) <= self._forward_half_rad and r < EMERGENCY_STOP_DISTANCE:
                    emergency = True
                    break
            angle += msg.angle_increment

        self._emstop_pub.publish(Bool(data=emergency))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
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
