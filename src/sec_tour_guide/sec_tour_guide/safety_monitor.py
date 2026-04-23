"""Monitors for safety hazards and publishes emergency stop commands."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector

from .utils import constants


class SafetyMonitor(Node):
    """
    Monitors for imminent obstacles and hazard detections, publishing a
    zero-velocity Twist message to a high-priority topic to trigger an
    emergency stop via twist_mux.
    """

    def __init__(self):
        super().__init__("safety_monitor")

        # Publisher for the emergency stop command
        self.publisher_ = self.create_publisher(
            Twist, constants.TOPIC_SAFETY_CMD, 10
        )

        # Subscriber to the laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            constants.TOPIC_SCAN,
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        # Subscriber to the Create 3 hazard detection system
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",  # This is a standard topic for Create 3
            self.hazard_callback,
            10,
        )

        self.get_logger().info("Safety Monitor has been started.")

    def scan_callback(self, msg: LaserScan):
        """
        Processes LaserScan data to detect obstacles directly in front of the robot.
        """
        # The front of the robot is around the 0-degree angle. We need to check
        # a small arc to account for the robot's width. Let's define the
        # forward arc as -15 to +15 degrees.
        # The RPLIDAR A1 on the TurtleBot 4 has a 360-degree view.
        # The 'ranges' array starts at the front of the robot (0 degrees) and
        # goes counter-clockwise.

        # Calculate the indices for the forward arc
        angle_min_rad = constants.FORWARD_ARC_MIN_DEG * (3.14159 / 180.0)
        angle_max_rad = constants.FORWARD_ARC_MAX_DEG * (3.14159 / 180.0)
        
        start_index = int((angle_min_rad - msg.angle_min) / msg.angle_increment)
        end_index = int((angle_max_rad - msg.angle_min) / msg.angle_increment)

        # Ensure indices are within the bounds of the ranges array
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) -1, end_index)
        
        # Check for obstacles within the emergency stop distance in the forward arc
        for i in range(start_index, end_index):
            if 0 < msg.ranges[i] < constants.EMERGENCY_STOP_DISTANCE:
                self.publish_emergency_stop("LaserScan detected obstacle")
                return

    def hazard_callback(self, msg: HazardDetectionVector):
        """
        Processes hazard detection messages from the Create 3 base.
        Any detected hazard is considered a reason to stop.
        """
        if msg.detections:
            # Log the specific hazard for debugging
            for detection in msg.detections:
                if detection.type != "backup_limit": # Ignore backup limit sensor
                    self.publish_emergency_stop(f"Hazard detected: {detection.type}")
                    return

    def publish_emergency_stop(self, reason: str):
        """
        Publishes a zero-velocity Twist message to stop the robot.
        """
        self.get_logger().warn(f"EMERGENCY STOP triggered: {reason}")
        stop_msg = Twist()
        # All velocity components are zero by default
        self.publisher_.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
