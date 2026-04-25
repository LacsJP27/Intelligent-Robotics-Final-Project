import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import random
import time

# Helper functions for working with angles and quaternions
def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

# Wraps angle to [-pi..pi]
def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def angle_diff(target: float, current: float) -> float:
    return wrap_to_pi(target - current)

# Timeout for escape behavior to prevent getting "stuck" 
# if something goes wrong (i.e. we fail to turn enough to 
# clear the obstacle, or we have a bad scan reading that 
# triggers escape but there's actually no obstacle there)
ESCAPE_TIMEOUT_SEC = 3.0
ESCAPE_COOLDOWN_S = 1.5
AVOID_COOLDOWN_S = 1.0

class Project1Controller(Node):

    def __init__(self):
        super().__init__('project1_controller')

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.key_sub = self.create_subscription(Twist, '/cmd_vel_key', self.key_callback, 10)

        # Control Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.escape_start_time = None  # for escape timeout

        # Internal State
        self.latest_scan = None
        self.manual_twist = Twist()
        self.is_colliding = False

        # Scan summaries (for behaviors 3/4)
        self.have_scan = False
        self.min_front = float('inf')
        self.min_left = float('inf')
        self.min_right = float('inf')

        # Odom pose (for behaviors 3/5)
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Distance tracking for behavior 5
        self.last_turn_x = None
        self.last_turn_y = None

        # Keyboard timeout so it doesn't "stick"
        self.last_key_time = 0.0
        self.key_timeout_s = 0.35

        # Constants
        self.one_ft_m = 0.3048
        self.front_half_angle = math.radians(30.0)   # front cone +/- 30 degrees
        self.collision_thresh_m = 0.23               # safety stop threshold
        self.forward_speed = 0.1
        self.turn_speed = 0.7

        # Behavior 3 (Escape) state
        self.escape_active = False
        self.escape_target_yaw = 0.0
        self.escape_cooldown_end = 0.0
        self.avoid_active = False
        self.avoid_target_yaw = 0.0
        self.avoid_cooldown_end = 0.0

        # Behavior 5 (Random turn) state
        self.random_turn_active = False
        self.random_target_yaw = 0.0

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

        # Compute min distances in front cone using scan geometry (handles 0..2pi wrap)
        min_left = float('inf')
        min_right = float('inf')
        min_front = float('inf')

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r):
                a = wrap_to_pi(angle + (math.pi / 2)) # IMPORTANT: converts [0..2pi] to [-pi..pi] around "front=0"
                if abs(a) <= self.front_half_angle:
                    if r < min_front:
                        min_front = r
                    if a > 0.0:
                        if r < min_left:
                            min_left = r
                    else:
                        if r < min_right:
                            min_right = r
            angle += msg.angle_increment

        self.min_front = min_front
        self.min_left = min_left
        self.min_right = min_right
        self.have_scan = True

        # Approx collision detection using scan
        self.is_colliding = (self.min_front < self.collision_thresh_m)

    # Note: we could use odom for more advanced behaviors like escape/avoid 
    # but for simplicity in this project we just use it for the random turn 
    # behavior since it requires tracking distance traveled which is more 
    # robust using odom than scan
    def odom_callback(self, msg: Odometry):
        # Store pose + yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True

        if self.last_turn_x is None:
            self.last_turn_x = self.x
            self.last_turn_y = self.y

    def key_callback(self, msg: Twist):
        self.manual_twist = msg
        self.last_key_time = time.time()

    # ---------- Helper checks + actions ----------
    def key_is_active(self) -> bool:
        return (time.time() - self.last_key_time) <= self.key_timeout_s

    def symmetric_obstacles(self) -> bool:
        # Both sides within 1ft and similar distances
        if (self.min_left < self.one_ft_m) and (self.min_right < self.one_ft_m):
            return abs(self.min_left - self.min_right) < 0.05
        return False

    def asymmetric_obstacles(self) -> bool:
        # Something within 1ft but not symmetric
        if min(self.min_left, self.min_right) < self.one_ft_m:
            return not self.symmetric_obstacles()
        return False

    def start_escape(self):
        # Escape: turn 180 +/- 30 degrees (fixed action pattern)
        self.escape_start_time = self.get_clock().now()
        delta = random.uniform(math.radians(150.0), math.radians(210.0))
        self.escape_target_yaw = wrap_to_pi(self.yaw + delta)
        self.escape_active = True
        self.random_turn_active = False  # interrupt any random turn in progress
        self.avoid_active = False

    def start_random_turn(self):
        # Random turn +/- 15 degrees
        delta = random.uniform(math.radians(-15.0), math.radians(15.0))
        self.random_target_yaw = wrap_to_pi(self.yaw + delta)
        self.random_turn_active = True
        self.escape_active = False  # interrupt any escape in progress
        self.avoid_active = False
    
    def start_avoid_turn(self):
        # Commit to turning 60-90° away from the closer side
        if self.min_left < self.min_right:
            delta = -random.uniform(math.radians(60.0), math.radians(90.0))
        else:
            delta = random.uniform(math.radians(60.0), math.radians(90.0))
        self.avoid_target_yaw = wrap_to_pi(self.yaw + delta)
        self.avoid_active = True
        self.escape_active = False        # mutual exclusion
        self.random_turn_active = False   # mutual exclusion

    def rotate_toward(self, target_yaw: float) -> TwistStamped:
        msg = TwistStamped()
        err = angle_diff(target_yaw, self.yaw)
        if abs(err) < 0.05:
            msg.twist.angular.z = 0.0
        else:
            msg.twist.angular.z = self.turn_speed if err > 0.0 else -self.turn_speed
        return msg

    def dist_since_last_turn(self) -> float:
        if self.last_turn_x is None:
            return 0.0
        return math.hypot(self.x - self.last_turn_x, self.y - self.last_turn_y)
    
    def front_is_blocked(self) -> bool:
        # front blocked close enough to warrant an escape regardless of symmetry
        return self.min_front < self.one_ft_m
    # --------------------------------------------

    def control_loop(self):
        if not (self.have_scan and self.have_odom):
            return

        msg = TwistStamped()

        self.get_logger().info("TEST")


        # PRIORITY 1: Halt on collision
        if self.is_colliding:
            # Guarantee no forward motion while colliding
            self.get_logger().info("COLLIDING")

            msg.twist.linear.x = 0.0
            self.cmd_pub.publish(msg)
            self.timer.cancel()
            return
            
        # PRIORITY 2: Keyboard commands (only if recent)
        if self.key_is_active():
            stamped = TwistStamped()
            stamped.twist = self.manual_twist
            self.cmd_pub.publish(stamped)
            return
        
                # Note: we check for escape first since it has a timeout 
        # and we don't want to interrupt it, but we check for the 
        # conditions that trigger it (symmetric obstacles or front blocked) 
        # here so that it will start as soon as those conditions are met even 
        # if we happen to be in the middle of another behavior like a random 
        # turn when that happens

            # Note: this check is after escape to ensure it doesn't 
        # interrupt that behavior but before random turn to ensure 
        # it does interrupt if we happen to detect an asymmetric 
        # obstacle during a random turn
        if self.asymmetric_obstacles():
            self.start_avoid_turn()
            msg = self.rotate_toward(self.avoid_target_yaw)
            self.cmd_pub.publish(msg)
            return
        elif (self.symmetric_obstacles() or self.front_is_blocked()) and time.time() > self.escape_cooldown_end:
            self.start_escape()
            msg = self.rotate_toward(self.escape_target_yaw)
            self.cmd_pub.publish(msg)
            return


        # PRIORITY 3: Escape (fixed action pattern)

        if self.escape_active and not self.avoid_active:
            msg = self.rotate_toward(self.escape_target_yaw)
            elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9
            
            # Escape is complete when we're facing the target direction 
            # or we hit the timeout, whichever comes first.
            if abs(angle_diff(self.escape_target_yaw, self.yaw)) < 0.05 or elapsed > ESCAPE_TIMEOUT_SEC:
                self.get_logger().info("ESCAPE COMPLETE" if elapsed <= ESCAPE_TIMEOUT_SEC else "ESCAPE TIMEOUT")
                self.escape_active = False
                self.escape_cooldown_end = time.time() + ESCAPE_COOLDOWN_S
            self.cmd_pub.publish(msg)
            return

            # Note: this check is after escape to ensure it doesn't 
        # interrupt that behavior but before random turn to ensure 
        # it does interrupt if we happen to detect an asymmetric 
        # obstacle during a random turn
        # if self.asymmetric_obstacles():
        #     self.start_avoid_turn()
        #     msg = self.rotate_toward(self.avoid_target_yaw)
        #     self.cmd_pub.publish(msg)
        #     return

        # PRIORITY 4: Avoid (reflex)
        if self.avoid_active and not self.escape_active:
            self.get_logger().info("AVOID")

            err = angle_diff(self.avoid_target_yaw, self.yaw)
            msg = TwistStamped()
            # msg.twist.angular.z = self.turn_speed if err > 0.0 else -self.turn_speed
            msg.twist.angular.z = self.turn_speed

            # if abs(angle_diff(self.avoid_target_yaw, self.yaw)) < 0.05:
            self.avoid_active = False
            #     # self.avoid_cooldown_end = time.time() + AVOID_COOLDOWN_S
            #     self.last_turn_x = self.x
            #     self.last_turn_y = self.y
            self.cmd_pub.publish(msg)
            return
   

        # PRIORITY 5: Random turn after every 1ft of forward travel
        if self.random_turn_active:
            msg = self.rotate_toward(self.random_target_yaw)
            # If we happen to hit the distance threshold during a random turn, 
            # #we want to finish that turn before starting another one, 
            # so we check distance in the separate if below instead of 
            # interrupting and starting a new random turn here
            if abs(angle_diff(self.random_target_yaw, self.yaw)) < 0.05:
                self.random_turn_active = False
                self.last_turn_x = self.x
                self.last_turn_y = self.y
            self.cmd_pub.publish(msg)
            return
        # Note: this check is after escape/avoid to ensure it doesn't 
        # interrupt those behaviors but before forward motion to 
        # ensure it does interrupt if we happen to hit the distance 
        # threshold during a forward drive
        if self.dist_since_last_turn() >= self.one_ft_m:
            self.start_random_turn()
            msg = self.rotate_toward(self.random_target_yaw)
            self.cmd_pub.publish(msg)
            return

        # PRIORITY 6: Drive forward
        msg.twist.linear.x = self.forward_speed

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Project1Controller()
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