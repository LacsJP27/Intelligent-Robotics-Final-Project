import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose

from sec_tour_guide.utils.constants import (
    WAYPOINT_DWELL_TIME,
    TELEOP_TIMEOUT_SEC,
    FSM_LOOP_RATE_HZ,
    TOPIC_CMD_VEL,
    TOPIC_TELEOP,
    TOPIC_TOUR_STATE,
    TOPIC_EMERGENCY,
    TOPIC_GROUP_DETECTED,
    GROUP_LOST_TIMEOUT_SEC
)


# ---------------------------------------------------------------------------
# FSM states
# ---------------------------------------------------------------------------
IDLE = 'IDLE'
NAVIGATING = 'NAVIGATING'
WAYPOINT_REACHED = 'WAYPOINT_REACHED'
EMERGENCY_STOP = 'EMERGENCY_STOP'
TELEOP_OVERRIDE = 'TELEOP_OVERRIDE'
TOUR_COMPLETE = 'TOUR_COMPLETE'
WAITING_FOR_GROUP = 'WAITING_FOR_GROUP'


def _yaw_to_quat(yaw: float):
    """Convert a yaw angle (radians) to a (qz, qw) quaternion pair."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TourStateMachine(Node):

    def __init__(self):
        super().__init__('tour_state_machine')

        # --- Parameters ---
        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        # Real TurtleBot 4 (Create 3) expects TwistStamped on /cmd_vel; the Gazebo
        # simulation expects plain Twist. Launch files set this per-environment.
        self.declare_parameter('use_stamped_cmd_vel', False)
        self._use_stamped = self.get_parameter('use_stamped_cmd_vel').value
        self._cmd_msg_type = TwistStamped if self._use_stamped else Twist

        # --- Waypoints ---
        self._waypoints = []          # list of (x, y, yaw)
        self._wp_idx = 0              # which waypoint we are heading to
        self._load_waypoints(wp_file)

        # --- Nav2 action client ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None      # handle to the current Nav2 goal
        self._goal_active = False     # True once Nav2 accepts and is executing the goal
        self._goal_pending = False    # True between send_goal_async and _goal_response_cb
        self._cancel_requested = False # True if we requested cancel while waiting for goal response
        # bt_navigator advertises its action server before its BT is fully loaded; during
        # that window goals are rejected with "Action server is inactive". Back off so
        # we don't hammer it at the FSM loop rate.
        self._retry_after = 0.0
        self._retry_cooldown_sec = 1.0
        # --- FSM state ---
        self._state = IDLE
        self._dwell_start = None      # wall-clock time when dwell began

        # --- Emergency stop flag (set by safety_monitor) ---
        self._emergency = False

        # --- Group detection flag (set by group_tracker) ---
        self._last_group_seen = time.time() # assume group there at startup
        self._group_detected_count = 0

        # --- Teleop tracking ---
        self._last_teleop_time = 0.0
        self._latest_teleop = self._cmd_msg_type()

        # --- Publishers ---
        self._cmd_pub = self.create_publisher(self._cmd_msg_type, TOPIC_CMD_VEL, 10)
        self._state_pub = self.create_publisher(String, TOPIC_TOUR_STATE, 10)

        # --- Subscribers ---
        self.create_subscription(Bool, TOPIC_EMERGENCY, self._emergency_cb, 10)
        self.create_subscription(self._cmd_msg_type, TOPIC_TELEOP, self._teleop_cb, 10)
        self.create_subscription(Bool, TOPIC_GROUP_DETECTED, self._group_detected_cb, 10)
        # --- Main loop timer ---
        period = 1.0 / FSM_LOOP_RATE_HZ
        self.create_timer(period, self._loop)

        self.get_logger().info(
            f'Tour state machine ready — {len(self._waypoints)} waypoints loaded.')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _emergency_cb(self, msg: Bool):
        self._emergency = msg.data

    def _teleop_cb(self, msg):
        self._latest_teleop = msg
        self._last_teleop_time = time.time()

    def _group_detected_cb(self, msg: Bool):
        if msg.data:
            self._last_group_seen = time.time()
            # self._group_detected_count += 1


    # -----------------------------------------------------------------------
    # Velocity helpers
    # -----------------------------------------------------------------------

    def _make_stop_cmd(self):
        """Zero-velocity command in whichever message type is configured."""
        if self._use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            return msg
        return Twist()

    def _publish_teleop(self):
        """Relay the latest teleop command, refreshing the timestamp if stamped.

        The Create 3 base rejects TwistStamped messages older than ~200 ms, so
        the original keyboard timestamp would be too stale by the time it
        reaches the robot over WiFi.
        """
        if self._use_stamped:
            self._latest_teleop.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(self._latest_teleop)

    # -----------------------------------------------------------------------
    # Nav2 helpers
    # -----------------------------------------------------------------------

    def _send_goal(self, x: float, y: float, yaw: float):
        # Non-blocking check — avoids freezing the executor while waiting
        if not self._nav_client.server_is_ready():
            self.get_logger().warn('Nav2 not ready yet, will retry...')
            self._state = IDLE   # IDLE retries every loop tick until server is ready
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz, qw = _yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_active = False
        self._goal_pending = True   # waiting for _goal_response_cb
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(
            f'[{self._state}] Sending goal {self._wp_idx + 1}/{len(self._waypoints)}: '
            f'({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)')

    def _goal_response_cb(self, future):
        self._goal_pending = False   # response received either way
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal — will retry.')
            self._retry_after = time.time() + self._retry_cooldown_sec
            self._state = IDLE
            return
        if self._cancel_requested:
            self.get_logger().info('Goal accepted but cancel requested — cancelling immediately.')
            handle.cancel_goal_async()
            self._cancel_requested = False
            return
        self._goal_handle = handle
        self._goal_active = True
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._goal_active = False
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Reached waypoint {self._wp_idx + 1}/{len(self._waypoints)}!')
            self._state = WAYPOINT_REACHED
            self._dwell_start = time.time()
        elif status == GoalStatus.STATUS_CANCELED:
            # We cancelled it intentionally — state already set by _cancel_goal caller
            pass
        else:
            # Nav2 aborted (no path found, obstacle, etc.) — go to IDLE so the
            # loop retries the same waypoint on the next tick.
            self.get_logger().warn(
                f'Nav2 aborted goal (status {status}), retrying waypoint '
                f'{self._wp_idx + 1}...')
            self._state = IDLE

    def _cancel_goal(self):
        if self._goal_handle is not None and self._goal_active:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self._goal_active = False
        elif self._goal_pending:
            self._cancel_requested = True

    # -----------------------------------------------------------------------
    # Main FSM loop
    # -----------------------------------------------------------------------

    def _loop(self):
        teleop_active = (time.time() - self._last_teleop_time) < TELEOP_TIMEOUT_SEC

        if (teleop_active 
            and self._state != EMERGENCY_STOP 
            and self._state != TELEOP_OVERRIDE):

            self.get_logger().info('Teleop active — overriding Nav2 commands.')
            self._cancel_goal()  # ensure Nav2 goal is cancelled immediately when teleop starts
            self._state = TELEOP_OVERRIDE

        # -- IDLE: wait for Nav2 to be ready, then start --
        elif self._state == IDLE:
            if (self._waypoints
                    and self._nav_client.server_is_ready()
                    and time.time() >= self._retry_after):
                self._state = NAVIGATING
                self._send_goal(*self._waypoints[self._wp_idx])

        # -- NAVIGATING: Nav2 is driving; watch for interruptions --
        elif self._state == NAVIGATING:
            if self._emergency:
                self.get_logger().warn('EMERGENCY STOP — obstacle < 0.25 m!')
                self._cancel_goal()
                self._state = EMERGENCY_STOP
            elif (time.time() - self._last_group_seen) > GROUP_LOST_TIMEOUT_SEC:
                self.get_logger().warn('Group lost — waiting for them to reappear.')
                self._cancel_goal()
                self._state = WAITING_FOR_GROUP
            elif not self._goal_active and not self._goal_pending:
                # No goal in flight — something dropped it without us noticing.
                # Fall back to IDLE so the next tick re-sends.
                self._state = IDLE

        # -- EMERGENCY_STOP: publish zero velocity until clear --
        elif self._state == EMERGENCY_STOP:
            self._cmd_pub.publish(self._make_stop_cmd())
            if not self._emergency:
                self.get_logger().info('Obstacle cleared — resuming.')
                self._state = NAVIGATING
                self._send_goal(*self._waypoints[self._wp_idx])

        # -- TELEOP_OVERRIDE: relay keyboard commands --
        elif self._state == TELEOP_OVERRIDE:
            self._publish_teleop()
            if not teleop_active:
                self.get_logger().info('Teleop inactive — resuming navigation.')
                self._state = NAVIGATING
                self._send_goal(*self._waypoints[self._wp_idx])

        elif self._state == WAITING_FOR_GROUP:
            self._cmd_pub.publish(self._make_stop_cmd())
            if (time.time() - self._last_group_seen) < GROUP_LOST_TIMEOUT_SEC:
                self.get_logger().info('Group re-detected — resuming navigation.')
                self._state = NAVIGATING
                self._send_goal(*self._waypoints[self._wp_idx]) # re-send same goal

        # -- WAYPOINT_REACHED: dwell, then advance --
        elif self._state == WAYPOINT_REACHED:
            self._cmd_pub.publish(self._make_stop_cmd())
            if time.time() - self._dwell_start >= WAYPOINT_DWELL_TIME:
                self._wp_idx += 1
                if self._wp_idx >= len(self._waypoints):
                    self.get_logger().info('Tour complete!')
                    self._state = TOUR_COMPLETE
                else:
                    self._state = NAVIGATING
                    self._send_goal(*self._waypoints[self._wp_idx])

        # -- TOUR_COMPLETE: sit still --
        elif self._state == TOUR_COMPLETE:
            self._cmd_pub.publish(self._make_stop_cmd())

        # Publish current state for monitoring
        self._state_pub.publish(String(data=self._state))

    # -----------------------------------------------------------------------
    # Waypoint loading
    # -----------------------------------------------------------------------

    def _load_waypoints(self, file_path: str):
        if not file_path:
            self.get_logger().error('No waypoints_file parameter set!')
            return
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            for wp in data['tour']['waypoints']:
                self._waypoints.append((
                    float(wp['x']),
                    float(wp['y']),
                    float(wp['yaw']),
                ))
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TourStateMachine()
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
