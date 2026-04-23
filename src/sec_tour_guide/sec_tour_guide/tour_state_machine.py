"""Main FSM for orchestrating the tour."""

import rclpy
from rclpy.node import Node


class TourStateMachine(Node):
    """The main state machine for the robot tour guide."""

    def __init__(self):
        super().__init__("tour_state_machine")
        self.get_logger().info("Tour State Machine has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = TourStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
