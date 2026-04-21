import numpy as np
from geometry_msgs.msg import Quaternion


def polar_to_cartesian(ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """Convert parallel polar arrays to an Nx2 Cartesian array [[x, y], ...]."""
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.column_stack((x, y))


def laserscan_angles(msg) -> np.ndarray:
    """Build the angle array corresponding to a LaserScan message's ranges."""
    n = len(msg.ranges)
    return msg.angle_min + np.arange(n) * msg.angle_increment


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = float(np.sin(yaw / 2.0))
    q.w = float(np.cos(yaw / 2.0))
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def bearing_to_point(x: float, y: float) -> float:
    """Return bearing angle in radians to a point given in base_link frame."""
    return float(np.arctan2(y, x))
