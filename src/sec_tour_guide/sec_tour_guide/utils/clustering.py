import math
import numpy as np
from sklearn.cluster import DBSCAN

# --- Rear-arc filter ---
REAR_ARC_MIN_RAD = math.radians(120.0)   # 0 = forward in base_link
REAR_ARC_MAX_RAD = math.radians(240.0)

# --- Range filter ---
RANGE_MIN = 0   # metres
RANGE_MAX = 5.0

# --- DBSCAN ---
DBSCAN_EPS = 0.3
DBSCAN_MIN_SAMPLES = 3


def filter_rear_arc(msg):
    """Return (angles_rad, ranges_m) arrays for points inside the rear arc."""
    angles = []
    ranges = []
    angle = msg.angle_min
    for r in msg.ranges:
        # Normalise angle to [0, 2π)
        a = angle % (2.0 * math.pi)
        if REAR_ARC_MIN_RAD <= a <= REAR_ARC_MAX_RAD:
            if math.isfinite(r) and RANGE_MIN <= r <= RANGE_MAX:
                angles.append(a)
                ranges.append(r)
        angle += msg.angle_increment
    return np.array(angles), np.array(ranges)


def polar_to_cartesian(angles, ranges):
    """Convert parallel angle/range arrays to an (N, 2) XY array."""
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.column_stack((x, y))


def cluster(points):
    """
    Run DBSCAN on an (N, 2) array.
    Returns a list of centroids (x, y) — one per cluster, noise excluded.
    Returns empty list if fewer than DBSCAN_MIN_SAMPLES points.
    """
    if len(points) < DBSCAN_MIN_SAMPLES:
        return []
    labels = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit_predict(points)
    centroids = []
    for label in set(labels):
        if label == -1:   # noise
            continue
        mask = labels == label
        centroids.append(points[mask].mean(axis=0))
    return centroids


def closest_distance(centroids):
    """Return the distance to the nearest centroid, or None if no clusters."""
    if not centroids:
        return None
    return float(min(math.hypot(c[0], c[1]) for c in centroids))
