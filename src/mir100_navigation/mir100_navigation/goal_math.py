"""Pure goal-pose calculations, kept independent of ROS runtime imports."""

import math


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return the planar quaternion (x, y, z, w) for a yaw angle."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
