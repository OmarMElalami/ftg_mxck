import math
from typing import Tuple

from geometry_msgs.msg import TransformStamped


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_to_2d(transform: TransformStamped) -> Tuple[float, float, float]:
    tx = float(transform.transform.translation.x)
    ty = float(transform.transform.translation.y)
    q = transform.transform.rotation
    yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
    return tx, ty, yaw


def range_is_valid(r: float) -> bool:
    return math.isfinite(r) and r > 0.0