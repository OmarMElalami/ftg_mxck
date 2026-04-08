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


def point_laser_to_base(
    r: float,
    theta_laser: float,
    tx: float,
    ty: float,
    yaw_base_from_laser: float,
) -> Tuple[float, float]:
    xl = r * math.cos(theta_laser)
    yl = r * math.sin(theta_laser)

    cy = math.cos(yaw_base_from_laser)
    sy = math.sin(yaw_base_from_laser)

    xb = tx + cy * xl - sy * yl
    yb = ty + sy * xl + cy * yl
    return xb, yb


def compute_front_center_in_scan(
    yaw_base_from_laser: float,
    base_front_center_rad: float = 0.0,
) -> float:
    return wrap_to_pi(base_front_center_rad - yaw_base_from_laser)


def angle_in_window(angle: float, center: float, half_width: float) -> bool:
    return abs(wrap_to_pi(angle - center)) <= half_width


def range_is_valid(r: float) -> bool:
    return math.isfinite(r) and r > 0.0
