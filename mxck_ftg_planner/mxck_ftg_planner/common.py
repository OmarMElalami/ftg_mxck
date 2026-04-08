from dataclasses import dataclass
from typing import Optional
import math


@dataclass
class SpeedPolicyConfig:
    cruise_speed_mps: float
    min_speed_mps: float
    stop_speed_mps: float
    caution_clearance_m: float
    stop_clearance_m: float
    steering_slowdown_start_rad: float
    steering_slowdown_full_rad: float
    max_abs_gap_angle_rad: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite_or_none(value: float) -> Optional[float]:
    if math.isfinite(value):
        return float(value)
    return None


def compute_speed_from_clearance_and_steering(
    clearance_m: Optional[float],
    gap_angle_rad: float,
    cfg: SpeedPolicyConfig,
) -> float:
    angle = abs(clamp(gap_angle_rad, -cfg.max_abs_gap_angle_rad, cfg.max_abs_gap_angle_rad))

    # Steering-based slowdown.
    if angle <= cfg.steering_slowdown_start_rad:
        steering_factor = 1.0
    elif angle >= cfg.steering_slowdown_full_rad:
        steering_factor = 0.0
    else:
        span = cfg.steering_slowdown_full_rad - cfg.steering_slowdown_start_rad
        steering_factor = 1.0 - (angle - cfg.steering_slowdown_start_rad) / span

    # Clearance-based slowdown.
    if clearance_m is None:
        clearance_factor = 0.5
    elif clearance_m <= cfg.stop_clearance_m:
        clearance_factor = 0.0
    elif clearance_m >= cfg.caution_clearance_m:
        clearance_factor = 1.0
    else:
        span = cfg.caution_clearance_m - cfg.stop_clearance_m
        clearance_factor = (clearance_m - cfg.stop_clearance_m) / span

    factor = min(steering_factor, clearance_factor)

    if factor <= 0.0:
        return cfg.stop_speed_mps

    return max(cfg.min_speed_mps, cfg.cruise_speed_mps * factor)
