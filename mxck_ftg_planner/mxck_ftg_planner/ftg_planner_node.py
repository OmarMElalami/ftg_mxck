#!/usr/bin/env python3
"""Unified FTG planner node for the MXCK scan-based FTG path.

Subscribes to follow_the_gap_v0 outputs (heading angle, gap_found) and
front_clearance from the scan preprocessor, then publishes gap_angle and
target_speed for the downstream ftg_command_node.
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String

from mxck_ftg_planner.common import (
    SpeedPolicyConfig,
    clamp,
    compute_speed_from_clearance_and_steering,
    finite_or_none,
)


class FtgPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__("ftg_planner_node")

        # --- input topics ---
        self.declare_parameter("heading_angle_topic", "/final_heading_angle")
        self.declare_parameter("gap_found_topic", "/gap_found")
        self.declare_parameter("front_clearance_topic", "/autonomous/ftg/front_clearance")

        # --- output topics ---
        self.declare_parameter("gap_angle_topic", "/autonomous/ftg/gap_angle")
        self.declare_parameter("target_speed_topic", "/autonomous/ftg/target_speed")
        self.declare_parameter("planner_status_topic", "/autonomous/ftg/planner_status")

        # --- behaviour ---
        self.declare_parameter("input_timeout_sec", 0.50)
        self.declare_parameter("max_abs_gap_angle_rad", 0.45)

        self.declare_parameter("cruise_speed_mps", 0.60)
        self.declare_parameter("min_speed_mps", 0.20)
        self.declare_parameter("stop_speed_mps", 0.00)

        self.declare_parameter("caution_clearance_m", 0.90)
        self.declare_parameter("stop_clearance_m", 0.35)

        self.declare_parameter("steering_slowdown_start_rad", 0.20)
        self.declare_parameter("steering_slowdown_full_rad", 0.45)

        # --- resolve values ---
        self.heading_angle_topic = str(self.get_parameter("heading_angle_topic").value)
        self.gap_found_topic = str(self.get_parameter("gap_found_topic").value)
        self.front_clearance_topic = str(self.get_parameter("front_clearance_topic").value)

        self.gap_angle_topic = str(self.get_parameter("gap_angle_topic").value)
        self.target_speed_topic = str(self.get_parameter("target_speed_topic").value)
        self.planner_status_topic = str(self.get_parameter("planner_status_topic").value)

        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)

        self.cfg = SpeedPolicyConfig(
            cruise_speed_mps=float(self.get_parameter("cruise_speed_mps").value),
            min_speed_mps=float(self.get_parameter("min_speed_mps").value),
            stop_speed_mps=float(self.get_parameter("stop_speed_mps").value),
            caution_clearance_m=float(self.get_parameter("caution_clearance_m").value),
            stop_clearance_m=float(self.get_parameter("stop_clearance_m").value),
            steering_slowdown_start_rad=float(self.get_parameter("steering_slowdown_start_rad").value),
            steering_slowdown_full_rad=float(self.get_parameter("steering_slowdown_full_rad").value),
            max_abs_gap_angle_rad=float(self.get_parameter("max_abs_gap_angle_rad").value),
        )

        # --- state ---
        self.latest_gap_found = False
        self.latest_heading_angle = 0.0
        self.latest_clearance = None

        self.last_heading_stamp = None
        self.last_gap_stamp = None
        self.last_clearance_stamp = None

        # --- publishers ---
        self.gap_angle_pub = self.create_publisher(Float32, self.gap_angle_topic, 10)
        self.target_speed_pub = self.create_publisher(Float32, self.target_speed_topic, 10)
        self.status_pub = self.create_publisher(String, self.planner_status_topic, 10)

        # --- subscribers ---
        self.create_subscription(Float32, self.heading_angle_topic, self._heading_cb, 10)
        self.create_subscription(Bool, self.gap_found_topic, self._gap_found_cb, 10)
        self.create_subscription(Float32, self.front_clearance_topic, self._clearance_cb, 10)

        # --- timer (20 Hz) ---
        self.timer = self.create_timer(0.05, self._timer_cb)

        self.get_logger().info(
            f"FTG planner started. Inputs: {self.heading_angle_topic}, "
            f"{self.gap_found_topic}, {self.front_clearance_topic} | Outputs: "
            f"{self.gap_angle_topic}, {self.target_speed_topic}, {self.planner_status_topic}"
        )

    # ------------------------------------------------------------------ helpers
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _inputs_are_fresh(self, now_s: float) -> bool:
        stamps = [self.last_heading_stamp, self.last_gap_stamp, self.last_clearance_stamp]
        if any(s is None for s in stamps):
            return False
        return all((now_s - s) <= self.input_timeout_sec for s in stamps)

    # --------------------------------------------------------------- callbacks
    def _heading_cb(self, msg: Float32) -> None:
        self.latest_heading_angle = float(msg.data)
        self.last_heading_stamp = self._now_sec()

    def _gap_found_cb(self, msg: Bool) -> None:
        self.latest_gap_found = bool(msg.data)
        self.last_gap_stamp = self._now_sec()

    def _clearance_cb(self, msg: Float32) -> None:
        self.latest_clearance = finite_or_none(float(msg.data))
        self.last_clearance_stamp = self._now_sec()

    # ------------------------------------------------------------ main loop
    def _timer_cb(self) -> None:
        now_s = self._now_sec()

        if not self._inputs_are_fresh(now_s):
            self.gap_angle_pub.publish(Float32(data=0.0))
            self.target_speed_pub.publish(Float32(data=self.cfg.stop_speed_mps))
            self.status_pub.publish(String(data="[PLANNER] waiting for fresh inputs"))
            return

        if not self.latest_gap_found:
            self.gap_angle_pub.publish(Float32(data=0.0))
            self.target_speed_pub.publish(Float32(data=self.cfg.stop_speed_mps))
            self.status_pub.publish(String(data="[PLANNER] no valid gap found -> stop"))
            return

        gap_angle = clamp(
            self.latest_heading_angle,
            -self.cfg.max_abs_gap_angle_rad,
            self.cfg.max_abs_gap_angle_rad,
        )
        speed = compute_speed_from_clearance_and_steering(
            self.latest_clearance,
            gap_angle,
            self.cfg,
        )

        self.gap_angle_pub.publish(Float32(data=float(gap_angle)))
        self.target_speed_pub.publish(Float32(data=float(speed)))

        clearance_text = "nan" if self.latest_clearance is None else f"{self.latest_clearance:.2f}"
        self.status_pub.publish(
            String(
                data=(
                    f"[PLANNER] gap_angle={gap_angle:+.3f} rad, "
                    f"target_speed={speed:.2f} m/s, "
                    f"front_clearance={clearance_text} m"
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = FtgPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
