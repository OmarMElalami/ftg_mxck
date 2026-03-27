#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import LaserScan


class CtuFtgAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('ctu_ftg_adapter_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('final_heading_topic', '/final_heading_angle')
        self.declare_parameter('gap_found_topic', '/gap_found')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('gap_angle_topic', '/autonomous/ftg/gap_angle')
        self.declare_parameter('target_speed_topic', '/autonomous/ftg/target_speed')
        self.declare_parameter('planner_status_topic', '/autonomous/ftg/planner_status')

        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('message_timeout_sec', 0.50)

        self.declare_parameter('use_scan_for_speed', True)
        self.declare_parameter('front_window_deg', 20.0)
        self.declare_parameter('default_front_clearance_m', 10.0)

        self.declare_parameter('heading_sign', 1.0)
        self.declare_parameter('heading_offset_rad', 0.0)
        self.declare_parameter('gap_angle_limit_rad', 0.60)

        self.declare_parameter('stop_distance_m', 0.30)
        self.declare_parameter('slow_distance_m', 0.55)
        self.declare_parameter('clear_distance_m', 0.90)

        self.declare_parameter('stop_speed_mps', 0.0)
        self.declare_parameter('crawl_speed_mps', 0.10)
        self.declare_parameter('slow_speed_mps', 0.18)
        self.declare_parameter('cruise_speed_mps', 0.25)

        self.final_heading_topic = str(self.get_parameter('final_heading_topic').value)
        self.gap_found_topic = str(self.get_parameter('gap_found_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)

        self.gap_angle_topic = str(self.get_parameter('gap_angle_topic').value)
        self.target_speed_topic = str(self.get_parameter('target_speed_topic').value)
        self.planner_status_topic = str(self.get_parameter('planner_status_topic').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.message_timeout_sec = float(self.get_parameter('message_timeout_sec').value)

        self.use_scan_for_speed = bool(self.get_parameter('use_scan_for_speed').value)
        self.front_window_deg = float(self.get_parameter('front_window_deg').value)
        self.default_front_clearance_m = float(self.get_parameter('default_front_clearance_m').value)

        self.heading_sign = float(self.get_parameter('heading_sign').value)
        self.heading_offset_rad = float(self.get_parameter('heading_offset_rad').value)
        self.gap_angle_limit_rad = float(self.get_parameter('gap_angle_limit_rad').value)

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.clear_distance_m = float(self.get_parameter('clear_distance_m').value)

        self.stop_speed_mps = float(self.get_parameter('stop_speed_mps').value)
        self.crawl_speed_mps = float(self.get_parameter('crawl_speed_mps').value)
        self.slow_speed_mps = float(self.get_parameter('slow_speed_mps').value)
        self.cruise_speed_mps = float(self.get_parameter('cruise_speed_mps').value)

        # -----------------------------
        # State
        # -----------------------------
        self.last_heading: Optional[float] = None
        self.last_gap_found: Optional[bool] = None
        self.last_scan: Optional[LaserScan] = None

        self.last_heading_time = None
        self.last_gap_time = None
        self.last_scan_time = None

        # -----------------------------
        # Publishers
        # -----------------------------
        self.pub_gap_angle = self.create_publisher(Float32, self.gap_angle_topic, 10)
        self.pub_target_speed = self.create_publisher(Float32, self.target_speed_topic, 10)
        self.pub_status = self.create_publisher(String, self.planner_status_topic, 10)

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.sub_heading = self.create_subscription(
            Float32,
            self.final_heading_topic,
            self.on_heading,
            10,
        )

        self.sub_gap_found = self.create_subscription(
            Bool,
            self.gap_found_topic,
            self.on_gap_found,
            10,
        )

        self.sub_scan = None
        if self.use_scan_for_speed:
            self.sub_scan = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.on_scan,
                10,
            )

        # -----------------------------
        # Timer
        # -----------------------------
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'CTU FTG adapter started. Inputs: '
            f'{self.final_heading_topic}, {self.gap_found_topic}, {self.scan_topic} | '
            f'Outputs: {self.gap_angle_topic}, {self.target_speed_topic}, {self.planner_status_topic}'
        )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------
    def on_heading(self, msg: Float32) -> None:
        self.last_heading = float(msg.data)
        self.last_heading_time = self.get_clock().now()

    def on_gap_found(self, msg: Bool) -> None:
        self.last_gap_found = bool(msg.data)
        self.last_gap_time = self.get_clock().now()

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def is_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= self.message_timeout_sec

    def compute_front_clearance(self) -> float:
        if (not self.use_scan_for_speed) or (self.last_scan is None):
            return self.default_front_clearance_m

        scan = self.last_scan
        half_window = math.radians(self.front_window_deg)

        min_valid = None
        angle = scan.angle_min

        for r in scan.ranges:
            if -half_window <= angle <= half_window:
                if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                    if (min_valid is None) or (r < min_valid):
                        min_valid = r
            angle += scan.angle_increment

        if min_valid is None:
            return self.default_front_clearance_m

        return float(min_valid)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    # --------------------------------------------------
    # Main output logic
    # --------------------------------------------------
    def on_timer(self) -> None:
        heading_fresh = self.is_fresh(self.last_heading_time)
        gap_fresh = self.is_fresh(self.last_gap_time)
        scan_fresh = self.is_fresh(self.last_scan_time) if self.use_scan_for_speed else True

        gap_angle = 0.0
        target_speed = 0.0
        front_clearance = self.default_front_clearance_m
        reason = 'waiting_for_inputs'

        if not heading_fresh:
            reason = 'heading_timeout'
        elif not gap_fresh:
            reason = 'gap_found_timeout'
        elif not bool(self.last_gap_found):
            reason = 'no_gap_found'
        elif self.use_scan_for_speed and not scan_fresh:
            reason = 'scan_timeout'
            target_speed = self.stop_speed_mps
        else:
            raw_angle = self.heading_sign * float(self.last_heading) + self.heading_offset_rad
            gap_angle = self.clamp(raw_angle, -self.gap_angle_limit_rad, self.gap_angle_limit_rad)

            front_clearance = self.compute_front_clearance()

            # Clearance-based speed only
            if front_clearance < self.stop_distance_m:
                target_speed = self.stop_speed_mps
                reason = 'stop_distance'
            elif front_clearance < self.slow_distance_m:
                target_speed = self.crawl_speed_mps
                reason = 'crawl_distance'
            elif front_clearance < self.clear_distance_m:
                target_speed = self.slow_speed_mps
                reason = 'slow_distance'
            else:
                target_speed = self.cruise_speed_mps
                reason = 'cruise'

        gap_msg = Float32()
        gap_msg.data = float(gap_angle)

        speed_msg = Float32()
        speed_msg.data = float(target_speed)

        status_msg = String()
        status_msg.data = (
            f'gap_found={self.last_gap_found}, '
            f'heading_fresh={heading_fresh}, gap_fresh={gap_fresh}, scan_fresh={scan_fresh}, '
            f'gap_angle_rad={gap_angle:+.4f}, '
            f'front_clearance_m={front_clearance:.3f}, '
            f'target_speed_mps={target_speed:.3f}, '
            f'reason={reason}'
        )

        self.pub_gap_angle.publish(gap_msg)
        self.pub_target_speed.publish(speed_msg)
        self.pub_status.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CtuFtgAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()