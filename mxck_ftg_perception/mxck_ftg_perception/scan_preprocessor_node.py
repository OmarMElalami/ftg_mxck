#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener, TransformException

from mxck_ftg_perception.common import (
    compute_front_center_in_scan,
    transform_to_2d,
    range_is_valid,
)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanPreprocessorNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_preprocessor_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('filtered_scan_topic', '/autonomous/ftg/scan_filtered')
        self.declare_parameter('front_clearance_topic', '/autonomous/ftg/front_clearance')
        self.declare_parameter('status_topic', '/autonomous/ftg/status')

        self.declare_parameter('front_center_deg', 0.0)
        self.declare_parameter('front_fov_deg', 120.0)

        self.declare_parameter('clip_min_range_m', 0.05)
        self.declare_parameter('clip_max_range_m', 8.0)

        self.declare_parameter('enable_moving_average', False)
        self.declare_parameter('moving_average_window', 3)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.filtered_scan_topic = str(self.get_parameter('filtered_scan_topic').value)
        self.front_clearance_topic = str(self.get_parameter('front_clearance_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)

        self.front_center_base_rad = math.radians(float(self.get_parameter('front_center_deg').value))
        self.front_half_fov_rad = math.radians(float(self.get_parameter('front_fov_deg').value)) / 2.0

        self.clip_min = float(self.get_parameter('clip_min_range_m').value)
        self.clip_max = float(self.get_parameter('clip_max_range_m').value)

        self.enable_moving_average = bool(self.get_parameter('enable_moving_average').value)
        self.moving_average_window = max(1, int(self.get_parameter('moving_average_window').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.filtered_pub = self.create_publisher(LaserScan, self.filtered_scan_topic, 10)
        self.clearance_pub = self.create_publisher(Float32, self.front_clearance_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.get_logger().info(
            f"Listening on {self.scan_topic}, publishing recentered front scan to {self.filtered_scan_topic}"
        )

    def _moving_average(self, values):
        if self.moving_average_window <= 1:
            return values[:]

        output = values[:]
        half = self.moving_average_window // 2

        for i in range(len(values)):
            if not math.isfinite(values[i]):
                continue

            acc = 0.0
            cnt = 0
            for j in range(max(0, i - half), min(len(values), i + half + 1)):
                if math.isfinite(values[j]):
                    acc += values[j]
                    cnt += 1

            if cnt > 0:
                output[i] = acc / cnt

        return output

    def scan_cb(self, msg: LaserScan) -> None:
        scan_frame = msg.header.frame_id or 'laser'

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, Time())
        except TransformException as exc:
            text = f"[PREPROCESSOR] TF lookup failed {self.base_frame} <- {scan_frame}: {exc}"
            self.get_logger().warn(text)
            self.status_pub.publish(String(data=text))
            return

        _, _, yaw_base_from_laser = transform_to_2d(tf_msg)
        front_center_in_scan = compute_front_center_in_scan(
            yaw_base_from_laser=yaw_base_from_laser,
            base_front_center_rad=self.front_center_base_rad,
        )

        effective_min = max(self.clip_min, float(msg.range_min))
        effective_max = min(self.clip_max, float(msg.range_max))

        selected = []
        front_valid_ranges = []

        intensities = list(msg.intensities) if msg.intensities else []

        for i, raw_r in enumerate(msg.ranges):
            theta = msg.angle_min + i * msg.angle_increment
            rel_theta = wrap_to_pi(theta - front_center_in_scan)

            if abs(rel_theta) > self.front_half_fov_rad:
                continue

            if range_is_valid(float(raw_r)):
                r = min(max(float(raw_r), effective_min), effective_max)
                front_valid_ranges.append(r)
            else:
                r = float('inf')

            inten = float(intensities[i]) if i < len(intensities) else 0.0
            selected.append((rel_theta, r, inten))

        if not selected:
            text = f"[PREPROCESSOR] no beams selected in front window"
            self.get_logger().warn(text)
            self.status_pub.publish(String(data=text))
            return

        selected.sort(key=lambda x: x[0])

        rel_angles = [a for a, _, _ in selected]
        filtered_ranges = [r for _, r, _ in selected]
        filtered_intensities = [it for _, _, it in selected]

        if self.enable_moving_average:
            filtered_ranges = self._moving_average(filtered_ranges)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = rel_angles[0]
        out.angle_increment = msg.angle_increment
        out.angle_max = out.angle_min + (len(filtered_ranges) - 1) * out.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = effective_min
        out.range_max = effective_max
        out.ranges = filtered_ranges
        out.intensities = filtered_intensities

        clearance = min(front_valid_ranges) if front_valid_ranges else float('nan')

        self.filtered_pub.publish(out)
        self.clearance_pub.publish(Float32(data=float(clearance)))

        if front_valid_ranges:
            status = (
                f"[PREPROCESSOR] recentered_front_scan=true, "
                f"scan_frame={scan_frame}, "
                f"front_center_in_scan={math.degrees(front_center_in_scan):+.1f} deg, "
                f"published_angle_min={math.degrees(out.angle_min):+.1f} deg, "
                f"published_angle_max={math.degrees(out.angle_max):+.1f} deg, "
                f"front_clearance={clearance:.2f} m"
            )
        else:
            status = (
                f"[PREPROCESSOR] recentered_front_scan=true, "
                f"scan_frame={scan_frame}, no valid front points"
            )

        self.status_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = ScanPreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    