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
    angle_in_window,
    transform_to_2d,
    range_is_valid,
)


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
        self.declare_parameter('outside_window_as_obstacle', True)

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
        self.outside_window_as_obstacle = bool(self.get_parameter('outside_window_as_obstacle').value)

        self.enable_moving_average = bool(self.get_parameter('enable_moving_average').value)
        self.moving_average_window = max(1, int(self.get_parameter('moving_average_window').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.filtered_pub = self.create_publisher(LaserScan, self.filtered_scan_topic, 10)
        self.clearance_pub = self.create_publisher(Float32, self.front_clearance_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.get_logger().info(
            f"Listening on {self.scan_topic}, publishing filtered scan to {self.filtered_scan_topic}"
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

        raw_ranges = list(msg.ranges)
        blocked_value = self.clip_min if self.outside_window_as_obstacle else float('inf')
        filtered_ranges = [blocked_value] * len(raw_ranges)

        front_valid_ranges = []
        front_indices = []

        for i, raw_r in enumerate(raw_ranges):
            theta = msg.angle_min + i * msg.angle_increment

            if not angle_in_window(theta, front_center_in_scan, self.front_half_fov_rad):
                continue

            if not range_is_valid(float(raw_r)):
                continue

            r = min(max(float(raw_r), self.clip_min), self.clip_max)
            filtered_ranges[i] = r
            front_valid_ranges.append(r)
            front_indices.append(i)

        if self.enable_moving_average and front_indices:
            smoothed = self._moving_average(front_valid_ranges)
            for idx, val in zip(front_indices, smoothed):
                filtered_ranges[idx] = val

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered_ranges
        out.intensities = list(msg.intensities) if msg.intensities else []

        clearance = min(front_valid_ranges) if front_valid_ranges else float('nan')

        self.filtered_pub.publish(out)
        self.clearance_pub.publish(Float32(data=float(clearance)))

        if front_valid_ranges:
            status = (
                f"[PREPROCESSOR] scan_frame={scan_frame}, "
                f"front_center_in_scan={math.degrees(front_center_in_scan):+.1f} deg, "
                f"front_fov={math.degrees(self.front_half_fov_rad * 2.0):.1f} deg, "
                f"front_clearance={clearance:.2f} m"
            )
        else:
            status = f"[PREPROCESSOR] scan_frame={scan_frame}, no valid front points"

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