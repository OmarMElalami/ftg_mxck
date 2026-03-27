#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

from mxck_ftg_perception.common import (
    transform_to_2d,
    point_laser_to_base,
    angle_in_window,
    range_is_valid,
)


class ScanFrontWindowCheck(Node):
    def __init__(self) -> None:
        super().__init__('scan_front_window_check')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('diagnostic_topic', '/autonomous/ftg/scan_check')
        self.declare_parameter('marker_topic', '/autonomous/ftg/scan_check_markers')

        self.declare_parameter('front_center_deg', 0.0)
        self.declare_parameter('front_fov_deg', 120.0)

        self.declare_parameter('clip_min_range_m', 0.05)
        self.declare_parameter('clip_max_range_m', 8.0)
        self.declare_parameter('beam_stride', 2)
        self.declare_parameter('log_every_n_scans', 10)
        self.declare_parameter('publish_markers', True)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.diag_topic = str(self.get_parameter('diagnostic_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)

        self.front_center_rad = math.radians(float(self.get_parameter('front_center_deg').value))
        self.front_half_fov_rad = math.radians(float(self.get_parameter('front_fov_deg').value)) / 2.0

        self.clip_min = float(self.get_parameter('clip_min_range_m').value)
        self.clip_max = float(self.get_parameter('clip_max_range_m').value)
        self.beam_stride = max(1, int(self.get_parameter('beam_stride').value))
        self.log_every_n_scans = max(1, int(self.get_parameter('log_every_n_scans').value))
        self.publish_markers = bool(self.get_parameter('publish_markers').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.diag_pub = self.create_publisher(String, self.diag_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.scan_count = 0
        self.get_logger().info(
            f"Listening on {self.scan_topic}, base_frame={self.base_frame}, "
            f"front_center={math.degrees(self.front_center_rad):.1f} deg, "
            f"front_fov={math.degrees(self.front_half_fov_rad * 2.0):.1f} deg"
        )

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan_count += 1
        scan_frame = msg.header.frame_id or 'laser'

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, Time())
        except TransformException as exc:
            if self.scan_count % self.log_every_n_scans == 0:
                self.get_logger().warn(f"TF lookup failed {self.base_frame} <- {scan_frame}: {exc}")
            return

        tx, ty, yaw = transform_to_2d(tf_msg)
        best_front = None
        best_any = None

        for i in range(0, len(msg.ranges), self.beam_stride):
            r = float(msg.ranges[i])
            if not range_is_valid(r):
                continue

            r = min(max(r, self.clip_min), self.clip_max)
            theta = msg.angle_min + i * msg.angle_increment
            xb, yb = point_laser_to_base(r, theta, tx, ty, yaw)
            rb = math.hypot(xb, yb)
            bearing_base = math.atan2(yb, xb)

            if best_any is None or rb < best_any[0]:
                best_any = (rb, bearing_base, xb, yb)

            if angle_in_window(bearing_base, self.front_center_rad, self.front_half_fov_rad):
                if best_front is None or rb < best_front[0]:
                    best_front = (rb, bearing_base, xb, yb)

        if self.scan_count % self.log_every_n_scans != 0:
            return

        if best_front is None:
            text = f"[SCAN_CHECK] no valid points in front window. scan_frame={scan_frame}"
            self.get_logger().warn(text)
            self.diag_pub.publish(String(data=text))
            return

        fr, fb, fx, fy = best_front
        side = "CENTER"
        if math.degrees(fb) > 3.0:
            side = "LEFT"
        elif math.degrees(fb) < -3.0:
            side = "RIGHT"

        if best_any is None:
            extra = ""
        else:
            ar, ab, ax, ay = best_any
            extra = f" | closest_any={ar:.2f} m @ {math.degrees(ab):+.1f} deg (x={ax:+.2f}, y={ay:+.2f})"

        text = (
            f"[SCAN_CHECK] front_closest={fr:.2f} m @ {math.degrees(fb):+.1f} deg ({side}) "
            f"(x={fx:+.2f}, y={fy:+.2f}){extra}"
        )
        self.get_logger().info(text)
        self.diag_pub.publish(String(data=text))

        if not self.publish_markers:
            return

        markers = MarkerArray()

        point_marker = Marker()
        point_marker.header.frame_id = self.base_frame
        point_marker.header.stamp = msg.header.stamp
        point_marker.ns = 'scan_front_window_check'
        point_marker.id = 1
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x = fx
        point_marker.pose.position.y = fy
        point_marker.pose.position.z = 0.05
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.10
        point_marker.scale.y = 0.10
        point_marker.scale.z = 0.10
        point_marker.color.a = 1.0
        point_marker.color.r = 1.0
        point_marker.color.g = 0.2
        point_marker.color.b = 0.2
        markers.markers.append(point_marker)

        arrow_marker = Marker()
        arrow_marker.header.frame_id = self.base_frame
        arrow_marker.header.stamp = msg.header.stamp
        arrow_marker.ns = 'scan_front_window_check'
        arrow_marker.id = 2
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = 0.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 0.05
        arrow_marker.pose.orientation.z = math.sin(fb / 2.0)
        arrow_marker.pose.orientation.w = math.cos(fb / 2.0)
        arrow_marker.scale.x = max(0.30, min(1.20, fr))
        arrow_marker.scale.y = 0.04
        arrow_marker.scale.z = 0.04
        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 0.2
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.2
        markers.markers.append(arrow_marker)

        self.marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = ScanFrontWindowCheck()   # bzw. ScanPreprocessorNode()
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