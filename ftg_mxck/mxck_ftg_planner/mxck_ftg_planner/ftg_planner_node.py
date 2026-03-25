#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

from mxck_ftg_planner.common import transform_to_2d, range_is_valid, wrap_to_pi


class FTGPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('ftg_planner_node')

        self.declare_parameter('scan_topic', '/autonomous/ftg/scan_filtered')
        self.declare_parameter('front_clearance_topic', '/autonomous/ftg/front_clearance')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('gap_angle_topic', '/autonomous/ftg/gap_angle')
        self.declare_parameter('target_speed_topic', '/autonomous/ftg/target_speed')
        self.declare_parameter('status_topic', '/autonomous/ftg/planner_status')
        self.declare_parameter('marker_topic', '/autonomous/ftg/planner_markers')

        self.declare_parameter('free_space_threshold_m', 1.00)
        self.declare_parameter('min_gap_beams', 5)
        self.declare_parameter('safety_bubble_radius_m', 0.35)
        self.declare_parameter('steering_limit_deg', 28.0)
        self.declare_parameter('center_bias_weight', 0.25)

        self.declare_parameter('stop_distance_m', 0.35)
        self.declare_parameter('slow_distance_m', 0.60)
        self.declare_parameter('cruise_distance_m', 1.20)
        self.declare_parameter('speed_stop_mps', 0.0)
        self.declare_parameter('speed_slow_mps', 0.25)
        self.declare_parameter('speed_medium_mps', 0.45)
        self.declare_parameter('speed_fast_mps', 0.70)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.front_clearance_topic = str(self.get_parameter('front_clearance_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.gap_angle_topic = str(self.get_parameter('gap_angle_topic').value)
        self.target_speed_topic = str(self.get_parameter('target_speed_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)

        self.free_space_threshold = float(self.get_parameter('free_space_threshold_m').value)
        self.min_gap_beams = max(1, int(self.get_parameter('min_gap_beams').value))
        self.safety_bubble_radius = float(self.get_parameter('safety_bubble_radius_m').value)
        self.steering_limit_rad = math.radians(float(self.get_parameter('steering_limit_deg').value))
        self.center_bias_weight = max(0.0, min(1.0, float(self.get_parameter('center_bias_weight').value)))

        self.stop_distance = float(self.get_parameter('stop_distance_m').value)
        self.slow_distance = float(self.get_parameter('slow_distance_m').value)
        self.cruise_distance = float(self.get_parameter('cruise_distance_m').value)

        self.speed_stop = float(self.get_parameter('speed_stop_mps').value)
        self.speed_slow = float(self.get_parameter('speed_slow_mps').value)
        self.speed_medium = float(self.get_parameter('speed_medium_mps').value)
        self.speed_fast = float(self.get_parameter('speed_fast_mps').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gap_angle_pub = self.create_publisher(Float32, self.gap_angle_topic, 10)
        self.target_speed_pub = self.create_publisher(Float32, self.target_speed_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.front_clearance_sub = self.create_subscription(Float32, self.front_clearance_topic, self.clearance_cb, 10)

        self.latest_front_clearance = float('nan')

        self.get_logger().info(
            f"FTG planner listening on {self.scan_topic}, publishing gap angle to {self.gap_angle_topic}"
        )

    def clearance_cb(self, msg: Float32) -> None:
        self.latest_front_clearance = float(msg.data)

    def _bubble_beams(self, scan: LaserScan, ranges: List[float], closest_idx: int) -> List[float]:
        r_closest = ranges[closest_idx]
        if not range_is_valid(r_closest) or r_closest <= 0.0 or scan.angle_increment <= 0.0:
            return ranges[:]

        half_angle = math.atan2(self.safety_bubble_radius, max(r_closest, 1e-3))
        bubble_beams = int(abs(half_angle / scan.angle_increment))

        out = ranges[:]
        for i in range(max(0, closest_idx - bubble_beams), min(len(ranges), closest_idx + bubble_beams + 1)):
            out[i] = 0.0
        return out

    def _find_gaps(self, ranges: List[float]) -> List[Tuple[int, int]]:
        gaps = []
        start = None
        for i, r in enumerate(ranges):
            free = range_is_valid(r) and r >= self.free_space_threshold
            if free and start is None:
                start = i
            elif not free and start is not None:
                if (i - start) >= self.min_gap_beams:
                    gaps.append((start, i - 1))
                start = None
        if start is not None and (len(ranges) - start) >= self.min_gap_beams:
            gaps.append((start, len(ranges) - 1))
        return gaps

    def _select_gap(self, scan: LaserScan, gaps: List[Tuple[int, int]], yaw_base_from_laser: float) -> Optional[Tuple[int, int]]:
        best_gap = None
        best_score = -1e9
        for start, end in gaps:
            mid = (start + end) // 2
            theta_scan = scan.angle_min + mid * scan.angle_increment
            theta_base = wrap_to_pi(yaw_base_from_laser + theta_scan)
            width_score = float(end - start + 1)
            center_penalty = abs(theta_base) / max(self.steering_limit_rad, 1e-3)
            score = width_score - self.center_bias_weight * center_penalty * width_score
            if score > best_score:
                best_score = score
                best_gap = (start, end)
        return best_gap

    def _best_point_in_gap(self, scan: LaserScan, ranges: List[float], gap: Tuple[int, int], yaw_base_from_laser: float) -> Tuple[int, float]:
        start, end = gap
        best_idx = (start + end) // 2
        best_score = -1e9
        for i in range(start, end + 1):
            r = ranges[i]
            if not range_is_valid(r):
                continue
            theta_scan = scan.angle_min + i * scan.angle_increment
            theta_base = wrap_to_pi(yaw_base_from_laser + theta_scan)
            score = r - self.center_bias_weight * abs(theta_base)
            if score > best_score:
                best_score = score
                best_idx = i
        theta_scan = scan.angle_min + best_idx * scan.angle_increment
        theta_base = wrap_to_pi(yaw_base_from_laser + theta_scan)
        theta_base = max(-self.steering_limit_rad, min(self.steering_limit_rad, theta_base))
        return best_idx, theta_base

    def _speed_policy(self, clearance: float) -> float:
        if not math.isfinite(clearance):
            return self.speed_stop
        if clearance <= self.stop_distance:
            return self.speed_stop
        if clearance <= self.slow_distance:
            return self.speed_slow
        if clearance <= self.cruise_distance:
            return self.speed_medium
        return self.speed_fast

    def scan_cb(self, msg: LaserScan) -> None:
        scan_frame = msg.header.frame_id or 'laser'
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, Time())
        except TransformException as exc:
            text = f"[PLANNER] TF lookup failed {self.base_frame} <- {scan_frame}: {exc}"
            self.get_logger().warn(text)
            self.status_pub.publish(String(data=text))
            return

        _, _, yaw_base_from_laser = transform_to_2d(tf_msg)
        ranges = [float(r) for r in msg.ranges]
        valid_indices = [i for i, r in enumerate(ranges) if range_is_valid(r)]

        if not valid_indices:
            text = "[PLANNER] no valid ranges in scan_filtered"
            self.status_pub.publish(String(data=text))
            self.gap_angle_pub.publish(Float32(data=0.0))
            self.target_speed_pub.publish(Float32(data=self.speed_stop))
            return

        closest_idx = min(valid_indices, key=lambda i: ranges[i])
        bubbled = self._bubble_beams(msg, ranges, closest_idx)
        gaps = self._find_gaps(bubbled)

        if not gaps:
            text = "[PLANNER] no valid gap found -> stop"
            self.status_pub.publish(String(data=text))
            self.gap_angle_pub.publish(Float32(data=0.0))
            self.target_speed_pub.publish(Float32(data=self.speed_stop))
            return

        selected_gap = self._select_gap(msg, gaps, yaw_base_from_laser)
        if selected_gap is None:
            text = "[PLANNER] gap selection failed -> stop"
            self.status_pub.publish(String(data=text))
            self.gap_angle_pub.publish(Float32(data=0.0))
            self.target_speed_pub.publish(Float32(data=self.speed_stop))
            return

        best_idx, target_angle = self._best_point_in_gap(msg, bubbled, selected_gap, yaw_base_from_laser)
        target_speed = self._speed_policy(self.latest_front_clearance)

        self.gap_angle_pub.publish(Float32(data=float(target_angle)))
        self.target_speed_pub.publish(Float32(data=float(target_speed)))

        start, end = selected_gap
        text = (
            f"[PLANNER] gap=({start},{end}), best_idx={best_idx}, "
            f"gap_angle={math.degrees(target_angle):+.1f} deg, "
            f"front_clearance={self.latest_front_clearance:.2f} m, "
            f"target_speed={target_speed:.2f} m/s"
        )
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))
        self._publish_markers(msg, best_idx, target_angle, yaw_base_from_laser)

    def _publish_markers(self, msg: LaserScan, best_idx: int, target_angle_base: float, yaw_base_from_laser: float) -> None:
        markers = MarkerArray()

        best_theta_scan = msg.angle_min + best_idx * msg.angle_increment
        best_r = float(msg.ranges[best_idx]) if 0 <= best_idx < len(msg.ranges) else 1.0
        if not range_is_valid(best_r):
            best_r = 1.0

        theta_base = wrap_to_pi(yaw_base_from_laser + best_theta_scan)
        x = best_r * math.cos(theta_base)
        y = best_r * math.sin(theta_base)

        point_marker = Marker()
        point_marker.header.frame_id = self.base_frame
        point_marker.header.stamp = msg.header.stamp
        point_marker.ns = 'ftg_planner'
        point_marker.id = 1
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x = x
        point_marker.pose.position.y = y
        point_marker.pose.position.z = 0.05
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.12
        point_marker.scale.y = 0.12
        point_marker.scale.z = 0.12
        point_marker.color.a = 1.0
        point_marker.color.r = 1.0
        point_marker.color.g = 0.8
        point_marker.color.b = 0.1
        markers.markers.append(point_marker)

        arrow = Marker()
        arrow.header.frame_id = self.base_frame
        arrow.header.stamp = msg.header.stamp
        arrow.ns = 'ftg_planner'
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.05
        arrow.pose.orientation.z = math.sin(target_angle_base / 2.0)
        arrow.pose.orientation.w = math.cos(target_angle_base / 2.0)
        arrow.scale.x = min(max(best_r, 0.40), 1.50)
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.a = 1.0
        arrow.color.r = 0.2
        arrow.color.g = 0.8
        arrow.color.b = 1.0
        markers.markers.append(arrow)

        self.marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = FTGPlannerNode()
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