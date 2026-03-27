#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from ackermann_msgs.msg import AckermannDriveStamped


class FTGCommandNode(Node):
    def __init__(self) -> None:
        super().__init__('ftg_command_node')

        self.declare_parameter('gap_angle_topic', '/autonomous/ftg/gap_angle')
        self.declare_parameter('target_speed_topic', '/autonomous/ftg/target_speed')

        self.declare_parameter('ackermann_topic', '/autonomous/ackermann_cmd')
        self.declare_parameter('status_topic', '/autonomous/ftg/control_status')
        self.declare_parameter('frame_id', 'base_link')

        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('command_timeout_sec', 0.5)

        self.declare_parameter('invert_steering', False)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('steering_limit_deg', 22.0)
        self.declare_parameter('enable_steering_smoothing', True)
        self.declare_parameter('steering_smoothing_alpha', 0.35)

        self.declare_parameter('speed_limit_mps', 0.35)
        self.declare_parameter('stop_on_timeout', True)

        self.declare_parameter('enable_turn_speed_scaling', True)
        self.declare_parameter('slowdown_start_deg', 10.0)
        self.declare_parameter('slowdown_full_deg', 22.0)
        self.declare_parameter('min_turn_speed_mps', 0.15)

        self.gap_angle_topic = str(self.get_parameter('gap_angle_topic').value)
        self.target_speed_topic = str(self.get_parameter('target_speed_topic').value)

        self.ackermann_topic = str(self.get_parameter('ackermann_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.steering_limit_rad = math.radians(float(self.get_parameter('steering_limit_deg').value))
        self.enable_steering_smoothing = bool(self.get_parameter('enable_steering_smoothing').value)
        self.steering_smoothing_alpha = float(self.get_parameter('steering_smoothing_alpha').value)

        self.speed_limit_mps = float(self.get_parameter('speed_limit_mps').value)
        self.stop_on_timeout = bool(self.get_parameter('stop_on_timeout').value)

        self.enable_turn_speed_scaling = bool(self.get_parameter('enable_turn_speed_scaling').value)
        self.slowdown_start_rad = math.radians(float(self.get_parameter('slowdown_start_deg').value))
        self.slowdown_full_rad = math.radians(float(self.get_parameter('slowdown_full_deg').value))
        self.min_turn_speed_mps = float(self.get_parameter('min_turn_speed_mps').value)

        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.gap_angle_sub = self.create_subscription(Float32, self.gap_angle_topic, self.gap_angle_cb, 10)
        self.target_speed_sub = self.create_subscription(Float32, self.target_speed_topic, self.target_speed_cb, 10)

        self.latest_gap_angle = 0.0
        self.latest_target_speed = 0.0
        self.last_gap_time = None
        self.last_speed_time = None
        self.filtered_steering = 0.0

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f"FTG command node listening on {self.gap_angle_topic} and {self.target_speed_topic}, "
            f"publishing to {self.ackermann_topic}"
        )

    def gap_angle_cb(self, msg: Float32) -> None:
        self.latest_gap_angle = float(msg.data)
        self.last_gap_time = self.get_clock().now()

    def target_speed_cb(self, msg: Float32) -> None:
        self.latest_target_speed = float(msg.data)
        self.last_speed_time = self.get_clock().now()

    def _is_fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= self.command_timeout_sec

    def _apply_turn_speed_scaling(self, speed: float, steering: float) -> float:
        if not self.enable_turn_speed_scaling:
            return speed

        a = abs(steering)

        if a <= self.slowdown_start_rad:
            return speed

        if a >= self.slowdown_full_rad:
            return min(speed, self.min_turn_speed_mps)

        ratio = (a - self.slowdown_start_rad) / max(self.slowdown_full_rad - self.slowdown_start_rad, 1e-6)
        scaled_limit = (1.0 - ratio) * speed + ratio * self.min_turn_speed_mps
        return min(speed, scaled_limit)

    def timer_cb(self) -> None:
        gap_fresh = self._is_fresh(self.last_gap_time)
        speed_fresh = self._is_fresh(self.last_speed_time)

        steering = float(self.latest_gap_angle)
        if self.invert_steering:
            steering = -steering

        steering *= self.steering_gain
        steering = max(-self.steering_limit_rad, min(self.steering_limit_rad, steering))

        if self.enable_steering_smoothing:
            alpha = max(0.0, min(1.0, self.steering_smoothing_alpha))
            self.filtered_steering = alpha * steering + (1.0 - alpha) * self.filtered_steering
        else:
            self.filtered_steering = steering

        speed = min(max(self.latest_target_speed, 0.0), self.speed_limit_mps)
        speed = self._apply_turn_speed_scaling(speed, self.filtered_steering)

        timeout = not (gap_fresh and speed_fresh)
        if timeout and self.stop_on_timeout:
            speed = 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = float(self.filtered_steering)
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = float(speed)
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.ackermann_pub.publish(msg)

        status = (
            f"[CONTROL] steering={math.degrees(self.filtered_steering):+.1f} deg, "
            f"speed={speed:.2f} m/s, fresh_gap={gap_fresh}, fresh_speed={speed_fresh}, timeout={timeout}"
        )
        self.status_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = FTGCommandNode()
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
