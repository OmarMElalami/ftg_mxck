#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, String


class FtgCommandNode(Node):
    """Convert planner-level FTG topics into /autonomous/ackermann_cmd."""

    def __init__(self) -> None:
        super().__init__("ftg_command_node")

        self.declare_parameter("gap_angle_topic", "/autonomous/ftg/gap_angle")
        self.declare_parameter("target_speed_topic", "/autonomous/ftg/target_speed")

        self.declare_parameter("ackermann_cmd_topic", "/autonomous/ackermann_cmd")
        self.declare_parameter("control_status_topic", "/autonomous/ftg/control_status")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("input_timeout_sec", 0.50)
        self.declare_parameter("publish_zero_on_stale", True)

        self.declare_parameter("angle_to_steering_gain", 1.00)
        self.declare_parameter("max_steering_angle_rad", 0.45)

        self.declare_parameter("min_speed_mps", 0.00)
        self.declare_parameter("max_speed_mps", 1.00)

        self.gap_angle_topic = str(self.get_parameter("gap_angle_topic").value)
        self.target_speed_topic = str(self.get_parameter("target_speed_topic").value)

        self.ackermann_cmd_topic = str(self.get_parameter("ackermann_cmd_topic").value)
        self.control_status_topic = str(self.get_parameter("control_status_topic").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.publish_zero_on_stale = bool(self.get_parameter("publish_zero_on_stale").value)

        self.angle_to_steering_gain = float(self.get_parameter("angle_to_steering_gain").value)
        self.max_steering_angle_rad = float(self.get_parameter("max_steering_angle_rad").value)

        self.min_speed_mps = float(self.get_parameter("min_speed_mps").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)

        self.latest_gap_angle = 0.0
        self.latest_target_speed = 0.0
        self.last_gap_angle_stamp = None
        self.last_target_speed_stamp = None

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.ackermann_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.control_status_topic, 10)

        self.create_subscription(Float32, self.gap_angle_topic, self.gap_angle_cb, 10)
        self.create_subscription(Float32, self.target_speed_topic, self.target_speed_cb, 10)

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            f"FTG command node listening on {self.gap_angle_topic} and "
            f"{self.target_speed_topic}, publishing to {self.ackermann_cmd_topic}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def gap_angle_cb(self, msg: Float32) -> None:
        self.latest_gap_angle = float(msg.data)
        self.last_gap_angle_stamp = self._now_sec()

    def target_speed_cb(self, msg: Float32) -> None:
        self.latest_target_speed = float(msg.data)
        self.last_target_speed_stamp = self._now_sec()

    def _publish_cmd(self, speed_mps: float, steering_rad: float, status: str) -> None:
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame_id
        cmd.drive.speed = float(speed_mps)
        cmd.drive.steering_angle = float(steering_rad)
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=status))

    def timer_cb(self) -> None:
        now_s = self._now_sec()

        if self.last_gap_angle_stamp is None or self.last_target_speed_stamp is None:
            if self.publish_zero_on_stale:
                self._publish_cmd(0.0, 0.0, "[CONTROL] waiting for planner inputs")
            return

        if ((now_s - self.last_gap_angle_stamp) > self.input_timeout_sec or
                (now_s - self.last_target_speed_stamp) > self.input_timeout_sec):
            if self.publish_zero_on_stale:
                self._publish_cmd(0.0, 0.0, "[CONTROL] stale planner inputs -> stop")
            return

        steering = self.latest_gap_angle * self.angle_to_steering_gain
        steering = max(-self.max_steering_angle_rad, min(self.max_steering_angle_rad, steering))

        speed = max(self.min_speed_mps, min(self.max_speed_mps, self.latest_target_speed))

        self._publish_cmd(
            speed,
            steering,
            (
                f"[CONTROL] speed={speed:.2f} m/s, "
                f"steering={steering:+.3f} rad"
            ),
        )


def main() -> None:
    rclpy.init()
    node = FtgCommandNode()
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
