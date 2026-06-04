#!/usr/bin/env python3
"""WujiHand homing: smoothly interpolate every joint to zero, then exit.

Usage:
  ros2 run wujihand_bringup home.py
  ros2 run wujihand_bringup home.py --ros-args -p hand_name:=left_hand -p duration:=1.5
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState

NUM_JOINTS = 20


def interpolate(start, t):
    """Linear interpolation from start positions toward zero. t clamped to [0, 1]."""
    t = max(0.0, min(1.0, t))
    return [p * (1.0 - t) for p in start]


class HomeNode(Node):
    def __init__(self):
        super().__init__("home")

        self.declare_parameter("hand_name", "hand_0")
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("rate", 100.0)
        self.declare_parameter("timeout", 5.0)

        hand_name = self.get_parameter("hand_name").get_parameter_value().string_value
        self.duration = self.get_parameter("duration").get_parameter_value().double_value
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value

        self.start = None
        self.start_time = None
        self.done_future = Future()

        self.cmd_pub = self.create_publisher(
            JointState, f"/{hand_name}/joint_commands", qos_profile_sensor_data
        )
        self.state_sub = self.create_subscription(
            JointState, f"/{hand_name}/joint_states", self._on_state, qos_profile_sensor_data
        )

        self.deadline = self.get_clock().now() + Duration(seconds=timeout)
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)
        self.get_logger().info(
            f"Homing {hand_name} over {self.duration:.1f}s; waiting for joint_states..."
        )

    def _on_state(self, msg):
        if self.start is None and len(msg.position) >= NUM_JOINTS:
            self.start = list(msg.position[:NUM_JOINTS])
            self.start_time = self.get_clock().now()
            self.get_logger().info("Got initial positions; moving to zero.")

    def _on_timer(self):
        if self.done_future.done():
            return

        now = self.get_clock().now()
        if self.start is None:
            if now >= self.deadline:
                self.get_logger().error(
                    "No joint_states before timeout; is the driver running?"
                )
                self.done_future.set_result(False)
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9
        t = 1.0 if self.duration <= 0.0 else elapsed / self.duration

        msg = JointState()
        msg.position = interpolate(self.start, t)
        self.cmd_pub.publish(msg)

        if t >= 1.0:
            self.get_logger().info("Reached zero pose; exiting.")
            self.done_future.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    node = HomeNode()
    try:
        rclpy.spin_until_future_complete(node, node.done_future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
