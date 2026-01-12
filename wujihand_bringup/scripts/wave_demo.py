#!/usr/bin/env python3
"""
WujiHand Wave Motion Demo

Demonstrates wave motion control:
- Wave motion: fingers move in sinusoidal pattern at 100Hz
- Uses time-based calculation to avoid timer jitter

Usage:
  1. Start the driver: ros2 launch wujihand_bringup wujihand.launch.py
  2. Run this demo: ros2 run wujihand_bringup wave_demo.py
  3. For specific hand: ros2 run wujihand_bringup wave_demo.py --ros-args -p hand_name:=left_hand
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WaveDemo(Node):
    def __init__(self):
        super().__init__("wave_demo")

        # Declare and get hand_name parameter
        self.declare_parameter("hand_name", "hand_0")
        hand_name = self.get_parameter("hand_name").get_parameter_value().string_value

        # Record start time for time-based calculation
        self.start_time = time.perf_counter()

        # Wave parameters
        self.frequency = 0.5  # Hz (one cycle per 2 seconds)
        self.amplitude = 0.8  # radians

        # Pre-allocate message (no names = faster O(n) processing in driver)
        self.msg = JointState()
        self.msg.position = [0.0] * 20

        # Publisher for joint commands (with namespace)
        topic_name = f"/{hand_name}/joint_commands"
        self.cmd_pub = self.create_publisher(JointState, topic_name, 10)

        # Wave motion timer (100Hz)
        self.wave_timer = self.create_timer(0.01, self.wave_callback)

        self.get_logger().info(f"WujiHand Wave Demo Started (100Hz)")
        self.get_logger().info(f"Publishing to: {topic_name}")
        self.get_logger().info("Press Ctrl+C to stop")

    def wave_callback(self):
        # Use elapsed time for smooth motion (immune to timer jitter)
        elapsed = time.perf_counter() - self.start_time
        phase = 2.0 * math.pi * self.frequency * elapsed

        # Sinusoidal wave: y = (1 - cos(phase)) * amplitude
        y = (1.0 - math.cos(phase)) * self.amplitude

        # Thumb (F1): keep still (indices 0-3 stay 0)

        # Index, Middle, Ring, Little (F2-F5): wave motion
        for finger in range(1, 5):
            base = finger * 4
            self.msg.position[base + 0] = y  # joint1
            self.msg.position[base + 2] = y  # joint3
            self.msg.position[base + 3] = y  # joint4

        self.cmd_pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
