#!/usr/bin/env python3
"""
WujiHand Wave Motion Demo

Demonstrates wave motion control with fingers moving in a sinusoidal pattern.
Uses a dedicated thread for consistent 100Hz publishing rate.

Usage:
  ros2 run wujihand_bringup wave_demo.py
  ros2 run wujihand_bringup wave_demo.py --ros-args -p hand_name:=left_hand
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# Constants
NUM_JOINTS = 20
PUBLISH_RATE = 100.0  # Hz
WAVE_FREQUENCY = 0.5  # Hz (one cycle per 2 seconds)
WAVE_AMPLITUDE = 0.8  # radians


class WaveDemo(Node):
    def __init__(self):
        super().__init__("wave_demo")

        # Parameters
        self.declare_parameter("hand_name", "hand_0")
        hand_name = self.get_parameter("hand_name").get_parameter_value().string_value

        # Publisher
        topic_name = f"/{hand_name}/joint_commands"
        self.cmd_pub = self.create_publisher(JointState, topic_name, qos_profile_sensor_data)

        # Pre-allocate message (no names = faster processing in driver)
        self.msg = JointState()
        self.msg.position = [0.0] * NUM_JOINTS

        # Thread control
        self.running = True
        self.pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.pub_thread.start()

        self.get_logger().info(f"Wave demo started ({PUBLISH_RATE:.0f}Hz) -> {topic_name}")

    def _publish_loop(self):
        """Dedicated thread for consistent rate publishing."""
        start_time = time.perf_counter()
        period = 1.0 / PUBLISH_RATE
        next_time = start_time + period
        last_pub_time = start_time
        max_gap = 0.0
        anomaly_count = 0

        while self.running:
            now = time.perf_counter()
            gap = now - last_pub_time
            if gap > max_gap:
                max_gap = gap
            # Count anomalies (gap > 20ms)
            if gap > period * 2:
                anomaly_count += 1
                self.get_logger().warn(f"Gap {gap*1000:.1f}ms detected (count: {anomaly_count})")
            last_pub_time = now

            elapsed = now - start_time
            phase = 2.0 * math.pi * WAVE_FREQUENCY * elapsed
            y = (1.0 - math.cos(phase)) * WAVE_AMPLITUDE

            # F2-F5 wave motion (skip F1 thumb)
            for finger in range(1, 5):
                base = finger * 4
                self.msg.position[base + 0] = y  # MCP
                self.msg.position[base + 2] = y  # PIP
                self.msg.position[base + 3] = y  # DIP

            self.cmd_pub.publish(self.msg)

            # Precise timing
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_time += period

    def destroy_node(self):
        self.running = False
        self.pub_thread.join(timeout=1.0)
        super().destroy_node()


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
