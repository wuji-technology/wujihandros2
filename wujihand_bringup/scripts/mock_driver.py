#!/usr/bin/env python3
"""
WujiHand Mock Driver

A simulation driver that mimics the real hardware driver without requiring actual hardware.
Useful for testing, development, and visualization without a physical device.

Usage:
  ros2 run wujihand_bringup mock_driver.py
  ros2 launch wujihand_bringup wujihand_mock.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from wujihand_msgs.msg import HandDiagnostics
from wujihand_msgs.srv import ResetError, SetEnabled


class MockDriver(Node):
    NUM_JOINTS = 20
    NUM_FINGERS = 5
    JOINTS_PER_FINGER = 4

    JOINT_NAMES = [
        "finger1_joint1",
        "finger1_joint2",
        "finger1_joint3",
        "finger1_joint4",
        "finger2_joint1",
        "finger2_joint2",
        "finger2_joint3",
        "finger2_joint4",
        "finger3_joint1",
        "finger3_joint2",
        "finger3_joint3",
        "finger3_joint4",
        "finger4_joint1",
        "finger4_joint2",
        "finger4_joint3",
        "finger4_joint4",
        "finger5_joint1",
        "finger5_joint2",
        "finger5_joint3",
        "finger5_joint4",
    ]

    def __init__(self):
        super().__init__("wujihand_driver")

        # Declare parameters (same as real driver)
        self.declare_parameter("serial_number", "MOCK_DEVICE")
        self.declare_parameter("publish_rate", 1000.0)
        self.declare_parameter("filter_cutoff_freq", 10.0)
        self.declare_parameter("diagnostics_rate", 10.0)
        self.declare_parameter("handedness", "right")
        self.declare_parameter("firmware_version", "0.0.0-mock")

        # Get parameters
        publish_rate = self.get_parameter("publish_rate").value
        diagnostics_rate = self.get_parameter("diagnostics_rate").value
        self.handedness = self.get_parameter("handedness").value

        # Joint state
        self.current_positions = [0.0] * self.NUM_JOINTS
        self.enabled = [True] * self.NUM_JOINTS

        # Create QoS for sensor data (compatible with robot_state_publisher)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, "joint_states", sensor_qos
        )
        self.diagnostics_pub = self.create_publisher(
            HandDiagnostics, "hand_diagnostics", 10
        )

        # Subscriber for commands
        self.cmd_sub = self.create_subscription(
            JointState, "joint_commands", self.command_callback, 10
        )

        # Services
        self.set_enabled_srv = self.create_service(
            SetEnabled, "set_enabled", self.set_enabled_callback
        )
        self.reset_error_srv = self.create_service(
            ResetError, "reset_error", self.reset_error_callback
        )

        # Pre-allocate messages
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = list(self.JOINT_NAMES)
        self.joint_state_msg.position = self.current_positions.copy()

        # Timers
        state_period = 1.0 / publish_rate
        self.state_timer = self.create_timer(state_period, self.publish_state)

        diag_period = 1.0 / diagnostics_rate
        self.diag_timer = self.create_timer(diag_period, self.publish_diagnostics)

        self.get_logger().info(
            f"WujiHand Mock Driver started ({self.handedness}, "
            f"state: {publish_rate:.1f} Hz, diagnostics: {diagnostics_rate:.1f} Hz)"
        )

    def command_callback(self, msg: JointState):
        """Handle incoming joint commands."""
        if msg.name:
            # Named joints - match by name
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in self.JOINT_NAMES:
                    idx = self.JOINT_NAMES.index(name)
                    self.current_positions[idx] = msg.position[i]
        else:
            # Position-only array - use index order
            for i, pos in enumerate(msg.position):
                if i < self.NUM_JOINTS:
                    self.current_positions[i] = pos

    def publish_state(self):
        """Publish current joint states."""
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.current_positions.copy()
        self.joint_state_pub.publish(self.joint_state_msg)

    def publish_diagnostics(self):
        """Publish mock diagnostics."""
        msg = HandDiagnostics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.handedness = self.handedness
        msg.system_temperature = 35.0  # Mock temperature
        msg.input_voltage = 24.0  # Mock voltage
        msg.joint_temperatures = [30.0] * self.NUM_JOINTS
        msg.error_codes = [0] * self.NUM_JOINTS
        msg.enabled = self.enabled.copy()
        self.diagnostics_pub.publish(msg)

    def set_enabled_callback(self, request, response):
        """Handle set_enabled service."""
        finger_id = request.finger_id
        joint_id = request.joint_id
        enabled = request.enabled

        if finger_id == 255 and joint_id == 255:
            # All joints
            self.enabled = [enabled] * self.NUM_JOINTS
            response.success = True
            response.message = (
                "All joints enabled" if enabled else "All joints disabled"
            )
        elif finger_id < self.NUM_FINGERS:
            if joint_id == 255:
                # All joints of a finger
                for j in range(self.JOINTS_PER_FINGER):
                    self.enabled[finger_id * self.JOINTS_PER_FINGER + j] = enabled
                response.success = True
                response.message = "Finger joints updated"
            elif joint_id < self.JOINTS_PER_FINGER:
                # Single joint
                self.enabled[finger_id * self.JOINTS_PER_FINGER + joint_id] = enabled
                response.success = True
                response.message = "Joint updated"
            else:
                response.success = False
                response.message = "Invalid joint_id"
        else:
            response.success = False
            response.message = "Invalid finger_id"

        return response

    def reset_error_callback(self, request, response):
        """Handle reset_error service."""
        # In mock mode, just return success
        response.success = True
        if request.finger_id == 255 and request.joint_id == 255:
            response.message = "All errors reset"
        elif request.finger_id < self.NUM_FINGERS:
            if request.joint_id == 255:
                response.message = "Finger errors reset"
            elif request.joint_id < self.JOINTS_PER_FINGER:
                response.message = "Joint error reset"
            else:
                response.success = False
                response.message = "Invalid joint_id"
        else:
            response.success = False
            response.message = "Invalid finger_id"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
