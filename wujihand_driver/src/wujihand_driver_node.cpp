// Copyright 2025 Wuji Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "wujihand_driver/wujihand_driver_node.hpp"

#include <chrono>
#include <functional>

namespace wujihand_driver {

const std::array<std::string, WujiHandDriverNode::NUM_JOINTS> WujiHandDriverNode::JOINT_NAMES = {
    "finger1_joint1", "finger1_joint2", "finger1_joint3", "finger1_joint4", "finger2_joint1",
    "finger2_joint2", "finger2_joint3", "finger2_joint4", "finger3_joint1", "finger3_joint2",
    "finger3_joint3", "finger3_joint4", "finger4_joint1", "finger4_joint2", "finger4_joint3",
    "finger4_joint4", "finger5_joint1", "finger5_joint2", "finger5_joint3", "finger5_joint4"};

WujiHandDriverNode::WujiHandDriverNode() : Node("wujihand_driver"), hardware_connected_(false) {
  // Declare parameters
  this->declare_parameter("serial_number", "");
  this->declare_parameter("publish_rate", 1000.0);
  this->declare_parameter("filter_cutoff_freq", 10.0);
  this->declare_parameter("diagnostics_rate", 10.0);

  // Get parameters
  serial_number_ = this->get_parameter("serial_number").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  filter_cutoff_freq_ = this->get_parameter("filter_cutoff_freq").as_double();
  diagnostics_rate_ = this->get_parameter("diagnostics_rate").as_double();

  // Initialize last target positions
  last_target_positions_.fill(0.0);

  // Declare read-only parameters for hardware info (must be declared before connect_hardware)
  this->declare_parameter("handedness", std::string(""));
  this->declare_parameter("firmware_version", std::string(""));
  this->declare_parameter("joint_upper_limits", std::vector<double>(NUM_JOINTS, 0.0));
  this->declare_parameter("joint_lower_limits", std::vector<double>(NUM_JOINTS, 0.0));

  // Connect to hardware
  if (!connect_hardware()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to WujiHand hardware");
    throw std::runtime_error("Hardware connection failed");
  }

  // Create publishers (use SensorDataQoS for compatibility with robot_state_publisher)
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  diagnostics_pub_ =
      this->create_publisher<wujihand_msgs::msg::HandDiagnostics>("hand_diagnostics", 10);

  // Create subscriber for joint commands (using SensorDataQoS for high-frequency data)
  cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_commands", rclcpp::SensorDataQoS(),
      std::bind(&WujiHandDriverNode::command_callback, this, std::placeholders::_1));

  // Create services
  set_enabled_srv_ = this->create_service<wujihand_msgs::srv::SetEnabled>(
      "set_enabled", std::bind(&WujiHandDriverNode::set_enabled_callback, this,
                               std::placeholders::_1, std::placeholders::_2));

  reset_error_srv_ = this->create_service<wujihand_msgs::srv::ResetError>(
      "reset_error", std::bind(&WujiHandDriverNode::reset_error_callback, this,
                               std::placeholders::_1, std::placeholders::_2));

  // Initialize pre-allocated JointState message with handedness prefix
  joint_state_msg_.name.reserve(NUM_JOINTS);
  joint_state_msg_.position.resize(NUM_JOINTS, 0.0);
  joint_state_msg_.effort.resize(NUM_JOINTS, 0.0);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_state_msg_.name.push_back(handedness_ + "_" + JOINT_NAMES[i]);
  }

  // Create state publish timer (high frequency)
  auto state_period = std::chrono::duration<double>(1.0 / publish_rate_);
  state_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(state_period),
                              std::bind(&WujiHandDriverNode::publish_state, this));

  // Create diagnostics publish timer (low frequency)
  auto diag_period = std::chrono::duration<double>(1.0 / diagnostics_rate_);
  diagnostics_timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(diag_period),
                              std::bind(&WujiHandDriverNode::publish_diagnostics, this));

  RCLCPP_INFO(this->get_logger(), "WujiHand driver started (state: %.1f Hz, diagnostics: %.1f Hz)",
              publish_rate_, diagnostics_rate_);
}

WujiHandDriverNode::~WujiHandDriverNode() {
  disconnect_hardware();
}

bool WujiHandDriverNode::connect_hardware() {
  try {
    const char* serial = serial_number_.empty() ? nullptr : serial_number_.c_str();
    hand_ = std::make_unique<wujihandcpp::device::Hand>(serial);

    // Create realtime controller with filter
    wujihandcpp::filter::LowPass filter(filter_cutoff_freq_);
    controller_ = hand_->realtime_controller<true>(filter);

    // Disable thread safety check for multi-threaded access
    hand_->disable_thread_safe_check();

    // Enable all joints
    hand_->write<wujihandcpp::data::joint::Enabled>(true);

    // Read handedness (0 = right, 1 = left)
    auto handedness_value = hand_->read<wujihandcpp::data::hand::Handedness>();
    handedness_ = (handedness_value == 0) ? "right" : "left";

    // Read full system firmware version (unified version for the entire system)
    auto version = hand_->read<wujihandcpp::data::hand::FullSystemFirmwareVersion>();
    wujihandcpp::data::FirmwareVersionData version_data(version);
    firmware_version_ = version_data.to_string();

    // Set joint limits (placeholder values - could be read from hardware in future)
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      joint_upper_limits_[i] = 1.57;  // ~90 degrees
      joint_lower_limits_[i] = 0.0;
    }
    // Abduction joints have different limits
    for (size_t f = 0; f < NUM_FINGERS; ++f) {
      size_t abd_idx = f * JOINTS_PER_FINGER;
      joint_upper_limits_[abd_idx] = 0.5;
      joint_lower_limits_[abd_idx] = -0.5;
    }

    // Read initial positions from hardware (not from controller cache)
    hand_->read<wujihandcpp::data::joint::ActualPosition>();
    double initial_positions[NUM_FINGERS][JOINTS_PER_FINGER];
    for (size_t f = 0; f < NUM_FINGERS; ++f) {
      for (size_t j = 0; j < JOINTS_PER_FINGER; ++j) {
        auto pos = hand_->finger(f).joint(j).get<wujihandcpp::data::joint::ActualPosition>();
        initial_positions[f][j] = pos;
        last_target_positions_[to_flat_index(f, j)] = pos;
      }
    }

    // Send initial target position to start realtime communication
    controller_->set_joint_target_position(initial_positions);

    // Update ROS parameters with hardware info so other nodes can query them
    this->set_parameter(rclcpp::Parameter("handedness", handedness_));
    this->set_parameter(rclcpp::Parameter("firmware_version", firmware_version_));
    this->set_parameter(rclcpp::Parameter("joint_upper_limits",
        std::vector<double>(joint_upper_limits_.begin(), joint_upper_limits_.end())));
    this->set_parameter(rclcpp::Parameter("joint_lower_limits",
        std::vector<double>(joint_lower_limits_.begin(), joint_lower_limits_.end())));

    hardware_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to WujiHand (%s)", handedness_.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", e.what());
    return false;
  }
}

void WujiHandDriverNode::disconnect_hardware() {
  if (hardware_connected_ && hand_) {
    try {
      // Disable all joints before disconnecting
      hand_->write<wujihandcpp::data::joint::Enabled>(false);
      RCLCPP_INFO(this->get_logger(), "Disconnected from WujiHand");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Error during disconnect: %s", e.what());
    }
  }
  controller_.reset();
  hand_.reset();
  hardware_connected_ = false;
}

void WujiHandDriverNode::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!hardware_connected_ || !controller_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Cannot send command: hardware not connected");
    return;
  }

  // Build position array from JointState message
  // Support both named joints and position-only arrays
  double positions[NUM_FINGERS][JOINTS_PER_FINGER] = {};

  if (!msg->name.empty()) {
    // Named joints - match by name (support both with and without handedness prefix)
    std::string prefix = handedness_ + "_";
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      for (size_t j = 0; j < NUM_JOINTS; ++j) {
        // Match either "finger1_joint1" or "right_finger1_joint1"
        if (msg->name[i] == JOINT_NAMES[j] || msg->name[i] == prefix + JOINT_NAMES[j]) {
          size_t f = j / JOINTS_PER_FINGER;
          size_t jj = j % JOINTS_PER_FINGER;
          positions[f][jj] = msg->position[i];
          last_target_positions_[j] = msg->position[i];
          break;
        }
      }
    }
  } else {
    // Position-only array - use index order
    for (size_t i = 0; i < msg->position.size() && i < NUM_JOINTS; ++i) {
      size_t f = i / JOINTS_PER_FINGER;
      size_t j = i % JOINTS_PER_FINGER;
      positions[f][j] = msg->position[i];
      last_target_positions_[i] = msg->position[i];
    }
  }

  // Send to hardware
  controller_->set_joint_target_position(positions);
}

void WujiHandDriverNode::publish_state() {
  if (!hardware_connected_ || !controller_) {
    return;
  }

  auto now = this->now();

  // Get actual positions and efforts from realtime controller
  // SDK 1.4.0+: TPDO proactively reports actual positions from hardware
  const auto& positions = controller_->get_joint_actual_position();
  const auto& efforts = controller_->get_joint_actual_effort();

  // Build and publish JointState message
  joint_state_msg_.header.stamp = now;
  for (size_t f = 0; f < NUM_FINGERS; ++f) {
    for (size_t j = 0; j < JOINTS_PER_FINGER; ++j) {
      size_t idx = to_flat_index(f, j);
      joint_state_msg_.position[idx] = positions[f][j].load();
      joint_state_msg_.effort[idx] = efforts[f][j].load();
    }
  }

  joint_state_pub_->publish(joint_state_msg_);
}

void WujiHandDriverNode::publish_diagnostics() {
  if (!hardware_connected_ || !hand_) {
    return;
  }

  // Skip if previous read is still in progress
  if (diagnostics_busy_) {
    return;
  }

  diagnostics_busy_ = true;

  // Read diagnostics in a separate thread to avoid blocking the main loop
  std::thread([this]() {
    try {
      wujihand_msgs::msg::HandDiagnostics msg;
      msg.header.stamp = this->now();
      msg.handedness = handedness_;

      // Use mutex to ensure thread-safe hardware access
      {
        std::lock_guard<std::mutex> lock(hardware_mutex_);
        msg.system_temperature = hand_->read<wujihandcpp::data::hand::Temperature>();
        msg.input_voltage = hand_->read<wujihandcpp::data::hand::InputVoltage>();

        for (size_t f = 0; f < NUM_FINGERS; ++f) {
          for (size_t j = 0; j < JOINTS_PER_FINGER; ++j) {
            size_t idx = to_flat_index(f, j);
            auto joint = hand_->finger(f).joint(j);
            msg.joint_temperatures[idx] = joint.read<wujihandcpp::data::joint::Temperature>();
            msg.error_codes[idx] = joint.read<wujihandcpp::data::joint::ErrorCode>();
            msg.enabled[idx] = (msg.error_codes[idx] == 0);
            msg.effort_limits[idx] = joint.read<wujihandcpp::data::joint::EffortLimit>();
          }
        }
      }

      diagnostics_pub_->publish(msg);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Error reading diagnostics: %s", e.what());
    }
    diagnostics_busy_ = false;
  }).detach();
}

void WujiHandDriverNode::set_enabled_callback(
    const std::shared_ptr<wujihand_msgs::srv::SetEnabled::Request> request,
    std::shared_ptr<wujihand_msgs::srv::SetEnabled::Response> response) {
  if (!hardware_connected_ || !hand_) {
    response->success = false;
    response->message = "Hardware not connected";
    return;
  }

  try {
    if (request->finger_id == 255 && request->joint_id == 255) {
      hand_->write<wujihandcpp::data::joint::Enabled>(request->enabled);
      response->success = true;
      response->message = request->enabled ? "All joints enabled" : "All joints disabled";
    } else if (request->finger_id < NUM_FINGERS) {
      if (request->joint_id == 255) {
        for (size_t j = 0; j < JOINTS_PER_FINGER; ++j) {
          hand_->finger(request->finger_id)
              .joint(j)
              .write<wujihandcpp::data::joint::Enabled>(request->enabled);
        }
        response->success = true;
        response->message = "Finger joints updated";
      } else if (request->joint_id < JOINTS_PER_FINGER) {
        hand_->finger(request->finger_id)
            .joint(request->joint_id)
            .write<wujihandcpp::data::joint::Enabled>(request->enabled);
        response->success = true;
        response->message = "Joint updated";
      } else {
        response->success = false;
        response->message = "Invalid joint_id";
      }
    } else {
      response->success = false;
      response->message = "Invalid finger_id";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error: ") + e.what();
  }
}

void WujiHandDriverNode::reset_error_callback(
    const std::shared_ptr<wujihand_msgs::srv::ResetError::Request> request,
    std::shared_ptr<wujihand_msgs::srv::ResetError::Response> response) {
  if (!hardware_connected_ || !hand_) {
    response->success = false;
    response->message = "Hardware not connected";
    return;
  }

  try {
    if (request->finger_id == 255 && request->joint_id == 255) {
      hand_->write<wujihandcpp::data::joint::ResetError>(1);
      response->success = true;
      response->message = "All errors reset";
    } else if (request->finger_id < NUM_FINGERS) {
      if (request->joint_id == 255) {
        for (size_t j = 0; j < JOINTS_PER_FINGER; ++j) {
          hand_->finger(request->finger_id).joint(j).write<wujihandcpp::data::joint::ResetError>(1);
        }
        response->success = true;
        response->message = "Finger errors reset";
      } else if (request->joint_id < JOINTS_PER_FINGER) {
        hand_->finger(request->finger_id)
            .joint(request->joint_id)
            .write<wujihandcpp::data::joint::ResetError>(1);
        response->success = true;
        response->message = "Joint error reset";
      } else {
        response->success = false;
        response->message = "Invalid joint_id";
      }
    } else {
      response->success = false;
      response->message = "Invalid finger_id";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error: ") + e.what();
  }
}

size_t WujiHandDriverNode::to_flat_index(size_t finger_id, size_t joint_id) {
  return finger_id * JOINTS_PER_FINGER + joint_id;
}

}  // namespace wujihand_driver
