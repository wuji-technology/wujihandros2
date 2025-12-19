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

#ifndef WUJIHAND_DRIVER__WUJIHAND_DRIVER_NODE_HPP_
#define WUJIHAND_DRIVER__WUJIHAND_DRIVER_NODE_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <wujihandcpp/data/hand.hpp>
#include <wujihandcpp/data/joint.hpp>
#include <wujihandcpp/device/controller.hpp>
#include <wujihandcpp/device/hand.hpp>
#include <wujihandcpp/filter/low_pass.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "wujihand_msgs/msg/hand_diagnostics.hpp"
#include "wujihand_msgs/srv/reset_error.hpp"
#include "wujihand_msgs/srv/set_enabled.hpp"

namespace wujihand_driver {

class WujiHandDriverNode : public rclcpp::Node {
 public:
  static constexpr size_t NUM_JOINTS = 20;
  static constexpr size_t NUM_FINGERS = 5;
  static constexpr size_t JOINTS_PER_FINGER = 4;

  WujiHandDriverNode();
  ~WujiHandDriverNode();

 private:
  // Joint names for JointState message
  static const std::array<std::string, NUM_JOINTS> JOINT_NAMES;

  // Hardware connection
  std::unique_ptr<wujihandcpp::device::Hand> hand_;
  std::unique_ptr<wujihandcpp::device::IController> controller_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<wujihand_msgs::msg::HandDiagnostics>::SharedPtr diagnostics_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

  // Services
  rclcpp::Service<wujihand_msgs::srv::SetEnabled>::SharedPtr set_enabled_srv_;
  rclcpp::Service<wujihand_msgs::srv::ResetError>::SharedPtr reset_error_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Parameters
  std::string serial_number_;
  std::string joint_prefix_;
  double publish_rate_;
  double filter_cutoff_freq_;
  double diagnostics_rate_;

  // Cached state
  std::array<double, NUM_JOINTS> last_target_positions_;
  std::array<double, NUM_JOINTS> joint_upper_limits_;
  std::array<double, NUM_JOINTS> joint_lower_limits_;
  bool hardware_connected_;
  std::string handedness_;
  std::string firmware_version_;
  std::atomic<bool> diagnostics_busy_{false};
  std::mutex hardware_mutex_;  // Mutex for thread-safe hardware access

  // Pre-allocated messages for efficiency
  sensor_msgs::msg::JointState joint_state_msg_;

  // Callbacks
  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_state();
  void publish_diagnostics();

  // Service callbacks
  void set_enabled_callback(const std::shared_ptr<wujihand_msgs::srv::SetEnabled::Request> request,
                            std::shared_ptr<wujihand_msgs::srv::SetEnabled::Response> response);
  void reset_error_callback(const std::shared_ptr<wujihand_msgs::srv::ResetError::Request> request,
                            std::shared_ptr<wujihand_msgs::srv::ResetError::Response> response);

  // Helper functions
  bool connect_hardware();
  void disconnect_hardware();
  static size_t to_flat_index(size_t finger_id, size_t joint_id);
};

}  // namespace wujihand_driver

#endif  // WUJIHAND_DRIVER__WUJIHAND_DRIVER_NODE_HPP_
