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

#ifndef WUJIHAND_DRIVER__TACTILE_DRIVER_NODE_HPP_
#define WUJIHAND_DRIVER__TACTILE_DRIVER_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <wujihandcpp/device/tactile_board.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wujihand_msgs/msg/tactile_diagnostics.hpp"
#include "wujihand_msgs/msg/tactile_frame.hpp"
#include "wujihand_msgs/srv/reset_tactile_counters.hpp"
#include "wujihand_msgs/srv/set_tactile_sample_rate.hpp"
#include "wujihand_msgs/srv/set_tactile_streaming.hpp"

namespace wujihand_driver {

class TactileDriverNode : public rclcpp::Node {
 public:
  TactileDriverNode();
  ~TactileDriverNode() override;

 private:
  void on_frame(const wujihandcpp::TactileFrame& frame);
  void publish_diagnostics();

  // Hardware
  std::unique_ptr<wujihandcpp::TactileBoard> board_;

  // Publishers
  rclcpp::Publisher<wujihand_msgs::msg::TactileFrame>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<wujihand_msgs::msg::TactileDiagnostics>::SharedPtr diag_pub_;

  // Each long-running callback gets its own MutuallyExclusive group so
  // they run in parallel under the MultiThreadedExecutor used in
  // tactile_driver_main.cpp. Without this, the 100 ms diagnostics timer
  // (issuing a 2-second-timeout SDK command) would starve the services.
  rclcpp::CallbackGroup::SharedPtr cb_group_diag_;
  rclcpp::CallbackGroup::SharedPtr cb_group_streaming_;
  rclcpp::CallbackGroup::SharedPtr cb_group_rate_;
  rclcpp::CallbackGroup::SharedPtr cb_group_reset_;

  // Services (lifecycle / config knobs from spec §3.3 + §3.4)
  rclcpp::Service<wujihand_msgs::srv::SetTactileStreaming>::SharedPtr svc_streaming_;
  rclcpp::Service<wujihand_msgs::srv::SetTactileSampleRate>::SharedPtr svc_rate_;
  rclcpp::Service<wujihand_msgs::srv::ResetTactileCounters>::SharedPtr svc_reset_;

  // Timer driving the diagnostics topic.
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Parameters that survive past construction (the rest are consumed in
  // the constructor and live only as locals).
  double image_rate_;
  std::string frame_id_;

  // Image-publish skip ratio. Read by on_frame on the SDK streaming
  // thread; written by the set_sample_rate service callback on a
  // different thread → atomic.
  std::atomic<int> image_skip_{1};

  // Frame counter for image downsampling. Only touched by on_frame on
  // the single SDK streaming consumer thread.
  uint32_t frame_counter_{0};

  // Diagnostics-poll backoff. publish_diagnostics is the only writer/
  // reader and runs on its own MutuallyExclusive callback group, so
  // these are single-threaded. The two counters are kept distinct
  // (failures vs. backoff phase) so a single failure cannot double-
  // increment the phase.
  int diag_consecutive_failures_{0};
  int diag_backoff_tick_{0};
};

}  // namespace wujihand_driver

#endif  // WUJIHAND_DRIVER__TACTILE_DRIVER_NODE_HPP_
