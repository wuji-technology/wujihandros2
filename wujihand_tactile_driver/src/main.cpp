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

#include "rclcpp/rclcpp.hpp"
#include "wujihand_tactile_driver/tactile_driver_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<wujihand_tactile_driver::TactileDriverNode>();
    // Multi-threaded executor so the 100 ms diagnostics timer (which
    // issues a 2-second-timeout SDK command) cannot starve service
    // callbacks. The node assigns the timer and the three services to
    // distinct mutually-exclusive callback groups.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("tactile_driver"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
