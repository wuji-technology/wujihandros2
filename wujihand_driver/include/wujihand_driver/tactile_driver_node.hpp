#pragma once

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
    // tactile_main.cpp. Without this, the 100 ms diagnostics timer
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

    // Parameters (immutable after construction).
    std::string serial_number_;
    double image_rate_;
    bool streaming_at_startup_;
    std::string frame_id_;

    // Mutable runtime state shared between the SDK streaming thread
    // (on_frame) and ROS service callbacks (set_sample_rate). Both fields
    // are atomic to avoid a data race when the rate is changed mid-stream.
    std::atomic<int> sample_rate_hz_{120};
    std::atomic<int> image_skip_{1};
    std::atomic<uint32_t> frame_counter_{0};

    // Diagnostics-poll backoff. consecutive_failures counts how many
    // consecutive diag attempts threw (incremented at most once per tick,
    // in the catch handler). backoff_tick is a separate phase counter that
    // is incremented exactly once per skipped tick once we are over the
    // failure threshold — keeping the two distinct avoids the
    // double-increment bug where a single failure would advance the backoff
    // phase by 2 (one in the pre-skip branch, one in the catch).
    std::atomic<int> diag_consecutive_failures_{0};
    std::atomic<int> diag_backoff_tick_{0};
};

}  // namespace wujihand_driver
