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

    // Services (lifecycle / config knobs from spec §3.3 + §3.4)
    rclcpp::Service<wujihand_msgs::srv::SetTactileStreaming>::SharedPtr svc_streaming_;
    rclcpp::Service<wujihand_msgs::srv::SetTactileSampleRate>::SharedPtr svc_rate_;
    rclcpp::Service<wujihand_msgs::srv::ResetTactileCounters>::SharedPtr svc_reset_;

    // Timer driving the diagnostics topic.
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Parameters
    std::string serial_number_;
    double image_rate_;
    int sample_rate_hz_;
    bool streaming_at_startup_;
    std::string frame_id_;

    // Image rate limiting
    int image_skip_;        // publish image every N frames
    std::atomic<uint32_t> frame_counter_{0};
};

}  // namespace wujihand_driver
