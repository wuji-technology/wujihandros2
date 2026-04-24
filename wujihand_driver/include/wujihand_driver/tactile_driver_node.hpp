#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <wujihandcpp/device/tactile_board.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wujihand_msgs/msg/tactile_frame.hpp"

namespace wujihand_driver {

class TactileDriverNode : public rclcpp::Node {
public:
    TactileDriverNode();
    ~TactileDriverNode() override;

private:
    void on_frame(const wujihandcpp::TactileFrame& frame);

    // Hardware
    std::unique_ptr<wujihandcpp::TactileBoard> board_;

    // Publishers
    rclcpp::Publisher<wujihand_msgs::msg::TactileFrame>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // Parameters
    std::string serial_number_;
    double image_rate_;
    int pressure_max_;
    std::string frame_id_;

    // Image rate limiting
    int image_skip_;        // publish image every N frames
    std::atomic<uint32_t> frame_counter_{0};
};

}  // namespace wujihand_driver
