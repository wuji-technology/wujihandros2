#include "wujihand_driver/tactile_driver_node.hpp"
#include "wujihand_driver/colormap.hpp"

#include <algorithm>
#include <cmath>

namespace wujihand_driver {

TactileDriverNode::TactileDriverNode()
    : Node("tactile_driver_node") {

    // Declare parameters
    this->declare_parameter("serial_number", "");
    this->declare_parameter("image_rate", 30.0);
    this->declare_parameter("pressure_max", 2135);
    this->declare_parameter("frame_id", "tactile_sensor_link");

    serial_number_ = this->get_parameter("serial_number").as_string();
    image_rate_ = this->get_parameter("image_rate").as_double();
    pressure_max_ = this->get_parameter("pressure_max").as_int();
    if (pressure_max_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "pressure_max must be positive, using default 2135");
        pressure_max_ = 2135;
    }
    if (image_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "image_rate must be positive, using default 30.0");
        image_rate_ = 30.0;
    }
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Compute how many raw frames to skip between image publishes.
    // Raw runs at 120 FPS; image_rate_ defaults to 30 → skip every 4th frame.
    image_skip_ = std::max(1, static_cast<int>(std::round(120.0 / image_rate_)));

    // Create publishers
    raw_pub_ = this->create_publisher<wujihand_msgs::msg::TactileFrame>(
        "tactile/raw", rclcpp::SensorDataQoS());
    // Use default QoS (Reliable) for image so RViz2 can subscribe without
    // QoS mismatch. The image is only 24x32 @ 30Hz (~7KB/s), so Reliable
    // has no performance impact. Raw frames stay Best Effort (120Hz).
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "tactile/image", 10);

    // Connect to tactile board
    const char* sn = serial_number_.empty() ? nullptr : serial_number_.c_str();
    board_ = std::make_unique<wujihandcpp::TactileBoard>(sn);

    if (!board_->connect()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to connect to tactile board");
        throw std::runtime_error("Tactile board connection failed");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to tactile board at 120 FPS");

    // Start streaming (push mode: callback drives publishing, no timer)
    board_->start_streaming(
        [this](const wujihandcpp::TactileFrame& frame) {
            this->on_frame(frame);
        });
}

TactileDriverNode::~TactileDriverNode() {
    // stop_streaming() is synchronous: it sets stop_requested, then joins the
    // reader thread. After join() returns, no more on_frame() callbacks can fire,
    // so it is safe to destroy publishers after this call.
    if (board_) {
        board_->stop_streaming();
    }
}

void TactileDriverNode::on_frame(const wujihandcpp::TactileFrame& frame) {
    // Detect disconnect (zero-initialized frame with crc_valid=false)
    if (!frame.crc_valid && frame.sequence == 0 && frame.timestamp_ms == 0) {
        RCLCPP_ERROR(this->get_logger(), "Tactile board disconnected");
        return;
    }

    auto now = this->get_clock()->now();

    // --- Publish raw TactileFrame (every frame, 120 Hz) ---
    auto raw_msg = wujihand_msgs::msg::TactileFrame();
    raw_msg.header.stamp = now;
    raw_msg.header.frame_id = frame_id_;
    raw_msg.hand = static_cast<uint8_t>(frame.hand);
    raw_msg.sequence = frame.sequence;
    raw_msg.device_timestamp = frame.timestamp_ms;

    // Copy pressure matrix (24x32 → flat 768)
    for (int r = 0; r < 24; ++r) {
        for (int c = 0; c < 32; ++c) {
            raw_msg.pressure[r * 32 + c] = frame.pressure[r][c];
        }
    }
    raw_pub_->publish(raw_msg);

    // --- Publish Image (every N frames, ~30 Hz) ---
    uint32_t count = frame_counter_.fetch_add(1, std::memory_order_relaxed);
    if (count % image_skip_ != 0) return;

    auto img_msg = sensor_msgs::msg::Image();
    img_msg.header.stamp = now;
    img_msg.header.frame_id = frame_id_;
    img_msg.height = 24;
    img_msg.width = 32;
    img_msg.encoding = "rgb8";
    img_msg.is_bigendian = false;
    img_msg.step = 32 * 3;
    img_msg.data.resize(24 * 32 * 3);

    const int pmax = pressure_max_;
    for (int r = 0; r < 24; ++r) {
        for (int c = 0; c < 32; ++c) {
            // Map pressure to colormap index.
            // Lower pressure value = higher force, so we invert:
            //   index = clamp((pmax - value) * 255 / pmax, 0, 255)
            int val = frame.pressure[r][c];
            int idx = (pmax > 0) ? std::clamp((pmax - val) * 255 / pmax, 0, 255) : 0;

            size_t pixel = (r * 32 + c) * 3;
            img_msg.data[pixel + 0] = JET_LUT[idx][0];
            img_msg.data[pixel + 1] = JET_LUT[idx][1];
            img_msg.data[pixel + 2] = JET_LUT[idx][2];
        }
    }
    image_pub_->publish(img_msg);
}

}  // namespace wujihand_driver
