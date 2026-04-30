#include "wujihand_driver/tactile_driver_node.hpp"
#include "wujihand_driver/colormap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

namespace wujihand_driver {

namespace {
// RGB triple used to render NaN ("invalid cell" sentinel) in the heatmap.
constexpr uint8_t NAN_R = 64;
constexpr uint8_t NAN_G = 64;
constexpr uint8_t NAN_B = 64;
}  // namespace

TactileDriverNode::TactileDriverNode()
    : Node("tactile_driver_node") {

    // -- Parameters --
    this->declare_parameter("serial_number", "");
    this->declare_parameter("image_rate", 30.0);
    this->declare_parameter("sample_rate_hz", 120);
    this->declare_parameter("streaming_at_startup", true);
    this->declare_parameter("frame_id", "tactile_sensor_link");

    serial_number_       = this->get_parameter("serial_number").as_string();
    image_rate_          = this->get_parameter("image_rate").as_double();
    int initial_rate     = this->get_parameter("sample_rate_hz").as_int();
    streaming_at_startup_ = this->get_parameter("streaming_at_startup").as_bool();
    frame_id_            = this->get_parameter("frame_id").as_string();

    if (initial_rate < 1 || initial_rate > 120) {
        RCLCPP_WARN(this->get_logger(),
                    "sample_rate_hz=%d out of 1..120, clamping to 120", initial_rate);
        initial_rate = 120;
    }
    if (image_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "image_rate must be positive, using default 30.0");
        image_rate_ = 30.0;
    }
    sample_rate_hz_.store(initial_rate, std::memory_order_relaxed);

    // Compute image-publish skip ratio against the *streaming* rate.
    image_skip_.store(
        std::max(1, static_cast<int>(std::round(
            static_cast<double>(initial_rate) / image_rate_))),
        std::memory_order_relaxed);

    // -- Publishers --
    raw_pub_ = this->create_publisher<wujihand_msgs::msg::TactileFrame>(
        "tactile/raw", rclcpp::SensorDataQoS());
    // Image uses default Reliable QoS — RViz2 Humble ignores QoS overrides
    // for the Image display (see wujihandros2 CLAUDE.md pitfall #1).
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "tactile/image", 10);
    diag_pub_ = this->create_publisher<wujihand_msgs::msg::TactileDiagnostics>(
        "tactile/diagnostics", 10);

    // -- Connect --
    const char* sn = serial_number_.empty() ? nullptr : serial_number_.c_str();
    board_ = std::make_unique<wujihandcpp::TactileBoard>(sn);
    if (!board_->connect()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to connect to tactile board");
        throw std::runtime_error("Tactile board connection failed");
    }

    // Identity log line — analogue of GET_DEVICE_INFO + GET_FW_BUILD.
    auto info  = board_->get_device_info();
    auto build = board_->get_fw_build();
    RCLCPP_INFO(this->get_logger(),
                "Tactile: SN=%s hw=%u.%u.%u.%u fw=%u.%u.%u(pre=%u) build=%s",
                info.serial.c_str(),
                info.hw_revision[0], info.hw_revision[1],
                info.hw_revision[2], info.hw_revision[3],
                info.fw_version[0], info.fw_version[1],
                info.fw_version[2], info.fw_version[3],
                build.git_short_sha.c_str());

    // Apply startup config.
    board_->set_sample_rate_hz(
        static_cast<uint16_t>(sample_rate_hz_.load(std::memory_order_relaxed)));
    board_->set_streaming(streaming_at_startup_);

    board_->set_disconnect_callback([this]() {
        RCLCPP_ERROR(this->get_logger(), "Tactile USB disconnected");
    });

    // -- Callback groups --
    // Distinct MutuallyExclusive groups so the diag timer (which can block
    // up to 2 s on an SDK command) cannot starve service callbacks. Each
    // group serializes its own callbacks; the executor parallelizes
    // across groups.
    cb_group_diag_      = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_streaming_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_rate_      = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_reset_     = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // -- Services --
    svc_streaming_ = this->create_service<wujihand_msgs::srv::SetTactileStreaming>(
        "set_tactile_streaming",
        [this](const std::shared_ptr<wujihand_msgs::srv::SetTactileStreaming::Request> req,
               std::shared_ptr<wujihand_msgs::srv::SetTactileStreaming::Response> resp) {
            try {
                board_->set_streaming(req->enable);
                resp->success = true;
            } catch (const std::exception& e) {
                resp->success = false;
                resp->message = e.what();
            }
        },
        rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_streaming_);

    svc_rate_ = this->create_service<wujihand_msgs::srv::SetTactileSampleRate>(
        "set_tactile_sample_rate",
        [this](const std::shared_ptr<wujihand_msgs::srv::SetTactileSampleRate::Request> req,
               std::shared_ptr<wujihand_msgs::srv::SetTactileSampleRate::Response> resp) {
            try {
                board_->set_sample_rate_hz(req->sample_rate_hz);
                int new_rate = static_cast<int>(req->sample_rate_hz);
                sample_rate_hz_.store(new_rate, std::memory_order_relaxed);
                image_skip_.store(
                    std::max(1, static_cast<int>(std::round(
                        static_cast<double>(new_rate) / image_rate_))),
                    std::memory_order_relaxed);
                resp->success = true;
            } catch (const std::exception& e) {
                resp->success = false;
                resp->message = e.what();
            }
        },
        rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_rate_);

    svc_reset_ = this->create_service<wujihand_msgs::srv::ResetTactileCounters>(
        "reset_tactile_counters",
        [this](const std::shared_ptr<wujihand_msgs::srv::ResetTactileCounters::Request>,
               std::shared_ptr<wujihand_msgs::srv::ResetTactileCounters::Response> resp) {
            try {
                board_->reset_counters();
                resp->success = true;
            } catch (const std::exception&) {
                resp->success = false;
            }
        },
        rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_reset_);

    // 10 Hz diagnostics timer (its own callback group so the 2 s SDK
    // timeout doesn't starve the services above).
    diag_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { publish_diagnostics(); },
        cb_group_diag_);

    // -- Streaming consumer --
    board_->start_streaming(
        [this](const wujihandcpp::TactileFrame& frame) {
            this->on_frame(frame);
        });

    RCLCPP_INFO(this->get_logger(),
                "Tactile streaming started at %d Hz (image @ ~%.1f Hz)",
                sample_rate_hz_.load(std::memory_order_relaxed), image_rate_);
}

TactileDriverNode::~TactileDriverNode() {
    if (board_) {
        // stop_streaming joins the demuxer's reader thread; after it returns
        // no on_frame() callbacks can fire so publishers are safe to destroy.
        board_->stop_streaming();
    }
}

void TactileDriverNode::on_frame(const wujihandcpp::TactileFrame& frame) {
    if (!frame.crc_valid) {
        // Should never reach the consumer (demuxer drops bad-CRC frames),
        // but keep this as a defensive guard.
        return;
    }

    auto now = this->get_clock()->now();

    // --- Raw TactileFrame (every frame) ---
    auto raw_msg = wujihand_msgs::msg::TactileFrame();
    raw_msg.header.stamp = now;
    raw_msg.header.frame_id = frame_id_;
    raw_msg.hand = static_cast<uint8_t>(frame.hand);
    raw_msg.sequence = frame.sequence;
    raw_msg.device_timestamp_ms = frame.timestamp_ms;
    static_assert(sizeof(raw_msg.pressure[0]) == sizeof(float),
                  "TactileFrame.msg pressure[] must be float32");
    std::memcpy(raw_msg.pressure.data(), &frame.pressure[0][0],
                768 * sizeof(float));
    raw_pub_->publish(raw_msg);

    // --- Image (downsampled to ~image_rate Hz) ---
    uint32_t count = frame_counter_.fetch_add(1, std::memory_order_relaxed);
    int skip = image_skip_.load(std::memory_order_relaxed);
    if (skip < 1) skip = 1;  // defensive
    if (count % static_cast<uint32_t>(skip) != 0) return;

    auto img_msg = sensor_msgs::msg::Image();
    img_msg.header.stamp = now;
    img_msg.header.frame_id = frame_id_;
    img_msg.height = 24;
    img_msg.width = 32;
    img_msg.encoding = "rgb8";
    img_msg.is_bigendian = false;
    img_msg.step = 32 * 3;
    img_msg.data.resize(24 * 32 * 3);

    for (int r = 0; r < 24; ++r) {
        for (int c = 0; c < 32; ++c) {
            float v = frame.pressure[r][c];
            size_t pixel = (r * 32 + c) * 3;
            if (std::isnan(v)) {
                img_msg.data[pixel + 0] = NAN_R;
                img_msg.data[pixel + 1] = NAN_G;
                img_msg.data[pixel + 2] = NAN_B;
                continue;
            }
            int idx = std::clamp(static_cast<int>(v * 255.0f), 0, 255);
            img_msg.data[pixel + 0] = JET_LUT[idx][0];
            img_msg.data[pixel + 1] = JET_LUT[idx][1];
            img_msg.data[pixel + 2] = JET_LUT[idx][2];
        }
    }
    image_pub_->publish(img_msg);
}

void TactileDriverNode::publish_diagnostics() {
    if (!board_) return;

    // Backoff: after N consecutive failures (e.g. device stuck or
    // disconnected), throttle to ~1 Hz so we do not hammer the SDK with a
    // 2 s blocking command every 100 ms.
    constexpr int FAILURE_THRESHOLD = 3;
    constexpr int BACKOFF_PERIOD = 10;  // proceed once every 10 ticks (~1 Hz)
    int failures = diag_consecutive_failures_.load(std::memory_order_relaxed);
    if (failures >= FAILURE_THRESHOLD) {
        int tick = diag_backoff_tick_.fetch_add(1, std::memory_order_relaxed);
        if (tick % BACKOFF_PERIOD != 0) return;
    }

    try {
        // try_get_diagnostics yields to user-issued service calls instead
        // of queueing behind them on the SDK's per-channel command mutex.
        // Lock-contention returns false: do nothing this tick (NOT counted
        // as a failure — the SDK is healthy, we just chose not to compete).
        //
        // Asymmetric guarantee: this prevents the diagnostics tick from
        // queueing BEHIND a service that already holds the SDK serializer.
        // The reverse case — diag wins the lock first, a service arrives
        // milliseconds later — still makes the service wait for diag's
        // command (worst case ~SDK command timeout). Accepted tradeoff:
        // services are user-issued and rare; the diag command is a single
        // round trip, so this asymmetry rarely surfaces in practice.
        wujihandcpp::TactileDiagnostics d;
        if (!board_->try_get_diagnostics(d)) return;

        wujihand_msgs::msg::TactileDiagnostics msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;
        msg.uptime_ms       = d.uptime_ms;
        msg.frame_count     = d.frame_count;
        msg.crc_err_count   = d.crc_err_count;
        msg.dropout_count   = d.dropout_count;
        msg.usb_reset_count = d.usb_reset_count;
        diag_pub_->publish(msg);
        diag_consecutive_failures_.store(0, std::memory_order_relaxed);
        diag_backoff_tick_.store(0, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        // Single source of truth for the failure counter — the pre-skip
        // branch above only advances diag_backoff_tick_, never failures.
        diag_consecutive_failures_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "get_diagnostics failed: %s", e.what());
    }
}

}  // namespace wujihand_driver
