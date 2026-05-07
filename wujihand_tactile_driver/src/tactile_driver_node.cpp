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

#include "wujihand_tactile_driver/tactile_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "wujihand_tactile_driver/colormap.hpp"

namespace wujihand_tactile_driver {

namespace {
// RGB triple used to render NaN ("invalid cell" sentinel) in the heatmap.
constexpr uint8_t NAN_R = 64;
constexpr uint8_t NAN_G = 64;
constexpr uint8_t NAN_B = 64;

// Bounded image-publish queue. 4 frames @ 30 Hz = ~133 ms drop budget.
constexpr size_t IMAGE_QUEUE_MAX = 4;

// Run a service action and populate a standard {success, message} response.
template <typename Response, typename Fn>
void service_reply(Response& resp, Fn&& fn) {
    try {
        fn();
        resp.success = true;
    } catch (const std::exception& e) {
        resp.success = false;
        resp.message = e.what();
    }
}

// Frames-to-skip ratio so the SDK's `sample_rate_hz` stream is downsampled
// to publish at ~`image_rate` Hz. Always >= 1 to avoid division by zero
// in on_frame's `count % skip` test.
int image_skip_for(int sample_rate_hz, double image_rate) {
    return std::max(1, static_cast<int>(
        std::round(static_cast<double>(sample_rate_hz) / image_rate)));
}
}  // namespace

TactileDriverNode::TactileDriverNode() : Node("tactile_driver_node") {
  // -- Parameters --
  this->declare_parameter("serial_number", "");
  this->declare_parameter("image_rate", 30.0);
  this->declare_parameter("sample_rate_hz", 120);
  this->declare_parameter("streaming_at_startup", true);
  this->declare_parameter("frame_id", "tactile_sensor_link");

  const std::string serial_number = this->get_parameter("serial_number").as_string();
  image_rate_ = this->get_parameter("image_rate").as_double();
  int initial_rate = this->get_parameter("sample_rate_hz").as_int();
  const bool streaming_at_startup = this->get_parameter("streaming_at_startup").as_bool();
  frame_id_ = this->get_parameter("frame_id").as_string();

  if (initial_rate < 1 || initial_rate > 120) {
    RCLCPP_WARN(this->get_logger(), "sample_rate_hz=%d out of 1..120, clamping to 120",
                initial_rate);
    initial_rate = 120;
  }
  if (image_rate_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "image_rate must be positive, using default 30.0");
    image_rate_ = 30.0;
  }

  // Image-publish skip ratio against the *streaming* rate.
  image_skip_.store(image_skip_for(initial_rate, image_rate_),
                    std::memory_order_relaxed);

  // -- Publishers --
  raw_pub_ = this->create_publisher<wujihand_tactile_msgs::msg::TactileFrame>(
      "tactile/raw", rclcpp::SensorDataQoS());
  // Image uses default Reliable QoS — RViz2 Humble ignores QoS overrides
  // for the Image display (see wujihandros2 CLAUDE.md pitfall #1).
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("tactile/image", 10);
  diag_pub_ = this->create_publisher<wujihand_tactile_msgs::msg::TactileDiagnostics>(
      "tactile/diagnostics", 10);

  // -- Connect --
  const char* sn = serial_number.empty() ? nullptr : serial_number.c_str();
  board_ = std::make_unique<wujihandcpp::tactile::Board>(sn);
  if (!board_->connect()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect to tactile board");
    throw std::runtime_error("Tactile board connection failed");
  }

  auto info = board_->get_device_info();
  auto build = board_->get_fw_build();
  RCLCPP_INFO(this->get_logger(), "Tactile: SN=%s hw=%u.%u.%u.%u fw=%u.%u.%u(pre=%u) build=%s",
              info.serial.c_str(), info.hw_revision[0], info.hw_revision[1], info.hw_revision[2],
              info.hw_revision[3], info.fw_version[0], info.fw_version[1], info.fw_version[2],
              info.fw_version[3], build.git_short_sha.c_str());

  board_->set_sample_rate_hz(static_cast<uint16_t>(initial_rate));
  board_->set_streaming(streaming_at_startup);

  board_->set_disconnect_callback([this]() {
    // Flip BEFORE logging so a diag tick concurrent with this callback
    // observes the new state. SDK contract guarantees this fires once.
    connected_.store(false, std::memory_order_release);
    RCLCPP_ERROR(this->get_logger(),
                 "Tactile USB disconnected. /tactile/diagnostics will continue "
                 "publishing with connected=false; restart the node to reconnect.");
  });

  // -- Callback groups --
  // Distinct MutuallyExclusive groups so the diag timer (which can block
  // up to 2 s on an SDK command) cannot starve service callbacks. Each
  // group serializes its own callbacks; the executor parallelizes across
  // groups.
  cb_group_diag_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_streaming_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_rate_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_reset_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // -- Services --
  // All under the `tactile/` prefix to match the topic namespace.
  svc_streaming_ = this->create_service<wujihand_tactile_msgs::srv::SetTactileStreaming>(
      "tactile/set_streaming",
      [this](const std::shared_ptr<wujihand_tactile_msgs::srv::SetTactileStreaming::Request> req,
             std::shared_ptr<wujihand_tactile_msgs::srv::SetTactileStreaming::Response> resp) {
        service_reply(*resp, [&] { board_->set_streaming(req->enable); });
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_streaming_);

  svc_rate_ = this->create_service<wujihand_tactile_msgs::srv::SetTactileSampleRate>(
      "tactile/set_sample_rate",
      [this](const std::shared_ptr<wujihand_tactile_msgs::srv::SetTactileSampleRate::Request> req,
             std::shared_ptr<wujihand_tactile_msgs::srv::SetTactileSampleRate::Response> resp) {
        service_reply(*resp, [&] {
          board_->set_sample_rate_hz(req->sample_rate_hz);
          image_skip_.store(image_skip_for(req->sample_rate_hz, image_rate_),
                            std::memory_order_relaxed);
        });
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_rate_);

  svc_reset_ = this->create_service<wujihand_tactile_msgs::srv::ResetTactileCounters>(
      "tactile/reset_counters",
      [this](const std::shared_ptr<wujihand_tactile_msgs::srv::ResetTactileCounters::Request>,
             std::shared_ptr<wujihand_tactile_msgs::srv::ResetTactileCounters::Response> resp) {
        service_reply(*resp, [&] { board_->reset_counters(); });
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(), cb_group_reset_);

  // 10 Hz diagnostics timer (its own callback group so the 2 s SDK
  // timeout doesn't starve the services above).
  diag_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { publish_diagnostics(); }, cb_group_diag_);

  // -- Image-publish worker --
  // Spawn BEFORE start_streaming so the queue has a consumer ready by
  // the time the SDK starts firing on_frame. Tearing down in reverse
  // order (~Node → stop image worker → stop_streaming → ~publishers)
  // ensures image_pub_ outlives the worker.
  image_worker_ = std::thread([this]() { this->image_worker_loop(); });

  // -- Streaming consumer --
  // Constructor-time exception safety: ~Node is NOT called when the
  // constructor itself throws, so a joinable image_worker_ at the
  // throw point would terminate via std::thread's destructor
  // contract. Catch any failure from start_streaming (which can throw
  // NotConnectedError if USB drops between connect() and here), tear
  // the worker down cleanly, and rethrow so the launch surfaces the
  // real cause.
  try {
    board_->start_streaming(
        [this](const wujihandcpp::tactile::Frame& frame) { this->on_frame(frame); });
  } catch (...) {
    image_worker_stop_.store(true, std::memory_order_release);
    image_queue_cv_.notify_all();
    if (image_worker_.joinable()) {
      image_worker_.join();
    }
    // disconnect joins the SDK reader thread + drops the demuxer so
    // the user-set disconnect callback (which writes to connected_)
    // can no longer fire before the node-owned members destruct.
    board_->disconnect();
    throw;
  }

  RCLCPP_INFO(this->get_logger(), "Tactile streaming started at %d Hz (image @ ~%.1f Hz)",
              initial_rate, image_rate_);
}

TactileDriverNode::~TactileDriverNode() {
  // Tear down SDK threads before node-owned callback state and publishers
  // destruct: disconnect() joins both the SDK streaming consumer and the
  // demuxer reader, so the user-set disconnect callback (which writes to
  // connected_) cannot fire after we leave this body.
  if (board_) {
    board_->disconnect();
  }
  image_worker_stop_.store(true, std::memory_order_release);
  image_queue_cv_.notify_all();
  if (image_worker_.joinable()) {
    image_worker_.join();
  }
}

void TactileDriverNode::on_frame(const wujihandcpp::tactile::Frame& frame) {
  // Belt-and-suspenders disconnect guard against a stray late callback.
  if (!connected_.load(std::memory_order_acquire)) {
    return;
  }
  // Keep publisher failures from escaping into the SDK streaming thread.
  try {
    auto now = this->get_clock()->now();

    // --- Raw TactileFrame (every frame) ---
    auto raw_msg = wujihand_tactile_msgs::msg::TactileFrame();
    raw_msg.header.stamp = now;
    raw_msg.header.frame_id = frame_id_;
    raw_msg.hand = static_cast<uint8_t>(frame.hand);
    raw_msg.sequence = frame.sequence;
    raw_msg.device_timestamp_ms = frame.timestamp_ms;
    static_assert(sizeof(raw_msg.pressure[0]) == sizeof(float),
                  "TactileFrame.msg pressure[] must be float32");
    std::memcpy(raw_msg.pressure.data(), &frame.pressure[0][0], 768 * sizeof(float));
    raw_pub_->publish(raw_msg);

    // --- Image (downsampled to ~image_rate Hz) ---
    // on_frame runs single-threaded on the SDK streaming consumer; counter
    // does not need to be atomic.
    const uint32_t count = frame_counter_++;
    const int skip = image_skip_.load(std::memory_order_relaxed);
    if (count % static_cast<uint32_t>(skip) != 0)
      return;

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

    // Hand the message off to the worker thread instead of publishing
    // synchronously here. Reliable image_pub_ + slow RViz subscriber
    // would otherwise backpressure into this SDK reader thread and
    // ultimately into the kernel CDC buffer, producing real frame
    // drops on the bus. Drop-oldest under queue pressure trades visual
    // smoothness in RViz for keeping the wire happy.
    {
      std::lock_guard<std::mutex> qlock(image_queue_mu_);
      if (image_queue_.size() >= IMAGE_QUEUE_MAX) {
        image_queue_.pop_front();
        // Throttled WARN — under a rare RViz stall this fires once in
        // a while, which is informative; under sustained backpressure
        // it caps log volume.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "tactile image queue full, dropping oldest "
                             "frame (slow subscriber?)");
      }
      image_queue_.push_back(std::move(img_msg));
    }
    image_queue_cv_.notify_one();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "tactile on_frame: publish failed: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "tactile on_frame: publish failed (non-std exception)");
  }
}

void TactileDriverNode::image_worker_loop() {
  while (true) {
    sensor_msgs::msg::Image msg;
    {
      std::unique_lock<std::mutex> qlock(image_queue_mu_);
      image_queue_cv_.wait(qlock, [this]() {
        return !image_queue_.empty() || image_worker_stop_.load(std::memory_order_acquire);
      });
      if (image_queue_.empty()) {
        // Stop signal raced with empty queue → exit.
        return;
      }
      msg = std::move(image_queue_.front());
      image_queue_.pop_front();
    }
    // publish() outside the lock; Reliable QoS may block here but only
    // this worker thread is affected. Catch everything so a stray rclcpp
    // exception cannot unwind the thread and std::terminate the process.
    try {
      image_pub_->publish(std::move(msg));
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "tactile image publish failed: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "tactile image publish failed (non-std exception)");
    }
  }
}

void TactileDriverNode::publish_diagnostics() {
  // Disconnected: keep publishing connected=false, skip SDK calls, and preserve
  // last counters so downstream delta trackers don't see a fake reset.
  if (!connected_.load(std::memory_order_acquire)) {
    wujihand_tactile_msgs::msg::TactileDiagnostics msg =
        have_last_diag_ ? last_diag_msg_
                        : wujihand_tactile_msgs::msg::TactileDiagnostics{};
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.connected = false;
    diag_pub_->publish(msg);
    return;
  }

  // Connected path: best-effort polling. After 3 consecutive command
  // failures (device stuck), throttle to ~1 Hz so we are not issuing a
  // 2 s blocking SDK call every 100 ms. The diag timer is in its own
  // MutuallyExclusive callback group, so this whole function is
  // single-threaded and the counters need not be atomic.
  constexpr int FAILURE_THRESHOLD = 3;
  constexpr int BACKOFF_PERIOD = 10;  // proceed once every 10 ticks (~1 Hz)
  if (diag_consecutive_failures_ >= FAILURE_THRESHOLD) {
    if (diag_backoff_tick_++ % BACKOFF_PERIOD != 0)
      return;
  }

  try {
    // Yield to in-flight service commands; skipped polls are not failures.
    wujihandcpp::tactile::Diagnostics d;
    if (!board_->try_get_diagnostics(d))
      return;

    wujihand_tactile_msgs::msg::TactileDiagnostics msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.connected = true;
    msg.uptime_ms = d.uptime_ms;
    msg.frame_count = d.frame_count;
    msg.crc_err_count = d.crc_err_count;
    msg.dropout_count = d.dropout_count;
    msg.usb_reset_count = d.usb_reset_count;
    diag_pub_->publish(msg);
    // Cache for the disconnect path so we can keep emitting frozen
    // counters instead of zeros after USB drops.
    last_diag_msg_ = msg;
    have_last_diag_ = true;
    diag_consecutive_failures_ = 0;
    diag_backoff_tick_ = 0;
  } catch (const std::exception& e) {
    ++diag_consecutive_failures_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "get_diagnostics failed: %s",
                         e.what());
  }
}

}  // namespace wujihand_tactile_driver
