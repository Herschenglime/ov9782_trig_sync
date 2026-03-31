// Copyright 2026 Herschenglime
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

#include <errno.h>
#include <sched.h>
#include <string.h>
#include <sys/resource.h>

#include <gpiod.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class TriggerSyncNode : public rclcpp::Node
{
public:
  TriggerSyncNode()
  : Node("trigger_sync_node")
  {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/unilidar/cloud");
    gpio_chip_ = declare_parameter<std::string>("gpio_chip", "/dev/gpiochip0");
    gpio_line_offset_ = declare_parameter<int>("gpio_line", -1);
    pulse_width_us_ = declare_parameter<int>("pulse_width_us", 50);
    active_high_ = declare_parameter<bool>("active_high", true);
    stats_period_s_ = declare_parameter<double>("stats_period_s", 1.0);
    latency_window_size_ = declare_parameter<int>("latency_window_size", 256);
    use_realtime_ = declare_parameter<bool>("use_realtime", false);
    realtime_priority_ = declare_parameter<int>("realtime_priority", 70);

    if (gpio_line_offset_ < 0) {
      throw std::runtime_error("Parameter 'gpio_line' must be >= 0");
    }
    if (pulse_width_us_ < 1) {
      throw std::runtime_error("Parameter 'pulse_width_us' must be >= 1");
    }
    if (latency_window_size_ < 10) {
      latency_window_size_ = 10;
      RCLCPP_WARN(get_logger(), "latency_window_size too small; clamped to 10");
    }
    if (stats_period_s_ <= 0.0) {
      stats_period_s_ = 1.0;
      RCLCPP_WARN(get_logger(), "stats_period_s must be > 0; using 1.0s");
    }

    if (use_realtime_) {
      configure_realtime();
    }

    init_gpio();

    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_,
      qos,
      std::bind(&TriggerSyncNode::on_lidar, this, std::placeholders::_1));

    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_s_),
      std::bind(&TriggerSyncNode::log_stats, this));

    RCLCPP_INFO(
      get_logger(),
      "Ready: topic=%s chip=%s line=%d pulse_width_us=%d active_high=%s",
      lidar_topic_.c_str(),
      gpio_chip_.c_str(),
      gpio_line_offset_,
      pulse_width_us_,
      active_high_ ? "true" : "false");
  }

  ~TriggerSyncNode() override
  {
    release_gpio();
  }

private:
  void configure_realtime()
  {
    sched_param sp {};
    sp.sched_priority = realtime_priority_;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to set SCHED_FIFO priority %d: %s",
        realtime_priority_,
        strerror(errno));
      return;
    }

    if (setpriority(PRIO_PROCESS, 0, -10) != 0) {
      RCLCPP_WARN(get_logger(), "Failed to raise nice priority: %s", strerror(errno));
    }

    RCLCPP_INFO(get_logger(), "Realtime scheduling enabled: SCHED_FIFO %d", realtime_priority_);
  }

  void init_gpio()
  {
    chip_ = gpiod_chip_open(gpio_chip_.c_str());
    if (chip_ == nullptr) {
      throw std::runtime_error("Failed to open gpio chip '" + gpio_chip_ + "': " + strerror(errno));
    }

    line_ = gpiod_chip_get_line(chip_, gpio_line_offset_);
    if (line_ == nullptr) {
      release_gpio();
      throw std::runtime_error(
              "Failed to get gpio line " + std::to_string(gpio_line_offset_) + ": " +
              strerror(errno));
    }

    const int inactive_level = active_high_ ? 0 : 1;
    const int ret = gpiod_line_request_output(line_, "ov9782_trig_sync", inactive_level);
    if (ret != 0) {
      release_gpio();
      throw std::runtime_error(
              "Failed to request gpio output line: " +
              std::string(strerror(errno)));
    }
  }

  void release_gpio()
  {
    if (line_ != nullptr) {
      gpiod_line_release(line_);
      line_ = nullptr;
    }
    if (chip_ != nullptr) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  void hold_pulse() const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::microseconds(pulse_width_us_);

    // Busy-wait is intentional here to reduce short-pulse scheduler jitter.
    while (std::chrono::steady_clock::now() < deadline) {
    }
  }

  void push_latency_ns(int64_t latency_ns)
  {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    if (latency_samples_ns_.size() >= static_cast<size_t>(latency_window_size_)) {
      latency_samples_ns_.erase(latency_samples_ns_.begin());
    }
    latency_samples_ns_.push_back(latency_ns);
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr)
  {
    incoming_count_.fetch_add(1, std::memory_order_relaxed);

    const auto callback_start = std::chrono::steady_clock::now();
    const int active_level = active_high_ ? 1 : 0;
    const int inactive_level = active_high_ ? 0 : 1;

    if (gpiod_line_set_value(line_, active_level) != 0) {
      gpio_error_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    const auto high_set = std::chrono::steady_clock::now();
    hold_pulse();

    if (gpiod_line_set_value(line_, inactive_level) != 0) {
      gpio_error_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    pulse_count_.fetch_add(1, std::memory_order_relaxed);

    const auto callback_end = std::chrono::steady_clock::now();
    const auto edge_latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      high_set - callback_start).count();
    const auto callback_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      callback_end - callback_start).count();

    push_latency_ns(edge_latency_ns);

    if (callback_duration_ns > 1'000'000) {
      slow_callback_count_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void log_stats()
  {
    const auto now_incoming = incoming_count_.load(std::memory_order_relaxed);
    const auto now_pulse = pulse_count_.load(std::memory_order_relaxed);
    const auto now_error = gpio_error_count_.load(std::memory_order_relaxed);
    const auto now_slow = slow_callback_count_.load(std::memory_order_relaxed);

    const auto d_incoming = now_incoming - prev_incoming_count_;
    const auto d_pulse = now_pulse - prev_pulse_count_;
    const auto d_error = now_error - prev_gpio_error_count_;
    const auto d_slow = now_slow - prev_slow_callback_count_;

    prev_incoming_count_ = now_incoming;
    prev_pulse_count_ = now_pulse;
    prev_gpio_error_count_ = now_error;
    prev_slow_callback_count_ = now_slow;

    std::vector<int64_t> samples;
    {
      std::lock_guard<std::mutex> lock(latency_mutex_);
      samples = latency_samples_ns_;
    }

    if (!samples.empty()) {
      std::sort(samples.begin(), samples.end());
      const int64_t min_ns = samples.front();
      const int64_t max_ns = samples.back();
      const size_t p95_idx = static_cast<size_t>(0.95 * static_cast<double>(samples.size() - 1));
      const int64_t p95_ns = samples[p95_idx];
      int64_t sum_ns = 0;
      for (const auto ns : samples) {
        sum_ns += ns;
      }
      const double avg_ns = static_cast<double>(sum_ns) / static_cast<double>(samples.size());

      RCLCPP_INFO(
        get_logger(),
        "rate Hz in=%lu pulse=%lu gpio_err=%lu slow_cb=%lu | "
        "edge_latency_ns min=%ld avg=%.0f p95=%ld max=%ld n=%zu",
        d_incoming,
        d_pulse,
        d_error,
        d_slow,
        min_ns,
        avg_ns,
        p95_ns,
        max_ns,
        samples.size());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "rate Hz in=%lu pulse=%lu gpio_err=%lu slow_cb=%lu | edge_latency_ns n=0",
      d_incoming,
      d_pulse,
      d_error,
      d_slow);
  }

  std::string lidar_topic_;
  std::string gpio_chip_;
  int gpio_line_offset_;
  int pulse_width_us_;
  bool active_high_;
  double stats_period_s_;
  int latency_window_size_;
  bool use_realtime_;
  int realtime_priority_;

  gpiod_chip * chip_ {nullptr};
  gpiod_line * line_ {nullptr};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::atomic<uint64_t> incoming_count_ {0};
  std::atomic<uint64_t> pulse_count_ {0};
  std::atomic<uint64_t> gpio_error_count_ {0};
  std::atomic<uint64_t> slow_callback_count_ {0};

  uint64_t prev_incoming_count_ {0};
  uint64_t prev_pulse_count_ {0};
  uint64_t prev_gpio_error_count_ {0};
  uint64_t prev_slow_callback_count_ {0};

  std::mutex latency_mutex_;
  std::vector<int64_t> latency_samples_ns_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<TriggerSyncNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("trigger_sync_node"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
