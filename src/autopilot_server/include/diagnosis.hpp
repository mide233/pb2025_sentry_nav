#pragma once

#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>

namespace diagnosis {
class Diagnosis {
public:
  Diagnosis(const uint16_t startup_timeout_sec = 15,
            const uint16_t expected_reporters = 4)
      : expected_reporters_(expected_reporters) {
    listener_thread_ = std::thread([this]() {
      int sock = socket(AF_UNIX, SOCK_STREAM, 0);
      struct sockaddr_un addr;
      addr.sun_family = AF_UNIX;
      strcpy(addr.sun_path, "/tmp/health_check.sock");
      unlink(addr.sun_path);

      if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(sock);
        return;
      }
      listen(sock, 5);

      while (running_) {
        int client = accept(sock, nullptr, nullptr);
        uint8_t status = is_healthy_ ? 0 : 1;
        if (write(client, &status, 1) < 0) {
          // Handle write error if needed
        }
        close(client);
      }
    });
    startup_time_ = std::chrono::steady_clock::now();
    timeout_ = std::chrono::seconds(startup_timeout_sec);
  };

  ~Diagnosis() {
    running_ = false;
    if (listener_thread_.joinable()) {
      listener_thread_.join();
    }
  }

  void update_health_callback(
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    int reporter_count = 0;
    for (const auto &status : msg->status) {
      if (status.level > 1 &&
          startup_time_ + timeout_ < std::chrono::steady_clock::now()) {
        is_healthy_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("HC"), "Health check failed");
        return;
      }
      reporter_count++;
    }
    is_healthy_ = reporter_count == expected_reporters_;
    if (!is_healthy_) {
      RCLCPP_ERROR(rclcpp::get_logger("HC"),
                   "Missing diagnostic reports: %d/%d", reporter_count,
                   expected_reporters_);
    }
  }

private:
  std::thread listener_thread_;
  std::atomic<bool> is_healthy_{true};
  std::atomic<bool> running_{true};

  std::chrono::duration<int64_t> timeout_{15};
  std::chrono::steady_clock::time_point startup_time_;
  uint16_t expected_reporters_ = 4;
};
} // namespace diagnosis