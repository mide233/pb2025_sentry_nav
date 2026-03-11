#pragma once

#include "fields.hpp"
#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.hpp>
#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <fstream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <vector>

namespace autopilot {
class Diagnosis {
public:
  Diagnosis(const uint16_t startup_timeout_sec,
            const uint16_t expected_reporters)
      : expected_reporters_(expected_reporters) {
    listener_thread_ = std::thread([this]() {
      int sock = socket(AF_UNIX, SOCK_STREAM, 0);
      struct sockaddr_un addr;
      addr.sun_family = AF_UNIX;
      strcpy(addr.sun_path, socket_path_.c_str());
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

  PilotDiag update_health_callback(
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg,
      bool no_fatal = false) {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> errors;
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> warnings;
    PilotDiag fatal_diag = no_fatal ? PilotDiag::WARNING : PilotDiag::FATAL;
    uint8_t reporter_count = 0;

    if (std::chrono::steady_clock::now() - startup_time_ < timeout_) {
      return PilotDiag::STARTING;
    }

    for (const auto &status : msg->status) {
      if (status.hardware_id == "") {
        continue;
      }

      if (status.level > 1) {
        errors.push_back(status);
      } else if (status.level == 1) {
        warnings.push_back(status);
      }
      reporter_count++;
    }
    if (!errors.empty()) {
      is_healthy_ = false;
      for (const auto &error : errors) {
        RCLCPP_ERROR(rclcpp::get_logger("HC"), "Error from %s: %s",
                     error.hardware_id.c_str(), error.message.c_str());
      }
      return fatal_diag;
    } else if (!warnings.empty()) {
      for (const auto &warning : warnings) {
        RCLCPP_WARN(rclcpp::get_logger("HC"), "Warning from %s: %s",
                    warning.hardware_id.c_str(), warning.message.c_str());
      }
      return PilotDiag::WARNING;
    } else if (expected_reporters_ != reporter_count) {
      return fatal_diag;
    } else {
      return PilotDiag::READY;
    }
  }

  void required_restart(bool required) { is_healthy_ = !required; }

  bool set_nav_mode(NavMode desired_mode) {
    // True for no work to do, false for restart required.
    if (current_nav_mode_ == NavMode::UNKNOWN) {
      if (std::filesystem::exists(nav_mode_path_)) {
        std::ifstream infile(nav_mode_path_);
        std::string mode_str;
        infile >> mode_str;
        infile.close();

        if (mode_str == "SLAM") {
          current_nav_mode_ = NavMode::SLAM;
        } else if (mode_str == "LOCA") {
          current_nav_mode_ = NavMode::RELOCATION;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("HC"), "Invalid nav mode in file: %s",
                       mode_str.c_str());
          return false;
        }
      } else {
        current_nav_mode_ = NavMode::RELOCATION;
        return write_nav_mode(current_nav_mode_);
      }
    }
    if (desired_mode == NavMode::UNKNOWN) {
      return true;
    }

    if (desired_mode == current_nav_mode_) {
      return true;
    } else {
      return !write_nav_mode(desired_mode);
    }
  }

private:
  bool write_nav_mode(const NavMode mode) {
    if (std::filesystem::exists(nav_mode_path_)) {
      std::filesystem::remove(nav_mode_path_);
    }

    std::ofstream outfile(nav_mode_path_);
    if (outfile.is_open()) {
      std::string mode_str = mode == NavMode::SLAM ? "SLAM" : "LOCA";
      outfile << mode_str;
      outfile.close();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("HC"),
                   "Failed to write nav mode file: %s", nav_mode_path_.c_str());
      return false;
    }
    return true;
  }

  const std::string socket_path_ = "/tmp/health_check.sock";
  const std::string nav_mode_path_ = "/tmp/nav_mode";

  NavMode current_nav_mode_ = NavMode::UNKNOWN;

  std::thread listener_thread_;
  std::atomic<bool> is_healthy_{true};
  std::atomic<bool> running_{true};

  std::chrono::duration<int64_t> timeout_{15};
  std::chrono::steady_clock::time_point startup_time_;
  uint16_t expected_reporters_ = 4;
};
} // namespace autopilot