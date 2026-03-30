#pragma once

#include "fields.hpp"
#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.hpp>
#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <thread>
#include <vector>

namespace autopilot::diagnosis {

struct DiagnosticStamped {
  diagnostic_msgs::msg::DiagnosticStatus status;
  rclcpp::Time timestamp;
};

class Diagnosis {
public:
  Diagnosis(rclcpp::Clock::SharedPtr clock, const uint16_t startup_timeout_sec,
            const uint16_t expected_slam_reporters,
            const uint16_t expected_relocation_reporters)
      : clock_(clock)
  // , expected_slam_reporters_(expected_slam_reporters),
  //   expected_relocation_reporters_(expected_relocation_reporters)
  {
    listener_thread_ = std::thread(&Diagnosis::healthcheck_server_thread, this);
    startup_time_ = std::chrono::steady_clock::now();
    startup_timeout_ = std::chrono::seconds(startup_timeout_sec);

    load_nav_mode();
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
    PilotDiag fatal_diag = no_fatal ? PilotDiag::WARNING : PilotDiag::FATAL;
    // uint8_t expected_reporters = current_nav_mode_ == NavMode::SLAM
    //                                  ? expected_slam_reporters_
    //                                  : expected_relocation_reporters_;

    for (const auto &status : msg->status) {
      if (status.hardware_id == "")
        continue;

      DiagnosticStamped diag_stamped{status, msg->header.stamp};
      if (status.level > 1) {
        diagnostics_errors_[get_diag_key(status)] = diag_stamped;
      } else if (status.level == 1) {
        diagnostics_warnings_[get_diag_key(status)] = diag_stamped;
      }
      diagnostics_all_[get_diag_key(status)] = diag_stamped;
    }
    clean_old_diagnostics();
    // uint16_t reporter_count = diagnostics_all_.size();

    if (std::chrono::steady_clock::now() - startup_time_ < startup_timeout_) {
      return PilotDiag::STARTING;
    }
    if (current_nav_mode_ == NavMode::UNKNOWN) {
      return fatal_diag;
    }

    // Do diagnosis.
    if (!diagnostics_errors_.empty()) {
      is_healthy_ = false;
      for (const auto &error : diagnostics_errors_) {
        RCLCPP_ERROR(rclcpp::get_logger("HC"), "Error from %s: %s",
                     error.second.status.hardware_id.c_str(),
                     error.second.status.message.c_str());
      }
      return fatal_diag;
    } else if (!diagnostics_warnings_.empty()) {
      for (const auto &warning : diagnostics_warnings_) {
        RCLCPP_WARN(rclcpp::get_logger("HC"), "Warning from %s: %s",
                    warning.second.status.hardware_id.c_str(),
                    warning.second.status.message.c_str());
      }
      return PilotDiag::WARNING;
      // Note: Due to apply of planB, we need temporarily disable the reporter
      // count check.
      // } else if (expected_reporters != reporter_count) {
      //   RCLCPP_ERROR(rclcpp::get_logger("HC"), "Expected %d reporters but got
      //   %d",
      //                expected_reporters, reporter_count);
      //   return fatal_diag;
    } else {
      return PilotDiag::READY;
    }
  }

  void required_restart(bool required) { is_healthy_ = !required; }

  // True for no work to do, false for restart required.
  bool set_nav_mode(NavMode desired_mode) {
    if (current_nav_mode_ == NavMode::UNKNOWN) {
      return false;
    }
    if (desired_mode == NavMode::UNKNOWN &&
        current_nav_mode_ == file_nav_mode_) {
      return true;
    }
    if (current_nav_mode_ != file_nav_mode_) {
      return false;
    }
    if (desired_mode == current_nav_mode_) {
      return true;
    }
    if (write_nav_mode(desired_mode)) {
      file_nav_mode_ = desired_mode;
      return false;
    }
    return true;
  }

  NavMode get_current_nav_mode() const { return current_nav_mode_; }

private:
  void healthcheck_server_thread() {
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) {
      return;
    }

    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, socket_path_.c_str());
    unlink(addr.sun_path);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
      close(sock);
      return;
    }
    listen(sock, 5);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (running_ && rclcpp::ok()) {
      int client = accept(sock, nullptr, nullptr);
      if (client < 0) {
        if (!running_)
          break;
        continue;
      }

      uint8_t status = is_healthy_ ? 0 : 1;
      ssize_t written = write(client, &status, 1);

      if (written > 0) {
        fsync(client);
      }
      usleep(10000); // 10ms
      close(client);
    }

    close(sock);
    unlink(addr.sun_path);
  }

  void load_nav_mode() {
    if (current_nav_mode_ == NavMode::UNKNOWN) {
      if (std::filesystem::exists(nav_mode_path_)) {
        std::ifstream infile(nav_mode_path_);
        std::string mode_str;
        infile >> mode_str;
        infile.close();

        if (mode_str == "SLAM") {
          current_nav_mode_ = file_nav_mode_ = NavMode::SLAM;
        } else if (mode_str == "LOCA") {
          current_nav_mode_ = file_nav_mode_ = NavMode::RELOCATION;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("HC"), "Invalid nav mode in file: %s",
                       mode_str.c_str());
        }
      } else {
        current_nav_mode_ = file_nav_mode_ = NavMode::RELOCATION;
        write_nav_mode(current_nav_mode_);
      }
    }
  }

  // True when successful, false on failure
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

  void clean_old_diagnostics() {
    auto now = clock_->now();
    auto do_clean = [this](std::map<std::string, DiagnosticStamped> &diags,
                           rclcpp::Time now) {
      for (auto it = diags.begin(); it != diags.end();) {
        if ((now - it->second.timestamp).seconds() > diagnostic_timeout_sec_) {
          it = diags.erase(it);
        } else {
          ++it;
        }
      }
    };
    do_clean(diagnostics_errors_, now);
    do_clean(diagnostics_warnings_, now);
    do_clean(diagnostics_all_, now);
  }

  std::string
  get_diag_key(const diagnostic_msgs::msg::DiagnosticStatus &status) {
    return status.hardware_id + ":" + status.name;
  }

  const std::string socket_path_ = "/tmp/health_check.sock";
  const std::string nav_mode_path_ = "/tmp/nav_mode";
  const uint8_t diagnostic_timeout_sec_ = 4;

  NavMode current_nav_mode_ = NavMode::UNKNOWN;
  NavMode file_nav_mode_ = NavMode::UNKNOWN;

  rclcpp::Clock::SharedPtr clock_;

  std::thread listener_thread_;
  std::atomic<bool> is_healthy_{true};
  std::atomic<bool> running_{true};

  std::map<std::string, DiagnosticStamped> diagnostics_errors_;
  std::map<std::string, DiagnosticStamped> diagnostics_warnings_;
  std::map<std::string, DiagnosticStamped> diagnostics_all_;

  std::chrono::duration<int64_t> startup_timeout_{15};
  std::chrono::steady_clock::time_point startup_time_;
  // uint16_t expected_slam_reporters_;
  // uint16_t expected_relocation_reporters_;
};
} // namespace autopilot::diagnosis