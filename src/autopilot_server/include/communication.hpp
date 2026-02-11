#pragma once

#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <functional>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace autopilot {
struct StateData {
  double gimbal_faceing[3];
};
struct PilotData {
  double chassis_vel[3]; // x, y, w
};
class Communication {
public:
  Communication() : sockfd_(-1), running_(false) {}

  ~Communication() {
    stop();
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

  bool startReceiving(const std::string &ip, uint16_t port,
                      std::function<void(const StateData &)> callback) {
    if (running_) {
      return false;
    }

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      return false;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip.c_str());

    if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    running_ = true;
    callback_ = callback;

    recv_thread_ = std::thread(&Communication::receiveThread, this);
    return true;
  }

  void stop() {
    running_ = false;
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

  bool sendPilotData(const PilotData &data, const std::string &ip,
                     uint16_t port) {
    int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0) {
      return false;
    }

    struct sockaddr_in dest_addr;
    std::memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(ip.c_str());

    ssize_t sent = sendto(send_sock, &data, sizeof(PilotData), 0,
                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (sent < 0) {
      last_sending_error_ = std::strerror(errno);
    }

    close(send_sock);
    return sent == sizeof(PilotData);
  }

private:
  void receiveThread() {
    StateData data;
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    while (running_) {
      ssize_t recv_len = recvfrom(sockfd_, &data, sizeof(StateData), 0,
                                  (struct sockaddr *)&sender_addr, &addr_len);

      if (recv_len == sizeof(StateData) && callback_) {
        callback_(data);
      } // TODO: Can error in recv happen frequently? If so, we might want to
        // log it or handle it differently.
    }
  }

  int sockfd_;
  std::string last_receiving_error_;
  std::string last_sending_error_;
  std::atomic<bool> running_;
  std::thread recv_thread_;
  std::function<void(const StateData &)> callback_;
};
} // namespace autopilot