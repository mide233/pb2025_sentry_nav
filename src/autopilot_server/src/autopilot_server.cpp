#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "communication.hpp"

namespace autopilot {
class AutopilotServer : public rclcpp::Node {
public:
  explicit AutopilotServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("autopilot_server", options) {
    RCLCPP_INFO(this->get_logger(), "Autopilot Server node has been started.");

    this->declare_parameter<std::string>("pilot_data_sending_host",
                                         "127.0.0.1");
    this->declare_parameter<int>("pilot_data_sending_port", 42331);
    this->declare_parameter<std::string>("state_data_receiving_host",
                                         "127.0.0.1");
    this->declare_parameter<int>("state_data_receiving_port", 42332);
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    this->get_parameter("pilot_data_sending_host", pilot_data_sending_host_);
    this->get_parameter("pilot_data_sending_port", pilot_data_sending_port_);
    this->get_parameter("state_data_receiving_host",
                        state_data_receiving_host_);
    this->get_parameter("state_data_receiving_port",
                        state_data_receiving_port_);

    gicp_control_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "small_gicp/reset_when_err", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        this->get_parameter("cmd_vel_topic").as_string(), 10,
        std::bind(&AutopilotServer::cmd_vel_callback, this,
                  std::placeholders::_1));

    if (!communication_.startReceiving(
            state_data_receiving_host_, state_data_receiving_port_,
            [this](const StateData &data) { state_data_callback(data); })) {
      RCLCPP_ERROR(this->get_logger(),
                   "Init state data receiving failed. Host: %s, Port: %d",
                   state_data_receiving_host_.c_str(),
                   state_data_receiving_port_);
    }
  }

private:
  void state_data_callback(const StateData &data) {
    auto msg = std_msgs::msg::Bool();
    msg.data = !data.autopilot_enabled;
    gicp_control_publisher_->publish(msg);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist msg) {
    PilotData pilot_data;
    pilot_data.chassis_vel[0] = msg.linear.x;
    pilot_data.chassis_vel[1] = msg.linear.y;
    pilot_data.chassis_vel[2] = 4.0;
    if (!communication_.sendPilotData(pilot_data, pilot_data_sending_host_,
                                      pilot_data_sending_port_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Send pilot data failed. Host: %s, Port: %d",
                   pilot_data_sending_host_.c_str(), pilot_data_sending_port_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gicp_control_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::string pilot_data_sending_host_;
  int pilot_data_sending_port_;
  std::string state_data_receiving_host_;
  int state_data_receiving_port_;

  Communication communication_;
};
} // namespace autopilot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autopilot::AutopilotServer)