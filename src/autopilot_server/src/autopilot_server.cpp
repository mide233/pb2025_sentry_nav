#include "rclcpp/rclcpp.hpp"
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

    this->get_parameter("pilot_data_sending_host", pilot_data_sending_host_);
    this->get_parameter("pilot_data_sending_port", pilot_data_sending_port_);
    this->get_parameter("state_data_receiving_host",
                        state_data_receiving_host_);
    this->get_parameter("state_data_receiving_port",
                        state_data_receiving_port_);

    if (!communication_.startReceiving(
            state_data_receiving_host_, state_data_receiving_port_,
            [this](const StateData &data) { state_data_callback(data); })) {
      RCLCPP_ERROR(this->get_logger(),
                   "Init state data receiving failed. Host: %s, Port: %d",
                   state_data_receiving_host_.c_str(),
                   state_data_receiving_port_);
    }

    PilotData pilot_data;
    pilot_data.chassis_vel[0] = 1.0;
    pilot_data.chassis_vel[1] = 2.0;
    pilot_data.chassis_vel[2] = 3.0;
    if (!communication_.sendPilotData(pilot_data, pilot_data_sending_host_,
                                      pilot_data_sending_port_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Send pilot data failed. Host: %s, Port: %d",
                   pilot_data_sending_host_.c_str(), pilot_data_sending_port_);
    }
  }

private:
  void state_data_callback(const StateData &data) {
    RCLCPP_INFO(this->get_logger(), "Received state data:(%f, %f, %f)",
                data.gimbal_faceing[0], data.gimbal_faceing[1],
                data.gimbal_faceing[2]);
  }

  std::string pilot_data_sending_host_;
  int pilot_data_sending_port_;
  std::string state_data_receiving_host_;
  int state_data_receiving_port_;

  Communication communication_;
};
} // namespace autopilot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autopilot::AutopilotServer)