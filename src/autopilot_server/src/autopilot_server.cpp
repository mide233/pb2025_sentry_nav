#include <autopilot_interfaces/msg/detail/vel_stamped__struct.hpp>
#include <autopilot_interfaces/msg/state.hpp>
#include <autopilot_interfaces/msg/vel_stamped.hpp>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
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
    state_publisher_ = this->create_publisher<autopilot_interfaces::msg::State>(
        "autopilot/state", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        this->get_parameter("cmd_vel_topic").as_string(), 10,
        std::bind(&AutopilotServer::cmd_vel_callback, this,
                  std::placeholders::_1));
    cmd_spin_sub_ =
        this->create_subscription<autopilot_interfaces::msg::VelStamped>(
            "autopilot/decision/spin", 10,
            [this](const autopilot_interfaces::msg::VelStamped::SharedPtr msg) {
              last_cmd_spin_msg_ = msg;
            });

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

    auto state_msg = autopilot_interfaces::msg::State();
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.current_hp = data.current_hp;
    state_msg.game_state = data.game_state;
    state_msg.state_remain_time = data.state_remain_time;
    state_msg.rfid_state = data.rfid_state;
    state_msg.center_area_state = data.center_area_state;
    state_msg.projectile_allowance = data.projectile_allowance;
    state_msg.auto_aim_tracking = data.auto_aim_tracking;
    state_publisher_->publish(state_msg);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist msg) {
    PilotData pilot_data;
    pilot_data.chassis_vel[0] = msg.linear.x;
    pilot_data.chassis_vel[1] = msg.linear.y;
    pilot_data.chassis_vel[2] = 2.0;

    rclcpp::Time spin_cmd_stamp(last_cmd_spin_msg_->header.stamp);
    if (last_cmd_spin_msg_) {
      if ((this->get_clock()->now() - spin_cmd_stamp).seconds() < 1) {
        pilot_data.chassis_vel[2] = last_cmd_spin_msg_->vel;
      }
    }

    if (!communication_.sendPilotData(pilot_data, pilot_data_sending_host_,
                                      pilot_data_sending_port_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Send pilot data failed. Host: %s, Port: %d",
                   pilot_data_sending_host_.c_str(), pilot_data_sending_port_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gicp_control_publisher_;
  rclcpp::Publisher<autopilot_interfaces::msg::State>::SharedPtr
      state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<autopilot_interfaces::msg::VelStamped>::SharedPtr
      cmd_spin_sub_;

  autopilot_interfaces::msg::VelStamped::SharedPtr last_cmd_spin_msg_;

  std::string pilot_data_sending_host_;
  int pilot_data_sending_port_;
  std::string state_data_receiving_host_;
  int state_data_receiving_port_;

  Communication communication_;
};
} // namespace autopilot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autopilot::AutopilotServer)