#include <autopilot_interfaces/msg/detail/vel_stamped__struct.hpp>
#include <autopilot_interfaces/msg/state.hpp>
#include <autopilot_interfaces/msg/vel_stamped.hpp>
#include <cstdint>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.hpp>
#include <functional>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "communication.hpp"
#include "diagnosis.hpp"
#include "fields.hpp"

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
    this->declare_parameter<int>("expected_slam_diagnosis_reporters", 2);
    this->declare_parameter<int>("expected_relocation_diagnosis_reporters", 3);

    this->declare_parameter<int>("startup_time", 15);

    this->get_parameter("pilot_data_sending_host", pilot_data_sending_host_);
    this->get_parameter("pilot_data_sending_port", pilot_data_sending_port_);
    this->get_parameter("state_data_receiving_host",
                        state_data_receiving_host_);
    this->get_parameter("state_data_receiving_port",
                        state_data_receiving_port_);
    this->get_parameter("expected_slam_diagnosis_reporters",
                        expected_slam_diagnosis_reporters_);
    this->get_parameter("expected_relocation_diagnosis_reporters",
                        expected_relocation_diagnosis_reporters_);
    this->get_parameter("startup_time", startup_time_);

    diagnosis_ = std::make_unique<Diagnosis>(
        startup_time_, expected_slam_diagnosis_reporters_,
        expected_relocation_diagnosis_reporters_);

    gicp_control_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "small_gicp/reset_when_err", 10);
    state_publisher_ = this->create_publisher<autopilot_interfaces::msg::State>(
        "autopilot/state", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        this->get_parameter("cmd_vel_topic").as_string(), 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          geometry_msgs::msg::TwistStamped cmd_vel_stamped;
          cmd_vel_stamped.header.stamp = this->get_clock()->now();
          cmd_vel_stamped.twist = *msg;
          *last_cmd_vel_msg_ = cmd_vel_stamped;
        });
    cmd_spin_sub_ =
        this->create_subscription<autopilot_interfaces::msg::VelStamped>(
            "autopilot/decision/spin", 10,
            [this](const autopilot_interfaces::msg::VelStamped::SharedPtr msg) {
              last_cmd_spin_msg_ = msg;
            });
    diagnostic_sub_ =
        this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", 10,
            [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
              last_pilot_diag_ = diagnosis_->update_health_callback(
                  msg, is_pilot_enabled_by_client_);
              diagnosis_->required_restart(
                  last_pilot_diag_ == PilotDiag::FATAL ||
                  !diagnosis_->set_nav_mode(desired_nav_mode_));
            });

    pilot_data_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), [this]() { pilot_data_update(); });

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
    is_pilot_enabled_by_client_ = data.autopilot_enabled;
    desired_nav_mode_ = data.desired_nav_mode;

    auto msg = std_msgs::msg::Bool();
    msg.data = !is_pilot_enabled_by_client_;
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
    state_msg.target_position[0] = data.target_position[0];
    state_msg.target_position[1] = data.target_position[1];
    state_msg.target_position[2] = data.target_position[2];
    state_publisher_->publish(state_msg);
  }

  void pilot_data_update() {
    PilotData pilot_data;
    pilot_data.chassis_vel[0] = 0.0;
    pilot_data.chassis_vel[1] = 0.0;
    pilot_data.chassis_vel[2] = 0.0;
    pilot_data.pilot_valid = false;

    pilot_data.pilot_state = last_pilot_diag_;
    pilot_data.current_nav_mode = diagnosis_->get_current_nav_mode();

    if (last_cmd_vel_msg_) {
      rclcpp::Time cmd_vel_stamp(last_cmd_vel_msg_->header.stamp);
      if ((this->get_clock()->now() - cmd_vel_stamp).seconds() < 0.5) {
        pilot_data.chassis_vel[0] = last_cmd_vel_msg_->twist.linear.x;
        pilot_data.chassis_vel[1] = last_cmd_vel_msg_->twist.linear.y;
        pilot_data.chassis_vel[2] = 2.0;
        pilot_data.pilot_valid = true;
      }
    }
    if (last_cmd_spin_msg_) {
      rclcpp::Time spin_cmd_stamp(last_cmd_spin_msg_->header.stamp);
      if ((this->get_clock()->now() - spin_cmd_stamp).seconds() < 1) {
        pilot_data.chassis_vel[2] = last_cmd_spin_msg_->vel;
        pilot_data.pilot_valid = true;
      }
    }

    if (!communication_.sendPilotData(pilot_data, pilot_data_sending_host_,
                                      pilot_data_sending_port_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Send pilot data failed. Host: %s, Port: %d",
                   pilot_data_sending_host_.c_str(), pilot_data_sending_port_);
    }
  }

  void diagnostic_callback(
      const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {}

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gicp_control_publisher_;
  rclcpp::Publisher<autopilot_interfaces::msg::State>::SharedPtr
      state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<autopilot_interfaces::msg::VelStamped>::SharedPtr
      cmd_spin_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_sub_;

  std::unique_ptr<Diagnosis> diagnosis_;
  PilotDiag last_pilot_diag_ = PilotDiag::STARTING;
  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_vel_msg_;
  autopilot_interfaces::msg::VelStamped::SharedPtr last_cmd_spin_msg_;
  rclcpp::TimerBase::SharedPtr pilot_data_update_timer_;
  bool is_pilot_enabled_by_client_ = false;
  NavMode desired_nav_mode_ = NavMode::UNKNOWN;

  std::string pilot_data_sending_host_;
  int pilot_data_sending_port_;
  std::string state_data_receiving_host_;
  int state_data_receiving_port_;
  uint16_t expected_slam_diagnosis_reporters_;
  uint16_t expected_relocation_diagnosis_reporters_;
  uint16_t startup_time_;

  Communication communication_;
};
} // namespace autopilot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autopilot::AutopilotServer)