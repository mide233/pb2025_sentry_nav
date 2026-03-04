#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <autopilot_interfaces/msg/detail/state__struct.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <memory>
#include <string>

using namespace BT;
namespace decision {
class BlackboardManager : public SyncActionNode {
public:
  static std::shared_ptr<autopilot_interfaces::msg::State> state_data;

  BlackboardManager(const std::string &name, const NodeConfig &config)
      : SyncActionNode(name, config){};
  ~BlackboardManager() override = default;
  NodeStatus tick() override {
    if (state_data) {
      rclcpp::Clock ros_clock(
          RCL_ROS_TIME); // use the same time source of msg header stamp
      rclcpp::Time state_data_time(state_data->header.stamp);

      if ((ros_clock.now() - state_data_time).seconds() < 2) {
        setOutput<uint16_t>("current_hp", state_data->current_hp);
        setOutput<uint8_t>("game_state", state_data->game_state);
        setOutput<uint16_t>("state_remain_time", state_data->state_remain_time);
        setOutput<uint8_t>("rfid_state", state_data->rfid_state);
        setOutput<uint8_t>("center_area_state", state_data->center_area_state);
        setOutput<int64_t>("projectile_allowance",
                           state_data->projectile_allowance);
        setOutput<bool>("auto_aim_tracking", state_data->auto_aim_tracking);
      } else {
        resetAllPorts();
      }
    } else {
      resetAllPorts();
    }

    return NodeStatus::SUCCESS;
  };
  static PortsList providedPorts() {
    return {OutputPort<uint16_t>("current_hp"),
            OutputPort<uint8_t>("game_state"),
            OutputPort<uint16_t>("state_remain_time"),
            OutputPort<uint8_t>("rfid_state"),
            OutputPort<uint8_t>("center_area_state"),
            OutputPort<int64_t>("projectile_allowance"),
            OutputPort<bool>("auto_aim_tracking")};
  };

  static void
  updateStateData(std::shared_ptr<autopilot_interfaces::msg::State> data) {
    state_data = data;
  }

private:
  void resetAllPorts() {
    setOutput<uint16_t>("current_hp", 0);
    setOutput<uint8_t>("game_state", 0);
    setOutput<uint16_t>("state_remain_time", 0);
    setOutput<uint8_t>("rfid_state", 0);
    setOutput<uint8_t>("center_area_state", 0);
    setOutput<int64_t>("projectile_allowance", 0);
    setOutput<bool>("auto_aim_tracking", false);
  }
};
} // namespace decision
