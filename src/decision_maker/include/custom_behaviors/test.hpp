#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <cstdint>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace decision {
class Test : public BT::SyncActionNode {
public:
  Test(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  ~Test() override = default;

  BT::NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("Test Node"), "hello ! %s %d",
                getInput<std::string>("msg").value().c_str(),
                getInput<uint16_t>("val").value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("msg", "dbg msg"),
            BT::InputPort<uint16_t>("val", "dbg val")};
  }

private:
};
} // namespace decision