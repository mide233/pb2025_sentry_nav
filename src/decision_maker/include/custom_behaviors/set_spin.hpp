#include "behaviortree_cpp/bt_factory.h"
#include <autopilot_interfaces/msg/detail/vel_stamped__struct.hpp>
#include <autopilot_interfaces/msg/vel_stamped.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>

namespace decision {
class SetSpin : public BT::SyncActionNode {
public:
  SetSpin(const std::string &name, const BT::NodeConfig &config,
          rclcpp::Publisher<autopilot_interfaces::msg::VelStamped>::SharedPtr
              spin_vel_pub)
      : BT::SyncActionNode(name, config), spin_vel_pub_(spin_vel_pub){};

  ~SetSpin() override = default;

  BT::NodeStatus tick() override {
    autopilot_interfaces::msg::VelStamped msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    msg.header.stamp = ros_clock.now();
    msg.vel = getInput<double>("spin_vel").value_or(0.0);
    spin_vel_pub_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort("spin_vel", "chassis spin velocity")};
  }

private:
  rclcpp::Publisher<autopilot_interfaces::msg::VelStamped>::SharedPtr
      spin_vel_pub_;
};
} // namespace decision