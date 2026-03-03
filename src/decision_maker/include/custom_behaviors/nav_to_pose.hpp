#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "custom_types.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;
namespace decision {
class NavToPose : public StatefulActionNode {
public:
  NavToPose(const std::string &name, const NodeConfig &config,
            rclcpp_action::Client<NavigateToPose>::SharedPtr action_client,
            std::shared_ptr<rclcpp::Logger> logger, const int send_goal_timeout)
      : StatefulActionNode(name, config), action_client_(action_client),
        logger_(logger), send_goal_timeout_(send_goal_timeout) {
    RCLCPP_DEBUG(*logger_, "NAV2POSE NODE LOADED");
  }

  NodeStatus onStart() override {
    RCLCPP_DEBUG(*logger_, "NAV2POSE NODE ON START");
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
      RCLCPP_ERROR(*logger_, "goal is not set");
      return NodeStatus::FAILURE;
    }

    navigation_goal_.pose = goal.value();
    navigation_goal_.pose.header.frame_id = "map";
    navigation_goal_.pose.header.stamp = rclcpp::Clock().now();

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(*logger_, "Action server not available");
      return NodeStatus::FAILURE;
    }

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future_goal_handle =
        action_client_->async_send_goal(navigation_goal_, send_goal_options);

    RCLCPP_DEBUG(*logger_, "send goal timeout ms: %d", send_goal_timeout_);

    // 使用 wait_for 替代 spin_until_future_complete
    if (future_goal_handle.wait_for(std::chrono::milliseconds(
            send_goal_timeout_)) != std::future_status::ready) {
      RCLCPP_ERROR(*logger_, "send goal failed");
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(*logger_, "goal handle is null");
      return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(*logger_, "Navigating to pose [%f, %f, %f]",
                goal->pose.position.x, goal->pose.position.y,
                goal->pose.position.z);
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override {
    auto new_goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!new_goal) {
      RCLCPP_ERROR(*logger_, "goal is not set");
      return NodeStatus::FAILURE;
    }

    if ((new_goal.value().pose.position.x -
         navigation_goal_.pose.pose.position.x) >
            std::numeric_limits<double>::epsilon() ||
        (new_goal.value().pose.position.y -
         navigation_goal_.pose.pose.position.y) >
            std::numeric_limits<double>::epsilon() ||
        (new_goal.value().pose.position.z -
         navigation_goal_.pose.pose.position.z) >
            std::numeric_limits<double>::epsilon()) {
      RCLCPP_INFO(*logger_, "goal updated");

      navigation_goal_.pose = new_goal.value();
      navigation_goal_.pose.header.frame_id = "map";
      navigation_goal_.pose.header.stamp = rclcpp::Clock().now();

      auto send_goal_options =
          rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      auto future_goal_handle =
          action_client_->async_send_goal(navigation_goal_, send_goal_options);

      if (future_goal_handle.wait_for(std::chrono::milliseconds(
              send_goal_timeout_)) != std::future_status::ready) {
        RCLCPP_ERROR(*logger_, "send goal failed");
        return NodeStatus::FAILURE;
      }

      goal_handle_ = future_goal_handle.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(*logger_, "goal handle is null");
        return NodeStatus::FAILURE;
      }
      RCLCPP_INFO(*logger_, "Navigating to pose [%f, %f, %f]",
                  new_goal->pose.position.x, new_goal->pose.position.y,
                  new_goal->pose.position.z);
    }

    switch (goal_handle_->get_status()) {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      RCLCPP_INFO(*logger_, "goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_SUCCEEDED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_CANCELED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_ABORTED");
      break;
    default:
      RCLCPP_INFO(*logger_, "goal status: ERROR CODE");
      break;
    }

    if (goal_handle_->get_status() ==
        action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      return NodeStatus::SUCCESS;
    } else if (goal_handle_->get_status() ==
                   action_msgs::msg::GoalStatus::STATUS_ABORTED ||
               goal_handle_->get_status() ==
                   action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      return NodeStatus::FAILURE;
    } else {
      return NodeStatus::RUNNING;
    }
  }

  void onHalted() override {
    RCLCPP_INFO(*logger_, "goal halted");
    if (goal_handle_) {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (cancel_future.wait_for(std::chrono::seconds(1)) !=
          std::future_status::ready) {
        RCLCPP_ERROR(*logger_, "cancel goal failed");
      }
      RCLCPP_INFO(*logger_, "goal canceled");
    }
  }

  static PortsList providedPorts() {
    const char *description = "goal send to navigator.";
    return {InputPort<geometry_msgs::msg::PoseStamped>("goal", description)};
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  NavigateToPoseGoalHandle::SharedPtr goal_handle_;
  std::shared_ptr<rclcpp::Logger> logger_;

  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  int send_goal_timeout_;
};
} // namespace decision