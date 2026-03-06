#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"
#include <autopilot_interfaces/msg/detail/vel_stamped__struct.hpp>
#include <autopilot_interfaces/msg/vel_stamped.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

namespace decision {
class FindEnemy : public BT::SyncActionNode {
public:
  FindEnemy(const std::string &name, const BT::NodeConfig &config,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string &base_frame, const std::string &odom_frame,
            const double desired_distance)
      : BT::SyncActionNode(name, config), tf_buffer_(tf_buffer),
        base_frame_(base_frame), odom_frame_(odom_frame),
        desired_distance_(desired_distance){};

  ~FindEnemy() override = default;

  BT::NodeStatus tick() override {

    auto target_position =
        getInput<std::string>("target_position").value_or("");
    if (target_position.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    auto target_position_parts = BT::splitString(target_position, ',');
    geometry_msgs::msg::PointStamped target_point;
    target_point.header.frame_id = base_frame_; // make it use parameter
    target_point.header.stamp = rclcpp::Time(0);
    target_point.point.x =
        BT::convertFromString<double>(target_position_parts[0]);
    target_point.point.y =
        BT::convertFromString<double>(target_position_parts[1]);
    target_point.point.z = 0;

    try {
      geometry_msgs::msg::PointStamped target_in_odom;
      tf_buffer_->transform(target_point, target_in_odom, odom_frame_);

      try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                        rclcpp::Time(0));

        std::stringstream nav_goal_stringstream;

        double self_x = transform.transform.translation.x;
        double self_y = transform.transform.translation.y;
        double target_x = target_in_odom.point.x;
        double target_y = target_in_odom.point.y;
        double d_x = target_x - self_x;
        double d_y = target_y - self_y;
        double distance = std::sqrt(d_x * d_x + d_y * d_y);
        if (distance > desired_distance_) {
          double scale = (distance - desired_distance_) / distance;

          nav_goal_stringstream
              << std::fixed << std::setprecision(1) << self_x + d_x * scale
              << "," << self_y + d_y * scale << ",0.0,0.0,0.0,0.0,0.0";
        } else {
          nav_goal_stringstream << std::fixed << std::setprecision(1) << self_x
                                << "," << self_y << ",0.0,0.0,0.0,0.0,0.0";
        }

        std::string nav_goal_str = nav_goal_stringstream.str();
        setOutput("nav_goal", nav_goal_str);

        RCLCPP_INFO(rclcpp::get_logger("FindEnemy"),
                    "Transformed target position to nav goal: %s",
                    nav_goal_str.c_str());

        return BT::NodeStatus::SUCCESS;

      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("FindEnemy"),
                     "Failed to lookup transform: %s", ex.what());
        return BT::NodeStatus::FAILURE;
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("FindEnemy"),
                   "Failed to transform target position: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("target_position",
                                   "xy target posi in base frame"),
        BT::OutputPort<std::string>("nav_goal", "xyzabcw goal in odom frame")};
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string base_frame_;
  std::string odom_frame_;
  double desired_distance_;
};
} // namespace decision