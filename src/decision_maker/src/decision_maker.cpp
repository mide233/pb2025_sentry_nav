#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "custom_behaviors/blackboard_manager.hpp"
#include "custom_behaviors/find_enemy.hpp"
#include "custom_behaviors/nav_to_pose.hpp"
#include "custom_behaviors/set_spin.hpp"
#include "custom_behaviors/test.hpp"
#include "custom_types.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autopilot_interfaces/msg/detail/state__struct.hpp>
#include <autopilot_interfaces/msg/state.hpp>
#include <autopilot_interfaces/msg/vel_stamped.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

namespace decision {
std::shared_ptr<autopilot_interfaces::msg::State>
    BlackboardManager::state_data = nullptr;

class DecisionMaker : public rclcpp::Node {
public:
  explicit DecisionMaker(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("decision_maker", options) {
    RCLCPP_INFO(this->get_logger(), "Decision maker node has been started");

    this->declare_parameter<std::string>("load_tree");
    this->declare_parameter<std::string>("custom_behaviors_export_path");
    this->declare_parameter<int>("groot_port", 5556);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("desired_enemy_distance_threshold", 1.0);
    this->get_parameter("load_tree", load_tree_name);
    this->get_parameter("custom_behaviors_export_path",
                        custom_behaviors_save_path);
    this->get_parameter("groot_port", groot_port);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("desired_enemy_distance_threshold",
                        desired_enemy_distance_threshold_);

    this->get_parameter_or("send_goal_timeout_ms", nav_send_goal_timeout_,
                           1000);

    auto tree_dir =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("decision_maker")) /
        "trees";

    RCLCPP_INFO(this->get_logger(), "Got tree file path: %s", tree_dir.c_str());

    node_logger_ = std::make_shared<rclcpp::Logger>(this->get_logger());
    nav_action_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    state_subscription_ =
        this->create_subscription<autopilot_interfaces::msg::State>(
            "autopilot/state", 10,
            [&](const autopilot_interfaces::msg::State::SharedPtr msg) {
              decision::BlackboardManager::updateStateData(msg);
            });
    spin_cmd_publisher_ =
        this->create_publisher<autopilot_interfaces::msg::VelStamped>(
            "autopilot/decision/spin", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    node_clock_ = this->get_clock();

    tree_factory_.registerNodeType<decision::Test>("Test");
    tree_factory_.registerNodeType<decision::NavToPose>(
        "NavToPose", nav_action_client_, node_logger_, nav_send_goal_timeout_);
    tree_factory_.registerNodeType<decision::SetSpin>("SetSpin",
                                                      spin_cmd_publisher_);
    tree_factory_.registerNodeType<decision::FindEnemy>(
        "FindEnemy", tf_buffer_, base_frame_, odom_frame_,
        desired_enemy_distance_threshold_);
    tree_factory_.registerNodeType<decision::BlackboardManager>(
        "BlackboardManager", node_clock_);
    save_custom_behaviors(tree_factory_, custom_behaviors_save_path);

    tree_ = tree_factory_.createTreeFromFile(tree_dir / load_tree_name);
    debug_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port);

    tree_tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(250),
                                               [this]() { tree_.tickOnce(); });
  }

private:
  void save_custom_behaviors(BT::BehaviorTreeFactory &factory,
                             std::string save_path) {
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream export_file(save_path);

    export_file << xml_models;
    export_file.close();
  }

  rclcpp::TimerBase::SharedPtr tree_tick_timer_;

  int nav_send_goal_timeout_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::Subscription<autopilot_interfaces::msg::State>::SharedPtr
      state_subscription_;
  rclcpp::Publisher<autopilot_interfaces::msg::VelStamped>::SharedPtr
      spin_cmd_publisher_;
  std::shared_ptr<rclcpp::Logger> node_logger_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rclcpp::Clock> node_clock_;

  std::string load_tree_name;
  std::string custom_behaviors_save_path;
  BT::Tree tree_;
  BT::BehaviorTreeFactory tree_factory_;

  int desired_enemy_distance_threshold_;
  std::string odom_frame_;
  std::string base_frame_;

  int groot_port;
  std::shared_ptr<BT::Groot2Publisher> debug_publisher_;
};
} // namespace decision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision::DecisionMaker)