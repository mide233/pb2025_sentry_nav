#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "custom_behaviors/nav_to_pose.hpp"
#include "custom_behaviors/test.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

namespace decision {
class DecisionMaker : public rclcpp::Node {
public:
  explicit DecisionMaker(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("decision_maker", options) {
    RCLCPP_INFO(this->get_logger(), "Decision maker node has been started");

    this->declare_parameter<std::string>("load_tree");
    this->declare_parameter<std::string>("custom_behaviors_export_path");
    this->declare_parameter<int>("groot_port", 5556);
    this->get_parameter("load_tree", load_tree_name);
    this->get_parameter("custom_behaviors_export_path",
                        custom_behaviors_save_path);
    this->get_parameter("groot_port", groot_port);

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

    tree_factory_.registerNodeType<decision::Test>("Test");
    tree_factory_.registerNodeType<decision::NavToPose>(
        "NavToPose", nav_action_client_, node_logger_, nav_send_goal_timeout_);
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

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  std::shared_ptr<rclcpp::Logger> node_logger_;

  int nav_send_goal_timeout_;

  std::string load_tree_name;
  std::string custom_behaviors_save_path;
  int groot_port;

  BT::Tree tree_;
  BT::BehaviorTreeFactory tree_factory_;

  std::shared_ptr<BT::Groot2Publisher> debug_publisher_;
};
} // namespace decision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision::DecisionMaker)