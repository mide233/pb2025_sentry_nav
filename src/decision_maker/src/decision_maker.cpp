#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "custom_behaviors/test.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
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

    auto tree_dir =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory("decision_maker")) /
        "trees";

    RCLCPP_INFO(this->get_logger(), "Got tree file path: %s", tree_dir.c_str());

    tree_factory_.registerNodeType<decision::Test>("Test");
    save_custom_behaviors(tree_factory_, custom_behaviors_save_path);

    tree_ = tree_factory_.createTreeFromFile(tree_dir / load_tree_name);
    debug_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port);

    tree_thread_ = std::thread([&]() {
      while (rclcpp::ok()) {
        tree_.tickOnce();
        tree_.sleep(std::chrono::milliseconds(5));
      }
    });
  }

  ~DecisionMaker() {
    if (tree_thread_.joinable())
      tree_thread_.join();
  }

private:
  void save_custom_behaviors(BT::BehaviorTreeFactory &factory,
                             std::string save_path) {
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream export_file(save_path);

    export_file << xml_models;
    export_file.close();
  }

  std::string load_tree_name;
  std::string custom_behaviors_save_path;
  int groot_port;

  BT::Tree tree_;
  std::thread tree_thread_;
  BT::BehaviorTreeFactory tree_factory_;

  std::shared_ptr<BT::Groot2Publisher> debug_publisher_;
};
} // namespace decision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision::DecisionMaker)