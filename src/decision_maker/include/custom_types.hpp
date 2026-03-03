#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/json_export.h"

#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace BT {
using StringView = std::string_view;

template <>
[[nodiscard]] inline geometry_msgs::msg::PointStamped
convertFromString<geometry_msgs::msg::PointStamped>(StringView str) {
  const auto parts = BT::splitString(str, ',');
  if (parts.size() != 3) {
    throw BT::RuntimeError("invalid input");
  }

  geometry_msgs::msg::PointStamped point;
  point.point.x = convertFromString<double>(parts[0]);
  point.point.y = convertFromString<double>(parts[1]);
  point.point.z = convertFromString<double>(parts[2]);
  return point;
}

inline void PointStampedToJson(nlohmann::json &dest,
                               const geometry_msgs::msg::PointStamped &point) {
  dest["x"] = point.point.x;
  dest["y"] = point.point.y;
  dest["z"] = point.point.z;
}

template <>
[[nodiscard]] inline geometry_msgs::msg::PoseStamped
convertFromString<geometry_msgs::msg::PoseStamped>(StringView str) {
  const auto parts = BT::splitString(str, ',');
  if (parts.size() != 7) {
    throw BT::RuntimeError("invalid input");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = convertFromString<double>(parts[0]);
  pose.pose.position.y = convertFromString<double>(parts[1]);
  pose.pose.position.z = convertFromString<double>(parts[2]);
  pose.pose.orientation.x = convertFromString<double>(parts[3]);
  pose.pose.orientation.y = convertFromString<double>(parts[4]);
  pose.pose.orientation.z = convertFromString<double>(parts[5]);
  pose.pose.orientation.w = convertFromString<double>(parts[6]);
  return pose;
}

inline void PoseStampedToJson(nlohmann::json &dest,
                              const geometry_msgs::msg::PoseStamped &pose) {
  dest["x"] = pose.pose.position.x;
  dest["y"] = pose.pose.position.y;
  dest["z"] = pose.pose.position.z;
  dest["qx"] = pose.pose.orientation.x;
  dest["qy"] = pose.pose.orientation.y;
  dest["qz"] = pose.pose.orientation.z;
  dest["qw"] = pose.pose.orientation.w;
}

} // namespace BT
