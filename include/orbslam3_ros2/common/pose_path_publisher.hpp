#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "System.h"

namespace orbslam3_ros2::common
{

class PosePathPublisher
{
public:
  PosePathPublisher(
    rclcpp::Node & node,
    std::string topic_prefix,
    std::string world_frame_id)
  : world_frame_id_(std::move(world_frame_id)),
    pose_pub_(node.create_publisher<geometry_msgs::msg::PoseStamped>(topic_prefix + "/camera_pose", 10)),
    path_pub_(node.create_publisher<nav_msgs::msg::Path>(topic_prefix + "/trajectory", 10))
  {
    path_msg_.header.frame_id = world_frame_id_;
  }

  void reset()
  {
    path_msg_.poses.clear();
  }

  bool publish(const Sophus::SE3f & twc, const builtin_interfaces::msg::Time & stamp)
  {
    if (!is_finite(twc)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = stamp;

    pose_msg.pose.position.x = twc.translation().x();
    pose_msg.pose.position.y = twc.translation().y();
    pose_msg.pose.position.z = twc.translation().z();

    auto q = twc.unit_quaternion();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    pose_pub_->publish(pose_msg);

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_msg);
    path_pub_->publish(path_msg_);
    return true;
  }

private:
  static bool is_finite(const Sophus::SE3f & pose)
  {
    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    return std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
           std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
           std::isfinite(q.z());
  }

  std::string world_frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
};

}  // namespace orbslam3_ros2::common

