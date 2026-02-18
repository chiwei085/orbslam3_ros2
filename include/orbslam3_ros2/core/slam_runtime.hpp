/**
 * @file slam_runtime.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for slam runtime.
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

#include "orbslam3_ros2/orbslam3/system.hpp"

namespace orbslam3_ros2::core::runtime
{

inline const char* tracking_state_to_string(const int state) noexcept {
    switch (state) {
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
            return "SYSTEM_NOT_READY";
        case ORB_SLAM3::Tracking::NO_IMAGES_YET:
            return "NO_IMAGES_YET";
        case ORB_SLAM3::Tracking::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case ORB_SLAM3::Tracking::OK:
            return "OK";
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return "RECENTLY_LOST";
        case ORB_SLAM3::Tracking::LOST:
            return "LOST";
        case ORB_SLAM3::Tracking::OK_KLT:
            return "OK_KLT";
        default:
            return "UNKNOWN";
    }
}

inline bool is_pose_publishable_tracking_state(const int state) noexcept {
    return state == ORB_SLAM3::Tracking::OK ||
           state == ORB_SLAM3::Tracking::OK_KLT ||
           state == ORB_SLAM3::Tracking::RECENTLY_LOST;
}

inline std::string resolve_camera_frame(
    const std::string& configured_camera_frame,
    const std::string& image_frame) {
    if (!configured_camera_frame.empty()) {
        return configured_camera_frame;
    }
    if (!image_frame.empty()) {
        return image_frame;
    }
    return "camera_link";
}

inline void publish_pose_outputs(
    const builtin_interfaces::msg::Time& stamp, const std::string& map_frame,
    const std::string& camera_frame, const Eigen::Vector3f& translation,
    const Eigen::Quaternionf& quat,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr&
        pose_pub,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom_pub,
    tf2_ros::TransformBroadcaster* tf_broadcaster) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame;
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    pose_pub->publish(pose_msg);

    if (odom_pub) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = camera_frame;
        odom_msg.pose.pose = pose_msg.pose;
        odom_pub->publish(odom_msg);
    }

    if (tf_broadcaster != nullptr) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame;
        tf_msg.child_frame_id = camera_frame;
        tf_msg.transform.translation.x = translation.x();
        tf_msg.transform.translation.y = translation.y();
        tf_msg.transform.translation.z = translation.z();
        tf_msg.transform.rotation.x = quat.x();
        tf_msg.transform.rotation.y = quat.y();
        tf_msg.transform.rotation.z = quat.z();
        tf_msg.transform.rotation.w = quat.w();
        tf_broadcaster->sendTransform(tf_msg);
    }
}

}  // namespace orbslam3_ros2::core::runtime
