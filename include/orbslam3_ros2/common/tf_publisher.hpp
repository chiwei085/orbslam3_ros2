#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <utility>

#include "System.h"

namespace orbslam3_ros2::common
{

class TfPublisher
{
public:
    explicit TfPublisher(rclcpp::Node& node) : tf_broadcaster_(node) {}

    void publish(const Sophus::SE3f& pose, const std::string& frame_id,
                 const std::string& child_frame_id,
                 const builtin_interfaces::msg::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = frame_id;
        tf_msg.child_frame_id = child_frame_id;

        tf_msg.transform.translation.x = pose.translation().x();
        tf_msg.transform.translation.y = pose.translation().y();
        tf_msg.transform.translation.z = pose.translation().z();

        auto q = pose.unit_quaternion();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();

        tf_broadcaster_.sendTransform(tf_msg);
    }

private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace orbslam3_ros2::common
