/**
 * @file path_node.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for path node.
 */
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>

class PathNode final : public rclcpp::Node
{
public:
    PathNode() : rclcpp::Node("path_node") {
        pose_topic_ = declare_parameter("pose_topic",
                                        std::string("/orbslam3/camera_pose"));
        path_topic_ =
            declare_parameter("path_topic", std::string("/orbslam3/path"));
        max_length_ = static_cast<std::size_t>(
            std::max(declare_parameter("path_max_length", std::int64_t{2000}),
                     std::int64_t{1}));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                on_pose_(msg);
            });
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            path_topic_, rclcpp::SystemDefaultsQoS());
        reset_srv_ = create_service<std_srvs::srv::Empty>(
            "/orbslam3/path/reset",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                std::lock_guard<std::mutex> lock(path_mutex_);
                path_msg_.poses.clear();
                path_msg_.header.frame_id.clear();
            });

        RCLCPP_INFO(get_logger(), "Path node started. pose='%s' -> path='%s'",
                    pose_topic_.c_str(), path_topic_.c_str());
    }

private:
    void on_pose_(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_msg_.header = msg->header;
        path_msg_.poses.push_back(*msg);
        while (path_msg_.poses.size() > max_length_) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }
        path_pub_->publish(path_msg_);
    }

private:
    std::string pose_topic_;
    std::string path_topic_;
    std::size_t max_length_{2000};
    std::mutex path_mutex_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathNode>());
    rclcpp::shutdown();
    return 0;
}
