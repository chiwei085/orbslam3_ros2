/**
 * @file tracking_cloud_node.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for tracking cloud node.
 */
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include "orbslam3_ros2/srv/get_tracked_points.hpp"

namespace
{
using namespace std::chrono_literals;
}

namespace Srv = orbslam3_ros2::srv;

class TrackingCloudNode final : public rclcpp::Node
{
public:
    TrackingCloudNode() : rclcpp::Node("tracking_cloud_node") {
        service_name_ =
            declare_parameter("tracked_points_service",
                              std::string("/orbslam3/get_tracked_points"));
        cloud_topic_ = declare_parameter(
            "cloud_topic", std::string("/orbslam3/tracking/points"));
        max_points_ =
            std::max(declare_parameter("max_points", std::int64_t{5000}),
                     std::int64_t{1});
        publish_hz_ = std::max(
            declare_parameter("publish_hz", std::int64_t{15}), std::int64_t{1});

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS());
        client_ = create_client<Srv::GetTrackedPoints>(service_name_);
        timer_ =
            create_wall_timer(std::chrono::milliseconds(1000 / publish_hz_),
                              [this]() { tick_(); });

        RCLCPP_INFO(get_logger(),
                    "Tracking cloud node started. service='%s' -> topic='%s'",
                    service_name_.c_str(), cloud_topic_.c_str());
    }

private:
    void tick_() {
        if (!client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Service not ready: %s",
                                 service_name_.c_str());
            return;
        }
        auto req = std::make_shared<Srv::GetTrackedPoints::Request>();
        req->max_points = static_cast<std::int32_t>(max_points_);
        auto future = client_->async_send_request(req);
        using FutureStatus = std::future_status;
        if (future.wait_for(50ms) != FutureStatus::ready) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Service timeout: %s", service_name_.c_str());
            return;
        }
        const auto resp = future.get();
        if (!resp->success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "GetTrackedPoints failed: %s",
                                 resp->message.c_str());
            return;
        }
        cloud_pub_->publish(resp->cloud);
    }

private:
    std::string service_name_;
    std::string cloud_topic_;
    std::int64_t max_points_{5000};
    std::int64_t publish_hz_{15};
    rclcpp::Client<Srv::GetTrackedPoints>::SharedPtr client_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingCloudNode>());
    rclcpp::shutdown();
    return 0;
}
