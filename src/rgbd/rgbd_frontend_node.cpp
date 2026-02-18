/**
 * @file rgbd_frontend_node.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for rgbd frontend node.
 */
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>

#include "orbslam3_ros2/msg/observations.hpp"
#include "orbslam3_ros2/rgbd/rgbd_config.hpp"
#include "orbslam3_ros2/rgbd/rgbd_image_convert.hpp"
#include "orbslam3_ros2/sync/rgbd_sync.hpp"

namespace Rgbd = orbslam3_ros2::rgbd;
namespace Sync = orbslam3_ros2::sync;
namespace Msg = orbslam3_ros2::msg;

class RgbdFrontendNode final : public rclcpp::Node
{
public:
    using ImgPtr = sensor_msgs::msg::Image::ConstSharedPtr;
    using Pair = std::tuple<ImgPtr, ImgPtr, std::uint64_t, std::int64_t>;

    RgbdFrontendNode() : rclcpp::Node("rgbd_frontend_node"), sync_() {
        const auto params = Rgbd::load_rgbd_frontend_params(*this);
        sync_.set_config(params.sync_cfg);
        obs_topic_ = params.observation_topic;
        convert_cfg_.depth_scale = params.depth_scale;
        convert_cfg_.depth_max_m = params.depth_max_m;
        convert_cfg_.depth_encoding_override = params.depth_encoding_override;
        convert_cfg_.rgb_clone = params.rgb_clone;
        convert_cfg_.depth_clone = params.depth_clone;

        const auto qos = rclcpp::SensorDataQoS().keep_last(
            static_cast<std::size_t>(params.qos_keep_last));
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            params.rgb_topic, qos, [this](const ImgPtr msg) { on_rgb_(msg); });
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            params.depth_topic, qos,
            [this](const ImgPtr msg) { on_depth_(msg); });
        obs_pub_ = create_publisher<Msg::Observations>(obs_topic_, qos);

        RCLCPP_INFO(get_logger(),
                    "RGBD frontend started. rgb='%s' depth='%s' -> obs='%s'",
                    params.rgb_topic.c_str(), params.depth_topic.c_str(),
                    obs_topic_.c_str());
    }

private:
    static std::optional<std::uint64_t> stamp_to_ns_(
        const builtin_interfaces::msg::Time& stamp) {
        const auto ns = rclcpp::Time(stamp).nanoseconds();
        if (ns < 0) {
            return std::nullopt;
        }
        return static_cast<std::uint64_t>(ns);
    }

    void publish_pair_(Pair&& pair) {
        const auto rgb_in = std::get<0>(pair);
        const auto depth_in = std::get<1>(pair);
        auto im_rgb_bgr = Rgbd::convert_rgb_to_bgr(
            rgb_in, convert_cfg_.rgb_clone, get_logger(), *get_clock());
        if (!im_rgb_bgr) {
            return;
        }
        auto im_depth_m = Rgbd::convert_depth_to_meters(
            depth_in, convert_cfg_, get_logger(), *get_clock());
        if (!im_depth_m) {
            return;
        }
        Rgbd::sanitize_depth_in_place(*im_depth_m, convert_cfg_.depth_max_m);

        cv_bridge::CvImage rgb_cv(
            rgb_in->header, sensor_msgs::image_encodings::BGR8, *im_rgb_bgr);
        cv_bridge::CvImage depth_cv(depth_in->header,
                                    sensor_msgs::image_encodings::TYPE_32FC1,
                                    *im_depth_m);
        auto rgb_msg = rgb_cv.toImageMsg();
        auto depth_msg = depth_cv.toImageMsg();

        Msg::Observations out;
        out.header = rgb_in->header;
        out.mode = Msg::Observations::MODE_RGBD;
        out.images.reserve(2);
        out.images.push_back(*rgb_msg);
        out.images.push_back(*depth_msg);
        out.camera_frame = out.header.frame_id;
        out.t_track_ns = std::get<2>(pair);
        out.dt_ns = std::get<3>(pair);
        obs_pub_->publish(out);
    }

    void on_rgb_(const ImgPtr& msg) {
        const auto t_ns = stamp_to_ns_(msg->header.stamp);
        if (!t_ns) {
            return;
        }
        std::optional<Pair> pair;
        {
            std::lock_guard lock(sync_mutex_);
            pair = sync_.push_rgb(msg, *t_ns);
        }
        if (pair) {
            publish_pair_(std::move(*pair));
        }
    }

    void on_depth_(const ImgPtr& msg) {
        const auto t_ns = stamp_to_ns_(msg->header.stamp);
        if (!t_ns) {
            return;
        }
        std::optional<Pair> pair;
        {
            std::lock_guard lock(sync_mutex_);
            pair = sync_.push_depth(msg, *t_ns);
        }
        if (pair) {
            publish_pair_(std::move(*pair));
        }
    }

private:
    std::mutex sync_mutex_;
    Sync::RgbdSync<ImgPtr, ImgPtr> sync_;
    Rgbd::ImageConvertConfig convert_cfg_{};
    std::string obs_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<Msg::Observations>::SharedPtr obs_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbdFrontendNode>());
    rclcpp::shutdown();
    return 0;
}
