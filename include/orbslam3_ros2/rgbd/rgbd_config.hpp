/**
 * @file rgbd_config.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for rgbd config.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <string>

#include "orbslam3_ros2/sync/rgbd_sync.hpp"

namespace orbslam3_ros2::rgbd::config
{

enum class RgbEncodingKind
{
    kBgr8,
    kRgb8,
    kMono8,
    kUnsupported
};

std::uint64_t ms_to_ns(std::uint64_t ms) noexcept;

bool is_depth_encoding_alias_compatible(
    const std::string& msg_encoding,
    const std::string& override_encoding) noexcept;

RgbEncodingKind classify_rgb_encoding(const std::string& encoding) noexcept;

}  // namespace orbslam3_ros2::rgbd::config

namespace orbslam3_ros2::rgbd
{

struct RgbdFrontendParams
{
    using ImgPtr = sensor_msgs::msg::Image::ConstSharedPtr;
    using SyncConfig = orbslam3_ros2::sync::RgbdSync<ImgPtr, ImgPtr>::Config;

    SyncConfig sync_cfg{};
    std::string rgb_topic{};
    std::string depth_topic{};
    std::string observation_topic{"/orbslam3/observations"};
    double depth_scale{0.001};
    double depth_max_m{0.0};
    std::string depth_encoding_override{};
    bool rgb_clone{false};
    bool depth_clone{false};
    std::int64_t qos_keep_last{10};
};

RgbdFrontendParams load_rgbd_frontend_params(rclcpp::Node& node);

}  // namespace orbslam3_ros2::rgbd
