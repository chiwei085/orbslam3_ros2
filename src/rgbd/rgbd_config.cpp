/**
 * @file rgbd_config.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for rgbd config.
 */
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cctype>
#include <string>

#include "orbslam3_ros2/rgbd/rgbd_config.hpp"

namespace orbslam3_ros2::rgbd::config
{
namespace
{
constexpr std::uint64_t kNsPerMs = 1'000'000ULL;
}  // namespace

std::uint64_t ms_to_ns(const std::uint64_t ms) noexcept {
    return ms * kNsPerMs;
}

bool is_depth_encoding_alias_compatible(
    const std::string& msg_encoding,
    const std::string& override_encoding) noexcept {
    auto to_lower_ascii = [](const std::string& s) {
        std::string out = s;
        for (char& ch : out) {
            ch =
                static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return out;
    };
    const std::string msg = to_lower_ascii(msg_encoding);
    const std::string ov = to_lower_ascii(override_encoding);
    if (msg == ov) {
        return true;
    }
    auto is_u16sc1_alias = [](const std::string& e) {
        return e == sensor_msgs::image_encodings::TYPE_16UC1 ||
               e == sensor_msgs::image_encodings::MONO16 || e == "16uc1" ||
               e == "mono16";
    };
    return is_u16sc1_alias(msg) && is_u16sc1_alias(ov);
}

RgbEncodingKind classify_rgb_encoding(const std::string& encoding) noexcept {
    if (encoding == sensor_msgs::image_encodings::BGR8) {
        return RgbEncodingKind::kBgr8;
    }
    if (encoding == sensor_msgs::image_encodings::RGB8) {
        return RgbEncodingKind::kRgb8;
    }
    if (encoding == sensor_msgs::image_encodings::MONO8) {
        return RgbEncodingKind::kMono8;
    }
    return RgbEncodingKind::kUnsupported;
}

}  // namespace orbslam3_ros2::rgbd::config

namespace orbslam3_ros2::rgbd
{
namespace
{

template <typename NodeT>
std::int64_t declare_i64_min_t(NodeT& node, const std::string& name,
                               const std::int64_t default_value,
                               const std::int64_t minimum) {
    return std::max(node.declare_parameter(name, default_value), minimum);
}

template <typename NodeT>
std::size_t declare_size_min_t(NodeT& node, const std::string& name,
                               const std::int64_t default_value,
                               const std::int64_t minimum) {
    return static_cast<std::size_t>(
        declare_i64_min_t(node, name, default_value, minimum));
}

template <typename NodeT>
RgbdFrontendParams load_rgbd_frontend_params_impl(NodeT& node) {
    RgbdFrontendParams params;
    params.sync_cfg.queue_size = declare_size_min_t(
        node, "sync_queue_size", std::int64_t{30}, std::int64_t{1});
    params.sync_cfg.max_dt_ns =
        config::ms_to_ns(static_cast<std::uint64_t>(declare_i64_min_t(
            node, "sync_max_dt_ms", std::int64_t{10}, std::int64_t{1})));
    params.sync_cfg.enable_span_guard =
        node.declare_parameter("sync_enable_span_guard", true);
    params.sync_cfg.max_span_ns =
        config::ms_to_ns(static_cast<std::uint64_t>(declare_i64_min_t(
            node, "sync_max_span_ms", std::int64_t{500}, std::int64_t{1})));
    params.sync_cfg.enable_huge_jump_reset =
        node.declare_parameter("sync_enable_huge_jump_reset", true);
    params.sync_cfg.huge_jump_ns =
        config::ms_to_ns(static_cast<std::uint64_t>(declare_i64_min_t(
            node, "sync_huge_jump_ms", std::int64_t{2000}, std::int64_t{1})));
    params.sync_cfg.reset_on_time_backwards =
        node.declare_parameter("sync_reset_on_time_backwards", true);
    params.sync_cfg.validate_and_sanitize();
    params.rgb_topic = node.declare_parameter(
        "rgb_topic", std::string("/camera/color/image_raw"));
    params.depth_topic = node.declare_parameter(
        "depth_topic", std::string("/camera/aligned_depth_to_color/image_raw"));
    params.observation_topic = node.declare_parameter(
        "observation_topic", std::string("/orbslam3/observations"));
    params.depth_scale = node.declare_parameter("depth_scale", 0.001);
    params.depth_max_m = node.declare_parameter("depth_max_m", 0.0);
    params.depth_encoding_override =
        node.declare_parameter("depth_encoding_override", std::string(""));
    params.rgb_clone = node.declare_parameter("rgb_clone", false);
    params.depth_clone = node.declare_parameter("depth_clone", false);
    const auto observation_rgbd_topic_compat =
        node.declare_parameter("observation_rgbd_topic", std::string(""));
    if (!observation_rgbd_topic_compat.empty()) {
        params.observation_topic = observation_rgbd_topic_compat;
        RCLCPP_WARN(node.get_logger(),
                    "Parameter 'observation_rgbd_topic' is deprecated; use "
                    "'observation_topic'.");
    }
    params.qos_keep_last = declare_i64_min_t(node, "qos_keep_last",
                                             std::int64_t{10}, std::int64_t{1});
    return params;
}

}  // namespace

RgbdFrontendParams load_rgbd_frontend_params(rclcpp::Node& node) {
    return load_rgbd_frontend_params_impl(node);
}

}  // namespace orbslam3_ros2::rgbd
