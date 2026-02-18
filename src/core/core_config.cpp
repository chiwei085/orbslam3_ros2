/**
 * @file core_config.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Configure-time parameter declaration and normalization.
 *
 * Rationale:
 * - Clamp invalid numeric values to keep runtime deterministic under bad
 * configs.
 * - Keep compatibility aliases to avoid breaking existing launch files.
 */
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

#include "orbslam3_ros2/core/core_config.hpp"

namespace orbslam3_ros2::core
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

}  // namespace

CoreParams load_core_params(rclcpp_lifecycle::LifecycleNode& node) {
    CoreParams params;
    params.map_frame = node.declare_parameter("map_frame", std::string("map"));
    params.camera_frame =
        node.declare_parameter("camera_frame", std::string(""));
    params.publish_odom = node.declare_parameter("publish_odom", false);
    params.publish_tf = node.declare_parameter("publish_tf", true);
    params.observation_topic = node.declare_parameter(
        "observation_topic", std::string("/orbslam3/observations"));
    params.camera_pose_topic = node.declare_parameter(
        "camera_pose_topic", std::string("/orbslam3/camera_pose"));
    params.odom_topic =
        node.declare_parameter("odom_topic", std::string("/orbslam3/odom"));
    params.voc_file = node.declare_parameter("voc_file", std::string(""));
    params.voc_uri = node.declare_parameter("voc_uri", std::string(""));
    params.settings_file =
        node.declare_parameter("settings_file", std::string(""));
    params.settings_uri =
        node.declare_parameter("settings_uri", std::string(""));
    params.use_viewer = node.declare_parameter("use_viewer", false);
    params.test_mode_skip_orb_init =
        node.declare_parameter("test_mode_skip_orb_init", false);
    params.test_mode_track_delay_ms = declare_i64_min_t(
        node, "test_mode_track_delay_ms", std::int64_t{0}, std::int64_t{0});
    params.qos_keep_last = declare_i64_min_t(node, "qos_keep_last",
                                             std::int64_t{10}, std::int64_t{1});
    params.pending_queue_size = declare_size_min_t(
        node, "pending_queue_size", std::int64_t{10}, std::int64_t{1});
    params.latest_keep_last = declare_size_min_t(
        node, "latest_keep_last", std::int64_t{3}, std::int64_t{1});
    params.latest_take = declare_size_min_t(node, "latest_take",
                                            std::int64_t{1}, std::int64_t{1});
    params.worker_time_budget_us = declare_i64_min_t(
        node, "worker_time_budget_us", std::int64_t{8000}, std::int64_t{0});
    const auto worker_time_budget_ms_compat = declare_i64_min_t(
        node, "worker_time_budget_ms", std::int64_t{-1}, std::int64_t{-1});
    // Trade-off: keep legacy parameter for compatibility, but canonicalize to
    // us.
    if (worker_time_budget_ms_compat >= 0) {
        params.worker_time_budget_us = worker_time_budget_ms_compat * 1000;
        RCLCPP_WARN(node.get_logger(),
                    "Parameter 'worker_time_budget_ms' is deprecated; use "
                    "'worker_time_budget_us'.");
    }
    params.worker_idle_sleep_us = declare_i64_min_t(
        node, "worker_idle_sleep_us", std::int64_t{0}, std::int64_t{0});
    params.no_work_wait_us = declare_i64_min_t(
        node, "no_work_wait_us", std::int64_t{2000}, std::int64_t{0});
    params.max_frame_age_ms = declare_i64_min_t(
        node, "max_frame_age_ms", std::int64_t{0}, std::int64_t{0});
    params.lost_auto_reset_enable =
        node.declare_parameter("lost_auto_reset_enable", false);
    params.lost_auto_reset_frames = declare_i64_min_t(
        node, "lost_auto_reset_frames", std::int64_t{30}, std::int64_t{1});
    params.pointcloud_max_points = declare_size_min_t(
        node, "pointcloud_max_points", std::int64_t{5000}, std::int64_t{1});
    const auto backlog_drop_keep_last_compat = declare_i64_min_t(
        node, "backlog_drop_keep_last", std::int64_t{-1}, std::int64_t{-1});
    // Legacy alias stays readable to prevent rollout breaks in existing
    // presets.
    if (backlog_drop_keep_last_compat >= 0) {
        params.latest_keep_last = static_cast<std::size_t>(
            std::max(backlog_drop_keep_last_compat, std::int64_t{1}));
        RCLCPP_WARN(node.get_logger(),
                    "Parameter 'backlog_drop_keep_last' is deprecated; use "
                    "'latest_keep_last'.");
    }
    params.stats_period_ms = declare_i64_min_t(
        node, "sync_stats_period_ms", std::int64_t{1000}, std::int64_t{1});
    params.diag_period_ms = declare_i64_min_t(
        node, "diagnostics_period_ms", std::int64_t{1000}, std::int64_t{200});
    return params;
}

}  // namespace orbslam3_ros2::core
