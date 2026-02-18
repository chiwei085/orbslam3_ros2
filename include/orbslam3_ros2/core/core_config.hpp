#pragma once
/**
 * @file core_config.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Parameter contract for orbslam3_core_node.
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace orbslam3_ros2::core
{

/**
 * @brief Declared configuration parameters for Orbslam3CoreNode.
 *
 * Contract:
 * - Fields mirror ROS parameters and are read during configure transition.
 * - Numeric queue/time fields are clamped to safe lower bounds.
 * - QoS-relevant topic names are explicit to preserve deterministic gates.
 */
struct CoreParams
{
    std::string map_frame{"map"};
    std::string camera_frame{};
    bool publish_odom{false};
    bool publish_tf{true};
    std::string observation_topic{"/orbslam3/observations"};
    std::string camera_pose_topic{"/orbslam3/camera_pose"};
    std::string odom_topic{"/orbslam3/odom"};
    std::string voc_file{};
    std::string voc_uri{};
    std::string settings_file{};
    std::string settings_uri{};
    bool use_viewer{false};
    bool test_mode_skip_orb_init{false};
    std::int64_t test_mode_track_delay_ms{0};
    std::int64_t qos_keep_last{10};
    std::size_t pending_queue_size{10};
    std::size_t latest_keep_last{3};
    std::size_t latest_take{1};
    std::int64_t worker_time_budget_us{8000};
    std::int64_t worker_idle_sleep_us{0};
    std::int64_t no_work_wait_us{2000};
    std::int64_t max_frame_age_ms{0};
    bool lost_auto_reset_enable{false};
    std::int64_t lost_auto_reset_frames{30};
    std::size_t pointcloud_max_points{5000};
    std::int64_t stats_period_ms{1000};
    std::int64_t diag_period_ms{1000};
};

/**
 * @brief Declare and load core node parameters.
 *
 * @param node Lifecycle node used to declare/read parameters.
 * @return Normalized parameter snapshot for configure-time initialization.
 *
 * @note Backward-compatible aliases are still accepted with deprecation
 * warnings.
 * @warning Values violating minimum bounds are clamped, not rejected.
 */
CoreParams load_core_params(rclcpp_lifecycle::LifecycleNode& node);

}  // namespace orbslam3_ros2::core
