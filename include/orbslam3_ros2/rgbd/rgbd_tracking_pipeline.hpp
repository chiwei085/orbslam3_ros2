/**
 * @file rgbd_tracking_pipeline.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for rgbd tracking pipeline.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "orbslam3_ros2/core/slam_runtime.hpp"
#include "orbslam3_ros2/orbslam3/system.hpp"
#include "orbslam3_ros2/rgbd/rgbd_image_convert.hpp"

namespace orbslam3_ros2::rgbd::pipeline
{

inline double track_time_seconds_from_ns(
    const std::uint64_t t_track_ns) noexcept {
    // Keep one canonical conversion for ORB-SLAM3 input timestamp.
    return static_cast<double>(t_track_ns) * 1e-9;
}

struct TrackingPipelineConfig
{
    bool rgb_clone{false};
    bool depth_clone{false};
};

struct TrackingPipelineInput
{
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    std::uint64_t t_track_ns{0};
};

struct TrackingPipelineResult
{
    std::uint64_t track_time_us{0};
    int tracking_state{ORB_SLAM3::Tracking::SYSTEM_NOT_READY};
    std::size_t tracked_map_points{0};
    bool has_pose{false};
    Sophus::SE3f twc{};
};

inline std::optional<TrackingPipelineResult> run_tracking_pipeline(
    ORB_SLAM3::System& system, const TrackingPipelineInput& input,
    const TrackingPipelineConfig& config, const rclcpp::Logger& logger,
    rclcpp::Clock& clock) {
    if (input.rgb->width != input.depth->width ||
        input.rgb->height != input.depth->height) {
        RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                             "RGB/depth size mismatch. rgb=%ux%u depth=%ux%u "
                             "(depth may not be aligned to RGB)",
                             input.rgb->width, input.rgb->height,
                             input.depth->width, input.depth->height);
    }
    if (input.rgb->header.frame_id != input.depth->header.frame_id) {
        RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                             "RGB/depth frame_id mismatch. rgb='%s' depth='%s' "
                             "(pipeline may be inconsistent)",
                             input.rgb->header.frame_id.c_str(),
                             input.depth->header.frame_id.c_str());
    }

    auto mats =
        orbslam3_ros2::rgbd::convert_canonical_observation_to_tracking_mats(
            input.rgb, input.depth, config.rgb_clone, config.depth_clone,
            logger, clock);
    if (!mats) {
        return std::nullopt;
    }

    // Sample depth statistics before entering ORB-SLAM3 to catch invalid
    // values (NaN/Inf/extreme range) that can destabilize optimization.
    {
        const cv::Mat& d = mats->depth_meters;
        const int stride = 8;
        std::size_t sample_count = 0;
        std::size_t non_zero_count = 0;
        std::size_t nan_inf_count = 0;
        float min_depth = std::numeric_limits<float>::infinity();
        float max_depth = 0.0f;
        for (int r = 0; r < d.rows; r += stride) {
            const float* row = d.ptr<float>(r);
            for (int c = 0; c < d.cols; c += stride) {
                const float v = row[c];
                ++sample_count;
                if (!std::isfinite(v)) {
                    ++nan_inf_count;
                    continue;
                }
                if (v > 0.0f) {
                    ++non_zero_count;
                    if (v < min_depth) {
                        min_depth = v;
                    }
                    if (v > max_depth) {
                        max_depth = v;
                    }
                }
            }
        }
        if (sample_count > 0) {
            const double non_zero_ratio = static_cast<double>(non_zero_count) /
                                          static_cast<double>(sample_count);
            const bool bad_range = (non_zero_count > 0) &&
                                   (min_depth <= 0.0f || max_depth > 20.0f);
            if (nan_inf_count > 0 || bad_range || non_zero_ratio < 0.005) {
                RCLCPP_WARN_THROTTLE(
                    logger, clock, 2000,
                    "Depth sanity check: samples=%zu non_zero_ratio=%.4f "
                    "nan_inf=%zu min=%.4f max=%.4f "
                    "(expected indoor depth roughly 0-20m).",
                    sample_count, non_zero_ratio, nan_inf_count,
                    std::isfinite(min_depth) ? min_depth : 0.0f, max_depth);
            }
        }
    }

    // ORB-SLAM3 tracking timestamp: always derived from ROS message stamp
    // (not system_clock / steady_clock).
    const double t_sec = track_time_seconds_from_ns(input.t_track_ns);
    const auto track_start = std::chrono::steady_clock::now();
    const Sophus::SE3f tcw = orbslam3_ros2::orbslam3::track_rgbd(
        system, mats->rgb_bgr, mats->depth_meters, t_sec);
    const auto track_end = std::chrono::steady_clock::now();

    const auto track_time_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(track_end -
                                                              track_start)
            .count());
    const auto tracking_state = system.GetTrackingState();
    const auto tracked_map_points = system.GetTrackedMapPoints().size();
    const auto has_pose =
        orbslam3_ros2::core::runtime::is_pose_publishable_tracking_state(
            tracking_state);
    return TrackingPipelineResult{track_time_us, tracking_state,
                                  tracked_map_points, has_pose, tcw.inverse()};
}

}  // namespace orbslam3_ros2::rgbd::pipeline
