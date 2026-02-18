/**
 * @file tracking_pipeline.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for tracking pipeline.
 */
#pragma once

#include "orbslam3_ros2/rgbd/rgbd_tracking_pipeline.hpp"

namespace orbslam3_ros2::core::pipeline
{

using TrackingPipelineConfig =
    orbslam3_ros2::rgbd::pipeline::TrackingPipelineConfig;
using TrackingPipelineInput =
    orbslam3_ros2::rgbd::pipeline::TrackingPipelineInput;
using TrackingPipelineResult =
    orbslam3_ros2::rgbd::pipeline::TrackingPipelineResult;

inline double track_time_seconds_from_ns(
    const std::uint64_t t_track_ns) noexcept {
    return orbslam3_ros2::rgbd::pipeline::track_time_seconds_from_ns(
        t_track_ns);
}

inline std::optional<TrackingPipelineResult> run_tracking_pipeline(
    ORB_SLAM3::System& system, const TrackingPipelineInput& input,
    const TrackingPipelineConfig& config, const rclcpp::Logger& logger,
    rclcpp::Clock& clock) {
    return orbslam3_ros2::rgbd::pipeline::run_tracking_pipeline(
        system, input, config, logger, clock);
}

}  // namespace orbslam3_ros2::core::pipeline
