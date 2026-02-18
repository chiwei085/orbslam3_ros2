/**
 * @file tracking_backend.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for tracking backend.
 */
#include <utility>

#include "orbslam3_ros2/core/tracking_backend.hpp"

namespace Pipeline = orbslam3_ros2::core::pipeline;

namespace orbslam3_ros2::core
{

TrackingBackendImpl::TrackingBackendImpl(
    Pipeline::TrackingPipelineConfig config)
    : config_(std::move(config)) {}

std::uint8_t TrackingBackendImpl::mode() const noexcept {
    return orbslam3_ros2::msg::Observations::MODE_RGBD;
}

const char* TrackingBackendImpl::mode_name() const noexcept {
    return "rgbd";
}

std::optional<TrackingBackendResult> TrackingBackendImpl::run(
    ORB_SLAM3::System& system,
    const orbslam3_ros2::msg::Observations& observation,
    const rclcpp::Logger& logger, rclcpp::Clock& clock) {
    if (observation.images.size() < 2) {
        RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                             "Observation requires 2 images, got %zu.",
                             observation.images.size());
        return std::nullopt;
    }
    const auto rgb =
        std::make_shared<sensor_msgs::msg::Image>(observation.images[0]);
    const auto depth =
        std::make_shared<sensor_msgs::msg::Image>(observation.images[1]);

    const Pipeline::TrackingPipelineInput input{rgb, depth,
                                                observation.t_track_ns};
    auto result =
        Pipeline::run_tracking_pipeline(system, input, config_, logger, clock);
    if (!result) {
        return std::nullopt;
    }

    return TrackingBackendResult{result->track_time_us, result->tracking_state,
                                 result->tracked_map_points, result->has_pose,
                                 result->twc};
}

}  // namespace orbslam3_ros2::core
