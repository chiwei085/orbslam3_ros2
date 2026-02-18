/**
 * @file tracking_backend.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for tracking backend.
 */
#pragma once

#include <optional>

#include "orbslam3_ros2/core/tracking_backend_interface.hpp"
#include "orbslam3_ros2/core/tracking_pipeline.hpp"

namespace orbslam3_ros2::core
{

class TrackingBackendImpl final : public TrackingBackend
{
public:
    explicit TrackingBackendImpl(
        orbslam3_ros2::core::pipeline::TrackingPipelineConfig config);

    std::uint8_t mode() const noexcept override;
    const char* mode_name() const noexcept override;

    std::optional<TrackingBackendResult> run(
        ORB_SLAM3::System& system,
        const orbslam3_ros2::msg::Observations& observation,
        const rclcpp::Logger& logger, rclcpp::Clock& clock) override;

private:
    orbslam3_ros2::core::pipeline::TrackingPipelineConfig config_{};
};

}  // namespace orbslam3_ros2::core
