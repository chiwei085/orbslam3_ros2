/**
 * @file tracking_backend_interface.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for tracking backend interface.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

#include "orbslam3_ros2/msg/observations.hpp"
#include "orbslam3_ros2/orbslam3/system.hpp"

namespace orbslam3_ros2::core
{

struct TrackingBackendResult
{
    std::uint64_t track_time_us{0};
    int tracking_state{ORB_SLAM3::Tracking::SYSTEM_NOT_READY};
    std::size_t tracked_map_points{0};
    bool has_pose{false};
    Sophus::SE3f twc{};
};

class TrackingBackend
{
public:
    virtual ~TrackingBackend() = default;

    virtual std::uint8_t mode() const noexcept = 0;
    virtual const char* mode_name() const noexcept = 0;

    virtual std::optional<TrackingBackendResult> run(
        ORB_SLAM3::System& system,
        const orbslam3_ros2::msg::Observations& observation,
        const rclcpp::Logger& logger, rclcpp::Clock& clock) = 0;
};

}  // namespace orbslam3_ros2::core
