/**
 * @file tracking_diagnostics.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for tracking diagnostics.
 */
#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

namespace orbslam3_ros2::core
{

class TrackingDiagnostics
{
public:
    void on_observation_input() noexcept;
    void on_pending_drop(std::uint64_t count) noexcept;
    void on_expired_drop(std::uint64_t count) noexcept;
    void on_track(std::uint64_t frame_stamp_ns, std::uint64_t track_time_us);
    void on_tracking_state(int tracking_state,
                           std::size_t tracked_map_points) noexcept;
    std::uint64_t pending_drop_count() const noexcept;
    std::uint64_t expired_drop_count() const noexcept;
    std::uint64_t track_time_us_last() const noexcept;

    void produce(diagnostic_updater::DiagnosticStatusWrapper& stat,
                 std::size_t pending_size,
                 std::size_t pending_queue_size) const;

private:
    static constexpr std::size_t kTrackWindowSize = 200;

    std::atomic<std::uint64_t> pending_drop_count_{0};
    std::atomic<std::uint64_t> expired_drop_count_{0};
    std::atomic<std::uint64_t> input_observation_count_{0};
    std::atomic<std::int64_t> last_frame_stamp_ns_{0};
    std::atomic<std::int64_t> last_track_wall_ns_{0};
    std::atomic<std::uint64_t> track_time_us_last_{0};
    std::atomic<std::uint64_t> track_time_us_sum_{0};
    std::atomic<std::uint64_t> track_sample_count_{0};
    std::atomic<int> tracking_state_{-1};
    std::atomic<std::uint64_t> tracked_map_points_{0};

    mutable std::mutex track_stats_mutex_;
    std::deque<std::uint64_t> track_time_window_us_;
};

}  // namespace orbslam3_ros2::core
