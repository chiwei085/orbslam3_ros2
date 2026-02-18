/**
 * @file tracking_diagnostics.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for tracking diagnostics.
 */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <chrono>
#include <vector>

#include "orbslam3_ros2/core/tracking_diagnostics.hpp"

namespace orbslam3_ros2::core
{

void TrackingDiagnostics::on_observation_input() noexcept {
    input_observation_count_.fetch_add(1, std::memory_order_relaxed);
}

void TrackingDiagnostics::on_pending_drop(const std::uint64_t count) noexcept {
    pending_drop_count_.fetch_add(count, std::memory_order_relaxed);
}

void TrackingDiagnostics::on_expired_drop(const std::uint64_t count) noexcept {
    expired_drop_count_.fetch_add(count, std::memory_order_relaxed);
}

void TrackingDiagnostics::on_track(const std::uint64_t frame_stamp_ns,
                                   const std::uint64_t track_time_us) {
    last_frame_stamp_ns_.store(static_cast<std::int64_t>(frame_stamp_ns),
                               std::memory_order_relaxed);
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    last_track_wall_ns_.store(static_cast<std::int64_t>(now_ns),
                              std::memory_order_relaxed);
    track_time_us_last_.store(track_time_us, std::memory_order_relaxed);
    track_time_us_sum_.fetch_add(track_time_us, std::memory_order_relaxed);
    track_sample_count_.fetch_add(1, std::memory_order_relaxed);

    std::lock_guard lock(track_stats_mutex_);
    track_time_window_us_.push_back(track_time_us);
    if (track_time_window_us_.size() > kTrackWindowSize) {
        track_time_window_us_.pop_front();
    }
}

void TrackingDiagnostics::on_tracking_state(
    const int tracking_state, const std::size_t tracked_map_points) noexcept {
    tracking_state_.store(tracking_state, std::memory_order_relaxed);
    tracked_map_points_.store(static_cast<std::uint64_t>(tracked_map_points),
                              std::memory_order_relaxed);
}

std::uint64_t TrackingDiagnostics::pending_drop_count() const noexcept {
    return pending_drop_count_.load(std::memory_order_relaxed);
}

std::uint64_t TrackingDiagnostics::expired_drop_count() const noexcept {
    return expired_drop_count_.load(std::memory_order_relaxed);
}

std::uint64_t TrackingDiagnostics::track_time_us_last() const noexcept {
    return track_time_us_last_.load(std::memory_order_relaxed);
}

void TrackingDiagnostics::produce(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    const std::size_t pending_size,
    const std::size_t pending_queue_size) const {
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    const auto last_track_ns =
        last_track_wall_ns_.load(std::memory_order_relaxed);

    const auto obs_in =
        input_observation_count_.load(std::memory_order_relaxed);
    const auto pending_drop =
        pending_drop_count_.load(std::memory_order_relaxed);
    const auto expired_drop =
        expired_drop_count_.load(std::memory_order_relaxed);
    const auto last_track_us =
        track_time_us_last_.load(std::memory_order_relaxed);
    const auto track_sum = track_time_us_sum_.load(std::memory_order_relaxed);
    const auto track_samples =
        track_sample_count_.load(std::memory_order_relaxed);
    const auto tracking_state = tracking_state_.load(std::memory_order_relaxed);
    const auto tracked_map_points =
        tracked_map_points_.load(std::memory_order_relaxed);

    std::uint64_t track_p95_us = 0;
    {
        std::lock_guard lock(track_stats_mutex_);
        if (!track_time_window_us_.empty()) {
            std::vector<std::uint64_t> sorted(track_time_window_us_.begin(),
                                              track_time_window_us_.end());
            const std::size_t p95_index = (sorted.size() * 95) / 100 > 0
                                              ? (sorted.size() * 95) / 100 - 1
                                              : 0;
            std::nth_element(sorted.begin(), sorted.begin() + p95_index,
                             sorted.end());
            track_p95_us = sorted[p95_index];
        }
    }

    const double track_avg_us = track_samples > 0
                                    ? static_cast<double>(track_sum) /
                                          static_cast<double>(track_samples)
                                    : 0.0;
    const bool no_track_yet = last_track_ns <= 0;
    const bool stale_track =
        !no_track_yet &&
        (now_ns - last_track_ns) > (5LL * 1000LL * 1000LL * 1000LL);
    const bool backlog_warn = pending_size >= pending_queue_size;

    if (stale_track) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                     "tracking stale");
    }
    else if (no_track_yet || backlog_warn) {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            no_track_yet ? "no successful track yet" : "pending backlog high");
    }
    else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     "tracking healthy");
    }

    stat.add("input_observation", obs_in);
    stat.add("pending_size", pending_size);
    stat.add("pending_drop", pending_drop);
    stat.add("expired_drop", expired_drop);
    stat.add("last_track_us", last_track_us);
    stat.add("track_avg_us", track_avg_us);
    stat.add("track_p95_us", track_p95_us);
    stat.add("last_frame_stamp_ns",
             last_frame_stamp_ns_.load(std::memory_order_relaxed));
    stat.add("tracking_time_source", "msg.header.stamp");
    stat.add("tracking_time_unit", "seconds(double)=stamp_ns*1e-9");
    stat.add("pose_convention", "T_wc (map->camera)");
    stat.add("tracking_state", tracking_state);
    stat.add("tracked_map_points", tracked_map_points);
}

}  // namespace orbslam3_ros2::core
