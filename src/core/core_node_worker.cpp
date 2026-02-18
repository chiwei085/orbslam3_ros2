/**
 * @file core_node_worker.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for core node worker.
 */
#include <chrono>
#include <cctype>
#include <cinttypes>
#include <filesystem>

#include "orbslam3_ros2/core/orbslam3_core_node.hpp"
#include "orbslam3_ros2/core/slam_runtime.hpp"

namespace
{
using namespace std::chrono_literals;

std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}
}  // namespace

namespace Orb = orbslam3_ros2::orbslam3;
namespace Runtime = orbslam3_ros2::core::runtime;

namespace orbslam3_ros2::core
{

bool Orbslam3CoreNode::start_worker_() {
    if (worker_.joinable() || worker_.running()) {
        return true;
    }
    stop_.store(false, std::memory_order_release);
    const auto started = worker_.start(
        [this]() { worker_loop_(); },
        [this](const std::string& message) {
            RCLCPP_ERROR(get_logger(), "Worker thread exception: %s",
                         message.c_str());
            stop_.store(true, std::memory_order_release);
            cmd_queue_.notify_all();
            command_completion_.notify_all();
            pending_cv_.notify_all();
        });
    return started;
}

void Orbslam3CoreNode::stop_worker_() {
    stop_.store(true, std::memory_order_release);
    cmd_queue_.notify_all();
    command_completion_.notify_all();
    pending_cv_.notify_all();
    worker_.join();
}

std::string Orbslam3CoreNode::to_lower_ascii_(std::string value) const {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

bool Orbslam3CoreNode::is_expired_(const std::int64_t now_ns,
                                   const ObservationPtr& obs) const noexcept {
    const auto max_frame_age_ns =
        max_frame_age_ns_.load(std::memory_order_relaxed);
    if (max_frame_age_ns <= 0) {
        return false;
    }
    const auto t_track_ns = static_cast<std::int64_t>(obs->t_track_ns);
    auto age_ns = now_ns - t_track_ns;
    if (age_ns < 0) {
        age_ns = 0;
    }
    return age_ns > max_frame_age_ns;
}

void Orbslam3CoreNode::on_observation_(
    const orbslam3_ros2::msg::Observations::SharedPtr& msg) {
    rx_obs_count_.fetch_add(1, std::memory_order_relaxed);
    if (!configured_.load(std::memory_order_acquire)) {
        ignored_obs_count_.fetch_add(1, std::memory_order_relaxed);
        ignored_not_configured_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Ignoring observation: core not configured yet.");
        return;
    }
    if (stop_.load(std::memory_order_acquire) ||
        shutdown_requested_.load(std::memory_order_acquire)) {
        ignored_obs_count_.fetch_add(1, std::memory_order_relaxed);
        ignored_stopping_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Ignoring observation: core is stopping or shutdown requested.");
        return;
    }
    if (!active_.load(std::memory_order_acquire)) {
        ignored_obs_count_.fetch_add(1, std::memory_order_relaxed);
        ignored_inactive_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Ignoring observation: lifecycle state inactive.");
        return;
    }
    diagnostics_.on_observation_input();

    if (!tracking_backend_) {
        ignored_obs_count_.fetch_add(1, std::memory_order_relaxed);
        ignored_no_backend_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Tracking backend is not initialized.");
        return;
    }
    if (msg->mode != tracking_backend_->mode()) {
        ignored_obs_count_.fetch_add(1, std::memory_order_relaxed);
        ignored_mode_mismatch_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Unsupported observation mode=%u in core. Current "
                             "backend handles mode=%u (%s).",
                             static_cast<unsigned>(msg->mode),
                             static_cast<unsigned>(tracking_backend_->mode()),
                             tracking_backend_->mode_name());
        return;
    }
    enqueue_observation_(msg);
    enqueued_obs_count_.fetch_add(1, std::memory_order_relaxed);
}

void Orbslam3CoreNode::enqueue_observation_(ObservationPtr observation) {
    {
        std::lock_guard lock(pending_mutex_);
        pending_observations_.push_back(std::move(observation));
        if (pending_observations_.size() > pending_queue_size_) {
            const std::size_t drop_count =
                pending_observations_.size() - pending_queue_size_;
            for (std::size_t i = 0; i < drop_count; ++i) {
                pending_observations_.pop_front();
            }
            diagnostics_.on_pending_drop(
                static_cast<std::uint64_t>(drop_count));
        }
    }
    pending_cv_.notify_one();
}

void Orbslam3CoreNode::enqueue_command_(SlamCommand command) {
    cmd_queue_.push(std::move(command));
    pending_cv_.notify_one();
}

bool Orbslam3CoreNode::has_pending_commands_() const {
    return !cmd_queue_.empty();
}

bool Orbslam3CoreNode::drain_commands_() {
    const auto local_commands = cmd_queue_.drain();
    if (local_commands.empty()) {
        return true;
    }

    for (const auto& cmd : local_commands) {
        switch (cmd.type) {
            case SlamCommandType::kReset:
                if (system_) {
                    Orb::reset_active_map(*system_);
                    RCLCPP_INFO(get_logger(),
                                "Accepted command: reset_active_map");
                }
                complete_command_(cmd, true, "reset_active_map accepted");
                break;
            case SlamCommandType::kShutdown:
                if (system_ &&
                    !system_shutdown_called_.load(std::memory_order_acquire)) {
                    Orb::shutdown_system(*system_);
                    system_shutdown_called_.store(true,
                                                  std::memory_order_release);
                }
                stop_.store(true, std::memory_order_release);
                cmd_queue_.notify_all();
                pending_cv_.notify_all();
                RCLCPP_INFO(get_logger(), "Accepted command: shutdown");
                complete_command_(cmd, true, "shutdown accepted");
                rclcpp::shutdown();
                return false;
            case SlamCommandType::kSaveTrajectory:
                if (!system_) {
                    complete_command_(cmd, false, "ORB-SLAM3 system not ready");
                    break;
                }
                if (cmd.path.empty()) {
                    complete_command_(cmd, false, "path must not be empty");
                    break;
                }
                try {
                    const auto format = to_lower_ascii_(cmd.format);
                    if (format == "tum") {
                        Orb::save_trajectory_tum(*system_, cmd.path);
                    }
                    else if (format == "kitti") {
                        Orb::save_trajectory_kitti(*system_, cmd.path);
                    }
                    else if (format == "euroc") {
                        Orb::save_trajectory_euroc(*system_, cmd.path);
                    }
                    else {
                        complete_command_(cmd, false,
                                          "unsupported format, use one of: "
                                          "tum, kitti, euroc");
                        break;
                    }
                    complete_command_(cmd, true,
                                      "trajectory saved to " + cmd.path);
                    RCLCPP_INFO(get_logger(),
                                "Accepted command: save_trajectory format=%s "
                                "path='%s'",
                                format.c_str(), cmd.path.c_str());
                }
                catch (const std::exception& e) {
                    complete_command_(
                        cmd, false,
                        std::string("save_trajectory failed: ") + e.what());
                }
                break;
            case SlamCommandType::kSaveMap:
                if (cmd.path.empty()) {
                    complete_command_(cmd, false, "path must not be empty");
                    break;
                }
                try {
                    namespace fs = std::filesystem;
                    const auto target = normalize_map_path_(cmd.path);
                    if (target.empty()) {
                        complete_command_(cmd, false, "invalid target path");
                        break;
                    }
                    if (target.has_parent_path()) {
                        fs::create_directories(target.parent_path());
                    }

                    std::string restart_error;
                    if (!recreate_system_(RecreateReason::kSaveMap, true,
                                          &restart_error)) {
                        complete_command_(
                            cmd, false,
                            "save_map failed while restarting system: " +
                                restart_error);
                        break;
                    }

                    fs::copy_file(runtime_atlas_file_abs_, target,
                                  fs::copy_options::overwrite_existing);
                    complete_command_(cmd, true,
                                      "map saved to " + target.string());
                    RCLCPP_INFO(get_logger(),
                                "Accepted command: save_map path='%s'",
                                target.string().c_str());
                }
                catch (const std::exception& e) {
                    complete_command_(
                        cmd, false,
                        std::string("save_map failed: ") + e.what());
                }
                break;
            case SlamCommandType::kLoadMap:
                if (cmd.path.empty()) {
                    complete_command_(cmd, false, "path must not be empty");
                    break;
                }
                try {
                    namespace fs = std::filesystem;
                    const auto source = normalize_map_path_(cmd.path);
                    if (source.empty() || !fs::exists(source)) {
                        complete_command_(
                            cmd, false,
                            "map file does not exist: " + source.string());
                        break;
                    }
                    fs::create_directories(
                        runtime_atlas_file_abs_.parent_path());
                    fs::copy_file(source, runtime_atlas_file_abs_,
                                  fs::copy_options::overwrite_existing);

                    std::string restart_error;
                    if (!recreate_system_(RecreateReason::kLoadMap, true,
                                          &restart_error)) {
                        complete_command_(
                            cmd, false,
                            "load_map failed while restarting system: " +
                                restart_error);
                        break;
                    }
                    complete_command_(cmd, true,
                                      "map loaded from " + source.string());
                    RCLCPP_INFO(get_logger(),
                                "Accepted command: load_map path='%s'",
                                source.string().c_str());
                }
                catch (const std::exception& e) {
                    complete_command_(
                        cmd, false,
                        std::string("load_map failed: ") + e.what());
                }
                break;
            case SlamCommandType::kToggleLocalizationMode:
                if (!system_) {
                    complete_command_(cmd, false, "ORB-SLAM3 system not ready");
                    break;
                }
                Orb::set_localization_mode(*system_, cmd.localization_mode);
                localization_mode_enabled_.store(cmd.localization_mode,
                                                 std::memory_order_relaxed);
                complete_command_(cmd, true,
                                  cmd.localization_mode
                                      ? "localization mode enabled"
                                      : "localization mode disabled");
                break;
        }
    }
    return true;
}

void Orbslam3CoreNode::worker_loop_() {
    while (!stop_.load(std::memory_order_acquire) && rclcpp::ok()) {
        if (!drain_commands_()) {
            break;
        }
        {
            std::unique_lock lock(pending_mutex_);
            pending_cv_.wait(lock, [this]() {
                return stop_.load(std::memory_order_acquire) ||
                       !pending_observations_.empty() ||
                       has_pending_commands_() || !rclcpp::ok();
            });
            if (stop_.load(std::memory_order_acquire) || !rclcpp::ok()) {
                break;
            }
        }
        if (!drain_commands_()) {
            break;
        }

        const auto round_start = std::chrono::steady_clock::now();
        const auto max_take = std::max<std::size_t>(
            std::size_t{1}, latest_take_.load(std::memory_order_relaxed));
        const auto keep = std::max<std::size_t>(
            std::size_t{1}, latest_keep_last_.load(std::memory_order_relaxed));
        const auto time_budget = worker_time_budget_us_ * 1us;
        const bool budget_enabled = worker_time_budget_us_ > 0;
        std::size_t processed = 0;
        bool hit_budget = false;
        std::size_t dropped_old = 0;
        std::uint64_t expired_dropped_this_round = 0;
        const auto now_ns = get_clock()->now().nanoseconds();

        local_observation_batch_.clear();
        local_observation_batch_.reserve(max_take);
        {
            std::lock_guard lock(pending_mutex_);
            while (pending_observations_.size() > keep) {
                pending_observations_.pop_front();
                ++dropped_old;
            }
            diagnostics_.on_pending_drop(
                static_cast<std::uint64_t>(dropped_old));

            while (!pending_observations_.empty() &&
                   local_observation_batch_.size() < max_take) {
                auto observation = std::move(pending_observations_.back());
                pending_observations_.pop_back();
                if (is_expired_(now_ns, observation)) {
                    ++expired_dropped_this_round;
                    diagnostics_.on_expired_drop(1);
                    continue;
                }
                local_observation_batch_.push_back(std::move(observation));
            }
        }

        const std::size_t taken = local_observation_batch_.size();
        if (taken == 0) {
            if (no_work_wait_us_ > 0) {
                std::unique_lock lock(pending_mutex_);
                pending_cv_.wait_for(lock, no_work_wait_us_ * 1us, [this]() {
                    return stop_.load(std::memory_order_acquire) ||
                           !rclcpp::ok() || !pending_observations_.empty() ||
                           has_pending_commands_();
                });
            }
            continue;
        }

        while (processed < local_observation_batch_.size()) {
            if (budget_enabled &&
                (std::chrono::steady_clock::now() - round_start) >=
                    time_budget &&
                processed > 0) {
                hit_budget = true;
                break;
            }
            if (!drain_commands_()) {
                break;
            }
            const auto& observation = local_observation_batch_[processed];
            handle_observation_(observation);
            last_observation_stamp_ns_.store(
                stamp_to_ns(observation->header.stamp),
                std::memory_order_relaxed);
            last_observation_t_track_ns_.store(observation->t_track_ns,
                                               std::memory_order_relaxed);
            last_observation_dt_ns_.store(observation->dt_ns,
                                          std::memory_order_relaxed);
            processed_obs_count_.fetch_add(1, std::memory_order_relaxed);
            ++processed;
        }
        if (stop_.load(std::memory_order_acquire)) {
            break;
        }

        if (processed < local_observation_batch_.size()) {
            std::lock_guard lock(pending_mutex_);
            for (std::size_t i = local_observation_batch_.size(); i > processed;
                 --i) {
                pending_observations_.push_front(
                    std::move(local_observation_batch_[i - 1]));
            }
        }

        std::size_t pending_size_after = 0;
        {
            std::lock_guard lock(pending_mutex_);
            pending_size_after = pending_observations_.size();
        }
        const bool hit_take = (taken == max_take) && (processed == max_take);

        if (pending_size_after >= pending_queue_size_ ||
            (pending_size_after > 0 && hit_take) ||
            (pending_size_after > 0 && hit_budget)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Pending backlog remains after worker round: pending=%zu "
                "(queue_size=%zu, taken=%zu, processed=%zu/%zu, "
                "dropped_old=%zu, "
                "expired_dropped=%" PRIu64
                ", hit_take=%s, hit_budget=%s). "
                "Consider tuning latest_keep_last/latest_take/time_budget "
                "params.",
                pending_size_after, pending_queue_size_, taken, processed,
                max_take, dropped_old, expired_dropped_this_round,
                hit_take ? "true" : "false", hit_budget ? "true" : "false");
        }

        idle_or_yield_(pending_size_after);
    }
}

void Orbslam3CoreNode::handle_observation_(const ObservationPtr& observation) {
    if (!system_) {
        if (test_mode_skip_orb_init_ && test_mode_track_delay_us_ > 0) {
            std::this_thread::sleep_for(test_mode_track_delay_us_ * 1us);
        }
        return;
    }
    if (!tracking_backend_) {
        return;
    }

    auto result = tracking_backend_->run(*system_, *observation, get_logger(),
                                         *get_clock());
    if (!result) {
        return;
    }
    track_result_count_.fetch_add(1, std::memory_order_relaxed);

    diagnostics_.on_track(observation->t_track_ns, result->track_time_us);
    tracking_state_.store(result->tracking_state, std::memory_order_relaxed);
    tracked_map_points_.store(
        static_cast<std::uint64_t>(result->tracked_map_points),
        std::memory_order_relaxed);
    update_tracked_points_cache_(pointcloud_max_points_);
    diagnostics_.on_tracking_state(result->tracking_state,
                                   result->tracked_map_points);

    if (result->tracking_state == ORB_SLAM3::Tracking::LOST &&
        lost_auto_reset_enable_.load(std::memory_order_relaxed)) {
        const auto lost_count =
            consecutive_lost_frames_.fetch_add(1, std::memory_order_relaxed) +
            1;
        const auto lost_threshold =
            lost_auto_reset_frames_.load(std::memory_order_relaxed);
        if (lost_count >= lost_threshold) {
            consecutive_lost_frames_.store(0, std::memory_order_relaxed);
            enqueue_command_(SlamCommand{SlamCommandType::kReset});
            RCLCPP_WARN(get_logger(),
                        "Tracking LOST for %ld frames, auto enqueue reset.",
                        static_cast<long>(lost_threshold));
        }
    }
    else {
        consecutive_lost_frames_.store(0, std::memory_order_relaxed);
    }

    if (!result->has_pose) {
        no_pose_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skip pose publish, tracking_state=%s (%d)",
            Runtime::tracking_state_to_string(result->tracking_state),
            result->tracking_state);
        return;
    }

    const auto observation_camera_frame = observation->camera_frame.empty()
                                              ? observation->header.frame_id
                                              : observation->camera_frame;
    const auto camera_frame = Runtime::resolve_camera_frame(
        camera_frame_param_, observation_camera_frame);
    if (!camera_frame_param_.empty() &&
        observation_camera_frame != camera_frame_param_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "camera_frame='%s' overrides image frame_id='%s'. "
                             "No extrinsic/axis "
                             "conversion is applied in this node. Provide "
                             "camera/base extrinsics "
                             "via external TF.",
                             camera_frame_param_.c_str(),
                             observation_camera_frame.c_str());
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_local;
    tf2_ros::TransformBroadcaster* tf_broadcaster_local = nullptr;
    {
        std::lock_guard lock(publisher_mutex_);
        if (publish_odom_.load(std::memory_order_relaxed)) {
            odom_pub_local = odom_pub_;
        }
        if (publish_tf_.load(std::memory_order_relaxed)) {
            if (!tf_broadcaster_) {
                tf_broadcaster_ =
                    std::make_unique<tf2_ros::TransformBroadcaster>(this);
            }
            tf_broadcaster_local = tf_broadcaster_.get();
        }
    }

    Runtime::publish_pose_outputs(observation->header.stamp, map_frame_,
                                  camera_frame, result->twc.translation(),
                                  result->twc.unit_quaternion(), pose_pub_,
                                  odom_pub_local, tf_broadcaster_local);
    pose_published_count_.fetch_add(1, std::memory_order_relaxed);
}

void Orbslam3CoreNode::idle_or_yield_(
    const std::size_t pending_size_after) const {
    if (worker_idle_sleep_us_ > 0 && pending_size_after == 0) {
        std::this_thread::sleep_for(worker_idle_sleep_us_ * 1us);
        return;
    }
    std::this_thread::yield();
}

}  // namespace orbslam3_ros2::core
