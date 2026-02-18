/**
 * @file core_node_config.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Configure/cleanup implementation for Orbslam3CoreNode runtime.
 *
 * Contract:
 * - Configure declares parameters, resolves resources, and creates interfaces.
 * - Cleanup removes interfaces and releases backend resources.
 * - Runtime counters are reset on each configure/cleanup boundary.
 *
 * Rationale:
 * - Lifecycle-scoped resource ownership makes startup/shutdown behavior
 * deterministic.
 */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "orbslam3_ros2/core/core_config.hpp"
#include "orbslam3_ros2/core/orbslam3_core_node.hpp"
#include "orbslam3_ros2/core/resource_resolver.hpp"
#include "orbslam3_ros2/core/slam_runtime.hpp"
#include "orbslam3_ros2/core/tracking_backend.hpp"

namespace
{
using namespace std::chrono_literals;

std::uint64_t read_process_memory_rss_kb() {
    std::ifstream status("/proc/self/status");
    if (!status.is_open()) {
        return 0;
    }
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream iss(line.substr(6));
            std::uint64_t value_kb = 0;
            iss >> value_kb;
            return value_kb;
        }
    }
    return 0;
}
}  // namespace

namespace Orb = orbslam3_ros2::orbslam3;
namespace Runtime = orbslam3_ros2::core::runtime;

namespace orbslam3_ros2::core
{

bool Orbslam3CoreNode::starts_with_trimmed_(const std::string& line,
                                            const std::string& key) {
    const auto pos = line.find_first_not_of(" \t");
    if (pos == std::string::npos) {
        return false;
    }
    return line.compare(pos, key.size(), key) == 0;
}

bool Orbslam3CoreNode::write_runtime_settings_(
    const std::optional<std::string>& load_stem,
    const std::optional<std::string>& save_stem, std::string* error_message) {
    std::ifstream in(settings_file_);
    if (!in.is_open()) {
        if (error_message) {
            *error_message =
                "Failed to open base settings file: " + settings_file_;
        }
        return false;
    }

    std::ostringstream filtered;
    std::string line;
    while (std::getline(in, line)) {
        if (starts_with_trimmed_(line, "System.LoadAtlasFromFile:") ||
            starts_with_trimmed_(line, "System.SaveAtlasToFile:")) {
            continue;
        }
        filtered << line << '\n';
    }

    if (load_stem.has_value()) {
        filtered << "System.LoadAtlasFromFile: \"" << *load_stem << "\"\n";
    }
    if (save_stem.has_value()) {
        filtered << "System.SaveAtlasToFile: \"" << *save_stem << "\"\n";
    }

    std::ofstream out(runtime_settings_file_);
    if (!out.is_open()) {
        if (error_message) {
            *error_message = "Failed to write runtime settings file: " +
                             runtime_settings_file_;
        }
        return false;
    }
    out << filtered.str();
    return true;
}

void Orbslam3CoreNode::prepare_runtime_paths_() {
    namespace fs = std::filesystem;
    runtime_dir_ = fs::temp_directory_path() / "orbslam3_ros2_runtime";
    fs::create_directories(runtime_dir_);

    runtime_atlas_stem_abs_ = runtime_dir_ / "atlas_runtime";
    runtime_atlas_file_abs_ = runtime_atlas_stem_abs_;
    runtime_atlas_file_abs_ += ".osa";
    runtime_settings_file_ = (runtime_dir_ / "settings_runtime.yaml").string();

    const auto cwd = fs::current_path();
    runtime_atlas_stem_rel_ =
        fs::relative(runtime_atlas_stem_abs_, cwd).generic_string();
}

std::filesystem::path Orbslam3CoreNode::normalize_map_path_(
    const std::string& request_path) const {
    namespace fs = std::filesystem;
    fs::path path(request_path);
    if (path.empty()) {
        return path;
    }
    if (path.extension() != ".osa") {
        path += ".osa";
    }
    if (path.is_relative()) {
        path = fs::current_path() / path;
    }
    return path.lexically_normal();
}

const char* Orbslam3CoreNode::recreate_reason_to_string_(
    RecreateReason reason) {
    switch (reason) {
        case RecreateReason::kConfigure:
            return "configure";
        case RecreateReason::kSaveMap:
            return "save_map";
        case RecreateReason::kLoadMap:
            return "load_map";
    }
    return "unknown";
}

bool Orbslam3CoreNode::recreate_system_(const RecreateReason reason,
                                        const bool load_internal,
                                        std::string* error_message) {
    namespace fs = std::filesystem;
    if (system_ && !system_shutdown_called_.load(std::memory_order_acquire)) {
        Orb::shutdown_system(*system_);
        system_shutdown_called_.store(true, std::memory_order_release);
    }
    system_.reset();

    std::optional<std::string> load_stem;
    if (load_internal) {
        if (!fs::exists(runtime_atlas_file_abs_)) {
            if (error_message) {
                *error_message = "Internal atlas file does not exist: " +
                                 runtime_atlas_file_abs_.string();
            }
            return false;
        }
        load_stem = runtime_atlas_stem_rel_;
    }

    std::string settings_error;
    if (!write_runtime_settings_(load_stem, runtime_atlas_stem_rel_,
                                 &settings_error)) {
        if (error_message) {
            *error_message = settings_error;
        }
        return false;
    }

    try {
        system_ =
            Orb::make_system(voc_file_, runtime_settings_file_, use_viewer_);
        system_shutdown_called_.store(false, std::memory_order_release);
    }
    catch (const std::exception& e) {
        if (error_message) {
            *error_message = std::string("ORB-SLAM3 init failed: ") + e.what();
        }
        {
            std::lock_guard lock(recreate_state_mutex_);
            last_recreate_error_ = error_message ? *error_message : e.what();
        }
        return false;
    }
    catch (...) {
        if (error_message) {
            *error_message = "ORB-SLAM3 init failed: unknown error";
        }
        {
            std::lock_guard lock(recreate_state_mutex_);
            last_recreate_error_ = error_message
                                       ? *error_message
                                       : "ORB-SLAM3 init failed: unknown error";
        }
        return false;
    }
    last_recreate_reason_.store(reason, std::memory_order_relaxed);
    system_recreate_count_.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard lock(recreate_state_mutex_);
        last_recreate_error_.clear();
    }
    return true;
}

Orbslam3CoreNode::CommandResult Orbslam3CoreNode::wait_command_result_(
    SlamCommand command) {
    if (stop_.load(std::memory_order_acquire)) {
        return {false, "Node is stopping."};
    }
    command.request_id = command_completion_.allocate_request_id();
    const auto request_id = command.request_id;
    enqueue_command_(std::move(command));

    CommandResult result;
    const auto wait_status = command_completion_.wait_and_take(
        request_id, std::chrono::milliseconds(30000),
        [this]() { return stop_.load(std::memory_order_acquire); }, &result);
    if (wait_status == CommandCompletion::WaitStatus::kTimeout) {
        return {false, "Timeout while waiting command execution."};
    }
    if (wait_status == CommandCompletion::WaitStatus::kStoppedWithoutResult) {
        return {false, "Node stopped before command was executed."};
    }
    return result;
}

void Orbslam3CoreNode::complete_command_(const SlamCommand& command,
                                         const bool success,
                                         const std::string& message) {
    command_completion_.complete(command.request_id, success, message);
}

std::string Orbslam3CoreNode::get_worker_exception_() const {
    return worker_.last_exception();
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::configure_resources_() {
    if (configured_.load(std::memory_order_acquire)) {
        RCLCPP_WARN(get_logger(),
                    "on_configure called while already configured.");
        return CallbackReturn::SUCCESS;
    }

    const auto params = load_core_params(*this);
    use_viewer_ = params.use_viewer;
    test_mode_skip_orb_init_ = params.test_mode_skip_orb_init;
    test_mode_track_delay_us_ = params.test_mode_track_delay_ms * 1000;
    map_frame_ = params.map_frame;
    camera_frame_param_ = params.camera_frame;
    publish_odom_.store(params.publish_odom, std::memory_order_relaxed);
    publish_tf_.store(params.publish_tf, std::memory_order_relaxed);
    observation_topic_ = params.observation_topic;
    camera_pose_topic_ = params.camera_pose_topic;
    odom_topic_ = params.odom_topic;
    pending_queue_size_ = params.pending_queue_size;
    latest_keep_last_.store(params.latest_keep_last, std::memory_order_relaxed);
    latest_take_.store(params.latest_take, std::memory_order_relaxed);
    worker_time_budget_us_ = params.worker_time_budget_us;
    worker_idle_sleep_us_ = params.worker_idle_sleep_us;
    no_work_wait_us_ = params.no_work_wait_us;
    max_frame_age_ns_.store(params.max_frame_age_ms > 0
                                ? params.max_frame_age_ms * 1'000'000
                                : std::int64_t{0},
                            std::memory_order_relaxed);
    lost_auto_reset_enable_.store(params.lost_auto_reset_enable,
                                  std::memory_order_relaxed);
    lost_auto_reset_frames_.store(params.lost_auto_reset_frames,
                                  std::memory_order_relaxed);
    pointcloud_max_points_ = params.pointcloud_max_points;

    if (!test_mode_skip_orb_init_) {
        const auto voc_resolved =
            resolve_voc_file("orbslam3_ros2", params.voc_file, params.voc_uri);
        const auto settings_resolved =
            resolve_settings_file("orbslam3_ros2", params.settings_file,
                                  params.settings_uri, "config/slam.yaml");
        voc_file_ = voc_resolved.path;
        settings_file_ = settings_resolved.path;

        if (voc_file_.empty() || settings_file_.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to resolve ORB resources.\n"
                         "voc resolved='%s'\n"
                         "settings resolved='%s'\n"
                         "voc searched: %s\n"
                         "settings searched: %s",
                         voc_file_.c_str(), settings_file_.c_str(),
                         format_attempts(voc_resolved.attempts).c_str(),
                         format_attempts(settings_resolved.attempts).c_str());
            return CallbackReturn::ERROR;
        }

        prepare_runtime_paths_();
        std::string runtime_settings_error;
        if (!write_runtime_settings_(std::nullopt, runtime_atlas_stem_rel_,
                                     &runtime_settings_error)) {
            RCLCPP_ERROR(get_logger(), "%s", runtime_settings_error.c_str());
            return CallbackReturn::ERROR;
        }
    }
    else {
        voc_file_.clear();
        settings_file_.clear();
        runtime_settings_file_.clear();
        runtime_dir_.clear();
        runtime_atlas_stem_abs_.clear();
        runtime_atlas_file_abs_.clear();
        runtime_atlas_stem_rel_.clear();
        RCLCPP_WARN(get_logger(),
                    "test_mode_skip_orb_init=true; ORB-SLAM3 system init is "
                    "disabled for interface/stress tests.");
    }

    tracking_backend_ =
        std::make_unique<orbslam3_ros2::core::TrackingBackendImpl>(
            orbslam3_ros2::core::pipeline::TrackingPipelineConfig{false,
                                                                  false});

    {
        std::lock_guard lock(pending_mutex_);
        pending_observations_.clear();
    }
    command_completion_.clear();
    shutdown_requested_.store(false, std::memory_order_release);
    stop_.store(true, std::memory_order_release);
    system_shutdown_called_.store(false, std::memory_order_release);
    tracking_state_.store(ORB_SLAM3::Tracking::SYSTEM_NOT_READY,
                          std::memory_order_relaxed);
    tracked_map_points_.store(0, std::memory_order_relaxed);
    processed_obs_count_.store(0, std::memory_order_relaxed);
    rx_obs_count_.store(0, std::memory_order_relaxed);
    enqueued_obs_count_.store(0, std::memory_order_relaxed);
    ignored_obs_count_.store(0, std::memory_order_relaxed);
    ignored_inactive_count_.store(0, std::memory_order_relaxed);
    ignored_not_configured_count_.store(0, std::memory_order_relaxed);
    ignored_stopping_count_.store(0, std::memory_order_relaxed);
    ignored_no_backend_count_.store(0, std::memory_order_relaxed);
    ignored_mode_mismatch_count_.store(0, std::memory_order_relaxed);
    track_result_count_.store(0, std::memory_order_relaxed);
    no_pose_count_.store(0, std::memory_order_relaxed);
    pose_published_count_.store(0, std::memory_order_relaxed);
    last_observation_stamp_ns_.store(0, std::memory_order_relaxed);
    last_observation_t_track_ns_.store(0, std::memory_order_relaxed);
    last_observation_dt_ns_.store(0, std::memory_order_relaxed);
    memory_rss_kb_.store(read_process_memory_rss_kb(),
                         std::memory_order_relaxed);
    system_recreate_count_.store(0, std::memory_order_relaxed);
    last_recreate_reason_.store(RecreateReason::kConfigure,
                                std::memory_order_relaxed);
    {
        std::lock_guard lock(recreate_state_mutex_);
        last_recreate_error_.clear();
    }
    {
        std::lock_guard lock(tracked_points_cache_mutex_);
        tracked_points_cache_.clear();
    }

    cam_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stats_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Lifecycle timing contract:
    // - Interfaces are created on configure for discoverability.
    // - Observation handling still enforces active/configured/stopping gates.
    setup_observation_io_(params.qos_keep_last);
    setup_control_interfaces_();
    setup_parameter_callback_();

    stats_timer_ = create_wall_timer(
        params.stats_period_ms * 1ms, [this]() { log_sync_stats_(); },
        stats_group_);
    diagnostics_timer_ = create_wall_timer(
        params.diag_period_ms * 1ms,
        [this]() { diagnostics_updater_.force_update(); }, stats_group_);

    if (!test_mode_skip_orb_init_) {
        std::string restart_error;
        if (!recreate_system_(RecreateReason::kConfigure, false,
                              &restart_error)) {
            RCLCPP_ERROR(get_logger(), "%s", restart_error.c_str());
            cleanup_resources_();
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 System initialized.");
        RCLCPP_INFO(get_logger(),
                    "Runtime atlas managed at '%s' (runtime settings '%s').",
                    runtime_atlas_file_abs_.string().c_str(),
                    runtime_settings_file_.c_str());
    }
    else {
        tracking_state_.store(ORB_SLAM3::Tracking::NOT_INITIALIZED,
                              std::memory_order_relaxed);
    }
    RCLCPP_INFO(
        get_logger(),
        "Core configured. observation_topic='%s', camera_pose_topic='%s', "
        "odom_topic='%s', qos_keep_last=%" PRId64 ", pending_queue_size=%zu",
        observation_topic_.c_str(), camera_pose_topic_.c_str(),
        odom_topic_.c_str(), params.qos_keep_last, pending_queue_size_);

    configured_.store(true, std::memory_order_release);
    return CallbackReturn::SUCCESS;
}

void Orbslam3CoreNode::cleanup_resources_() {
    stop_worker_();
    join_export_threads_();
    on_set_parameters_callback_handle_.reset();
    stats_timer_.reset();
    diagnostics_timer_.reset();
    obs_sub_.reset();
    reset_srv_.reset();
    shutdown_srv_.reset();
    get_status_srv_.reset();
    get_tracked_points_srv_.reset();
    save_trajectory_srv_.reset();
    save_map_srv_.reset();
    load_map_srv_.reset();
    set_localization_mode_srv_.reset();
    export_pointcloud_action_server_.reset();
    pose_pub_.reset();
    odom_pub_.reset();
    snapshot_pointcloud_pub_.reset();
    tf_broadcaster_.reset();
    cam_group_.reset();
    stats_group_.reset();
    cmd_group_.reset();

    if (system_ && !system_shutdown_called_.load(std::memory_order_acquire)) {
        Orb::shutdown_system(*system_);
        system_shutdown_called_.store(true, std::memory_order_release);
    }
    system_.reset();
    {
        std::lock_guard lock(pending_mutex_);
        pending_observations_.clear();
    }
    command_completion_.clear();
    tracking_backend_.reset();
    {
        std::lock_guard lock(tracked_points_cache_mutex_);
        tracked_points_cache_.clear();
    }
    processed_obs_count_.store(0, std::memory_order_relaxed);
    rx_obs_count_.store(0, std::memory_order_relaxed);
    enqueued_obs_count_.store(0, std::memory_order_relaxed);
    ignored_obs_count_.store(0, std::memory_order_relaxed);
    ignored_inactive_count_.store(0, std::memory_order_relaxed);
    ignored_not_configured_count_.store(0, std::memory_order_relaxed);
    ignored_stopping_count_.store(0, std::memory_order_relaxed);
    ignored_no_backend_count_.store(0, std::memory_order_relaxed);
    ignored_mode_mismatch_count_.store(0, std::memory_order_relaxed);
    track_result_count_.store(0, std::memory_order_relaxed);
    no_pose_count_.store(0, std::memory_order_relaxed);
    pose_published_count_.store(0, std::memory_order_relaxed);
    last_observation_stamp_ns_.store(0, std::memory_order_relaxed);
    last_observation_t_track_ns_.store(0, std::memory_order_relaxed);
    last_observation_dt_ns_.store(0, std::memory_order_relaxed);
    configured_.store(false, std::memory_order_release);
}

void Orbslam3CoreNode::log_sync_stats_() {
    std::size_t pending_size = 0;
    const std::uint64_t pending_drop_count = diagnostics_.pending_drop_count();
    const std::uint64_t expired_drop_count = diagnostics_.expired_drop_count();
    memory_rss_kb_.store(read_process_memory_rss_kb(),
                         std::memory_order_relaxed);
    {
        std::lock_guard lock(pending_mutex_);
        pending_size = pending_observations_.size();
    }

    RCLCPP_INFO(
        get_logger(),
        "core: pending=%zu pending_drop=%" PRIu64 " expired_drop=%" PRIu64
        " rx_obs=%" PRIu64 " enqueued_obs=%" PRIu64 " ignored_obs=%" PRIu64
        " ignored_inactive=%" PRIu64 " ignored_not_configured=%" PRIu64
        " ignored_stopping=%" PRIu64 " ignored_no_backend=%" PRIu64
        " ignored_mode_mismatch=%" PRIu64 " track_result=%" PRIu64
        " no_pose=%" PRIu64 " pose_published=%" PRIu64 " processed_obs=%" PRIu64
        " rss_kb=%" PRIu64 " tracking_state=%s tracked_map_points=%" PRIu64,
        pending_size, pending_drop_count, expired_drop_count,
        rx_obs_count_.load(std::memory_order_relaxed),
        enqueued_obs_count_.load(std::memory_order_relaxed),
        ignored_obs_count_.load(std::memory_order_relaxed),
        ignored_inactive_count_.load(std::memory_order_relaxed),
        ignored_not_configured_count_.load(std::memory_order_relaxed),
        ignored_stopping_count_.load(std::memory_order_relaxed),
        ignored_no_backend_count_.load(std::memory_order_relaxed),
        ignored_mode_mismatch_count_.load(std::memory_order_relaxed),
        track_result_count_.load(std::memory_order_relaxed),
        no_pose_count_.load(std::memory_order_relaxed),
        pose_published_count_.load(std::memory_order_relaxed),
        processed_obs_count_.load(std::memory_order_relaxed),
        memory_rss_kb_.load(std::memory_order_relaxed),
        Runtime::tracking_state_to_string(
            tracking_state_.load(std::memory_order_relaxed)),
        tracked_map_points_.load(std::memory_order_relaxed));
}

void Orbslam3CoreNode::produce_diagnostics_(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto pending_size = [&]() {
        std::lock_guard lock(pending_mutex_);
        return pending_observations_.size();
    }();
    diagnostics_.produce(stat, pending_size, pending_queue_size_);

    const auto worker_exception = get_worker_exception_();
    const bool worker_alive = worker_.running();
    stat.add("worker_alive", worker_alive ? "true" : "false");
    stat.add("worker_exception", worker_exception);
    stat.add("rx_obs",
             static_cast<int>(rx_obs_count_.load(std::memory_order_relaxed)));
    stat.add(
        "enqueued_obs",
        static_cast<int>(enqueued_obs_count_.load(std::memory_order_relaxed)));
    stat.add(
        "ignored_obs",
        static_cast<int>(ignored_obs_count_.load(std::memory_order_relaxed)));
    stat.add("ignored_inactive", static_cast<int>(ignored_inactive_count_.load(
                                     std::memory_order_relaxed)));
    stat.add("ignored_not_configured",
             static_cast<int>(ignored_not_configured_count_.load(
                 std::memory_order_relaxed)));
    stat.add("ignored_stopping", static_cast<int>(ignored_stopping_count_.load(
                                     std::memory_order_relaxed)));
    stat.add("ignored_no_backend",
             static_cast<int>(
                 ignored_no_backend_count_.load(std::memory_order_relaxed)));
    stat.add("ignored_mode_mismatch",
             static_cast<int>(
                 ignored_mode_mismatch_count_.load(std::memory_order_relaxed)));
    stat.add(
        "track_result",
        static_cast<int>(track_result_count_.load(std::memory_order_relaxed)));
    stat.add("no_pose",
             static_cast<int>(no_pose_count_.load(std::memory_order_relaxed)));
    stat.add("pose_published", static_cast<int>(pose_published_count_.load(
                                   std::memory_order_relaxed)));
    stat.add("system_recreate_count",
             static_cast<int>(
                 system_recreate_count_.load(std::memory_order_relaxed)));
    stat.add("last_recreate_reason",
             recreate_reason_to_string_(
                 last_recreate_reason_.load(std::memory_order_relaxed)));
    {
        std::lock_guard lock(recreate_state_mutex_);
        stat.add("last_recreate_error", last_recreate_error_);
    }
    if (!worker_exception.empty()) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                     "worker exception");
    }
}

}  // namespace orbslam3_ros2::core
