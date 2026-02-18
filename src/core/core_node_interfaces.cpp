/**
 * @file core_node_interfaces.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief ROS 2 topic/service/action wiring for Orbslam3CoreNode.
 *
 * Contract:
 * - Observation input uses SensorDataQoS (best-effort, volatile).
 * - Pose/Odom outputs use explicit reliable+volatile QoS.
 * - Snapshot point cloud uses transient-local QoS for late joiners.
 *
 * Rationale:
 * - Deterministic local gates require stable QoS and predictable interface
 * timing.
 */
#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <cstdint>
#include <utility>

#include "orbslam3_ros2/core/orbslam3_core_node.hpp"
#include "orbslam3_ros2/core/slam_runtime.hpp"

namespace Runtime = orbslam3_ros2::core::runtime;
namespace Srv = orbslam3_ros2::srv;

namespace orbslam3_ros2::core
{
namespace
{
builtin_interfaces::msg::Time ns_to_stamp(std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    if (ns < 0) {
        ns = 0;
    }
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
}
}  // namespace

void Orbslam3CoreNode::setup_observation_io_(const std::int64_t qos_keep_last) {
    // Observation stream is sensor-paced and may drop under load by design.
    const auto qos = rclcpp::SensorDataQoS().keep_last(
        static_cast<std::size_t>(qos_keep_last));

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = cam_group_;
    obs_sub_ = create_subscription<orbslam3_ros2::msg::Observations>(
        observation_topic_, qos,
        [this](const orbslam3_ros2::msg::Observations::SharedPtr msg) {
            on_observation_(msg);
        },
        sub_opt);

    // Explicit QoS: SystemDefaultsQoS differs across RMW and breaks gate
    // determinism.
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        camera_pose_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    if (publish_odom_.load(std::memory_order_relaxed)) {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    }
    snapshot_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orbslam3/map/points_snapshot",
        rclcpp::QoS(1).transient_local().reliable());
    if (publish_tf_.load(std::memory_order_relaxed)) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
}

void Orbslam3CoreNode::setup_control_interfaces_() {
    // Lifecycle contract:
    // - Interfaces are created during configure.
    // - Processing/publication remains gated by activate/deactivate state.
    // This avoids "silent success" from pre-created interfaces before
    // activation.
    reset_srv_ = create_service<std_srvs::srv::Empty>(
        "/orbslam3/reset",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
            enqueue_command_(SlamCommand{SlamCommandType::kReset});
        },
        rmw_qos_profile_services_default, cmd_group_);

    shutdown_srv_ = create_service<std_srvs::srv::Empty>(
        "/orbslam3/shutdown",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
            shutdown_requested_.store(true, std::memory_order_release);
            enqueue_command_(SlamCommand{SlamCommandType::kShutdown});
        },
        rmw_qos_profile_services_default, cmd_group_);

    get_status_srv_ = create_service<Srv::GetStatus>(
        "/orbslam3/get_status",
        [this](const std::shared_ptr<Srv::GetStatus::Request>,
               std::shared_ptr<Srv::GetStatus::Response> response) {
            // Gate semantics:
            // - Counters are monotonic snapshots for machine-checkable
            // pass/fail.
            // - "running" reports worker+process liveness, not tracking
            // success.
            const auto pending_size = [&]() {
                std::lock_guard lock(pending_mutex_);
                return pending_observations_.size();
            }();
            response->running = !stop_.load(std::memory_order_acquire) &&
                                rclcpp::ok() && worker_.running();
            response->stop_requested = stop_.load(std::memory_order_relaxed);
            response->has_system = static_cast<bool>(system_);
            response->shutdown_requested =
                shutdown_requested_.load(std::memory_order_relaxed);
            response->worker_alive = worker_.running();
            response->worker_exception = get_worker_exception_();
            response->tracking_state =
                tracking_state_.load(std::memory_order_relaxed);
            response->tracking_state_name =
                Runtime::tracking_state_to_string(response->tracking_state);
            response->tracked_map_points =
                tracked_map_points_.load(std::memory_order_relaxed);
            response->pending_size = pending_size;
            response->pending_queue_size = pending_queue_size_;
            response->pending_drop_count = diagnostics_.pending_drop_count();
            response->expired_drop_count = diagnostics_.expired_drop_count();
            response->processed_obs_count =
                processed_obs_count_.load(std::memory_order_relaxed);
            response->rx_obs_count =
                rx_obs_count_.load(std::memory_order_relaxed);
            response->enqueued_obs_count =
                enqueued_obs_count_.load(std::memory_order_relaxed);
            response->ignored_obs_count =
                ignored_obs_count_.load(std::memory_order_relaxed);
            response->ignored_inactive_count =
                ignored_inactive_count_.load(std::memory_order_relaxed);
            response->ignored_not_configured_count =
                ignored_not_configured_count_.load(std::memory_order_relaxed);
            response->ignored_stopping_count =
                ignored_stopping_count_.load(std::memory_order_relaxed);
            response->ignored_no_backend_count =
                ignored_no_backend_count_.load(std::memory_order_relaxed);
            response->ignored_mode_mismatch_count =
                ignored_mode_mismatch_count_.load(std::memory_order_relaxed);
            response->track_result_count =
                track_result_count_.load(std::memory_order_relaxed);
            response->no_pose_count =
                no_pose_count_.load(std::memory_order_relaxed);
            response->pose_published_count =
                pose_published_count_.load(std::memory_order_relaxed);
            response->last_observation_stamp = ns_to_stamp(
                last_observation_stamp_ns_.load(std::memory_order_relaxed));
            response->last_observation_t_track_ns =
                last_observation_t_track_ns_.load(std::memory_order_relaxed);
            response->last_observation_dt_ns =
                last_observation_dt_ns_.load(std::memory_order_relaxed);
            response->memory_rss_kb =
                memory_rss_kb_.load(std::memory_order_relaxed);
            response->track_time_us_last = diagnostics_.track_time_us_last();
        },
        rmw_qos_profile_services_default, cmd_group_);

    get_tracked_points_srv_ = create_service<Srv::GetTrackedPoints>(
        "/orbslam3/get_tracked_points",
        [this](const std::shared_ptr<Srv::GetTrackedPoints::Request> request,
               std::shared_ptr<Srv::GetTrackedPoints::Response> response) {
            const auto now = get_clock()->now();
            const auto max_points =
                request->max_points > 0
                    ? static_cast<std::size_t>(request->max_points)
                    : pointcloud_max_points_;
            response->cloud = build_tracked_points_cloud_from_cache_(
                now, std::max<std::size_t>(1, max_points));
            response->success = true;
            response->message = "ok";
        },
        rmw_qos_profile_services_default, cmd_group_);

    save_trajectory_srv_ = create_service<Srv::SaveTrajectory>(
        "/orbslam3/save_trajectory",
        [this](const std::shared_ptr<Srv::SaveTrajectory::Request> request,
               std::shared_ptr<Srv::SaveTrajectory::Response> response) {
            SlamCommand command;
            command.type = SlamCommandType::kSaveTrajectory;
            command.format = request->format;
            command.path = request->path;
            const auto result = wait_command_result_(std::move(command));
            response->success = result.success;
            response->message = result.message;
        },
        rmw_qos_profile_services_default, cmd_group_);

    save_map_srv_ = create_service<Srv::SaveMap>(
        "/orbslam3/save_map",
        [this](const std::shared_ptr<Srv::SaveMap::Request> request,
               std::shared_ptr<Srv::SaveMap::Response> response) {
            SlamCommand command;
            command.type = SlamCommandType::kSaveMap;
            command.path = request->path;
            const auto result = wait_command_result_(std::move(command));
            response->success = result.success;
            response->message = result.message;
        },
        rmw_qos_profile_services_default, cmd_group_);

    load_map_srv_ = create_service<Srv::LoadMap>(
        "/orbslam3/load_map",
        [this](const std::shared_ptr<Srv::LoadMap::Request> request,
               std::shared_ptr<Srv::LoadMap::Response> response) {
            SlamCommand command;
            command.type = SlamCommandType::kLoadMap;
            command.path = request->path;
            const auto result = wait_command_result_(std::move(command));
            response->success = result.success;
            response->message = result.message;
        },
        rmw_qos_profile_services_default, cmd_group_);

    set_localization_mode_srv_ = create_service<Srv::SetLocalizationMode>(
        "/orbslam3/set_localization_mode",
        [this](const std::shared_ptr<Srv::SetLocalizationMode::Request> request,
               std::shared_ptr<Srv::SetLocalizationMode::Response> response) {
            SlamCommand command;
            command.type = SlamCommandType::kToggleLocalizationMode;
            command.localization_mode = request->localization_mode;
            const auto result = wait_command_result_(std::move(command));
            response->success = result.success;
            response->message = result.message;
        },
        rmw_qos_profile_services_default, cmd_group_);

    export_pointcloud_action_server_ =
        rclcpp_action::create_server<ExportPointCloud>(
            this, "/orbslam3/map/export_pointcloud",
            [this](const rclcpp_action::GoalUUID& uuid,
                   std::shared_ptr<const ExportPointCloud::Goal> goal) {
                (void)uuid;
                return handle_export_pointcloud_goal_(goal);
            },
            [this](
                const std::shared_ptr<GoalHandleExportPointCloud> goal_handle) {
                return handle_export_pointcloud_cancel_(goal_handle);
            },
            [this](
                const std::shared_ptr<GoalHandleExportPointCloud> goal_handle) {
                handle_export_pointcloud_accepted_(goal_handle);
            });
}

void Orbslam3CoreNode::setup_parameter_callback_() {
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "accepted";

            auto publish_tf = publish_tf_.load(std::memory_order_relaxed);
            auto publish_odom = publish_odom_.load(std::memory_order_relaxed);
            auto latest_keep_last =
                latest_keep_last_.load(std::memory_order_relaxed);
            auto latest_take = latest_take_.load(std::memory_order_relaxed);
            auto max_frame_age_ns =
                max_frame_age_ns_.load(std::memory_order_relaxed);
            auto lost_auto_reset_enable =
                lost_auto_reset_enable_.load(std::memory_order_relaxed);
            auto lost_auto_reset_frames =
                lost_auto_reset_frames_.load(std::memory_order_relaxed);

            for (const auto& parameter : parameters) {
                const auto& name = parameter.get_name();
                if (name == "publish_tf") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_BOOL) {
                        result.successful = false;
                        result.reason = "publish_tf must be bool.";
                        return result;
                    }
                    publish_tf = parameter.as_bool();
                    continue;
                }
                if (name == "publish_odom") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_BOOL) {
                        result.successful = false;
                        result.reason = "publish_odom must be bool.";
                        return result;
                    }
                    publish_odom = parameter.as_bool();
                    continue;
                }
                if (name == "max_frame_age_ms") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_INTEGER) {
                        result.successful = false;
                        result.reason = "max_frame_age_ms must be integer.";
                        return result;
                    }
                    const auto ms = parameter.as_int();
                    if (ms < 0) {
                        result.successful = false;
                        result.reason = "max_frame_age_ms must be >= 0.";
                        return result;
                    }
                    max_frame_age_ns = ms > 0 ? ms * 1'000'000 : 0;
                    continue;
                }
                if (name == "latest_keep_last") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_INTEGER) {
                        result.successful = false;
                        result.reason = "latest_keep_last must be integer.";
                        return result;
                    }
                    const auto value = parameter.as_int();
                    if (value < 1) {
                        result.successful = false;
                        result.reason = "latest_keep_last must be >= 1.";
                        return result;
                    }
                    latest_keep_last = static_cast<std::size_t>(value);
                    continue;
                }
                if (name == "latest_take") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_INTEGER) {
                        result.successful = false;
                        result.reason = "latest_take must be integer.";
                        return result;
                    }
                    const auto value = parameter.as_int();
                    if (value < 1) {
                        result.successful = false;
                        result.reason = "latest_take must be >= 1.";
                        return result;
                    }
                    latest_take = static_cast<std::size_t>(value);
                    continue;
                }
                if (name == "use_viewer") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_BOOL) {
                        result.successful = false;
                        result.reason = "use_viewer must be bool.";
                        return result;
                    }
                    const auto value = parameter.as_bool();
                    if (value != use_viewer_) {
                        result.successful = false;
                        result.reason =
                            "use_viewer cannot be changed at runtime.";
                        return result;
                    }
                    continue;
                }
                if (name == "lost_auto_reset_enable") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_BOOL) {
                        result.successful = false;
                        result.reason = "lost_auto_reset_enable must be bool.";
                        return result;
                    }
                    lost_auto_reset_enable = parameter.as_bool();
                    continue;
                }
                if (name == "lost_auto_reset_frames") {
                    if (parameter.get_type() !=
                        rclcpp::ParameterType::PARAMETER_INTEGER) {
                        result.successful = false;
                        result.reason =
                            "lost_auto_reset_frames must be integer.";
                        return result;
                    }
                    const auto value = parameter.as_int();
                    if (value < 1) {
                        result.successful = false;
                        result.reason = "lost_auto_reset_frames must be >= 1.";
                        return result;
                    }
                    lost_auto_reset_frames = value;
                    continue;
                }
                if (name == "sync_enable_span_guard" ||
                    name == "sync_enable_huge_jump_reset" ||
                    name == "sync_reset_on_time_backwards" ||
                    name == "sync_max_dt_ms" || name == "sync_queue_size" ||
                    name == "sync_max_span_ms" || name == "sync_huge_jump_ms" ||
                    name == "depth_scale" || name == "depth_max_m" ||
                    name == "depth_encoding_override" || name == "rgb_clone" ||
                    name == "depth_clone") {
                    result.successful = false;
                    result.reason =
                        "sync_*/depth_* convert parameters are frontend-only "
                        "in split architecture.";
                    return result;
                }
            }

            publish_odom_.store(publish_odom, std::memory_order_relaxed);
            publish_tf_.store(publish_tf, std::memory_order_relaxed);
            latest_keep_last_.store(latest_keep_last,
                                    std::memory_order_relaxed);
            latest_take_.store(latest_take, std::memory_order_relaxed);
            max_frame_age_ns_.store(max_frame_age_ns,
                                    std::memory_order_relaxed);
            lost_auto_reset_enable_.store(lost_auto_reset_enable,
                                          std::memory_order_relaxed);
            lost_auto_reset_frames_.store(lost_auto_reset_frames,
                                          std::memory_order_relaxed);

            {
                std::lock_guard lock(publisher_mutex_);
                if (publish_odom && !odom_pub_) {
                    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
                        odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10))
                                         .reliable()
                                         .durability_volatile());
                }
                if (publish_tf && !tf_broadcaster_) {
                    tf_broadcaster_ =
                        std::make_unique<tf2_ros::TransformBroadcaster>(this);
                }
            }

            return result;
        });
}

}  // namespace orbslam3_ros2::core
