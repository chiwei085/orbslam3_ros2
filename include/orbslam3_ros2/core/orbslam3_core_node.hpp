#pragma once
/**
 * @file orbslam3_core_node.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Lifecycle core node contract for ORB-SLAM3 integration.
 *
 * Contract:
 * - Observation input is event-driven and consumed from SensorDataQoS.
 * - Pose/Odom publications use explicit QoS to avoid RMW-dependent defaults.
 * - Topic/service availability follows ROS 2 lifecycle transitions.
 *
 * Rationale:
 * - Local deterministic gates assert both transport behavior and runtime
 * counters.
 */

#include <Eigen/Core>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "orbslam3_ros2/action/export_point_cloud.hpp"
#include "orbslam3_ros2/core/command_completion.hpp"
#include "orbslam3_ros2/core/guarded_worker.hpp"
#include "orbslam3_ros2/core/slam_command_queue.hpp"
#include "orbslam3_ros2/core/tracking_backend_interface.hpp"
#include "orbslam3_ros2/core/tracking_diagnostics.hpp"
#include "orbslam3_ros2/msg/observations.hpp"
#include "orbslam3_ros2/orbslam3/system.hpp"
#include "orbslam3_ros2/srv/get_status.hpp"
#include "orbslam3_ros2/srv/get_tracked_points.hpp"
#include "orbslam3_ros2/srv/load_map.hpp"
#include "orbslam3_ros2/srv/save_map.hpp"
#include "orbslam3_ros2/srv/save_trajectory.hpp"
#include "orbslam3_ros2/srv/set_localization_mode.hpp"

namespace orbslam3_ros2::core
{

/**
 * @brief Lifecycle core that owns ORB-SLAM3 backend and publishes pose/TF.
 *
 * Invariants:
 * - Pose/TF is published only when the node is active and tracking produced a
 * valid pose.
 * - Status counters are monotonic and safe for concurrent callback/worker
 * access.
 * - Runtime resources are created in configure/activate and released in
 * deactivate/cleanup.
 */
class Orbslam3CoreNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
    using ObservationPtr = orbslam3_ros2::msg::Observations::SharedPtr;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn;
    using ExportPointCloud = orbslam3_ros2::action::ExportPointCloud;
    using GoalHandleExportPointCloud =
        rclcpp_action::ServerGoalHandle<ExportPointCloud>;
    using CommandResult = CommandCompletion::Result;

    Orbslam3CoreNode();
    ~Orbslam3CoreNode() override;

    /**
     * @brief Configure runtime resources and external interfaces.
     * @return Lifecycle transition result.
     * @post On success, services and topic interfaces are created but
     * publication remains inactive.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    /**
     * @brief Activate publishers and worker processing.
     * @return Lifecycle transition result.
     * @post On success, observation callbacks may enqueue work and output
     * topics may publish.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    /**
     * @brief Deactivate publishers while keeping configuration.
     * @return Lifecycle transition result.
     * @post Observation input may still arrive but is contractually ignored
     * while inactive.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    /**
     * @brief Release configured resources and reset runtime state.
     * @return Lifecycle transition result.
     * @post Interfaces are removed and backend resources are released.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    /**
     * @brief Stop processing and perform final teardown.
     * @return Lifecycle transition result.
     * @post Worker is stopped and no further publications are emitted.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
    static bool starts_with_trimmed_(const std::string& line,
                                     const std::string& key);
    bool write_runtime_settings_(const std::optional<std::string>& load_stem,
                                 const std::optional<std::string>& save_stem,
                                 std::string* error_message = nullptr);
    void prepare_runtime_paths_();
    std::filesystem::path normalize_map_path_(
        const std::string& request_path) const;
    enum class RecreateReason : std::uint8_t
    {
        kConfigure,
        kSaveMap,
        kLoadMap,
    };
    static const char* recreate_reason_to_string_(RecreateReason reason);
    bool recreate_system_(RecreateReason reason, bool load_internal,
                          std::string* error_message);

    CommandResult wait_command_result_(SlamCommand command);
    void complete_command_(const SlamCommand& command, bool success,
                           const std::string& message);
    std::string get_worker_exception_() const;

    CallbackReturn configure_resources_();
    void cleanup_resources_();
    /**
     * @brief Create observation subscription and output publishers.
     *
     * @param qos_keep_last History depth for observation subscription.
     * @pre Called from configure phase before activation.
     * @post Subscription and publishers are created with deterministic QoS
     * contract.
     */
    void setup_observation_io_(std::int64_t qos_keep_last);
    void setup_control_interfaces_();
    void setup_parameter_callback_();

    bool start_worker_();
    void stop_worker_();
    std::string to_lower_ascii_(std::string value) const;
    bool is_expired_(std::int64_t now_ns,
                     const ObservationPtr& obs) const noexcept;

    /**
     * @brief Enqueue an observation for tracking.
     *
     * @param msg Observation message; header stamp is used as tracking time
     * source.
     * @pre Callback may run concurrently on ROS executor threads.
     * @post Counters are updated and message is queued or ignored according to
     * lifecycle state.
     * @warning Message is ignored when inactive, unconfigured, stopping, or
     * backend is unavailable.
     */
    void on_observation_(
        const orbslam3_ros2::msg::Observations::SharedPtr& msg);
    void enqueue_observation_(ObservationPtr observation);
    void enqueue_command_(SlamCommand command);
    bool has_pending_commands_() const;
    bool drain_commands_();
    void worker_loop_();
    void handle_observation_(const ObservationPtr& observation);

    sensor_msgs::msg::PointCloud2 build_tracked_points_cloud_from_cache_(
        const rclcpp::Time& stamp, std::size_t max_points) const;
    void update_tracked_points_cache_(std::size_t max_points);
    bool write_cloud_as_xyz_pcd_(const sensor_msgs::msg::PointCloud2& cloud,
                                 const std::string& output_path,
                                 std::string* error_message) const;

    rclcpp_action::GoalResponse handle_export_pointcloud_goal_(
        const std::shared_ptr<const ExportPointCloud::Goal>& goal);
    rclcpp_action::CancelResponse handle_export_pointcloud_cancel_(
        const std::shared_ptr<GoalHandleExportPointCloud>);
    void handle_export_pointcloud_accepted_(
        const std::shared_ptr<GoalHandleExportPointCloud> goal_handle);
    void join_export_threads_();

    void idle_or_yield_(std::size_t pending_size_after) const;
    void log_sync_stats_();
    void produce_diagnostics_(
        diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
    std::string voc_file_;
    std::string settings_file_;
    std::string runtime_settings_file_;
    std::filesystem::path runtime_dir_;
    std::filesystem::path runtime_atlas_stem_abs_;
    std::filesystem::path runtime_atlas_file_abs_;
    std::string runtime_atlas_stem_rel_;

    bool use_viewer_{false};
    bool test_mode_skip_orb_init_{false};
    std::int64_t test_mode_track_delay_us_{0};
    std::string map_frame_{"map"};
    std::string camera_frame_param_{};
    std::string observation_topic_{"/orbslam3/observations"};
    std::string camera_pose_topic_{"/orbslam3/camera_pose"};
    std::string odom_topic_{"/orbslam3/odom"};
    std::atomic<bool> publish_odom_{false};
    std::atomic<bool> publish_tf_{true};

    std::size_t pending_queue_size_{10};
    std::atomic<std::size_t> latest_keep_last_{3};
    std::atomic<std::size_t> latest_take_{1};
    std::int64_t worker_time_budget_us_{8000};
    std::int64_t worker_idle_sleep_us_{0};
    std::int64_t no_work_wait_us_{2000};
    std::atomic<std::int64_t> max_frame_age_ns_{0};
    std::atomic<bool> lost_auto_reset_enable_{false};
    std::atomic<std::int64_t> lost_auto_reset_frames_{30};
    std::size_t pointcloud_max_points_{5000};
    std::atomic<std::int64_t> consecutive_lost_frames_{0};
    std::atomic<bool> localization_mode_enabled_{false};
    TrackingDiagnostics diagnostics_{};

    std::mutex pending_mutex_;
    std::mutex publisher_mutex_;
    std::mutex export_threads_mutex_;
    CommandCompletion command_completion_{};
    std::condition_variable pending_cv_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> configured_{false};
    std::atomic<bool> active_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> system_shutdown_called_{false};
    std::atomic<int> tracking_state_{ORB_SLAM3::Tracking::SYSTEM_NOT_READY};
    std::atomic<std::uint64_t> tracked_map_points_{0};
    std::atomic<std::uint64_t> processed_obs_count_{0};
    std::atomic<std::uint64_t> rx_obs_count_{0};
    std::atomic<std::uint64_t> enqueued_obs_count_{0};
    std::atomic<std::uint64_t> ignored_obs_count_{0};
    std::atomic<std::uint64_t> ignored_inactive_count_{0};
    std::atomic<std::uint64_t> ignored_not_configured_count_{0};
    std::atomic<std::uint64_t> ignored_stopping_count_{0};
    std::atomic<std::uint64_t> ignored_no_backend_count_{0};
    std::atomic<std::uint64_t> ignored_mode_mismatch_count_{0};
    std::atomic<std::uint64_t> track_result_count_{0};
    std::atomic<std::uint64_t> no_pose_count_{0};
    std::atomic<std::uint64_t> pose_published_count_{0};
    std::atomic<std::int64_t> last_observation_stamp_ns_{0};
    std::atomic<std::uint64_t> last_observation_t_track_ns_{0};
    std::atomic<std::int64_t> last_observation_dt_ns_{0};
    std::atomic<std::uint64_t> memory_rss_kb_{0};
    std::atomic<std::uint64_t> system_recreate_count_{0};
    std::atomic<RecreateReason> last_recreate_reason_{
        RecreateReason::kConfigure};
    mutable std::mutex recreate_state_mutex_;
    std::string last_recreate_error_{};
    mutable std::mutex tracked_points_cache_mutex_;
    std::vector<Eigen::Vector3f> tracked_points_cache_;
    std::deque<ObservationPtr> pending_observations_;
    SlamCommandQueue cmd_queue_;
    std::vector<ObservationPtr> local_observation_batch_;
    std::vector<std::thread> export_threads_;
    std::unique_ptr<TrackingBackend> tracking_backend_;
    // system_ is accessed from worker_ and teardown after worker_ joined.
    std::unique_ptr<ORB_SLAM3::System> system_;
    GuardedWorker worker_{};
    diagnostic_updater::Updater diagnostics_updater_;
    rclcpp::CallbackGroup::SharedPtr cam_group_;
    rclcpp::CallbackGroup::SharedPtr stats_group_;
    rclcpp::CallbackGroup::SharedPtr cmd_group_;

    rclcpp::Subscription<orbslam3_ros2::msg::Observations>::SharedPtr obs_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv_;
    rclcpp::Service<orbslam3_ros2::srv::GetStatus>::SharedPtr get_status_srv_;
    rclcpp::Service<orbslam3_ros2::srv::GetTrackedPoints>::SharedPtr
        get_tracked_points_srv_;
    rclcpp::Service<orbslam3_ros2::srv::SaveTrajectory>::SharedPtr
        save_trajectory_srv_;
    rclcpp::Service<orbslam3_ros2::srv::SaveMap>::SharedPtr save_map_srv_;
    rclcpp::Service<orbslam3_ros2::srv::LoadMap>::SharedPtr load_map_srv_;
    rclcpp::Service<orbslam3_ros2::srv::SetLocalizationMode>::SharedPtr
        set_localization_mode_srv_;
    rclcpp_action::Server<ExportPointCloud>::SharedPtr
        export_pointcloud_action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        snapshot_pointcloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_parameters_callback_handle_;
};

}  // namespace orbslam3_ros2::core
