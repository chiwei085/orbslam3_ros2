#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "orbslam3_ros2/action/export_point_cloud.hpp"
#include "orbslam3_ros2/srv/get_status.hpp"
#include "orbslam3_ros2/srv/get_tracked_points.hpp"
#include "orbslam3_ros2/srv/save_trajectory.hpp"
#include "orbslam3_ros2/srv/set_localization_mode.hpp"

class ServiceStormNode final : public rclcpp::Node
{
public:
    ServiceStormNode()
        : Node("svc_storm_node"), started_(std::chrono::steady_clock::now()) {
        runtime_s_ = std::max(0.0, declare_parameter("runtime_s", 0.0));
        max_in_flight_ = std::max(
            1, static_cast<int>(declare_parameter("max_in_flight", 32)));
        tracked_points_max_ = std::max(
            1, static_cast<int>(declare_parameter("tracked_points_max", 500)));
        save_format_ = declare_parameter("save_trajectory_format", std::string("tum"));
        save_path_ = declare_parameter("save_trajectory_path", std::string("/tmp/orbslam3_test_trajectory.tum"));

        reset_hz_ = std::max(0.0, declare_parameter("reset_rate_hz", 0.0));
        get_status_hz_ =
            std::max(0.0, declare_parameter("get_status_rate_hz", 20.0));
        tracked_points_hz_ = std::max(
            0.0, declare_parameter("get_tracked_points_rate_hz", 10.0));
        localization_toggle_hz_ = std::max(
            0.0, declare_parameter("set_localization_mode_rate_hz", 1.0));
        save_trajectory_hz_ = std::max(
            0.0, declare_parameter("save_trajectory_rate_hz", 0.33));
        export_hz_ = std::max(0.0, declare_parameter("export_rate_hz", 0.0));

        reset_client_ = create_client<std_srvs::srv::Empty>("/orbslam3/reset");
        status_client_ =
            create_client<orbslam3_ros2::srv::GetStatus>("/orbslam3/get_status");
        tracked_points_client_ = create_client<orbslam3_ros2::srv::GetTrackedPoints>(
            "/orbslam3/get_tracked_points");
        localization_client_ =
            create_client<orbslam3_ros2::srv::SetLocalizationMode>(
                "/orbslam3/set_localization_mode");
        save_trajectory_client_ =
            create_client<orbslam3_ros2::srv::SaveTrajectory>(
                "/orbslam3/save_trajectory");
        export_client_ = rclcpp_action::create_client<orbslam3_ros2::action::ExportPointCloud>(
            this, "/orbslam3/map/export_pointcloud");

        create_timer_if_needed_(reset_hz_, [this]() { send_reset_(); });
        create_timer_if_needed_(get_status_hz_, [this]() { send_get_status_(); });
        create_timer_if_needed_(tracked_points_hz_,
                                [this]() { send_get_tracked_points_(); });
        create_timer_if_needed_(localization_toggle_hz_,
                                [this]() { send_toggle_localization_(); });
        create_timer_if_needed_(save_trajectory_hz_,
                                [this]() { send_save_trajectory_(); });
        create_timer_if_needed_(export_hz_, [this]() { send_export_goal_(); });

        monitor_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&ServiceStormNode::on_monitor_, this));

        RCLCPP_INFO(get_logger(),
                    "svc_storm_node started rates reset=%.2f status=%.2f tracked=%.2f "
                    "localization=%.2f save=%.2f export=%.2f Hz",
                    reset_hz_, get_status_hz_, tracked_points_hz_,
                    localization_toggle_hz_, save_trajectory_hz_, export_hz_);
    }

private:
    struct ServiceStats
    {
        std::size_t sent{0};
        std::size_t completed{0};
        std::size_t failed{0};
        double max_rtt_ms{0.0};
    };

    template <typename ServiceT, typename Func>
    void record_completion_(ServiceStats& stats,
                            const std::chrono::steady_clock::time_point begin,
                            const std::shared_future<typename ServiceT::Response::SharedPtr>&
                                response,
                            Func&& success_predicate) {
        ++stats.completed;
        const auto rtt_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - begin)
                                .count();
        stats.max_rtt_ms = std::max(stats.max_rtt_ms, rtt_ms);

        auto ok = false;
        try {
            const auto resp = response.get();
            ok = success_predicate(resp);
        }
        catch (...) {
            ok = false;
        }
        if (!ok) {
            ++stats.failed;
        }
    }

    void create_timer_if_needed_(const double rate_hz,
                                 const std::function<void()>& cb) {
        if (rate_hz <= 0.0) {
            return;
        }
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate_hz));
        timers_.push_back(create_wall_timer(period, cb));
    }

    bool allow_send_(const std::size_t in_flight) const {
        return static_cast<int>(in_flight) < max_in_flight_;
    }

    void send_reset_() {
        if (!reset_client_->service_is_ready() || !allow_send_(reset_in_flight_)) {
            return;
        }
        ++reset_stats_.sent;
        ++reset_in_flight_;
        const auto begin = std::chrono::steady_clock::now();
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        reset_client_->async_send_request(
            request,
            [this, begin](
                const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response) {
                record_completion_<std_srvs::srv::Empty>(
                    reset_stats_, begin, response,
                    [](const std_srvs::srv::Empty::Response::SharedPtr&) {
                        return true;
                    });
                if (reset_in_flight_ > 0) {
                    --reset_in_flight_;
                }
            });
    }

    void send_get_status_() {
        if (!status_client_->service_is_ready() || !allow_send_(status_in_flight_)) {
            return;
        }
        ++status_stats_.sent;
        ++status_in_flight_;
        const auto begin = std::chrono::steady_clock::now();
        auto request = std::make_shared<orbslam3_ros2::srv::GetStatus::Request>();
        status_client_->async_send_request(
            request,
            [this, begin](const rclcpp::Client<orbslam3_ros2::srv::GetStatus>::SharedFuture
                              response) {
                record_completion_<orbslam3_ros2::srv::GetStatus>(
                    status_stats_, begin, response,
                    [](const orbslam3_ros2::srv::GetStatus::Response::SharedPtr&) {
                        return true;
                    });
                if (status_in_flight_ > 0) {
                    --status_in_flight_;
                }
            });
    }

    void send_get_tracked_points_() {
        if (!tracked_points_client_->service_is_ready() ||
            !allow_send_(tracked_in_flight_)) {
            return;
        }
        ++tracked_stats_.sent;
        ++tracked_in_flight_;
        const auto begin = std::chrono::steady_clock::now();
        auto request =
            std::make_shared<orbslam3_ros2::srv::GetTrackedPoints::Request>();
        request->max_points = tracked_points_max_;
        tracked_points_client_->async_send_request(
            request,
            [this, begin](const rclcpp::Client<orbslam3_ros2::srv::GetTrackedPoints>::SharedFuture
                              response) {
                record_completion_<orbslam3_ros2::srv::GetTrackedPoints>(
                    tracked_stats_, begin, response,
                    [](const orbslam3_ros2::srv::GetTrackedPoints::Response::SharedPtr& resp) {
                        return resp->success;
                    });
                if (tracked_in_flight_ > 0) {
                    --tracked_in_flight_;
                }
            });
    }

    void send_toggle_localization_() {
        if (!localization_client_->service_is_ready() ||
            !allow_send_(localization_in_flight_)) {
            return;
        }
        ++localization_stats_.sent;
        ++localization_in_flight_;
        const auto begin = std::chrono::steady_clock::now();
        auto request =
            std::make_shared<orbslam3_ros2::srv::SetLocalizationMode::Request>();
        localization_toggle_ = !localization_toggle_;
        request->localization_mode = localization_toggle_;
        localization_client_->async_send_request(
            request,
            [this, begin](const rclcpp::Client<orbslam3_ros2::srv::SetLocalizationMode>::SharedFuture
                              response) {
                record_completion_<orbslam3_ros2::srv::SetLocalizationMode>(
                    localization_stats_, begin, response,
                    [](const orbslam3_ros2::srv::SetLocalizationMode::Response::SharedPtr&) {
                        return true;
                    });
                if (localization_in_flight_ > 0) {
                    --localization_in_flight_;
                }
            });
    }

    void send_save_trajectory_() {
        if (!save_trajectory_client_->service_is_ready() ||
            !allow_send_(save_in_flight_)) {
            return;
        }
        ++save_stats_.sent;
        ++save_in_flight_;
        const auto begin = std::chrono::steady_clock::now();
        auto request =
            std::make_shared<orbslam3_ros2::srv::SaveTrajectory::Request>();
        request->format = save_format_;
        request->path = save_path_;
        save_trajectory_client_->async_send_request(
            request,
            [this, begin](const rclcpp::Client<orbslam3_ros2::srv::SaveTrajectory>::SharedFuture
                              response) {
                record_completion_<orbslam3_ros2::srv::SaveTrajectory>(
                    save_stats_, begin, response,
                    [](const orbslam3_ros2::srv::SaveTrajectory::Response::SharedPtr&) {
                        return true;
                    });
                if (save_in_flight_ > 0) {
                    --save_in_flight_;
                }
            });
    }

    void send_export_goal_() {
        if (!export_client_->action_server_is_ready() || export_goal_active_) {
            return;
        }
        orbslam3_ros2::action::ExportPointCloud::Goal goal;
        goal.scope = "tracking";
        goal.max_points = 500;
        goal.publish_snapshot = false;
        goal.include_cloud_in_result = false;

        rclcpp_action::Client<orbslam3_ros2::action::ExportPointCloud>::SendGoalOptions
            options;
        options.goal_response_callback =
            [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<
                       orbslam3_ros2::action::ExportPointCloud>>& handle) {
                if (!handle) {
                    ++export_failed_;
                    export_goal_active_ = false;
                    return;
                }
                ++export_accepted_;
                export_goal_active_ = true;
            };
        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<
                       orbslam3_ros2::action::ExportPointCloud>::WrappedResult&
                       result) {
                ++export_completed_;
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    ++export_failed_;
                }
                export_goal_active_ = false;
            };

        ++export_sent_;
        export_client_->async_send_goal(goal, options);
    }

    void on_monitor_() {
        const auto elapsed_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          started_)
                .count();
        if (runtime_s_ > 0.0 && elapsed_s >= runtime_s_) {
            rclcpp::shutdown();
            return;
        }

        static int tick = 0;
        ++tick;
        if ((tick % 10) != 0) {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "storm stats status=%zu/%zu fail=%zu max=%.2fms tracked=%zu/%zu "
            "fail=%zu max=%.2fms reset=%zu/%zu fail=%zu save=%zu/%zu fail=%zu "
            "export=%zu/%zu fail=%zu",
            status_stats_.completed, status_stats_.sent, status_stats_.failed,
            status_stats_.max_rtt_ms, tracked_stats_.completed,
            tracked_stats_.sent, tracked_stats_.failed, tracked_stats_.max_rtt_ms,
            reset_stats_.completed, reset_stats_.sent, reset_stats_.failed,
            save_stats_.completed, save_stats_.sent, save_stats_.failed,
            export_completed_, export_sent_, export_failed_);
    }

    double runtime_s_{0.0};
    int max_in_flight_{32};
    int tracked_points_max_{500};
    std::string save_format_{"tum"};
    std::string save_path_{"/tmp/orbslam3_test_trajectory.tum"};

    double reset_hz_{0.0};
    double get_status_hz_{20.0};
    double tracked_points_hz_{10.0};
    double localization_toggle_hz_{1.0};
    double save_trajectory_hz_{0.33};
    double export_hz_{0.0};

    ServiceStats reset_stats_{};
    ServiceStats status_stats_{};
    ServiceStats tracked_stats_{};
    ServiceStats localization_stats_{};
    ServiceStats save_stats_{};

    std::size_t reset_in_flight_{0};
    std::size_t status_in_flight_{0};
    std::size_t tracked_in_flight_{0};
    std::size_t localization_in_flight_{0};
    std::size_t save_in_flight_{0};

    bool localization_toggle_{false};
    bool export_goal_active_{false};
    std::size_t export_sent_{0};
    std::size_t export_accepted_{0};
    std::size_t export_completed_{0};
    std::size_t export_failed_{0};

    std::chrono::steady_clock::time_point started_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    rclcpp::Client<orbslam3_ros2::srv::GetStatus>::SharedPtr status_client_;
    rclcpp::Client<orbslam3_ros2::srv::GetTrackedPoints>::SharedPtr
        tracked_points_client_;
    rclcpp::Client<orbslam3_ros2::srv::SetLocalizationMode>::SharedPtr
        localization_client_;
    rclcpp::Client<orbslam3_ros2::srv::SaveTrajectory>::SharedPtr
        save_trajectory_client_;
    rclcpp_action::Client<orbslam3_ros2::action::ExportPointCloud>::SharedPtr
        export_client_;

    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceStormNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
