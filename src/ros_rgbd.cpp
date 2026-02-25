#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "orbslam3_ros2/common/pose_path_publisher.hpp"
#include "orbslam3_ros2/common/reset_service.hpp"
#include "orbslam3_ros2/common/viewer_backend_policy.hpp"

#include "ShutdownTrace.h"

namespace
{

using Image = sensor_msgs::msg::Image;
using ApproximateSyncPolicy =
    message_filters::sync_policies::ApproximateTime<Image, Image>;

std::string rgb_encoding_from_settings(const std::string& settings_file,
                                       rclcpp::Logger logger) {
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_WARN(logger, "Failed to open settings_file for Camera.RGB: %s",
                    settings_file.c_str());
        return {};
    }

    auto camera_rgb = 1;
    auto camera_rgb_node = fs["Camera.RGB"];
    if (camera_rgb_node.empty()) {
        RCLCPP_WARN(logger,
                    "Camera.RGB not found in settings_file, using incoming "
                    "RGB encoding");
        return {};
    }
    camera_rgb_node >> camera_rgb;

    return camera_rgb == 1 ? sensor_msgs::image_encodings::RGB8
                           : sensor_msgs::image_encodings::BGR8;
}

bool is_supported_depth_encoding(const std::string& encoding) {
    using sensor_msgs::image_encodings::TYPE_16UC1;
    using sensor_msgs::image_encodings::TYPE_32FC1;
    return encoding == TYPE_16UC1 || encoding == TYPE_32FC1;
}

tf2::Transform tf2_from_sophus(const Sophus::SE3f& pose) {
    const auto t = pose.translation();
    const auto q = pose.unit_quaternion();
    return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
                          tf2::Vector3(t.x(), t.y(), t.z()));
}

void trace_shutdown(const char* tag, const std::string& msg) {
#ifdef ORBROS_SHUTDOWN_TRACE
    ORB_SLAM3::shutdown_trace::log(tag, msg);
#else
    (void)tag;
    (void)msg;
#endif
}
}  // namespace

class RgbdNode final : public rclcpp::Node
{
public:
    RgbdNode()
        : rclcpp::Node("orb_slam3"),
          topic_prefix_("/" + std::string(get_name())),
          tf_broadcaster_(*this),
          rgb_sub_(),
          depth_sub_(),
          sync_(ApproximateSyncPolicy(10), rgb_sub_, depth_sub_) {
        declare_parameter("voc_file", "file_not_set");
        declare_parameter("settings_file", "file_not_set");
        declare_parameter("map_frame_id", "map");
        declare_parameter("cam_frame_id", "camera_link");
        declare_parameter("rgb_topic", "/camera/rgb/image_raw");
        declare_parameter("depth_topic", "/camera/depth_registered/image_raw");
        declare_parameter("enable_pangolin", true);
        declare_parameter("viewer_backend", "auto");
        declare_parameter("shutdown_after_sec", 0.0);

        auto voc_file = get_parameter("voc_file").as_string();
        auto settings_file = get_parameter("settings_file").as_string();
        map_frame_id_ = get_parameter("map_frame_id").as_string();
        cam_frame_id_ = get_parameter("cam_frame_id").as_string();
        auto rgb_topic = get_parameter("rgb_topic").as_string();
        auto depth_topic = get_parameter("depth_topic").as_string();
        auto enable_pangolin = get_parameter("enable_pangolin").as_bool();
        auto viewer_backend = get_parameter("viewer_backend").as_string();
        auto shutdown_after_sec =
            get_parameter("shutdown_after_sec").as_double();

        RCLCPP_INFO(get_logger(),
                    "[tf] standalone publishing dynamic TF only: %s->%s",
                    map_frame_id_.c_str(), cam_frame_id_.c_str());

        if (voc_file == "file_not_set" || settings_file == "file_not_set") {
            RCLCPP_FATAL(
                get_logger(),
                "Please provide voc_file and settings_file in the launch file");
            throw std::runtime_error(
                "Missing voc_file/settings_file parameters");
        }

        const auto viewer_env =
            orbslam3_ros2::common::get_viewer_backend_env_from_process();
        auto viewer_policy =
            orbslam3_ros2::common::evaluate_viewer_backend_policy(
                enable_pangolin, viewer_backend, viewer_env);

        if (!viewer_policy.parse_warning.empty()) {
            RCLCPP_WARN(get_logger(), "%s",
                        viewer_policy.parse_warning.c_str());
        }
        if (viewer_policy.warn_wayland_teardown) {
            RCLCPP_WARN(get_logger(),
                        "[viewer] viewer_backend=wayland requested; Wayland "
                        "backend teardown may crash in some environments/WSLg. "
                        "If it does, use auto or x11.");
        }

        if (!orbslam3_ros2::common::apply_viewer_backend_env_policy(
                viewer_policy)) {
            RCLCPP_ERROR(get_logger(),
                         "[viewer] failed to apply env override "
                         "(WAYLAND_DISPLAY=%s) -> disabling viewer",
                         viewer_policy.wayland_display_override.c_str());
            viewer_policy.effective_enable = false;
            viewer_policy.reason = "failed to force X11 -> disabling viewer";
        }

        RCLCPP_INFO(
            get_logger(),
            "[viewer] enable=%d backend=%s detected_wayland=%d DISPLAY=%s "
            "WAYLAND_DISPLAY=%s XDG_SESSION_TYPE=%s -> %s",
            enable_pangolin ? 1 : 0, viewer_policy.backend_name.c_str(),
            viewer_policy.detected_wayland ? 1 : 0,
            viewer_env.display.empty() ? "<empty>" : viewer_env.display.c_str(),
            viewer_env.wayland_display.empty()
                ? "<empty>"
                : viewer_env.wayland_display.c_str(),
            viewer_env.xdg_session_type.empty()
                ? "<empty>"
                : viewer_env.xdg_session_type.c_str(),
            viewer_policy.reason.c_str());
        const auto wayland_after =
            orbslam3_ros2::common::get_env_string("WAYLAND_DISPLAY");
        RCLCPP_INFO(get_logger(),
                    "[viewer] effective_enable=%d WAYLAND_DISPLAY(after)=%s",
                    viewer_policy.effective_enable ? 1 : 0,
                    wayland_after.empty() ? "<empty>" : wayland_after.c_str());

        enable_pangolin = viewer_policy.effective_enable;

        slam_ = std::make_unique<ORB_SLAM3::System>(
            voc_file, settings_file, ORB_SLAM3::System::RGBD, enable_pangolin);
        rgb_encoding_ = rgb_encoding_from_settings(settings_file, get_logger());

        pose_path_publisher_ =
            std::make_unique<orbslam3_ros2::common::PosePathPublisher>(
                *this, topic_prefix_, map_frame_id_);

        reset_service_ = std::make_unique<orbslam3_ros2::common::ResetService>(
            *this, topic_prefix_, slam_mutex_,
            [this, publisher = pose_path_publisher_.get()] {
                if (slam_) {
                    slam_->Reset();
                    publisher->reset();
                }
            });

        auto qos = rclcpp::SensorDataQoS();
        rgb_sub_.subscribe(this, rgb_topic, qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, depth_topic, qos.get_rmw_qos_profile());

        sync_.registerCallback(&RgbdNode::grab_rgbd, this);

        if (shutdown_after_sec > 0.0) {
            auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(shutdown_after_sec));
            shutdown_timer_ = create_wall_timer(delay, [this]() {
                if (shutdown_timer_) {
                    shutdown_timer_->cancel();
                }
                trace_shutdown("wrapper.timer",
                               state_string("timer_fired invoking shutdown"));
                shutdown_slam("timer");
                rclcpp::shutdown();
            });
            trace_shutdown("wrapper.timer", state_string("timer_armed"));
        }

        RCLCPP_INFO(get_logger(), "orb_slam3 RGB-D node ready");
    }

    ~RgbdNode() override {
        trace_shutdown("wrapper.dtor", state_string("enter"));
        shutdown_slam("destructor");
        trace_shutdown("wrapper.dtor", state_string("exit"));
    }

    void shutdown_slam(const char* origin = "unknown") {
        bool expected = false;
        if (!shutdown_started_.compare_exchange_strong(expected, true)) {
            trace_shutdown(
                "wrapper.shutdown",
                state_string(std::string("skip_already_started origin=") +
                             origin));
            return;
        }

        trace_shutdown("wrapper.shutdown",
                       state_string(std::string("begin origin=") + origin));

        accept_callbacks_.store(false, std::memory_order_release);
        rgb_sub_.unsubscribe();
        depth_sub_.unsubscribe();
        trace_shutdown("wrapper.shutdown", state_string("unsubscribed_topics"));

        while (callbacks_in_flight_.load(std::memory_order_acquire) != 0) {
            trace_shutdown("wrapper.shutdown",
                           state_string("waiting_callbacks"));
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        std::scoped_lock lock(slam_mutex_);
        trace_shutdown("wrapper.shutdown",
                       state_string("locked_before_shutdown"));
        if (!slam_ || slam_shutdown_) {
            trace_shutdown("wrapper.shutdown",
                           state_string("skip_no_slam_or_already_shutdown"));
            return;
        }
        trace_shutdown("wrapper.shutdown",
                       state_string("calling_slam_shutdown"));
        slam_->Shutdown();
        slam_shutdown_ = true;
        trace_shutdown("wrapper.shutdown",
                       state_string("slam_shutdown_returned"));
    }

private:
    void grab_rgbd(const Image::ConstSharedPtr& msg_rgb,
                   const Image::ConstSharedPtr& msg_depth) {
        if (!accept_callbacks_.load(std::memory_order_acquire)) {
            trace_shutdown("wrapper.cb", state_string("drop_before_enter"));
            return;
        }
        CallbackGuard cb_guard(*this);
        if (!accept_callbacks_.load(std::memory_order_acquire)) {
            trace_shutdown("wrapper.cb", state_string("drop_after_guard"));
            return;
        }

        cv_bridge::CvImageConstPtr cv_rgb;
        cv_bridge::CvImageConstPtr cv_depth;

        try {
            if (rgb_encoding_.empty()) {
                cv_rgb = cv_bridge::toCvShare(msg_rgb);
            }
            else {
                cv_rgb = cv_bridge::toCvShare(msg_rgb, rgb_encoding_);
            }
            if (!is_supported_depth_encoding(msg_depth->encoding)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Unsupported depth encoding '%s' (expected 16UC1/32FC1)",
                    msg_depth->encoding.c_str());
                return;
            }
            cv_depth = cv_bridge::toCvShare(msg_depth);
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        Sophus::SE3f tcw;
        {
            std::scoped_lock lock(slam_mutex_);
            if (!slam_) {
                trace_shutdown("wrapper.cb", state_string("slam_null"));
                return;
            }
            auto stamp = rclcpp::Time(msg_rgb->header.stamp).seconds();
            tcw = slam_->TrackRGBD(cv_rgb->image, cv_depth->image, stamp);
        }

        auto twc = tcw.inverse();
        auto stamp = msg_rgb->header.stamp;
        if (!pose_path_publisher_->publish(twc, stamp)) {
            return;
        }

        publish_camera_tf(twc, stamp);
    }

    void publish_camera_tf(const Sophus::SE3f& twc,
                           const builtin_interfaces::msg::Time& stamp) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame_id_;
        tf_msg.child_frame_id = cam_frame_id_;
        tf_msg.transform = tf2::toMsg(tf2_from_sophus(twc));
        tf_broadcaster_.sendTransform(tf_msg);
    }

    struct CallbackGuard
    {
        explicit CallbackGuard(RgbdNode& self) : self_(self) {
            self_.callbacks_in_flight_.fetch_add(1, std::memory_order_acq_rel);
            trace_shutdown("wrapper.cb", self_.state_string("enter"));
        }
        ~CallbackGuard() {
            self_.callbacks_in_flight_.fetch_sub(1, std::memory_order_acq_rel);
            trace_shutdown("wrapper.cb", self_.state_string("exit"));
        }
        RgbdNode& self_;
    };

    std::string state_string(const std::string& label) const {
        std::ostringstream oss;
        oss << label << " slam_shutdown_=" << (slam_shutdown_ ? 1 : 0)
            << " shutdown_started="
            << shutdown_started_.load(std::memory_order_relaxed)
            << " accept_callbacks="
            << accept_callbacks_.load(std::memory_order_relaxed)
            << " callbacks_in_flight="
            << callbacks_in_flight_.load(std::memory_order_relaxed);
        return oss.str();
    }

private:
    std::string topic_prefix_;
    std::string map_frame_id_;
    std::string cam_frame_id_;
    std::string rgb_encoding_;

    std::mutex slam_mutex_;
    bool slam_shutdown_{false};
    std::atomic<bool> shutdown_started_{false};
    std::atomic<bool> accept_callbacks_{true};
    std::atomic<int> callbacks_in_flight_{0};
    std::unique_ptr<ORB_SLAM3::System> slam_;

    std::unique_ptr<orbslam3_ros2::common::PosePathPublisher>
        pose_path_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<orbslam3_ros2::common::ResetService> reset_service_;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Synchronizer<ApproximateSyncPolicy> sync_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdNode>();
    rclcpp::spin(node);
    trace_shutdown("wrapper.main", "spin returned; invoking shutdown_slam");
    node->shutdown_slam("main_after_spin");
    rclcpp::shutdown();
    trace_shutdown("wrapper.main", "rclcpp::shutdown returned");
    return 0;
}
