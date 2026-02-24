#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "orbslam3_ros2/common/pose_path_publisher.hpp"
#include "orbslam3_ros2/common/reset_service.hpp"
#include "orbslam3_ros2/common/tf_publisher.hpp"

namespace
{

using Image = sensor_msgs::msg::Image;
using ApproximateSyncPolicy =
    message_filters::sync_policies::ApproximateTime<Image, Image>;

auto rgb_encoding_from_settings(const std::string& settings_file,
                                rclcpp::Logger logger) -> std::string {
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

auto is_supported_depth_encoding(const std::string& encoding) -> bool {
    using sensor_msgs::image_encodings::TYPE_16UC1;
    using sensor_msgs::image_encodings::TYPE_32FC1;
    return encoding == TYPE_16UC1 || encoding == TYPE_32FC1;
}
}  // namespace

class RgbdNode final : public rclcpp::Node
{
public:
    RgbdNode()
        : rclcpp::Node("orb_slam3"),
          topic_prefix_("/" + std::string(get_name())),
          tf_publisher_(*this),
          rgb_sub_(),
          depth_sub_(),
          sync_(ApproximateSyncPolicy(10), rgb_sub_, depth_sub_) {
        declare_parameter("voc_file", "file_not_set");
        declare_parameter("settings_file", "file_not_set");
        declare_parameter("world_frame_id", "map");
        declare_parameter("cam_frame_id", "camera");
        declare_parameter("rgb_topic", "/camera/rgb/image_raw");
        declare_parameter("depth_topic", "/camera/depth_registered/image_raw");
        declare_parameter("enable_pangolin", true);

        auto voc_file = get_parameter("voc_file").as_string();
        auto settings_file = get_parameter("settings_file").as_string();
        world_frame_id_ = get_parameter("world_frame_id").as_string();
        cam_frame_id_ = get_parameter("cam_frame_id").as_string();
        auto rgb_topic = get_parameter("rgb_topic").as_string();
        auto depth_topic = get_parameter("depth_topic").as_string();
        auto enable_pangolin = get_parameter("enable_pangolin").as_bool();

        if (voc_file == "file_not_set" || settings_file == "file_not_set") {
            RCLCPP_FATAL(
                get_logger(),
                "Please provide voc_file and settings_file in the launch file");
            throw std::runtime_error(
                "Missing voc_file/settings_file parameters");
        }

        slam_ = std::make_unique<ORB_SLAM3::System>(
            voc_file, settings_file, ORB_SLAM3::System::RGBD, enable_pangolin);
        rgb_encoding_ = rgb_encoding_from_settings(settings_file, get_logger());

        pose_path_publisher_ =
            std::make_unique<orbslam3_ros2::common::PosePathPublisher>(
                *this, topic_prefix_, world_frame_id_);

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

        RCLCPP_INFO(get_logger(), "orb_slam3 RGB-D node ready");
    }

    ~RgbdNode() override { shutdown_slam(); }

    void shutdown_slam() {
        std::scoped_lock lock(slam_mutex_);
        if (!slam_ || slam_shutdown_) {
            return;
        }
        slam_->Shutdown();
        slam_shutdown_ = true;
    }

private:
    void grab_rgbd(const Image::ConstSharedPtr& msg_rgb,
                   const Image::ConstSharedPtr& msg_depth) {
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
        tf_publisher_.publish(twc, world_frame_id_, cam_frame_id_, stamp);
    }

private:
    std::string topic_prefix_;
    std::string world_frame_id_;
    std::string cam_frame_id_;
    std::string rgb_encoding_;

    std::mutex slam_mutex_;
    bool slam_shutdown_{false};
    std::unique_ptr<ORB_SLAM3::System> slam_;

    std::unique_ptr<orbslam3_ros2::common::PosePathPublisher>
        pose_path_publisher_;
    orbslam3_ros2::common::TfPublisher tf_publisher_;
    std::unique_ptr<orbslam3_ros2::common::ResetService> reset_service_;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Synchronizer<ApproximateSyncPolicy> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdNode>();
    rclcpp::spin(node);
    node->shutdown_slam();
    rclcpp::shutdown();
    return 0;
}
