#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "orbslam3_ros2/msg/observations.hpp"

namespace
{
std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time ns_to_stamp(const std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    const auto clamped_ns = std::max<std::int64_t>(ns, 0);
    stamp.sec = static_cast<std::int32_t>(clamped_ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(clamped_ns % 1000000000LL);
    return stamp;
}
}  // namespace

class ObservationsPublisherNode final : public rclcpp::Node
{
public:
    ObservationsPublisherNode()
        : Node("obs_pub_node"),
          started_(std::chrono::steady_clock::now()),
          next_publish_(started_) {
        topic_ = declare_parameter("observation_topic",
                                   std::string("/orbslam3/observations"));
        camera_frame_ =
            declare_parameter("camera_frame", std::string("camera_link"));
        mode_ = static_cast<std::uint8_t>(
            declare_parameter("mode", static_cast<int>(
                                         orbslam3_ros2::msg::Observations::MODE_RGBD)));
        rate_hz_ = std::max(0.1, declare_parameter("rate_hz", 30.0));
        burst_rate_hz_ = std::max(0.0, declare_parameter("burst_rate_hz", 0.0));
        burst_start_s_ = std::max(0.0, declare_parameter("burst_start_s", 0.0));
        burst_duration_s_ =
            std::max(0.0, declare_parameter("burst_duration_s", 0.0));
        runtime_s_ = std::max(0.0, declare_parameter("runtime_s", 0.0));
        stamp_policy_ = declare_parameter("stamp_policy", std::string("ros_now"));
        width_ = std::max(2, static_cast<int>(declare_parameter("image_w", 64)));
        height_ =
            std::max(2, static_cast<int>(declare_parameter("image_h", 48)));
        dt_ns_override_ = declare_parameter("dt_ns", static_cast<std::int64_t>(0));
        qos_depth_ =
            std::max(1, static_cast<int>(declare_parameter("qos_depth", 10)));

        pub_ = create_publisher<orbslam3_ros2::msg::Observations>(
            topic_, rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(qos_depth_)));

        const auto initial_rate = active_rate_hz_(0.0);
        synthetic_time_ns_ = get_clock()->now().nanoseconds();
        synthetic_step_ns_ = hz_to_dt_ns_(initial_rate);

        tick_timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&ObservationsPublisherNode::on_tick_, this));

        RCLCPP_INFO(get_logger(),
                    "obs_pub_node started topic=%s rate=%.2fHz burst_rate=%.2fHz "
                    "burst=[%.2fs, %.2fs] stamp_policy=%s image=%dx%d",
                    topic_.c_str(), rate_hz_, burst_rate_hz_, burst_start_s_,
                    burst_duration_s_, stamp_policy_.c_str(), width_, height_);
    }

private:
    static std::int64_t hz_to_dt_ns_(const double hz) {
        const auto safe_hz = std::max(0.1, hz);
        return static_cast<std::int64_t>(1000000000.0 / safe_hz);
    }

    double active_rate_hz_(const double elapsed_s) const {
        if (burst_rate_hz_ <= 0.0 || burst_duration_s_ <= 0.0) {
            return rate_hz_;
        }
        if (elapsed_s >= burst_start_s_ &&
            elapsed_s < (burst_start_s_ + burst_duration_s_)) {
            return burst_rate_hz_;
        }
        return rate_hz_;
    }

    void on_tick_() {
        const auto now_steady = std::chrono::steady_clock::now();
        const auto elapsed_s =
            std::chrono::duration<double>(now_steady - started_).count();
        if (runtime_s_ > 0.0 && elapsed_s >= runtime_s_) {
            rclcpp::shutdown();
            return;
        }

        const auto current_rate_hz = active_rate_hz_(elapsed_s);
        const auto current_period =
            std::chrono::nanoseconds(hz_to_dt_ns_(current_rate_hz));

        if (now_steady < next_publish_) {
            return;
        }
        next_publish_ += current_period;
        if (next_publish_ < now_steady) {
            next_publish_ = now_steady + current_period;
        }

        orbslam3_ros2::msg::Observations obs;
        obs.mode = mode_;
        obs.camera_frame = camera_frame_;

        if (stamp_policy_ == "synthetic") {
            obs.header.stamp = ns_to_stamp(synthetic_time_ns_);
            synthetic_step_ns_ = hz_to_dt_ns_(current_rate_hz);
            synthetic_time_ns_ += synthetic_step_ns_;
        }
        else {
            obs.header.stamp = get_clock()->now();
            synthetic_step_ns_ = hz_to_dt_ns_(current_rate_hz);
        }

        obs.header.frame_id = camera_frame_;
        const auto stamp_ns = stamp_to_ns(obs.header.stamp);
        obs.t_track_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(0, stamp_ns));
        obs.dt_ns = dt_ns_override_ != 0 ? dt_ns_override_ : synthetic_step_ns_;

        sensor_msgs::msg::Image rgb;
        rgb.header = obs.header;
        rgb.height = static_cast<std::uint32_t>(height_);
        rgb.width = static_cast<std::uint32_t>(width_);
        rgb.encoding = "rgb8";
        rgb.is_bigendian = false;
        rgb.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_ * 3);
        rgb.data.resize(static_cast<std::size_t>(width_ * height_ * 3), 127);

        sensor_msgs::msg::Image depth;
        depth.header = obs.header;
        depth.height = static_cast<std::uint32_t>(height_);
        depth.width = static_cast<std::uint32_t>(width_);
        depth.encoding = "16UC1";
        depth.is_bigendian = false;
        depth.step = static_cast<sensor_msgs::msg::Image::_step_type>(width_ * 2);
        depth.data.resize(static_cast<std::size_t>(width_ * height_ * 2), 0);

        obs.images.reserve(2);
        obs.images.push_back(std::move(rgb));
        obs.images.push_back(std::move(depth));

        pub_->publish(std::move(obs));
        ++published_count_;
        if (published_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), "published observations=%zu", published_count_);
        }
    }

    std::string topic_;
    std::string camera_frame_;
    std::string stamp_policy_;
    std::uint8_t mode_{};
    double rate_hz_{30.0};
    double burst_rate_hz_{0.0};
    double burst_start_s_{0.0};
    double burst_duration_s_{0.0};
    double runtime_s_{0.0};
    int width_{64};
    int height_{48};
    int qos_depth_{10};
    std::int64_t dt_ns_override_{0};
    std::int64_t synthetic_time_ns_{0};
    std::int64_t synthetic_step_ns_{33333333};
    std::size_t published_count_{0};

    std::chrono::steady_clock::time_point started_;
    std::chrono::steady_clock::time_point next_publish_;

    rclcpp::Publisher<orbslam3_ros2::msg::Observations>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObservationsPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
