/**
 * @file rgbd_image_convert.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for rgbd image convert.
 */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <limits>

#include "orbslam3_ros2/rgbd/rgbd_config.hpp"
#include "orbslam3_ros2/rgbd/rgbd_image_convert.hpp"

namespace orbslam3_ros2::rgbd
{

std::optional<cv::Mat> convert_rgb_to_bgr(const ImgPtr& rgb,
                                          const bool rgb_clone,
                                          const rclcpp::Logger& logger,
                                          rclcpp::Clock& clock) {
    cv::Mat im_rgb_bgr;
    const std::string& rgb_encoding = rgb->encoding;
    const auto rgb_kind = config::classify_rgb_encoding(rgb_encoding);
    if (rgb_kind == config::RgbEncodingKind::kUnsupported) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Unsupported RGB encoding '%s' (supported: bgr8, rgb8, mono8)",
            rgb_encoding.c_str());
        return std::nullopt;
    }

    try {
        if (rgb_kind == config::RgbEncodingKind::kBgr8) {
            const auto cv_rgb =
                cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::BGR8);
            im_rgb_bgr = rgb_clone ? cv_rgb->image.clone() : cv_rgb->image;
        }
        else if (rgb_kind == config::RgbEncodingKind::kRgb8) {
            const auto cv_rgb =
                cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::RGB8);
            cv::cvtColor(cv_rgb->image, im_rgb_bgr, cv::COLOR_RGB2BGR);
        }
        else {
            const auto cv_rgb =
                cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_rgb->image, im_rgb_bgr, cv::COLOR_GRAY2BGR);
        }
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                             "RGB cv_bridge convert failed. encoding='%s' (%s)",
                             rgb_encoding.c_str(), e.what());
        return std::nullopt;
    }
    return im_rgb_bgr;
}

std::optional<cv::Mat> convert_depth_to_meters(const ImgPtr& depth,
                                               const ImageConvertConfig& config,
                                               const rclcpp::Logger& logger,
                                               rclcpp::Clock& clock) {
    cv::Mat im_depth_m;
    try {
        const std::string& msg_depth_encoding = depth->encoding;
        std::string depth_encoding = msg_depth_encoding;
        if (!config.depth_encoding_override.empty()) {
            if (!config::is_depth_encoding_alias_compatible(
                    msg_depth_encoding, config.depth_encoding_override)) {
                RCLCPP_WARN_THROTTLE(
                    logger, clock, 2000,
                    "Ignoring incompatible depth_encoding_override='%s' for "
                    "message encoding '%s'. Using message encoding. "
                    "Only 16UC1<->mono16 alias is allowed.",
                    config.depth_encoding_override.c_str(),
                    msg_depth_encoding.c_str());
                depth_encoding = msg_depth_encoding;
            }
            else {
                depth_encoding = config.depth_encoding_override;
            }
        }

        if (depth_encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            depth_encoding == sensor_msgs::image_encodings::MONO16) {
            const auto cv_d = cv_bridge::toCvShare(
                depth, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_d->image.convertTo(im_depth_m, CV_32FC1, config.depth_scale);
        }
        else if (depth_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            const auto cv_d = cv_bridge::toCvShare(
                depth, sensor_msgs::image_encodings::TYPE_32FC1);
            if (config.depth_scale == 1.0) {
                im_depth_m =
                    config.depth_clone ? cv_d->image.clone() : cv_d->image;
            }
            else {
                cv_d->image.convertTo(im_depth_m, CV_32FC1, config.depth_scale);
            }
        }
        else {
            RCLCPP_WARN_THROTTLE(
                logger, clock, 2000,
                "Unsupported depth encoding: msg='%s' effective='%s' "
                "(supported: 16UC1, mono16, 32FC1)",
                msg_depth_encoding.c_str(), depth_encoding.c_str());
            return std::nullopt;
        }
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Depth cv_bridge convert failed. encoding='%s' (%s)",
            depth->encoding.c_str(), e.what());
        return std::nullopt;
    }
    return im_depth_m;
}

void sanitize_depth_in_place(cv::Mat& depth_meters, const double depth_max_m) {
    const float max_depth = depth_max_m > 0.0
                                ? static_cast<float>(depth_max_m)
                                : std::numeric_limits<float>::max();
    for (int r = 0; r < depth_meters.rows; ++r) {
        float* row = depth_meters.ptr<float>(r);
        for (int c = 0; c < depth_meters.cols; ++c) {
            float& d = row[c];
            if (!(d > 0.0F) || d != d || d > max_depth) {
                d = 0.0F;
            }
        }
    }
}

std::optional<TrackingMats> convert_canonical_observation_to_tracking_mats(
    const ImgPtr& rgb, const ImgPtr& depth, const bool rgb_clone,
    const bool depth_clone, const rclcpp::Logger& logger,
    rclcpp::Clock& clock) {
    try {
        const auto cv_rgb =
            cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::BGR8);
        const auto cv_depth = cv_bridge::toCvShare(
            depth, sensor_msgs::image_encodings::TYPE_32FC1);
        TrackingMats mats;
        mats.rgb_bgr = rgb_clone ? cv_rgb->image.clone() : cv_rgb->image;
        mats.depth_meters =
            depth_clone ? cv_depth->image.clone() : cv_depth->image;
        return mats;
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Canonical observation convert failed. Expected rgb=bgr8 "
            "depth=32FC1 (%s)",
            e.what());
        return std::nullopt;
    }
}

}  // namespace orbslam3_ros2::rgbd
