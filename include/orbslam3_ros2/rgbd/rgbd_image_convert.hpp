/**
 * @file rgbd_image_convert.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for rgbd image convert.
 */
#pragma once

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <optional>
#include <string>

namespace orbslam3_ros2::rgbd
{

using ImgPtr = sensor_msgs::msg::Image::ConstSharedPtr;

struct ImageConvertConfig
{
    double depth_scale{0.001};
    double depth_max_m{0.0};
    std::string depth_encoding_override{};
    bool rgb_clone{false};
    bool depth_clone{false};
};

struct TrackingMats
{
    cv::Mat rgb_bgr;
    cv::Mat depth_meters;
};

std::optional<cv::Mat> convert_rgb_to_bgr(const ImgPtr& rgb, bool rgb_clone,
                                          const rclcpp::Logger& logger,
                                          rclcpp::Clock& clock);

std::optional<cv::Mat> convert_depth_to_meters(const ImgPtr& depth,
                                               const ImageConvertConfig& config,
                                               const rclcpp::Logger& logger,
                                               rclcpp::Clock& clock);

void sanitize_depth_in_place(cv::Mat& depth_meters, double depth_max_m);

std::optional<TrackingMats> convert_canonical_observation_to_tracking_mats(
    const ImgPtr& rgb, const ImgPtr& depth, bool rgb_clone, bool depth_clone,
    const rclcpp::Logger& logger, rclcpp::Clock& clock);

}  // namespace orbslam3_ros2::rgbd
