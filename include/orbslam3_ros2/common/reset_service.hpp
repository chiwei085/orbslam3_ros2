#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace orbslam3_ros2::common
{

class ResetService
{
public:
  using ResetFn = std::function<void()>;

  ResetService(
    rclcpp::Node & node,
    std::string topic_prefix,
    std::mutex & reset_mutex,
    ResetFn reset_fn)
  : reset_mutex_(reset_mutex),
    reset_fn_(std::move(reset_fn))
  {
    auto service_name = topic_prefix + "/reset";
    service_ = node.create_service<std_srvs::srv::Empty>(
      service_name,
      [&](const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        (void)request_header;
        (void)request;
        (void)response;
        std::scoped_lock lock(reset_mutex_);
        reset_fn_();
      });
  }

private:
  std::mutex & reset_mutex_;
  ResetFn reset_fn_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
};

}  // namespace orbslam3_ros2::common

