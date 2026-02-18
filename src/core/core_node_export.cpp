/**
 * @file core_node_export.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for core node export.
 */
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <filesystem>
#include <fstream>
#include <utility>

#include "orbslam3_ros2/core/orbslam3_core_node.hpp"

namespace Orb = orbslam3_ros2::orbslam3;

namespace orbslam3_ros2::core
{

sensor_msgs::msg::PointCloud2
Orbslam3CoreNode::build_tracked_points_cloud_from_cache_(
    const rclcpp::Time& stamp, const std::size_t max_points) const {
    sensor_msgs::msg::PointCloud2 cloud;
    const builtin_interfaces::msg::Time stamp_msg = stamp;
    cloud.header.stamp = stamp_msg;
    cloud.header.frame_id = map_frame_;
    cloud.height = 1;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    std::vector<Eigen::Vector3f> tracked_points_copy;
    {
        std::lock_guard lock(tracked_points_cache_mutex_);
        tracked_points_copy = tracked_points_cache_;
    }
    if (tracked_points_copy.empty()) {
        modifier.resize(0);
        return cloud;
    }

    const auto count = std::min(max_points, tracked_points_copy.size());
    modifier.resize(count);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (std::size_t i = 0; i < count; ++i) {
        const auto& p = tracked_points_copy[i];
        *iter_x = p.x();
        *iter_y = p.y();
        *iter_z = p.z();
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    return cloud;
}

void Orbslam3CoreNode::update_tracked_points_cache_(
    const std::size_t max_points) {
    if (!system_) {
        std::lock_guard lock(tracked_points_cache_mutex_);
        tracked_points_cache_.clear();
        return;
    }
    auto tracked_points = Orb::get_tracked_map_points_xyz(*system_, max_points);
    {
        std::lock_guard lock(tracked_points_cache_mutex_);
        tracked_points_cache_.swap(tracked_points);
    }
}

bool Orbslam3CoreNode::write_cloud_as_xyz_pcd_(
    const sensor_msgs::msg::PointCloud2& cloud, const std::string& output_path,
    std::string* error_message) const {
    namespace fs = std::filesystem;
    fs::path path(output_path);
    if (path.empty()) {
        if (error_message) {
            *error_message = "output_path is empty.";
        }
        return false;
    }
    if (path.extension() != ".pcd") {
        path += ".pcd";
    }
    if (path.is_relative()) {
        path = fs::current_path() / path;
    }
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path());
    }

    const auto point_count = static_cast<std::size_t>(cloud.width);
    std::ofstream out(path.string());
    if (!out.is_open()) {
        if (error_message) {
            *error_message = "failed to open output file: " + path.string();
        }
        return false;
    }

    out << "# .PCD v0.7\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z\n";
    out << "SIZE 4 4 4\n";
    out << "TYPE F F F\n";
    out << "COUNT 1 1 1\n";
    out << "WIDTH " << point_count << "\n";
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << "POINTS " << point_count << "\n";
    out << "DATA ascii\n";

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (std::size_t i = 0; i < point_count;
         ++i, ++iter_x, ++iter_y, ++iter_z) {
        out << *iter_x << " " << *iter_y << " " << *iter_z << "\n";
    }
    return true;
}

rclcpp_action::GoalResponse Orbslam3CoreNode::handle_export_pointcloud_goal_(
    const std::shared_ptr<const ExportPointCloud::Goal>& goal) {
    if (!configured_.load(std::memory_order_acquire)) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->scope != "tracking" && goal->scope != "map") {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Orbslam3CoreNode::handle_export_pointcloud_cancel_(
    const std::shared_ptr<GoalHandleExportPointCloud>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Orbslam3CoreNode::handle_export_pointcloud_accepted_(
    const std::shared_ptr<GoalHandleExportPointCloud> goal_handle) {
    std::thread worker([this, goal_handle]() {
        auto feedback = std::make_shared<ExportPointCloud::Feedback>();
        feedback->progress = 0.1f;
        goal_handle->publish_feedback(feedback);

        auto result = std::make_shared<ExportPointCloud::Result>();
        const auto& goal = goal_handle->get_goal();
        if (goal->scope == "map") {
            result->success = false;
            result->message =
                "scope=map is not available via current ORB-SLAM3 public API; "
                "use save_map/load_map for reproducible map state.";
            goal_handle->abort(result);
            return;
        }

        const auto now = get_clock()->now();
        const auto max_points = std::max<std::size_t>(
            std::size_t{1}, goal->max_points > 0
                                ? static_cast<std::size_t>(goal->max_points)
                                : pointcloud_max_points_);
        auto cloud = build_tracked_points_cloud_from_cache_(now, max_points);
        feedback->progress = 0.6f;
        goal_handle->publish_feedback(feedback);

        if (!goal->output_path.empty()) {
            std::string error;
            if (!write_cloud_as_xyz_pcd_(cloud, goal->output_path, &error)) {
                result->success = false;
                result->message = error;
                goal_handle->abort(result);
                return;
            }
            result->output_path = goal->output_path;
        }

        if (goal->publish_snapshot && snapshot_pointcloud_pub_) {
            snapshot_pointcloud_pub_->publish(cloud);
        }
        if (goal->include_cloud_in_result) {
            result->cloud = cloud;
        }

        feedback->progress = 1.0f;
        goal_handle->publish_feedback(feedback);
        result->success = true;
        result->message = "tracking pointcloud snapshot exported.";
        goal_handle->succeed(result);
    });

    {
        std::lock_guard lock(export_threads_mutex_);
        export_threads_.push_back(std::move(worker));
    }
}

void Orbslam3CoreNode::join_export_threads_() {
    std::vector<std::thread> local;
    {
        std::lock_guard lock(export_threads_mutex_);
        local.swap(export_threads_);
    }
    for (auto& t : local) {
        if (t.joinable()) {
            t.join();
        }
    }
}

}  // namespace orbslam3_ros2::core
