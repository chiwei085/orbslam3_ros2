/**
 * @file system.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for system.
 */
#pragma once

// Wrapper header to avoid ambiguous direct includes of third-party System.h.
#include <System.h>
#include <MapPoint.h>

#include <memory>
#include <string>
#include <vector>

namespace orbslam3_ros2::orbslam3
{

inline std::unique_ptr<ORB_SLAM3::System> make_system(
    const std::string& voc_file, const std::string& settings_file,
    const bool use_viewer) {
    return std::make_unique<ORB_SLAM3::System>(
        voc_file, settings_file, ORB_SLAM3::System::RGBD, use_viewer);
}

inline std::unique_ptr<ORB_SLAM3::System> make_rgbd_system(
    const std::string& voc_file, const std::string& settings_file,
    const bool use_viewer) {
    return make_system(voc_file, settings_file, use_viewer);
}

inline Sophus::SE3f track_rgbd(ORB_SLAM3::System& system, const cv::Mat& rgb,
                               const cv::Mat& depth, const double timestamp) {
    return system.TrackRGBD(rgb, depth, timestamp);
}

inline void reset_active_map(ORB_SLAM3::System& system) {
    system.ResetActiveMap();
}

inline void shutdown_system(ORB_SLAM3::System& system) {
    system.Shutdown();
}

inline void save_trajectory_tum(ORB_SLAM3::System& system,
                                const std::string& filename) {
    system.SaveTrajectoryTUM(filename);
}

inline void save_trajectory_kitti(ORB_SLAM3::System& system,
                                  const std::string& filename) {
    system.SaveTrajectoryKITTI(filename);
}

inline void save_trajectory_euroc(ORB_SLAM3::System& system,
                                  const std::string& filename) {
    system.SaveTrajectoryEuRoC(filename);
}

inline void set_localization_mode(ORB_SLAM3::System& system,
                                  const bool localization_mode) {
    if (localization_mode) {
        system.ActivateLocalizationMode();
        return;
    }
    system.DeactivateLocalizationMode();
}

inline std::vector<Eigen::Vector3f> get_tracked_map_points_xyz(
    ORB_SLAM3::System& system, const std::size_t max_points) {
    const auto tracked_points = system.GetTrackedMapPoints();
    std::vector<Eigen::Vector3f> out;
    out.reserve(max_points);
    for (auto* point : tracked_points) {
        if (!point || point->isBad()) {
            continue;
        }
        out.push_back(point->GetWorldPos());
        if (out.size() >= max_points) {
            break;
        }
    }
    return out;
}

}  // namespace orbslam3_ros2::orbslam3
