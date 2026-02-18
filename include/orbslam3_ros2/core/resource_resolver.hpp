/**
 * @file resource_resolver.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for resource resolver.
 */
#pragma once

#include <string>
#include <vector>

namespace orbslam3_ros2::core
{

struct ResolvedPath
{
    std::string path{};
    std::vector<std::string> attempts{};
};

std::string format_attempts(const std::vector<std::string>& attempts);

ResolvedPath resolve_voc_file(const std::string& package_name,
                              const std::string& voc_file,
                              const std::string& voc_uri);

ResolvedPath resolve_settings_file(const std::string& package_name,
                                   const std::string& settings_file,
                                   const std::string& settings_uri,
                                   const std::string& default_relative_path);

}  // namespace orbslam3_ros2::core
