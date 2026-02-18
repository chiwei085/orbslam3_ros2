/**
 * @file resource_resolver.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for resource resolver.
 */
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orbslam3_ros2/core/resource_resolver.hpp"

namespace orbslam3_ros2::core
{
namespace
{
bool starts_with(const std::string& value, const std::string& prefix) {
    return value.rfind(prefix, 0) == 0;
}

std::optional<std::pair<std::string, std::string>> parse_package_uri(
    const std::string& uri) {
    constexpr const char* kPrefix = "package://";
    if (!starts_with(uri, kPrefix)) {
        return std::nullopt;
    }
    const std::string tail =
        uri.substr(std::char_traits<char>::length(kPrefix));
    const auto slash_pos = tail.find('/');
    if (slash_pos == std::string::npos || slash_pos == 0 ||
        slash_pos + 1 >= tail.size()) {
        return std::nullopt;
    }
    return std::make_pair(tail.substr(0, slash_pos),
                          tail.substr(slash_pos + 1));
}

std::string resolve_existing_path(const std::string& value,
                                  std::vector<std::string>& attempts) {
    if (value.empty()) {
        return "";
    }

    if (starts_with(value, "package://")) {
        const auto parsed = parse_package_uri(value);
        if (!parsed) {
            attempts.push_back("INVALID_URI:" + value);
            return "";
        }
        try {
            const auto share_dir =
                ament_index_cpp::get_package_share_directory(parsed->first);
            const auto candidate =
                (std::filesystem::path(share_dir) / parsed->second).string();
            attempts.push_back(candidate);
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
        catch (const std::exception& e) {
            attempts.push_back("PACKAGE_NOT_FOUND:" + parsed->first + " (" +
                               e.what() + ")");
        }
        return "";
    }

    attempts.push_back(value);
    if (std::filesystem::exists(value)) {
        return value;
    }
    return "";
}

std::string resolve_from_share(const std::string& package_name,
                               const std::string& relative_path,
                               std::vector<std::string>& attempts) {
    try {
        const auto share_dir =
            ament_index_cpp::get_package_share_directory(package_name);
        const auto candidate =
            (std::filesystem::path(share_dir) / relative_path).string();
        attempts.push_back(candidate);
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    catch (const std::exception& e) {
        attempts.push_back("PACKAGE_NOT_FOUND:" + package_name + " (" +
                           e.what() + ")");
    }
    return "";
}
}  // namespace

std::string format_attempts(const std::vector<std::string>& attempts) {
    if (attempts.empty()) {
        return "<none>";
    }
    std::string out;
    for (std::size_t i = 0; i < attempts.size(); ++i) {
        out += attempts[i];
        if (i + 1 < attempts.size()) {
            out += " | ";
        }
    }
    return out;
}

ResolvedPath resolve_voc_file(const std::string& package_name,
                              const std::string& voc_file,
                              const std::string& voc_uri) {
    ResolvedPath result;
    if (!voc_uri.empty()) {
        result.path = resolve_existing_path(voc_uri, result.attempts);
        return result;
    }
    if (!voc_file.empty()) {
        result.path = resolve_existing_path(voc_file, result.attempts);
        return result;
    }
    result.path =
        resolve_from_share(package_name, "vocab/ORBvoc.txt", result.attempts);
    return result;
}

ResolvedPath resolve_settings_file(const std::string& package_name,
                                   const std::string& settings_file,
                                   const std::string& settings_uri,
                                   const std::string& default_relative_path) {
    ResolvedPath result;
    if (!settings_uri.empty()) {
        result.path = resolve_existing_path(settings_uri, result.attempts);
        return result;
    }
    if (!settings_file.empty()) {
        result.path = resolve_existing_path(settings_file, result.attempts);
        return result;
    }
    result.path = resolve_from_share(package_name, default_relative_path,
                                     result.attempts);
    return result;
}

}  // namespace orbslam3_ros2::core
