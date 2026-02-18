/**
 * @file command_completion.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for command completion.
 */
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

namespace orbslam3_ros2::core
{

class CommandCompletion
{
public:
    struct Result
    {
        bool success{false};
        std::string message{};
    };

    enum class WaitStatus : std::uint8_t
    {
        kResult,
        kTimeout,
        kStoppedWithoutResult,
    };

    std::uint64_t allocate_request_id();
    void complete(std::uint64_t request_id, bool success,
                  const std::string& message);
    WaitStatus wait_and_take(std::uint64_t request_id,
                             std::chrono::milliseconds timeout,
                             const std::function<bool()>& stop_predicate,
                             Result* out_result);
    void clear();
    void notify_all();

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::unordered_map<std::uint64_t, Result> results_;
    std::atomic<std::uint64_t> next_request_id_{1};
};

}  // namespace orbslam3_ros2::core
