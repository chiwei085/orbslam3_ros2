/**
 * @file slam_command_queue.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for slam command queue.
 */
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace orbslam3_ros2::core
{

enum class SlamCommandType : std::uint8_t
{
    kReset,
    kShutdown,
    kSaveTrajectory,
    kSaveMap,
    kLoadMap,
    kToggleLocalizationMode,
};

struct SlamCommand
{
    SlamCommandType type{SlamCommandType::kReset};
    std::uint64_t request_id{0};
    std::string format{};
    std::string path{};
    bool localization_mode{false};
};

class SlamCommandQueue
{
public:
    void push(const SlamCommand& command) {
        {
            std::lock_guard lock(mutex_);
            queue_.push_back(command);
        }
        cv_.notify_one();
    }

    std::deque<SlamCommand> drain() {
        std::deque<SlamCommand> local;
        {
            std::lock_guard lock(mutex_);
            local.swap(queue_);
        }
        return local;
    }

    bool empty() const {
        std::lock_guard lock(mutex_);
        return queue_.empty();
    }

    void notify_all() { cv_.notify_all(); }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<SlamCommand> queue_;
};

}  // namespace orbslam3_ros2::core
