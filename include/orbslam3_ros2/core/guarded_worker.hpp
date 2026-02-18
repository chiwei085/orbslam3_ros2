/**
 * @file guarded_worker.hpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Declaration for guarded worker.
 */
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace orbslam3_ros2::core
{

class GuardedWorker
{
public:
    using ExceptionCallback = std::function<void(const std::string&)>;

    GuardedWorker() = default;
    ~GuardedWorker();

    GuardedWorker(const GuardedWorker&) = delete;
    GuardedWorker& operator=(const GuardedWorker&) = delete;

    bool start(std::function<void()> loop_fn,
               ExceptionCallback on_exception = {});
    void join();
    bool joinable() const noexcept;
    bool running() const noexcept;

    void clear_exception();
    std::string last_exception() const;

private:
    void set_exception_(const std::string& message);

    std::thread worker_;
    std::atomic<bool> running_{false};
    mutable std::mutex exception_mutex_;
    std::string last_exception_;
};

}  // namespace orbslam3_ros2::core
