/**
 * @file guarded_worker.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for guarded worker.
 */
#include <exception>
#include <utility>

#include "orbslam3_ros2/core/guarded_worker.hpp"

namespace orbslam3_ros2::core
{

GuardedWorker::~GuardedWorker() {
    join();
}

bool GuardedWorker::start(std::function<void()> loop_fn,
                          ExceptionCallback on_exception) {
    if (worker_.joinable() || running_.load(std::memory_order_acquire)) {
        return false;
    }
    clear_exception();
    worker_ = std::thread([this, loop_fn = std::move(loop_fn),
                           on_exception = std::move(on_exception)]() mutable {
        running_.store(true, std::memory_order_release);
        try {
            loop_fn();
        }
        catch (const std::exception& e) {
            const std::string message =
                std::string("std::exception: ") + e.what();
            set_exception_(message);
            if (on_exception) {
                on_exception(message);
            }
        }
        catch (...) {
            constexpr const char* kMessage = "unknown exception";
            set_exception_(kMessage);
            if (on_exception) {
                on_exception(kMessage);
            }
        }
        running_.store(false, std::memory_order_release);
    });
    return true;
}

void GuardedWorker::join() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool GuardedWorker::joinable() const noexcept {
    return worker_.joinable();
}

bool GuardedWorker::running() const noexcept {
    return running_.load(std::memory_order_relaxed);
}

void GuardedWorker::clear_exception() {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    last_exception_.clear();
}

std::string GuardedWorker::last_exception() const {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    return last_exception_;
}

void GuardedWorker::set_exception_(const std::string& message) {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    last_exception_ = message;
}

}  // namespace orbslam3_ros2::core
