/**
 * @file command_completion.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Implementation for command completion.
 */
#include <utility>

#include "orbslam3_ros2/core/command_completion.hpp"

namespace orbslam3_ros2::core
{

std::uint64_t CommandCompletion::allocate_request_id() {
    return next_request_id_.fetch_add(1, std::memory_order_relaxed);
}

void CommandCompletion::complete(const std::uint64_t request_id,
                                 const bool success,
                                 const std::string& message) {
    if (request_id == 0) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        results_[request_id] = Result{success, message};
    }
    cv_.notify_all();
}

CommandCompletion::WaitStatus CommandCompletion::wait_and_take(
    const std::uint64_t request_id, const std::chrono::milliseconds timeout,
    const std::function<bool()>& stop_predicate, Result* out_result) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = cv_.wait_for(lock, timeout, [&]() {
        return stop_predicate() || results_.find(request_id) != results_.end();
    });
    if (!ready) {
        return WaitStatus::kTimeout;
    }

    const auto it = results_.find(request_id);
    if (it == results_.end()) {
        return WaitStatus::kStoppedWithoutResult;
    }

    if (out_result) {
        *out_result = std::move(it->second);
    }
    results_.erase(it);
    return WaitStatus::kResult;
}

void CommandCompletion::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    results_.clear();
}

void CommandCompletion::notify_all() {
    cv_.notify_all();
}

}  // namespace orbslam3_ros2::core
