/**
 * @file core_node_main.cpp
 * @author Yeh Chi Wei <yehchiwei.work@gmail.com>
 * @brief Lifecycle entrypoints and process main for orbslam3_core_node.
 *
 * Contract:
 * - `autostart_lifecycle=true` performs configure+activate before spinning.
 * - `autostart_lifecycle=false` keeps node silent until external transitions.
 *
 * Failure Mode:
 * - Process exits with code 1 when configure or activate transition fails.
 */
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "orbslam3_ros2/core/orbslam3_core_node.hpp"

namespace orbslam3_ros2::core
{

Orbslam3CoreNode::Orbslam3CoreNode()
    : LifecycleNode("orbslam3_core_node"), diagnostics_updater_(this) {
    diagnostics_updater_.setHardwareID("orbslam3_core");
    diagnostics_updater_.add("tracking_pipeline", this,
                             &Orbslam3CoreNode::produce_diagnostics_);
}

Orbslam3CoreNode::~Orbslam3CoreNode() {
    stop_worker_();
    join_export_threads_();
    if (system_ && !system_shutdown_called_.load(std::memory_order_acquire)) {
        system_->Shutdown();
        system_shutdown_called_.store(true, std::memory_order_release);
    }
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::on_configure(
    const rclcpp_lifecycle::State&) {
    return configure_resources_();
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::on_activate(
    const rclcpp_lifecycle::State&) {
    if (!configured_.load(std::memory_order_acquire)) {
        return CallbackReturn::ERROR;
    }
    active_.store(true, std::memory_order_release);
    if (!start_worker_()) {
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::on_deactivate(
    const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_release);
    stop_worker_();
    return CallbackReturn::SUCCESS;
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::on_cleanup(
    const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_release);
    cleanup_resources_();
    return CallbackReturn::SUCCESS;
}

Orbslam3CoreNode::CallbackReturn Orbslam3CoreNode::on_shutdown(
    const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_release);
    cleanup_resources_();
    return CallbackReturn::SUCCESS;
}

}  // namespace orbslam3_ros2::core

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    auto node = std::make_shared<orbslam3_ros2::core::Orbslam3CoreNode>();
    // autostart_lifecycle:
    // - true: convenience mode for standalone runs and local smoke cycles.
    // - false: external lifecycle manager owns timing of topic/service
    // readiness.
    const auto autostart_lifecycle =
        node->declare_parameter("autostart_lifecycle", true);

    if (autostart_lifecycle) {
        const auto state_after_config = node->trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        if (state_after_config.id() !=
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            RCLCPP_FATAL(node->get_logger(),
                         "Lifecycle configure transition failed.");
            rclcpp::shutdown();
            return 1;
        }
        const auto state_after_activate = node->trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        if (state_after_activate.id() !=
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_FATAL(node->get_logger(),
                         "Lifecycle activate transition failed.");
            rclcpp::shutdown();
            return 1;
        }
    }
    else {
        RCLCPP_INFO(node->get_logger(),
                    "autostart_lifecycle=false; waiting for external lifecycle"
                    " manager to configure/activate.");
    }
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
