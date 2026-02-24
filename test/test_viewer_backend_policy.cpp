#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>
#include <string>

#include "orbslam3_ros2/common/viewer_backend_policy.hpp"

namespace
{

using orbslam3_ros2::common::apply_viewer_backend_env_policy;
using orbslam3_ros2::common::evaluate_viewer_backend_policy;
using orbslam3_ros2::common::get_env_string;
using orbslam3_ros2::common::ViewerBackendEnv;

class ScopedEnvVar final
{
public:
    explicit ScopedEnvVar(const char* name) : name_(name) {
        const char* current = std::getenv(name_);
        if (current) {
            old_value_ = std::string(current);
        }
    }

    ~ScopedEnvVar() {
        if (old_value_) {
            ::setenv(name_, old_value_->c_str(), 1);
        }
        else {
            ::unsetenv(name_);
        }
    }

private:
    const char* name_;
    std::optional<std::string> old_value_;
};

struct ScopedViewerEnv final
{
    ScopedEnvVar display{"DISPLAY"};
    ScopedEnvVar wayland{"WAYLAND_DISPLAY"};
    ScopedEnvVar xdg{"XDG_SESSION_TYPE"};
};

ViewerBackendEnv make_env(const std::string& display,
                          const std::string& wayland, const std::string& xdg) {
    return ViewerBackendEnv{display, wayland, xdg};
}

TEST(ViewerBackendPolicy, AutoOnWaylandWithDisplayForcesX11) {
    auto result = evaluate_viewer_backend_policy(
        true, "auto", make_env(":0", "wayland-0", ""));

    EXPECT_TRUE(result.effective_enable);
    EXPECT_TRUE(result.detected_wayland);
    EXPECT_TRUE(result.should_set_wayland_display);
    EXPECT_EQ(result.wayland_display_override, "0");
    EXPECT_EQ(result.reason, "forcing X11 (WAYLAND_DISPLAY=0)");
}

TEST(ViewerBackendPolicy, AutoOnWaylandWithoutDisplayDisablesViewer) {
    auto result = evaluate_viewer_backend_policy(
        true, "auto", make_env("", "wayland-0", "wayland"));

    EXPECT_FALSE(result.effective_enable);
    EXPECT_TRUE(result.detected_wayland);
    EXPECT_FALSE(result.should_set_wayland_display);
    EXPECT_EQ(result.reason,
              "Wayland detected but DISPLAY is empty -> disabling viewer");
}

TEST(ViewerBackendPolicy, AutoWithoutWaylandKeepsDefaultBackend) {
    auto result =
        evaluate_viewer_backend_policy(true, "auto", make_env(":0", "", "x11"));

    EXPECT_TRUE(result.effective_enable);
    EXPECT_FALSE(result.detected_wayland);
    EXPECT_FALSE(result.should_set_wayland_display);
    EXPECT_EQ(result.reason, "keeping default backend");
}

TEST(ViewerBackendPolicy, X11WithoutDisplayDisablesViewer) {
    auto result = evaluate_viewer_backend_policy(
        true, "x11", make_env("", "wayland-0", "wayland"));

    EXPECT_FALSE(result.effective_enable);
    EXPECT_FALSE(result.should_set_wayland_display);
    EXPECT_EQ(result.reason,
              "viewer_backend=x11 but DISPLAY is empty -> disabling viewer");
}

TEST(ViewerBackendPolicy, WaylandModeKeepsWaylandAndWarns) {
    auto result = evaluate_viewer_backend_policy(
        true, "wayland", make_env(":0", "wayland-0", "wayland"));

    EXPECT_TRUE(result.effective_enable);
    EXPECT_FALSE(result.should_set_wayland_display);
    EXPECT_TRUE(result.warn_wayland_teardown);
    EXPECT_EQ(result.reason, "allowing Wayland/default backend selection");
}

TEST(ViewerBackendPolicy, InvalidBackendFallsBackToAuto) {
    auto result = evaluate_viewer_backend_policy(
        true, "bad-value", make_env(":0", "wayland-0", ""));

    EXPECT_EQ(result.backend_name, "auto");
    EXPECT_FALSE(result.parse_warning.empty());
    EXPECT_TRUE(result.should_set_wayland_display);
}

TEST(ViewerBackendPolicy, ApplyPolicyMutatesWaylandDisplayOnlyWhenRequested) {
    ScopedViewerEnv scoped;
    ::setenv("WAYLAND_DISPLAY", "wayland-0", 1);

    auto result = evaluate_viewer_backend_policy(
        true, "auto", make_env(":0", "wayland-0", ""));
    ASSERT_TRUE(result.should_set_wayland_display);
    ASSERT_TRUE(apply_viewer_backend_env_policy(result));
    EXPECT_EQ(get_env_string("WAYLAND_DISPLAY"), "0");

    auto no_change = evaluate_viewer_backend_policy(
        true, "wayland", make_env(":0", "wayland-0", "wayland"));
    ASSERT_FALSE(no_change.should_set_wayland_display);
    ASSERT_TRUE(apply_viewer_backend_env_policy(no_change));
    EXPECT_EQ(get_env_string("WAYLAND_DISPLAY"), "0");
}

}  // namespace
