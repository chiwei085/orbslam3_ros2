#pragma once

/// @file viewer_backend_policy.hpp
/// @brief Viewer backend selection policy for avoiding Pangolin Wayland teardown crashes.
///
/// This header keeps the decision logic separate from ROS node startup code so the
/// policy can be tested without creating a ROS graph, ORB-SLAM3 system, or Pangolin
/// window/context.

#include <cctype>
#include <cstdlib>
#include <string>
#include <utility>

namespace orbslam3_ros2::common {

/// @brief Requested viewer backend mode from user-facing parameter `viewer_backend`.
enum class ViewerBackendMode {
    /// Let policy detect Wayland and choose a safer runtime path.
    Auto,
    /// Force X11/XWayland by masking Wayland for this process.
    X11,
    /// Allow Wayland backend selection and accept teardown crash risk.
    Wayland,
};

/// @brief Snapshot of process environment variables relevant to viewer backend selection.
struct ViewerBackendEnv {
    /// Raw `DISPLAY` value. Empty means X11/XWayland is not available to this process.
    std::string display;
    /// Raw `WAYLAND_DISPLAY` value. `"0"` is treated as intentionally disabled.
    std::string wayland_display;
    /// Raw `XDG_SESSION_TYPE` value (e.g. `wayland`, `x11`).
    std::string xdg_session_type;
};

/// @brief Result of viewer backend policy evaluation.
///
/// The result is split into:
/// - normalized input (`mode`, `backend_name`)
/// - environment probes (`has_display`, `detected_wayland`, ...)
/// - actions (`effective_enable`, `should_set_wayland_display`)
/// - operator-facing diagnostics (`reason`, `parse_warning`)
struct ViewerPolicyResult {
    /// Original `enable_pangolin` request from caller.
    bool requested_enable{false};
    /// Effective viewer enable after policy fallback/disable rules.
    bool effective_enable{false};
    /// Normalized backend mode derived from user input.
    ViewerBackendMode mode{ViewerBackendMode::Auto};
    /// Lowercase backend string (`auto`, `x11`, `wayland`) after normalization.
    std::string backend_name{"auto"};
    /// `DISPLAY` is present and non-empty.
    bool has_display{false};
    /// `WAYLAND_DISPLAY` is present, non-empty, and not `"0"`.
    bool wayland_display_active{false};
    /// `XDG_SESSION_TYPE` indicates Wayland.
    bool xdg_wayland{false};
    /// Wayland detected by either `WAYLAND_DISPLAY` or `XDG_SESSION_TYPE`.
    bool detected_wayland{false};
    /// Caller should mutate process env before Pangolin init.
    bool should_set_wayland_display{false};
    /// Replacement value for `WAYLAND_DISPLAY` (currently `"0"`).
    std::string wayland_display_override;
    /// Caller should emit warning when explicit Wayland backend is requested.
    bool warn_wayland_teardown{false};
    /// Human-readable reason for logs / debugging.
    std::string reason;
    /// Warning for invalid backend string that was normalized to `auto`.
    std::string parse_warning;
};

/// @brief Read one environment variable as a string.
/// @param name Environment variable name to query.
/// @return Current value, or empty string if the variable is unset.
inline auto get_env_string(const char* name) -> std::string {
    const char* value = std::getenv(name);
    return value ? std::string(value) : std::string{};
}

/// @brief Capture viewer-related environment variables from the current process.
/// @return Snapshot of `DISPLAY`, `WAYLAND_DISPLAY`, and `XDG_SESSION_TYPE`.
/// @note This is side-effect free and is intended to feed `evaluate_viewer_backend_policy()`.
inline auto get_viewer_backend_env_from_process() -> ViewerBackendEnv {
    return {
        get_env_string("DISPLAY"),
        get_env_string("WAYLAND_DISPLAY"),
        get_env_string("XDG_SESSION_TYPE"),
    };
}

/// @brief ASCII lowercase helper used for tolerant parameter parsing.
/// @param value Input string.
/// @return Lowercased copy using ASCII rules.
/// @note Policy parameters are ASCII tokens (`auto|x11|wayland`).
inline auto to_lower_ascii(std::string value) -> std::string {
    // Keep parsing locale-independent. Parameter values are ASCII tokens.
    for (char& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

/// @brief Parse user-facing backend string into normalized mode + canonical name.
/// @param value Raw parameter value from caller.
/// @return Pair of parsed mode and canonical lowercase backend string.
/// @note Invalid values intentionally fall back to `auto`; caller reports warning text.
inline auto parse_viewer_backend_mode(const std::string& value)
    -> std::pair<ViewerBackendMode, std::string> {
    auto normalized = to_lower_ascii(value);
    if (normalized == "x11") {
        return {ViewerBackendMode::X11, normalized};
    }
    if (normalized == "wayland") {
        return {ViewerBackendMode::Wayland, normalized};
    }
    return {ViewerBackendMode::Auto, normalized == "auto" ? normalized : "auto"};
}

/// @brief Compute viewer backend policy from caller intent and environment snapshot.
/// @param enable_pangolin Requested viewer enable flag from wrapper parameter.
/// @param viewer_backend Raw backend selector string (`auto|x11|wayland`, case-insensitive).
/// @param env Environment snapshot used for backend selection and fallback decisions.
/// @return A policy result describing effective viewer enable, diagnostics, and any
///         process-local env mutation the caller should apply before Pangolin init.
/// @note This function does not mutate process environment. The wrapper applies the
///       requested env change after logging and error handling.
/// @warning `viewer_backend=wayland` keeps Wayland backend selection enabled. In some
///          Wayland/WSLg environments, Pangolin teardown may crash on shutdown.
/// @details
/// Policy summary (effective backend is encoded by `effective_enable` +
/// `should_set_wayland_display`/`wayland_display_override`):
///
/// | `enable_pangolin` | `viewer_backend` | Environment signal | Output (`effective_enable`, env mutation) |
/// | --- | --- | --- | --- |
/// | `false` | any | any | disabled, no env mutation |
/// | `true` | `auto` | no Wayland detected | enabled, no env mutation (keep default backend selection) |
/// | `true` | `auto` | Wayland detected + `DISPLAY` present | enabled, set `WAYLAND_DISPLAY=0` (force X11/XWayland) |
/// | `true` | `auto` | Wayland detected + no `DISPLAY` | disabled, no env mutation |
/// | `true` | `x11` | `DISPLAY` present | enabled, set `WAYLAND_DISPLAY=0` (force X11/XWayland) |
/// | `true` | `x11` | no `DISPLAY` | disabled, no env mutation |
/// | `true` | `wayland` | any | enabled, no env mutation (caller warns) |
///
/// Design choices:
/// - `WAYLAND_DISPLAY=0` is used instead of unsetting it because this is the path
///   observed to reliably avoid Pangolin's Wayland backend in affected environments.
///   The mutation is process-local only (`setenv()` in this process).
/// - `auto` disables viewer when Wayland is detected but `DISPLAY` is missing to
///   avoid a hard-fail trying to force X11 on systems without XWayland export.
///
/// @code
/// using orbslam3_ros2::common::ViewerBackendEnv;
/// using orbslam3_ros2::common::evaluate_viewer_backend_policy;
///
/// // Wayland session with XWayland available -> force X11 by masking Wayland.
/// auto a = evaluate_viewer_backend_policy(
///     true, "auto", ViewerBackendEnv{":0", "wayland-0", "wayland"});
/// // a.effective_enable == true
/// // a.should_set_wayland_display == true
/// // a.wayland_display_override == "0"   // caller applies setenv("WAYLAND_DISPLAY","0",1)
///
/// // Wayland session without DISPLAY -> disable viewer in auto mode.
/// auto b = evaluate_viewer_backend_policy(
///     true, "auto", ViewerBackendEnv{"", "wayland-0", "wayland"});
/// // b.effective_enable == false
/// // b.should_set_wayland_display == false
/// @endcode
inline auto evaluate_viewer_backend_policy(
    bool enable_pangolin, const std::string& viewer_backend,
    const ViewerBackendEnv& env) -> ViewerPolicyResult {
    ViewerPolicyResult result;
    result.requested_enable = enable_pangolin;
    result.effective_enable = enable_pangolin;
    result.has_display = !env.display.empty();
    // "0" means caller already masked Wayland. Do not treat it as active Wayland.
    result.wayland_display_active =
        !env.wayland_display.empty() && env.wayland_display != "0";
    // XDG_SESSION_TYPE is only a hint. WAYLAND_DISPLAY remains the stronger signal.
    result.xdg_wayland = to_lower_ascii(env.xdg_session_type) == "wayland";
    result.detected_wayland = result.wayland_display_active || result.xdg_wayland;

    auto [mode, backend_name] = parse_viewer_backend_mode(viewer_backend);
    result.mode = mode;
    result.backend_name = std::move(backend_name);

    if (to_lower_ascii(viewer_backend) != result.backend_name) {
        // Keep parser permissive. Wrapper can warn without changing control flow.
        result.parse_warning =
            "Unknown viewer_backend='" + viewer_backend +
            "', falling back to 'auto' (expected auto|x11|wayland)";
    }

    if (!enable_pangolin) {
        // Preserve explicit user disable. Policy never re-enables the viewer.
        result.reason = "viewer disabled (enable_pangolin=false)";
        return result;
    }

    switch (result.mode) {
    case ViewerBackendMode::Auto:
        if (!result.detected_wayland) {
            result.reason = "keeping default backend";
        } else if (!result.has_display) {
            // Avoid forcing X11 when no X11/XWayland endpoint is exposed.
            result.effective_enable = false;
            result.reason = "Wayland detected but DISPLAY is empty -> disabling viewer";
        } else {
            // Process-local mask. Pangolin backend selection happens later in wrapper.
            result.should_set_wayland_display = true;
            result.wayland_display_override = "0";
            result.reason = "forcing X11 (WAYLAND_DISPLAY=0)";
        }
        break;
    case ViewerBackendMode::X11:
        if (!result.has_display) {
            // Explicit x11 request but no X display. Disable instead of hard-failing later.
            result.effective_enable = false;
            result.reason = "viewer_backend=x11 but DISPLAY is empty -> disabling viewer";
        } else {
            result.should_set_wayland_display = true;
            result.wayland_display_override = "0";
            result.reason = "forcing X11 (WAYLAND_DISPLAY=0)";
        }
        break;
    case ViewerBackendMode::Wayland:
        result.warn_wayland_teardown = true;
        result.reason = "allowing Wayland/default backend selection";
        break;
    }

    return result;
}

/// @brief Apply the environment mutation requested by a previously computed policy.
/// @param result Policy result from `evaluate_viewer_backend_policy()`.
/// @return `true` on success, or `false` if `setenv()` failed.
/// @note This only mutates the current process environment and must run before
///       Pangolin creates any window/context.
/// @warning Calling this after Pangolin initialization is too late to affect backend
///          selection and will not prevent Wayland teardown issues.
inline auto apply_viewer_backend_env_policy(const ViewerPolicyResult& result)
    -> bool {
    if (!result.should_set_wayland_display) {
        // No mutation requested. Keep caller path simple.
        return true;
    }
    // Override only in this process. Parent shell and system session are unchanged.
    return ::setenv("WAYLAND_DISPLAY", result.wayland_display_override.c_str(), 1) ==
           0;
}

}  // namespace orbslam3_ros2::common
