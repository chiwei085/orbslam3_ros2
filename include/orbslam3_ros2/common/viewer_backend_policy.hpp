#pragma once

/**
 * @file viewer_backend_policy.hpp
 * @brief Viewer backend selection policy for avoiding Pangolin Wayland teardown crashes
 *
 * This policy exists to keep backend selection logic out of ROS node startup code.
 * It can be tested without creating a ROS graph, ORB-SLAM3 system, or Pangolin window.
 *
 * The policy may request a process-local environment override (`WAYLAND_DISPLAY=0`)
 * so Pangolin does not select its Wayland backend in affected environments.
 * The override is applied only in the current process via `setenv()`.
 */

#include <cctype>
#include <cstdlib>
#include <string>
#include <utility>

namespace orbslam3_ros2::common {

/**
 * @brief Requested viewer backend mode from user-facing parameter `viewer_backend`
 */
enum class ViewerBackendMode {
    Auto,    // Detect Wayland and choose a safer path.
    X11,     // Force X11/XWayland by masking Wayland in this process.
    Wayland, // Allow Wayland backend selection and accept teardown risk.
};

/**
 * @brief Snapshot of process environment variables used by backend policy
 */
struct ViewerBackendEnv {
    std::string display;          // Raw DISPLAY value.
    std::string wayland_display;  // Raw WAYLAND_DISPLAY value.
    std::string xdg_session_type; // Raw XDG_SESSION_TYPE value.
};

/**
 * @brief Result of backend policy evaluation
 *
 * The wrapper logs this result and applies any requested environment mutation
 * before Pangolin creates a window/context.
 */
struct ViewerPolicyResult {
    bool requested_enable{false};           // Original enable_pangolin input.
    bool effective_enable{false};           // Effective viewer enable after policy.
    ViewerBackendMode mode{ViewerBackendMode::Auto}; // Normalized backend mode.
    std::string backend_name{"auto"};       // Canonical backend string.
    bool has_display{false};                // DISPLAY is present and non-empty.
    bool wayland_display_active{false};     // WAYLAND_DISPLAY is active and not "0".
    bool xdg_wayland{false};                // XDG_SESSION_TYPE indicates wayland.
    bool detected_wayland{false};           // Any Wayland hint is present.
    bool should_set_wayland_display{false}; // Caller should mutate WAYLAND_DISPLAY.
    std::string wayland_display_override;   // Value to write into WAYLAND_DISPLAY.
    bool warn_wayland_teardown{false};      // Caller should warn about teardown risk.
    std::string reason;                     // Human-readable policy decision.
    std::string parse_warning;              // Warning for invalid backend input.
};

/**
 * @brief Read one environment variable as a string
 * @param name Environment variable name to query
 * @return Current value, or empty string if the variable is unset
 */
inline auto get_env_string(const char* name) -> std::string {
    const char* value = std::getenv(name);
    return value ? std::string(value) : std::string{};
}

/**
 * @brief Capture viewer-related environment variables from the current process
 * @return Snapshot of `DISPLAY`, `WAYLAND_DISPLAY`, and `XDG_SESSION_TYPE`
 * @note This function is side-effect free. It exists so tests can bypass process
 *       environment reads and pass a synthetic `ViewerBackendEnv`.
 */
inline auto get_viewer_backend_env_from_process() -> ViewerBackendEnv {
    return {
        get_env_string("DISPLAY"),
        get_env_string("WAYLAND_DISPLAY"),
        get_env_string("XDG_SESSION_TYPE"),
    };
}

/**
 * @brief ASCII lowercase helper for tolerant backend string parsing
 * @param value Input string
 * @return Lowercased copy using ASCII rules
 * @note Policy tokens are ASCII (`auto|x11|wayland`). Locale-dependent behavior
 *       is not needed here.
 */
inline auto to_lower_ascii(std::string value) -> std::string {
    // Keep parsing locale-independent. Parameter values are ASCII tokens.
    for (char& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

/**
 * @brief Parse user backend string into normalized mode and canonical name
 * @param value Raw parameter value from caller
 * @return Pair of parsed mode and canonical lowercase backend string
 * @note Invalid values intentionally fall back to `auto`. The caller can emit a
 *       warning based on `ViewerPolicyResult::parse_warning`.
 */
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

/**
 * @brief Compute viewer backend policy from caller intent and environment snapshot
 * @param enable_pangolin Requested viewer enable flag from wrapper parameter
 * @param viewer_backend Raw backend selector string (`auto|x11|wayland`, case-insensitive)
 * @param env Environment snapshot used for backend selection and fallback decisions
 * @return Policy result with effective viewer enable, normalized backend, warning flags,
 *         and any process-local env mutation to apply before Pangolin initialization
 * @note This function does not mutate the process environment. The wrapper applies
 *       the requested env change after logging and error handling.
 * @note `WAYLAND_DISPLAY=0` is used instead of unsetting `WAYLAND_DISPLAY` because
 *       it was observed to be the more reliable way to block Pangolin Wayland backend
 *       selection in affected environments. The mutation remains process-local.
 * @warning `viewer_backend=wayland` allows Wayland backend selection. In some
 *          Wayland/WSLg environments, Pangolin teardown may crash on shutdown.
 * @details
 * Rules (effective backend is encoded by `effective_enable` plus env mutation):
 *
 * | `enable_pangolin` | `viewer_backend` | Env signal | `effective_enable` | Env mutation |
 * | --- | --- | --- | --- | --- |
 * | `false` | any | any | `false` | none |
 * | `true` | `auto` | no Wayland detected | `true` | none |
 * | `true` | `auto` | Wayland detected + `DISPLAY` present | `true` | `WAYLAND_DISPLAY=0` |
 * | `true` | `auto` | Wayland detected + no `DISPLAY` | `false` | none |
 * | `true` | `x11` | `DISPLAY` present | `true` | `WAYLAND_DISPLAY=0` |
 * | `true` | `x11` | no `DISPLAY` | `false` | none |
 * | `true` | `wayland` | any | `true` | none |
 *
 * `auto` disables the viewer when Wayland is detected and `DISPLAY` is missing.
 * This avoids a hard-fail path when X11/XWayland is not exported to the process.
 *
 * @code
 * using orbslam3_ros2::common::ViewerBackendEnv;
 * using orbslam3_ros2::common::evaluate_viewer_backend_policy;
 *
 * // Wayland session with DISPLAY available -> keep viewer enabled and force X11.
 * auto a = evaluate_viewer_backend_policy(
 *     true, "auto", ViewerBackendEnv{":0", "wayland-0", "wayland"});
 * // a.effective_enable == true
 * // a.should_set_wayland_display == true
 * // a.wayland_display_override == "0"
 * @endcode
 *
 * @code
 * // Wayland session without DISPLAY -> disable viewer in auto mode.
 * auto b = evaluate_viewer_backend_policy(
 *     true, "auto", ViewerBackendEnv{"", "wayland-0", "wayland"});
 * // b.effective_enable == false
 * // b.should_set_wayland_display == false
 * @endcode
 */
inline auto evaluate_viewer_backend_policy(
    bool enable_pangolin, const std::string& viewer_backend,
    const ViewerBackendEnv& env) -> ViewerPolicyResult {
    ViewerPolicyResult result;
    result.requested_enable = enable_pangolin;
    result.effective_enable = enable_pangolin;
    result.has_display = !env.display.empty();
    // "0" means Wayland was already masked. Treat it as inactive.
    result.wayland_display_active =
        !env.wayland_display.empty() && env.wayland_display != "0";
    // XDG_SESSION_TYPE is a hint. WAYLAND_DISPLAY remains the stronger signal.
    result.xdg_wayland = to_lower_ascii(env.xdg_session_type) == "wayland";
    result.detected_wayland = result.wayland_display_active || result.xdg_wayland;

    auto [mode, backend_name] = parse_viewer_backend_mode(viewer_backend);
    result.mode = mode;
    result.backend_name = std::move(backend_name);

    if (to_lower_ascii(viewer_backend) != result.backend_name) {
        // Keep parsing permissive. Wrapper can warn without changing control flow.
        result.parse_warning =
            "Unknown viewer_backend='" + viewer_backend +
            "', falling back to 'auto' (expected auto|x11|wayland)";
    }

    if (!enable_pangolin) {
        // Preserve explicit disable. Policy never re-enables the viewer.
        result.reason = "viewer disabled (enable_pangolin=false)";
        return result;
    }

    switch (result.mode) {
    case ViewerBackendMode::Auto:
        if (!result.detected_wayland) {
            result.reason = "keeping default backend";
        } else if (!result.has_display) {
            // No X11 endpoint is available. Disable instead of forcing a failing path.
            result.effective_enable = false;
            result.reason = "Wayland detected but DISPLAY is empty -> disabling viewer";
        } else {
            // Pangolin backend selection happens later in wrapper startup.
            result.should_set_wayland_display = true;
            result.wayland_display_override = "0";
            result.reason = "forcing X11 (WAYLAND_DISPLAY=0)";
        }
        break;
    case ViewerBackendMode::X11:
        if (!result.has_display) {
            // Explicit x11 request with no DISPLAY. Disable instead of hard-failing later.
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

/**
 * @brief Apply environment mutation requested by a previously computed policy
 * @param result Policy result from `evaluate_viewer_backend_policy()`
 * @return `true` on success, or `false` if `setenv()` failed
 * @note This mutates only the current process environment. It must run before
 *       Pangolin creates any window or EGL context to affect backend selection.
 * @warning Calling this after Pangolin initialization is too late and will not
 *          prevent Wayland teardown crashes.
 * @details
 * The only mutation currently supported is `WAYLAND_DISPLAY=<override>`, where the
 * override is usually `"0"` to prevent Wayland backend selection.
 */
inline auto apply_viewer_backend_env_policy(const ViewerPolicyResult& result)
    -> bool {
    if (!result.should_set_wayland_display) {
        // No mutation requested. Keep caller path simple.
        return true;
    }
    // Override only in this process. Parent shell and system session stay unchanged.
    return ::setenv("WAYLAND_DISPLAY", result.wayland_display_override.c_str(), 1) ==
           0;
}

}  // namespace orbslam3_ros2::common

