import os
import time
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription

# Example runs (viewer/save validation flow):
# 1) WSLg / Wayland auto (default viewer backend):
#    ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py
# 2) Force X11 viewer backend:
#    ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py viewer_backend:=x11
# 3) Explicit Wayland (may crash on teardown in some WSLg setups):
#    ros2 launch orbslam3_ros2 realsense_d435i_rgbd_wsl_save.launch.py viewer_backend:=wayland


def _prepend_ld_library_path(prefix: str) -> str:
    current = os.environ.get("LD_LIBRARY_PATH", "")
    return f"{prefix}:{current}" if current else prefix


def _resolve_helper_script(pkg_share: Path, ws_root: Path, name: str) -> str:
    candidates = [
        pkg_share
        / name,  # installed via `install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME})`
        pkg_share / "scripts" / name,  # if install path later changes to keep subdir
        ws_root / "src" / "orbslam3_ros2" / "scripts" / name,  # source-tree fallback
    ]
    for path in candidates:
        if path.exists():
            return str(path)
    # Keep a deterministic path for error reporting if all candidates are missing.
    return str(candidates[0])


def _as_bool(text: str) -> bool:
    return text.strip().lower() in {"1", "true", "yes", "on"}


def _lc_str(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _on_wait_gate_exit(orb_start_action):
    def _handler(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg="[gate] RGB-D topics ready, starting ORB-SLAM3"), orb_start_action]
        return [
            LogInfo(
                msg=f"[gate] topic readiness failed (exit={event.returncode}), shutting down launch"
            ),
            EmitEvent(event=Shutdown(reason="wait_for_topics failed")),
        ]

    return _handler


def _make_realsense_actions(
    context, *, launch_realsense_script: str, realsense_exec: str, run_and_tee_script: str
):
    if not _as_bool(_lc_str(context, "start_realsense")):
        return []

    realsense_log_file = _lc_str(context, "realsense_log_file")
    use_sudo = _as_bool(_lc_str(context, "realsense_with_sudo"))

    tee_prefix = [
        "python3",
        run_and_tee_script,
        "--log-file",
        realsense_log_file,
        "--",
    ]
    rs_ros_args = [
        "--ros-args",
        "--remap",
        "__ns:=/camera",
        "--remap",
        "__node:=camera",
        "-p",
        "enable_sync:=true",
        "-p",
        "align_depth.enable:=true",
        "-p",
        "enable_gyro:=false",
        "-p",
        "enable_accel:=false",
        "-p",
        "enable_infra1:=false",
        "-p",
        "enable_infra2:=false",
        "-p",
        "rgb_camera.color_profile:=640,480,15",
        "-p",
        "depth_module.depth_profile:=640,480,15",
    ]

    if use_sudo:
        cmd = (
            tee_prefix
            + [
                "sudo",
                "-E",
                "python3",
                launch_realsense_script,
                "--exec-path",
                realsense_exec,
                "--",
            ]
            + rs_ros_args
        )
    else:
        cmd = tee_prefix + [realsense_exec] + rs_ros_args

    print("[launch.user] realsense cmd:", cmd, flush=True)
    return [
        GroupAction(
            scoped=True,
            actions=[
                SetEnvironmentVariable(name="FASTDDS_BUILTIN_TRANSPORTS", value="UDPv4"),
                SetEnvironmentVariable(
                    name="LD_LIBRARY_PATH", value=_prepend_ld_library_path("/usr/local/lib")
                ),
                SetEnvironmentVariable(name="ROS_LOG_DIR", value="/tmp/roslog_realsense"),
                ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True, shell=False),
            ],
        )
    ]


def _make_orb_bundle_actions(
    context,
    *,
    ws_root: str,
    orb_exec: str,
    check_outputs_script: str,
    extract_asan_script: str,
    run_and_tee_script: str,
    launch_start_ts: str,
):
    orb_log_file = _lc_str(context, "orb_log_file")
    check_outputs_enabled = _as_bool(_lc_str(context, "check_outputs"))
    dump_asan_enabled = _as_bool(_lc_str(context, "dump_asan_first_error"))
    asan_options = _lc_str(context, "asan_options")
    lsan_options = _lc_str(context, "lsan_options")
    debug_mode = _as_bool(_lc_str(context, "debug_mode"))
    effective_lsan_options = lsan_options
    if debug_mode and not effective_lsan_options:
        effective_lsan_options = "detect_leaks=0"

    enable_pangolin = _lc_str(context, "enable_pangolin")
    orb_cmd = [
        "python3",
        run_and_tee_script,
        "--log-file",
        orb_log_file,
        "--",
        orb_exec,
        "--ros-args",
        "-r",
        "__node:=orb_slam3",
        "-p",
        f"voc_file:={_lc_str(context, 'voc_file')}",
        "-p",
        f"settings_file:={_lc_str(context, 'settings_file')}",
        "-p",
        "rgb_topic:=/camera/camera/color/image_raw",
        "-p",
        "depth_topic:=/camera/camera/aligned_depth_to_color/image_raw",
        "-p",
        f"map_frame_id:={_lc_str(context, 'map_frame_id')}",
        "-p",
        f"cam_frame_id:={_lc_str(context, 'cam_frame_id')}",
        "-p",
        f"enable_pangolin:={enable_pangolin}",
        "-p",
        f"viewer_backend:={_lc_str(context, 'viewer_backend')}",
        "-p",
        f"shutdown_after_sec:={_lc_str(context, 'orb_shutdown_after_sec')}",
    ]
    print("[launch.user] orb cmd:", orb_cmd, flush=True)
    orb_additional_env = {}
    if asan_options:
        orb_additional_env["ASAN_OPTIONS"] = asan_options
    if effective_lsan_options:
        orb_additional_env["LSAN_OPTIONS"] = effective_lsan_options
    if orb_additional_env:
        print("[launch.user] orb extra env:", orb_additional_env, flush=True)

    orb_proc = ExecuteProcess(
        cmd=orb_cmd,
        output="screen",
        emulate_tty=True,
        shell=False,
        cwd=ws_root,
        additional_env=orb_additional_env or None,
    )

    actions: list[Any] = [
        GroupAction(
            scoped=True,
            actions=[
                SetEnvironmentVariable(name="FASTDDS_BUILTIN_TRANSPORTS", value="UDPv4"),
                SetEnvironmentVariable(name="ROS_LOG_DIR", value="/tmp/roslog_orb"),
                orb_proc,
            ],
        )
    ]

    if check_outputs_enabled:
        output_check_cmd = [
            "python3",
            check_outputs_script,
            "--settings-file",
            _lc_str(context, "settings_file"),
            "--workspace-root",
            ws_root,
            "--timeout-sec",
            _lc_str(context, "output_check_timeout_sec"),
            "--min-mtime",
            launch_start_ts,
            "--require-nonempty",
        ]
        print("[launch.user] check_outputs cmd:", output_check_cmd, flush=True)
        output_check_proc = ExecuteProcess(
            cmd=output_check_cmd,
            output="screen",
            emulate_tty=True,
            shell=False,
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=orb_proc,
                    on_exit=[
                        LogInfo(msg="[check] ORB exited, validating atlas/save outputs"),
                        TimerAction(period=1.0, actions=[output_check_proc]),
                    ],
                )
            )
        )

    if dump_asan_enabled:
        asan_extract_cmd = [
            "python3",
            extract_asan_script,
            "--input-log",
            orb_log_file,
            "--output-file",
            "/tmp/orbslam3_first_asan.txt",
        ]
        print("[launch.user] extract_asan cmd:", asan_extract_cmd, flush=True)
        extract_proc = ExecuteProcess(
            cmd=asan_extract_cmd,
            output="screen",
            emulate_tty=True,
            shell=False,
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=orb_proc,
                    on_exit=[
                        LogInfo(msg="[debug] extracting first ASAN error (if any) from ORB log"),
                        ExecuteProcess(
                            cmd=["rm", "-f", "/tmp/orbslam3_first_asan.txt"],
                            output="screen",
                            shell=False,
                        ),
                        extract_proc,
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("orbslam3_ros2"))
    orb_prefix = Path(get_package_prefix("orbslam3_ros2"))
    realsense_prefix = Path(get_package_prefix("realsense2_camera"))
    ws_root = pkg_share.parents[
        3
    ]  # .../colcon_ws/install/orbslam3_ros2/share/orbslam3_ros2 -> colcon_ws
    realsense_exec = realsense_prefix / "lib" / "realsense2_camera" / "realsense2_camera_node"
    orb_exec = orb_prefix / "lib" / "orbslam3_ros2" / "ros_rgbd"
    launch_realsense_script = _resolve_helper_script(pkg_share, ws_root, "launch_realsense.py")
    wait_for_topics_script = _resolve_helper_script(pkg_share, ws_root, "wait_for_topics.py")
    check_outputs_script = _resolve_helper_script(pkg_share, ws_root, "check_outputs.py")
    run_and_tee_script = _resolve_helper_script(pkg_share, ws_root, "run_and_tee.py")
    extract_asan_script = _resolve_helper_script(pkg_share, ws_root, "extract_asan_first_error.py")

    default_voc = str(pkg_share / "Vocabulary" / "ORBvoc.txt")
    # `install(DIRECTORY launch/ config/ DESTINATION share/${PROJECT_NAME})` flattens
    # `config/` contents under the package share root in the install tree.
    default_settings = str(pkg_share / "RGB-D" / "RealSense_D435i_save.yaml")
    launch_start_ts = f"{time.time():.6f}"

    voc_file_arg = DeclareLaunchArgument("voc_file", default_value=default_voc)
    settings_file_arg = DeclareLaunchArgument("settings_file", default_value=default_settings)
    map_frame_id_arg = DeclareLaunchArgument("map_frame_id", default_value="map")
    cam_frame_id_arg = DeclareLaunchArgument("cam_frame_id", default_value="camera_link")
    enable_pangolin_arg = DeclareLaunchArgument("enable_pangolin", default_value="true")
    viewer_backend_arg = DeclareLaunchArgument("viewer_backend", default_value="auto")
    realsense_with_sudo_arg = DeclareLaunchArgument("realsense_with_sudo", default_value="true")
    start_realsense_arg = DeclareLaunchArgument("start_realsense", default_value="false")
    wait_for_topics_arg = DeclareLaunchArgument("wait_for_topics", default_value="true")
    topics_timeout_sec_arg = DeclareLaunchArgument("topics_timeout_sec", default_value="20.0")
    orb_start_delay_sec_arg = DeclareLaunchArgument("orb_start_delay_sec", default_value="5.0")
    orb_shutdown_after_sec_arg = DeclareLaunchArgument(
        "orb_shutdown_after_sec", default_value="0.0"
    )
    check_outputs_arg = DeclareLaunchArgument("check_outputs", default_value="false")
    output_check_timeout_sec_arg = DeclareLaunchArgument(
        "output_check_timeout_sec", default_value="8.0"
    )
    asan_options_arg = DeclareLaunchArgument("asan_options", default_value="")
    lsan_options_arg = DeclareLaunchArgument("lsan_options", default_value="")
    debug_mode_arg = DeclareLaunchArgument("debug_mode", default_value="false")
    orb_log_file_arg = DeclareLaunchArgument("orb_log_file", default_value="/tmp/orbslam3_orb.log")
    realsense_log_file_arg = DeclareLaunchArgument(
        "realsense_log_file", default_value="/tmp/orbslam3_realsense.log"
    )
    dump_asan_first_error_arg = DeclareLaunchArgument("dump_asan_first_error", default_value="true")

    realsense_group = OpaqueFunction(
        function=lambda context: _make_realsense_actions(
            context,
            launch_realsense_script=launch_realsense_script,
            realsense_exec=str(realsense_exec),
            run_and_tee_script=run_and_tee_script,
        )
    )

    orb_group = OpaqueFunction(
        function=lambda context: _make_orb_bundle_actions(
            context,
            ws_root=str(ws_root),
            orb_exec=str(orb_exec),
            check_outputs_script=check_outputs_script,
            extract_asan_script=extract_asan_script,
            run_and_tee_script=run_and_tee_script,
            launch_start_ts=launch_start_ts,
        )
    )

    wait_for_topics_proc = ExecuteProcess(
        cmd=[
            "python3",
            wait_for_topics_script,
            "--rgb-topic",
            "/camera/camera/color/image_raw",
            "--depth-topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
            "--timeout-sec",
            LaunchConfiguration("topics_timeout_sec"),
        ],
        output="screen",
        emulate_tty=True,
        shell=False,
        condition=IfCondition(LaunchConfiguration("wait_for_topics")),
    )

    # Fallback for users who disable the readiness gate.
    delayed_orb = TimerAction(
        period=LaunchConfiguration("orb_start_delay_sec"),
        actions=[orb_group],
        condition=UnlessCondition(LaunchConfiguration("wait_for_topics")),
    )

    start_orb_after_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_topics_proc,
            on_exit=_on_wait_gate_exit(orb_group),
        ),
        condition=IfCondition(LaunchConfiguration("wait_for_topics")),
    )

    return LaunchDescription(
        [
            voc_file_arg,
            settings_file_arg,
            map_frame_id_arg,
            cam_frame_id_arg,
            enable_pangolin_arg,
            viewer_backend_arg,
            realsense_with_sudo_arg,
            start_realsense_arg,
            wait_for_topics_arg,
            topics_timeout_sec_arg,
            orb_start_delay_sec_arg,
            orb_shutdown_after_sec_arg,
            check_outputs_arg,
            output_check_timeout_sec_arg,
            asan_options_arg,
            lsan_options_arg,
            debug_mode_arg,
            orb_log_file_arg,
            realsense_log_file_arg,
            dump_asan_first_error_arg,
            realsense_group,
            wait_for_topics_proc,
            start_orb_after_gate,
            delayed_orb,
        ]
    )
