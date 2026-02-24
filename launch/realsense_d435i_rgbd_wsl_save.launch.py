import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepend_ld_library_path(prefix: str) -> str:
    current = os.environ.get("LD_LIBRARY_PATH", "")
    return f"{prefix}:{current}" if current else prefix


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("orbslam3_ros2"))
    ws_root = pkg_share.parents[2]  # .../colcon_ws/install/orbslam3_ros2/share/orbslam3_ros2 -> colcon_ws

    default_voc = str(pkg_share / "Vocabulary" / "ORBvoc.txt")
    default_settings = str(pkg_share / "config" / "RGB-D" / "RealSense_D435i_save.yaml")

    voc_file_arg = DeclareLaunchArgument("voc_file", default_value=default_voc)
    settings_file_arg = DeclareLaunchArgument("settings_file", default_value=default_settings)
    enable_pangolin_arg = DeclareLaunchArgument("enable_pangolin", default_value="true")
    realsense_with_sudo_arg = DeclareLaunchArgument("realsense_with_sudo", default_value="true")

    # Known-good WSL workaround:
    # - force custom librealsense in /usr/local
    # - force Fast DDS UDP transport (avoid root/user SHM issue)
    # - lower bandwidth so depth/aligned depth actually streams in WSL
    rs_cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"source {ws_root}/install/setup.bash && "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && "
        f"export LD_LIBRARY_PATH={_prepend_ld_library_path('/usr/local/lib')} && "
        "export ROS_LOG_DIR=/tmp/roslog_realsense && mkdir -p /tmp/roslog_realsense && "
        "ros2 run realsense2_camera realsense2_camera_node --ros-args "
        "-r __ns:=/camera -r __node:=camera "
        "-p enable_sync:=true "
        "-p align_depth.enable:=true "
        "-p enable_gyro:=false "
        "-p enable_accel:=false "
        "-p enable_infra1:=false "
        "-p enable_infra2:=false "
        "-p rgb_camera.color_profile:=640,480,15 "
        "-p depth_module.depth_profile:=640,480,15"
    )

    realsense_proc = ExecuteProcess(
        cmd=["sudo", "/bin/bash", "-lc", rs_cmd],
        output="screen",
        emulate_tty=True,
        shell=False,
    )

    orb_node = Node(
        package="orbslam3_ros2",
        executable="ros_rgbd",
        name="orb_slam3",
        output="screen",
        additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
        parameters=[
            {
                "voc_file": LaunchConfiguration("voc_file"),
                "settings_file": LaunchConfiguration("settings_file"),
                "rgb_topic": "/camera/camera/color/image_raw",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "world_frame_id": "world",
                "cam_frame_id": "camera",
                "enable_pangolin": ParameterValue(
                    LaunchConfiguration("enable_pangolin"), value_type=bool
                ),
            }
        ],
    )

    # Delay ORB startup a bit so the RealSense node can settle and start publishing RGB-D.
    delayed_orb = TimerAction(period=5.0, actions=[orb_node])

    return LaunchDescription(
        [
            voc_file_arg,
            settings_file_arg,
            enable_pangolin_arg,
            realsense_with_sudo_arg,  # kept for future extension; current launch always uses sudo path.
            realsense_proc,
            delayed_orb,
        ]
    )

