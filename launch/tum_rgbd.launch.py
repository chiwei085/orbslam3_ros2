from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("orbslam3_ros2"))

    default_voc = str(pkg_share / "Vocabulary" / "ORBvoc.txt")
    default_settings = str(pkg_share / "config" / "RGB-D" / "TUM1.yaml")

    # Refer thien94 launch defaults to TUM1; switch to TUM2.yaml or TUM3.yaml for other TUM sequences.
    voc_file_arg = DeclareLaunchArgument("voc_file", default_value=default_voc)
    settings_file_arg = DeclareLaunchArgument("settings_file", default_value=default_settings)
    rgb_topic_arg = DeclareLaunchArgument("rgb_topic", default_value="/camera/rgb/image_raw")
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/depth_registered/image_raw"
    )
    enable_pangolin_arg = DeclareLaunchArgument("enable_pangolin", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    orb_slam3_node = Node(
        package="orbslam3_ros2",
        executable="ros_rgbd",
        name="orb_slam3",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "voc_file": LaunchConfiguration("voc_file"),
                "settings_file": LaunchConfiguration("settings_file"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "world_frame_id": "world",
                "cam_frame_id": "camera",
                "enable_pangolin": ParameterValue(
                    LaunchConfiguration("enable_pangolin"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            voc_file_arg,
            settings_file_arg,
            rgb_topic_arg,
            depth_topic_arg,
            enable_pangolin_arg,
            use_sim_time_arg,
            orb_slam3_node,
        ]
    )
