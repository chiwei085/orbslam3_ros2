from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def _build_nodes(context):
    preset = LaunchConfiguration("preset").perform(context)
    use_path_node = LaunchConfiguration("use_path_node")
    use_tracking_cloud_node = LaunchConfiguration("use_tracking_cloud_node")

    preset_overrides = {
        "high_fps": {
            "rgbd_frontend_node": {
                "qos_keep_last": 20,
                "sync_max_dt_ms": 8,
                "sync_queue_size": 60,
            },
            "orbslam3_core_node": {
                "pending_queue_size": 20,
                "latest_keep_last": 6,
                "latest_take": 2,
                "worker_time_budget_us": 12000,
                "pointcloud_max_points": 8000,
            },
            "tracking_cloud_node": {
                "publish_hz": 20,
                "max_points": 8000,
            },
            "path_node": {
                "path_max_length": 4000,
            },
        },
        "low_cpu": {
            "rgbd_frontend_node": {
                "qos_keep_last": 8,
                "sync_max_dt_ms": 15,
                "sync_queue_size": 20,
            },
            "orbslam3_core_node": {
                "pending_queue_size": 8,
                "latest_keep_last": 2,
                "latest_take": 1,
                "worker_time_budget_us": 6000,
                "pointcloud_max_points": 3000,
            },
            "tracking_cloud_node": {
                "publish_hz": 8,
                "max_points": 3000,
            },
            "path_node": {
                "path_max_length": 1200,
            },
        },
    }
    selected = preset_overrides.get(preset, preset_overrides["high_fps"])

    config_file = PathJoinSubstitution([FindPackageShare("orbslam3_ros2"), "config", "rgbd.yaml"])

    frontend = Node(
        package="orbslam3_ros2",
        executable="rgbd_frontend_node",
        name="rgbd_frontend_node",
        output="screen",
        parameters=[config_file, selected["rgbd_frontend_node"]],
    )

    core = Node(
        package="orbslam3_ros2",
        executable="orbslam3_core_node",
        name="orbslam3_core_node",
        output="screen",
        parameters=[config_file, selected["orbslam3_core_node"]],
    )

    path = Node(
        package="orbslam3_ros2",
        executable="path_node",
        name="path_node",
        output="screen",
        parameters=[config_file, selected["path_node"]],
        condition=IfCondition(use_path_node),
    )
    tracking_cloud = Node(
        package="orbslam3_ros2",
        executable="tracking_cloud_node",
        name="tracking_cloud_node",
        output="screen",
        parameters=[config_file, selected["tracking_cloud_node"]],
        condition=IfCondition(use_tracking_cloud_node),
    )
    return [frontend, core, path, tracking_cloud]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("preset", default_value="high_fps"),
            DeclareLaunchArgument("use_path_node", default_value="true"),
            DeclareLaunchArgument("use_tracking_cloud_node", default_value="true"),
            OpaqueFunction(function=_build_nodes),
        ]
    )
