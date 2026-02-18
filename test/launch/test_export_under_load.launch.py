import glob
import math
import os
import time
import unittest

import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from orbslam3_ros2.action import ExportPointCloud
from orbslam3_ros2.srv import GetStatus
from rclpy.action import ActionClient
from std_srvs.srv import Empty

import launch


def _build_test_env():
    candidates = sorted(glob.glob(os.path.expanduser("~/.conan2/p/b/pango*/p/lib")))
    current = os.environ.get("LD_LIBRARY_PATH", "")
    if candidates:
        prefix = ":".join(candidates)
        ld_library_path = f"{prefix}:{current}" if current else prefix
    else:
        ld_library_path = current
    return {"LD_LIBRARY_PATH": ld_library_path} if ld_library_path else {}


TEST_ENV = _build_test_env()


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _percentile(values, p):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, int(math.ceil(p * len(ordered)) - 1)))
    return float(ordered[idx])


def generate_test_description():
    core = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="orbslam3_core_node",
        name="orbslam3_core_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[
            {
                "test_mode_skip_orb_init": True,
                "test_mode_track_delay_ms": 8,
                "publish_tf": False,
            }
        ],
    )

    obs_pub = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="obs_pub_node",
        name="obs_pub_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[{"rate_hz": 30.0, "runtime_s": 35.0}],
    )

    svc_storm = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="svc_storm_node",
        name="svc_storm_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[
            {
                "runtime_s": 30.0,
                "get_status_rate_hz": 15.0,
                "get_tracked_points_rate_hz": 10.0,
                "set_localization_mode_rate_hz": 1.0,
                "export_rate_hz": 0.1,
            }
        ],
    )

    ld = launch.LaunchDescription([core, obs_pub, svc_storm, launch_testing.actions.ReadyToTest()])
    return ld, {"core": core, "obs_pub": obs_pub, "svc_storm": svc_storm}


class TestExportUnderLoad(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_export_under_load_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, req, timeout_sec=2.0):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call timeout")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def _run_export_goal(self, action_client, timeout_sec=5.0):
        goal = ExportPointCloud.Goal()
        goal.scope = "tracking"
        goal.max_points = 200
        goal.publish_snapshot = False
        goal.include_cloud_in_result = False

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=timeout_sec)
        self.assertTrue(send_future.done(), "goal send timed out")
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)
        self.assertTrue(result_future.done(), "goal result timed out")
        wrapped = result_future.result()
        self.assertTrue(wrapped.result.success, wrapped.result.message)

    def test_export_action_and_status_responsiveness(self):
        status_client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        shutdown_client = self.node.create_client(Empty, "/orbslam3/shutdown")
        export_client = ActionClient(self.node, ExportPointCloud, "/orbslam3/map/export_pointcloud")

        self.assertTrue(status_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(shutdown_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(export_client.wait_for_server(timeout_sec=5.0))

        processed_before = None
        processed_after = None
        latencies_ms = []
        for _ in range(3):
            before = self._call(status_client, GetStatus.Request(), timeout_sec=1.0)
            if processed_before is None:
                processed_before = before.processed_obs_count
            self._run_export_goal(export_client, timeout_sec=5.0)
            started = time.monotonic()
            status = self._call(status_client, GetStatus.Request(), timeout_sec=1.0)
            self.assertLess(time.monotonic() - started, 1.0)
            self.assertEqual(status.worker_exception, "")
            self.assertTrue(status.worker_alive)
            processed_after = status.processed_obs_count
            if status.last_observation_stamp.sec > 0 or status.last_observation_stamp.nanosec > 0:
                latencies_ms.append(
                    max(
                        0.0,
                        (time.monotonic() - _stamp_to_sec(status.last_observation_stamp)) * 1000.0,
                    )
                )
            time.sleep(1.0)

        self.assertIsNotNone(processed_before)
        self.assertIsNotNone(processed_after)
        self.assertGreater(processed_after - processed_before, 50)
        if latencies_ms:
            self.assertLess(_percentile(latencies_ms, 0.99), 300.0)

        shutdown_started = time.monotonic()
        self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)
        self.assertLess(time.monotonic() - shutdown_started, 2.0)


@launch_testing.post_shutdown_test()
class TestExportShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
