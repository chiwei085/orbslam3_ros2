import glob
import os
import pathlib
import sys
import time
import unittest

import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from orbslam3_ros2.srv import GetStatus
from std_srvs.srv import Empty

import launch

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1] / "utils"))
from metrics import compute_processed_hz, percentile


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
OBSERVE_WINDOW_S = 25.0


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
                "observation_topic": "/orbslam3/observations",
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
        parameters=[{"rate_hz": 30.0, "runtime_s": 40.0}],
    )

    svc_storm = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="svc_storm_node",
        name="svc_storm_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[
            {
                "runtime_s": 35.0,
                "reset_rate_hz": 5.0,
                "get_status_rate_hz": 20.0,
                "get_tracked_points_rate_hz": 10.0,
                "set_localization_mode_rate_hz": 1.0,
                "save_trajectory_rate_hz": 0.33,
            }
        ],
    )

    ld = launch.LaunchDescription([core, obs_pub, svc_storm, launch_testing.actions.ReadyToTest()])
    return ld, {"core": core, "obs_pub": obs_pub, "svc_storm": svc_storm}


class TestThreadSafetyStorm(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_thread_safety_storm_client")

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

    def test_status_stays_responsive_under_storm(self):
        status_client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        shutdown_client = self.node.create_client(Empty, "/orbslam3/shutdown")

        self.assertTrue(status_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(shutdown_client.wait_for_service(timeout_sec=5.0))

        start = time.monotonic()
        rtt_ms_samples = []
        processed_counts = []
        processed_times = []
        pending_drop_samples = []

        last_processed = None
        while (time.monotonic() - start) < OBSERVE_WINDOW_S:
            call_start = time.monotonic()
            status = self._call(status_client, GetStatus.Request(), timeout_sec=1.0)
            call_elapsed_ms = (time.monotonic() - call_start) * 1000.0
            now = time.monotonic()

            rtt_ms_samples.append(call_elapsed_ms)
            processed_counts.append(status.processed_obs_count)
            processed_times.append(now)
            pending_drop_samples.append(status.pending_drop_count)

            if last_processed is not None:
                self.assertGreaterEqual(status.processed_obs_count, last_processed)
            last_processed = status.processed_obs_count

            self.assertEqual(status.worker_exception, "")
            self.assertTrue(status.worker_alive)
            self.assertLessEqual(status.pending_size, status.pending_queue_size)
            time.sleep(0.5)

        self.assertGreaterEqual(len(rtt_ms_samples), 30)
        self.assertLess(percentile(rtt_ms_samples, 0.99), 100.0)

        processed_delta = processed_counts[-1] - processed_counts[0]
        self.assertGreater(processed_delta, 100)
        processed_hz = compute_processed_hz(processed_times, processed_counts)
        self.assertGreaterEqual(processed_hz, 15.0)

        drop_delta = pending_drop_samples[-1] - pending_drop_samples[0]
        total_handled = processed_delta + max(0, drop_delta)
        drop_ratio = float(drop_delta) / float(total_handled) if total_handled > 0 else 0.0
        self.assertLess(drop_ratio, 0.8)

        self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)


@launch_testing.post_shutdown_test()
class TestStormShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
