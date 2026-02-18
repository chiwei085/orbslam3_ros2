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
from orbslam3_ros2.srv import GetStatus
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
                "test_mode_track_delay_ms": 4,
                "pending_queue_size": 10,
                "latest_keep_last": 4,
                "latest_take": 2,
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
        parameters=[{"rate_hz": 15.0, "runtime_s": 85.0}],
    )

    ld = launch.LaunchDescription([core, obs_pub, launch_testing.actions.ReadyToTest()])
    return ld, {"core": core, "obs_pub": obs_pub}


class TestLongRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_long_run_client")

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

    def test_status_stable_for_extended_window(self):
        status_client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        shutdown_client = self.node.create_client(Empty, "/orbslam3/shutdown")

        self.assertTrue(status_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(shutdown_client.wait_for_service(timeout_sec=5.0))

        start = time.monotonic()
        samples = 0
        first_processed = None
        last_processed = None
        first_processed_t = None
        last_processed_t = None
        rss_samples_kb = []
        latencies_ms = []
        while (time.monotonic() - start) < 70.0:
            status = self._call(status_client, GetStatus.Request(), timeout_sec=1.0)
            now = time.monotonic()
            self.assertEqual(status.worker_exception, "")
            self.assertTrue(status.worker_alive)
            self.assertLessEqual(status.pending_size, status.pending_queue_size)
            if first_processed is None:
                first_processed = status.processed_obs_count
                first_processed_t = now
            if last_processed is not None:
                self.assertGreaterEqual(status.processed_obs_count, last_processed)
            last_processed = status.processed_obs_count
            last_processed_t = now
            if status.memory_rss_kb > 0:
                rss_samples_kb.append(status.memory_rss_kb)
            if status.last_observation_stamp.sec > 0 or status.last_observation_stamp.nanosec > 0:
                latencies_ms.append(
                    max(0.0, (now - _stamp_to_sec(status.last_observation_stamp)) * 1000.0)
                )
            samples += 1
            time.sleep(1.0)

        self.assertGreaterEqual(samples, 50)
        self.assertIsNotNone(first_processed)
        self.assertIsNotNone(last_processed)
        self.assertIsNotNone(first_processed_t)
        self.assertIsNotNone(last_processed_t)
        self.assertGreater(last_processed - first_processed, 200)
        processed_hz = (last_processed - first_processed) / max(
            1e-6, (last_processed_t - first_processed_t)
        )
        self.assertGreaterEqual(processed_hz, 8.0)
        if rss_samples_kb:
            self.assertLess(max(rss_samples_kb) - min(rss_samples_kb), 100 * 1024)
        if latencies_ms:
            self.assertLess(_percentile(latencies_ms, 0.99), 300.0)

        shutdown_start = time.monotonic()
        self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)
        self.assertLess(time.monotonic() - shutdown_start, 2.0)


@launch_testing.post_shutdown_test()
class TestLongRunShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
