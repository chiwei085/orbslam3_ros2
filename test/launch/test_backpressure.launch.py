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
from metrics import compute_latency_ms, compute_processed_hz


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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


BURST_START_S = 10.0
BURST_END_S = 30.0
TEST_WINDOW_S = 35.0
RECOVERY_DEADLINE_S = 3.0


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
                "test_mode_track_delay_ms": 15,
                "observation_topic": "/orbslam3/observations",
                "pending_queue_size": 8,
                "latest_keep_last": 4,
                "latest_take": 1,
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
        parameters=[
            {
                "rate_hz": 30.0,
                "burst_rate_hz": 90.0,
                "burst_start_s": BURST_START_S,
                "burst_duration_s": BURST_END_S - BURST_START_S,
                "runtime_s": 42.0,
            }
        ],
    )

    ld = launch.LaunchDescription([core, obs_pub, launch_testing.actions.ReadyToTest()])
    return ld, {"core": core, "obs_pub": obs_pub}


class TestBackpressure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_backpressure_client")

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

    def test_backpressure_is_bounded(self):
        status_client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        shutdown_client = self.node.create_client(Empty, "/orbslam3/shutdown")

        self.assertTrue(status_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(shutdown_client.wait_for_service(timeout_sec=5.0))

        start = time.monotonic()
        processed_counts = []
        processed_times = []
        latency_recv_ros_s = []
        latency_src_ros_s = []

        first_sample = None
        last_sample = None
        burst_start_drop = None
        burst_end_drop = None
        post_start_drop = None
        post_end_drop = None
        recovered_pending_zero_at = None
        last_status = None

        while (time.monotonic() - start) < TEST_WINDOW_S:
            status = self._call(status_client, GetStatus.Request(), timeout_sec=1.0)
            now_mono = time.monotonic()
            elapsed = now_mono - start
            now_ros_s = float(self.node.get_clock().now().nanoseconds) * 1e-9

            if first_sample is None:
                first_sample = now_mono
            last_sample = now_mono
            last_status = status

            self.assertLessEqual(status.pending_size, status.pending_queue_size)
            processed_counts.append(status.processed_obs_count)
            processed_times.append(now_mono)

            if status.last_observation_stamp.sec > 0 or status.last_observation_stamp.nanosec > 0:
                latency_recv_ros_s.append(now_ros_s)
                latency_src_ros_s.append(_stamp_to_sec(status.last_observation_stamp))

            if elapsed >= BURST_START_S and burst_start_drop is None:
                burst_start_drop = status.pending_drop_count
            if elapsed >= BURST_END_S and burst_end_drop is None:
                burst_end_drop = status.pending_drop_count
                post_start_drop = status.pending_drop_count
            if (
                elapsed >= BURST_END_S
                and status.pending_size == 0
                and recovered_pending_zero_at is None
            ):
                recovered_pending_zero_at = now_mono
            if elapsed >= BURST_END_S:
                post_end_drop = status.pending_drop_count

            time.sleep(0.5)

        self.assertIsNotNone(first_sample)
        self.assertIsNotNone(last_sample)
        self.assertIsNotNone(last_status)
        self.assertIsNotNone(burst_start_drop)
        self.assertIsNotNone(burst_end_drop)
        self.assertIsNotNone(post_start_drop)
        self.assertIsNotNone(post_end_drop)

        processed_delta = processed_counts[-1] - processed_counts[0]
        self.assertGreaterEqual(processed_delta, 300)

        processed_hz = compute_processed_hz(processed_times, processed_counts)
        self.assertGreaterEqual(processed_hz, 10.0)

        burst_drop_delta = burst_end_drop - burst_start_drop
        self.assertGreaterEqual(burst_drop_delta, 50)

        self.assertIsNotNone(recovered_pending_zero_at)
        self.assertLessEqual(recovered_pending_zero_at - (start + BURST_END_S), RECOVERY_DEADLINE_S)

        post_drop_delta = post_end_drop - post_start_drop
        self.assertLessEqual(post_drop_delta, 5)

        lat_mean_ms, lat_p99_ms = compute_latency_ms(latency_recv_ros_s, latency_src_ros_s)
        self.assertGreaterEqual(lat_mean_ms, 0.0)
        self.assertLess(lat_p99_ms, 300.0)

        self.assertEqual(last_status.pending_size, 0)

        shutdown_start = time.monotonic()
        self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)
        self.assertLess(time.monotonic() - shutdown_start, 2.0)


@launch_testing.post_shutdown_test()
class TestBackpressureShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
