import glob
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


CORE_PARAMS = {
    "test_mode_skip_orb_init": True,
    "observation_topic": "/orbslam3/observations",
    "publish_tf": False,
}


def generate_test_description():
    core = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="orbslam3_core_node",
        name="orbslam3_core_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[CORE_PARAMS],
    )

    obs_pub = launch_ros.actions.Node(
        package="orbslam3_ros2",
        executable="obs_pub_node",
        name="obs_pub_node",
        output="screen",
        additional_env=TEST_ENV,
        parameters=[{"rate_hz": 5.0, "runtime_s": 6.0}],
    )

    ld = launch.LaunchDescription(
        [
            core,
            obs_pub,
            launch_testing.actions.ReadyToTest(),
        ]
    )
    return ld, {"core": core, "obs_pub": obs_pub}


class TestRgbdSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_rgbd_smoke_client")

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

    def test_smoke_services_and_shutdown(self):
        status_client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        reset_client = self.node.create_client(Empty, "/orbslam3/reset")
        shutdown_client = self.node.create_client(Empty, "/orbslam3/shutdown")

        self.assertTrue(status_client.wait_for_service(timeout_sec=2.0))
        self.assertTrue(reset_client.wait_for_service(timeout_sec=2.0))
        self.assertTrue(shutdown_client.wait_for_service(timeout_sec=2.0))

        start = time.monotonic()
        status = self._call(status_client, GetStatus.Request(), timeout_sec=2.0)
        self.assertLess(time.monotonic() - start, 2.0)
        self.assertTrue(status.worker_alive)
        self.assertEqual(status.worker_exception, "")
        self.assertGreaterEqual(status.processed_obs_count, 0)

        self._call(reset_client, Empty.Request(), timeout_sec=2.0)

        shutdown_begin = time.monotonic()
        self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)
        self.assertLess(time.monotonic() - shutdown_begin, 2.0)


@launch_testing.post_shutdown_test()
class TestSmokeShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
