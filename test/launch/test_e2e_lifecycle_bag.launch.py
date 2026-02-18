import glob
import json
import os
import pathlib
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import launch.actions
import launch_testing
import launch_testing.actions
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from orbslam3_ros2.srv import GetStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

import launch


def _build_test_env():
    candidates = sorted(glob.glob(os.path.expanduser("~/.conan2/p/b/pango*/p/lib")))
    current = os.environ.get("LD_LIBRARY_PATH", "")
    env = os.environ.copy()
    if candidates:
        prefix = ":".join(candidates)
        env["LD_LIBRARY_PATH"] = f"{prefix}:{current}" if current else prefix
    return env


TEST_ENV = _build_test_env()
CORE_NODE = "/orbslam3_core_node"
POSE_TOPIC = "/orbslam3/camera_pose"
SHUTDOWN_SERVICE = "/orbslam3/shutdown"
DEFAULT_CAMERA_FRAME = "camera_link"
DEFAULT_GATE_MODE = "full"


def generate_test_description():
    keep_alive = launch.actions.ExecuteProcess(
        cmd=["python3", "-c", "import time; time.sleep(3600)"], output="screen"
    )
    return launch.LaunchDescription([keep_alive, launch_testing.actions.ReadyToTest()])


class TestE2ELifecycleBag(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bag_path = cls._required_path_static("ORBSLAM3_E2E_BAG_PATH")
        cls.voc_file = cls._required_path_static("ORBSLAM3_E2E_VOC_FILE")
        cls.settings_file = cls._required_path_static("ORBSLAM3_E2E_SETTINGS_FILE")
        cls.gate_mode = os.environ.get("ORBSLAM3_E2E_GATE_MODE", DEFAULT_GATE_MODE).strip().lower()
        if cls.gate_mode not in ("smoke", "full"):
            raise unittest.SkipTest(
                f"ORBSLAM3_E2E_GATE_MODE must be smoke/full, got: {cls.gate_mode}"
            )
        play_seconds_raw = os.environ.get("ORBSLAM3_E2E_PLAY_SECONDS", "").strip()
        cls.play_seconds = float(play_seconds_raw) if play_seconds_raw else None
        summary_file_raw = os.environ.get("ORBSLAM3_E2E_SUMMARY_FILE", "").strip()
        cls.summary_file = pathlib.Path(summary_file_raw) if summary_file_raw else None
        cls.summary = {
            "gate_mode": cls.gate_mode,
            "bag_path": cls.bag_path,
            "voc_file": cls.voc_file,
            "settings_file": cls.settings_file,
            "play_seconds": cls.play_seconds,
            "pass": False,
            "cycles": [],
        }

        rclpy.init()
        cls.node = rclpy.create_node("test_e2e_lifecycle_bag_client")
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls._pose_event = threading.Event()
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cls._pose_sub = cls.node.create_subscription(
            PoseStamped, POSE_TOPIC, cls._on_pose, pose_qos
        )
        cls._qos_override_file = pathlib.Path(tempfile.gettempdir()) / "orbslam3_e2e_bag_qos.yaml"
        cls._qos_override_file.write_text(
            "/orbslam3/observations:\n"
            "  reliability: best_effort\n"
            "  history: keep_last\n"
            "  depth: 10\n"
            "  durability: volatile\n",
            encoding="utf-8",
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_pose(cls, _msg):
        cls._pose_event.set()

    @classmethod
    def _required_path_static(cls, env_name):
        value = os.environ.get(env_name, "").strip()
        if not value:
            raise unittest.SkipTest(f"{env_name} is not set")
        path = pathlib.Path(value)
        if not path.exists():
            raise unittest.SkipTest(f"{env_name} path does not exist: {value}")
        return str(path)

    @classmethod
    def _write_summary(cls):
        if cls.summary_file is None:
            return
        cls.summary_file.parent.mkdir(parents=True, exist_ok=True)
        cls.summary_file.write_text(json.dumps(cls.summary, indent=2) + "\n", encoding="utf-8")

    def _call(self, client, req, timeout_sec=3.0):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call timeout")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def _change_state(self, transition_id):
        client = self.node.create_client(ChangeState, f"{CORE_NODE}/change_state")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        req = ChangeState.Request()
        req.transition.id = transition_id
        resp = self._call(client, req, timeout_sec=5.0)
        self.assertTrue(resp.success, f"lifecycle transition failed: {transition_id}")

    def _get_state_label(self):
        client = self.node.create_client(GetState, f"{CORE_NODE}/get_state")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        resp = self._call(client, GetState.Request(), timeout_sec=5.0)
        return resp.current_state.label

    def _wait_until(self, predicate, timeout_sec, message):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        self.fail(message)

    def _wait_for_pose_once(self, timeout_sec):
        self._pose_event.clear()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._pose_event.is_set():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        return False

    def _get_status(self):
        client = self.node.create_client(GetStatus, "/orbslam3/get_status")
        self.assertTrue(client.wait_for_service(timeout_sec=3.0))
        return self._call(client, GetStatus.Request(), timeout_sec=3.0)

    def _spawn(self, cmd, log_path):
        logf = open(log_path, "w", encoding="utf-8")
        proc = subprocess.Popen(
            cmd,
            stdout=logf,
            stderr=subprocess.STDOUT,
            env=TEST_ENV,
            preexec_fn=os.setsid,
        )
        return proc, logf

    def _terminate_proc(self, proc, logf):
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
        logf.close()

    def _run_cycle(self, cycle_id, bag_path, voc_file, settings_file):
        camera_frame = os.environ.get("ORBSLAM3_E2E_CAMERA_FRAME", DEFAULT_CAMERA_FRAME)
        core_log = f"/tmp/orbslam3_e2e_core_{cycle_id}.log"
        bag_log = f"/tmp/orbslam3_e2e_bag_{cycle_id}.log"

        core_cmd = [
            "ros2",
            "run",
            "orbslam3_ros2",
            "orbslam3_core_node",
            "--ros-args",
            "-p",
            "autostart_lifecycle:=false",
            "-p",
            "test_mode_skip_orb_init:=false",
            "-p",
            f"voc_file:={voc_file}",
            "-p",
            f"settings_file:={settings_file}",
            "-p",
            f"camera_frame:={camera_frame}",
        ]
        core_proc, core_logf = self._spawn(core_cmd, core_log)
        bag_proc = None
        bag_logf = None
        cycle_summary = {
            "cycle_id": cycle_id,
            "core_log": core_log,
            "bag_log": bag_log,
            "pose_ready": False,
            "tf_lookup_success": False,
            "status": None,
            "error": "",
        }
        try:
            self._wait_until(
                lambda: self._get_state_label() == "unconfigured",
                timeout_sec=5.0,
                message="core did not enter unconfigured within 5s",
            )
            self._wait_until(
                lambda: self.node.count_publishers(POSE_TOPIC) == 0,
                timeout_sec=5.0,
                message="camera_pose publisher should not exist while unconfigured",
            )

            self._change_state(Transition.TRANSITION_CONFIGURE)
            self._change_state(Transition.TRANSITION_ACTIVATE)
            self._wait_until(
                lambda: self._get_state_label() == "active",
                timeout_sec=5.0,
                message="core did not enter active within 5s",
            )
            self._wait_until(
                lambda: self.node.count_publishers(POSE_TOPIC) >= 1,
                timeout_sec=5.0,
                message="camera_pose publisher not ready within 5s",
            )
            self._wait_until(
                lambda: self.node.count_subscribers("/orbslam3/observations") >= 1,
                timeout_sec=5.0,
                message="core observation subscriber not ready within 5s",
            )

            bag_cmd = [
                "ros2",
                "bag",
                "play",
                bag_path,
                "--clock",
                "--qos-profile-overrides-path",
                str(self._qos_override_file),
            ]
            if self.play_seconds is not None and self.play_seconds > 0.0:
                bag_cmd += ["--playback-duration", str(self.play_seconds)]
            bag_proc, bag_logf = self._spawn(bag_cmd, bag_log)

            self.assertTrue(
                self._wait_for_pose_once(timeout_sec=5.0),
                "pose stream not ready within 5s after activation",
            )
            cycle_summary["pose_ready"] = True
            self._wait_until(
                lambda: self.node.count_publishers("/tf") >= 1,
                timeout_sec=5.0,
                message="TF publisher not ready within 5s (check publish_tf/map_frame/camera_frame)",
            )
            self._wait_until(
                lambda: self._lookup_tf_ok("map", camera_frame),
                timeout_sec=10.0,
                message=f"TF lookup map->{camera_frame} failed within 10s",
            )
            cycle_summary["tf_lookup_success"] = True
            status = self._get_status()
            cycle_summary["status"] = {
                "rx_obs_count": int(status.rx_obs_count),
                "enqueued_obs_count": int(status.enqueued_obs_count),
                "processed_obs_count": int(status.processed_obs_count),
                "pose_published_count": int(status.pose_published_count),
                "ignored_obs_count": int(status.ignored_obs_count),
            }
            self.assertGreaterEqual(status.rx_obs_count, 1)
            self.assertGreaterEqual(status.enqueued_obs_count, 1)
            self.assertGreaterEqual(status.processed_obs_count, 1)
            self.assertGreaterEqual(status.pose_published_count, 1)
            self.assertEqual(status.ignored_obs_count, 0)

            shutdown_client = self.node.create_client(Empty, SHUTDOWN_SERVICE)
            self.assertTrue(shutdown_client.wait_for_service(timeout_sec=3.0))
            self._call(shutdown_client, Empty.Request(), timeout_sec=2.0)

            self._wait_until(
                lambda: core_proc.poll() is not None,
                timeout_sec=5.0,
                message="core process did not exit within 5s after /shutdown",
            )
            self._wait_until(
                lambda: self.node.count_publishers(POSE_TOPIC) == 0,
                timeout_sec=5.0,
                message="camera_pose publisher still present after shutdown",
            )
        except Exception as exc:
            cycle_summary["error"] = str(exc)
            raise
        finally:
            self.summary["cycles"].append(cycle_summary)
            self._write_summary()
            if bag_proc is not None and bag_logf is not None:
                self._terminate_proc(bag_proc, bag_logf)
            self._terminate_proc(core_proc, core_logf)

    def _lookup_tf_ok(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return True
        except TransformException:
            return False

    def test_two_cycle_e2e(self):
        self._run_cycle("cycle1", self.bag_path, self.voc_file, self.settings_file)
        if self.gate_mode == "full":
            self._run_cycle("cycle2", self.bag_path, self.voc_file, self.settings_file)
        self.summary["pass"] = True
        self._write_summary()
