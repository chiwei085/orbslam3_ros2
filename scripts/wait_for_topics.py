#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool: {value}")


class TopicGate(Node):
    def __init__(self, rgb_topic: str, depth_topic: str, require_message: bool) -> None:
        super().__init__("wait_for_topics_gate")
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.require_message = require_message
        self.rgb_ready = False
        self.depth_ready = False

        self.create_subscription(Image, rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)

    def _on_rgb(self, _msg: Image) -> None:
        if not self.rgb_ready:
            self.rgb_ready = True
            self.get_logger().info(f"received RGB message on {self.rgb_topic}")

    def _on_depth(self, _msg: Image) -> None:
        if not self.depth_ready:
            self.depth_ready = True
            self.get_logger().info(f"received depth message on {self.depth_topic}")

    def publishers_ready(self) -> bool:
        return (
            self.count_publishers(self.rgb_topic) > 0
            and self.count_publishers(self.depth_topic) > 0
        )

    def messages_ready(self) -> bool:
        return self.rgb_ready and self.depth_ready


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait until RGB-D topics are ready.")
    parser.add_argument("--rgb-topic", required=True)
    parser.add_argument("--depth-topic", required=True)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--require-message", type=parse_bool, default=True)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = TopicGate(args.rgb_topic, args.depth_topic, args.require_message)
    deadline = time.monotonic() + args.timeout_sec
    next_log = 0.0

    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            pubs_ready = node.publishers_ready()
            msgs_ready = node.messages_ready()
            if args.require_message:
                if msgs_ready:
                    node.get_logger().info("topic gate passed (messages received)")
                    return 0
            elif pubs_ready:
                node.get_logger().info("topic gate passed (publishers detected)")
                return 0

            now = time.monotonic()
            if now >= next_log:
                node.get_logger().info(
                    f"waiting: rgb_pubs={node.count_publishers(args.rgb_topic)} "
                    f"depth_pubs={node.count_publishers(args.depth_topic)} "
                    f"rgb_msg={1 if node.rgb_ready else 0} "
                    f"depth_msg={1 if node.depth_ready else 0} "
                    f"require_message={1 if args.require_message else 0}"
                )
                next_log = now + 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        "[wait_for_topics] timeout waiting for RGB-D readiness",
        file=sys.stderr,
        flush=True,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
