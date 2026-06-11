#!/usr/bin/env python3
import json
import os
import tempfile

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Robot2PlanFileBridge(Node):
    def __init__(self):
        super().__init__("robot2_plan_file_bridge")
        self._paths = {
            "robot1": "/tmp/robot1_plan.json",
            "robot2": "/tmp/robot2_plan.json",
        }
        for namespace in self._paths:
            self.create_subscription(
                Path,
                f"/{namespace}/plan",
                lambda msg, namespace=namespace: self._on_plan(namespace, msg),
                10,
            )
        self.get_logger().info("Writing /robot1/plan and /robot2/plan to /tmp/*_plan.json")

    def _on_plan(self, namespace, msg):
        points = [
            [float(p.pose.position.x), float(p.pose.position.y)]
            for p in msg.poses
        ]
        payload = {
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "points": points,
        }
        path = self._paths[namespace]
        directory = os.path.dirname(path)
        fd, tmp_path = tempfile.mkstemp(prefix=f"{namespace}_plan_", suffix=".json", dir=directory)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(tmp_path, path)
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)


def main(args=None):
    rclpy.init(args=args)
    node = Robot2PlanFileBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
