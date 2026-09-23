#!/usr/bin/env python3
"""Replay a rosbag segment and measure the CUDA detector. Reference is not truth."""

import argparse
import bisect
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from lidar_perception_system.msg import MovingBucketTrack
from diagnostic_msgs.msg import DiagnosticArray


class Monitor(Node):
    def __init__(self):
        super().__init__("cuda_bag_benchmark")
        self.counts = {"received": 0, "valid": 0, "direct": 0, "predicted": 0,
                       "invalid": 0, "reference": 0}
        self.outputs = []
        self.references = []
        self.last_diagnostic = {}
        self.create_subscription(MovingBucketTrack, "/moving_bucket_detector/target",
                                 self.on_target, 100)
        self.create_subscription(PointStamped, "/opponent_robot/bucket_target",
                                 self.on_reference, 100)
        self.create_subscription(DiagnosticArray,
                                 "/moving_bucket_detector/diagnostics",
                                 self.on_diagnostic, 10)

    @staticmethod
    def stamp(value):
        return value.sec + value.nanosec / 1e9

    def on_target(self, msg):
        self.counts["received"] += 1
        if msg.valid:
            self.counts["valid"] += 1
            if msg.observation_mode == MovingBucketTrack.DIRECT:
                self.counts["direct"] += 1
            elif msg.observation_mode == MovingBucketTrack.PREDICTED:
                self.counts["predicted"] += 1
            self.outputs.append((self.stamp(msg.header.stamp), msg.position.x,
                                 msg.position.y, msg.position.z))
        else:
            self.counts["invalid"] += 1

    def on_reference(self, msg):
        self.counts["reference"] += 1
        self.references.append((self.stamp(msg.header.stamp), msg.point.x,
                                msg.point.y, msg.point.z))

    def on_diagnostic(self, msg):
        for status in msg.status:
            if status.name == "moving_bucket_detector":
                self.last_diagnostic = {value.key: value.value for value in status.values}


def stop(group):
    if group.poll() is None:
        os.killpg(group.pid, signal.SIGINT)
        try:
            group.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(group.pid, signal.SIGTERM)
            group.wait(timeout=5)


def compare(outputs, references):
    references.sort()
    times = [row[0] for row in references]
    errors = []
    for sample in outputs:
        index = bisect.bisect_left(times, sample[0])
        neighbors = references[max(index - 1, 0):min(index + 1, len(references))]
        if not neighbors:
            continue
        near = min(neighbors, key=lambda row: abs(row[0] - sample[0]))
        if abs(near[0] - sample[0]) <= 0.3:
            errors.append(((sample[1] - near[1]) ** 2 +
                           (sample[2] - near[2]) ** 2) ** 0.5)
    errors.sort()
    return {"matched": len(errors),
            "xy_delta_median_m": errors[len(errors) // 2] if errors else None,
            "xy_delta_p95_m": errors[int((len(errors) - 1) * 0.95)] if errors else None,
            "within_1m": sum(error <= 1.0 for error in errors)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--offset", type=float, default=65)
    parser.add_argument("--seconds", type=float, default=50)
    parser.add_argument("--output", type=Path, default=Path("/tmp/lps_cuda_benchmark.json"))
    args = parser.parse_args()
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/lps_cuda_ros_logs"
    env["ROS2CLI_DISABLE_DAEMON"] = "1"
    Path(env["ROS_LOG_DIR"]).mkdir(exist_ok=True)
    with open("/tmp/lps_cuda_launch.log", "w") as launch_log, open(
            "/tmp/lps_cuda_bag.log", "w") as bag_log:
        launch = subprocess.Popen(
            ["ros2", "launch", "lidar_perception_system", "moving_bucket.launch.py"],
            stdout=launch_log, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)
        rclpy.init()
        monitor = Monitor()
        bag = None
        try:
            warmup = time.monotonic() + 3.0
            while time.monotonic() < warmup:
                rclpy.spin_once(monitor, timeout_sec=0.05)
            bag = subprocess.Popen(
                ["ros2", "bag", "play", str(args.bag), "--start-offset",
                 str(args.offset), "--topics", "/livox/lidar", "/tf", "/tf_static",
                 "/opponent_robot/bucket_target"],
                stdout=bag_log, stderr=subprocess.STDOUT, env=env,
                start_new_session=True)
            end = time.monotonic() + args.seconds
            while time.monotonic() < end and bag.poll() is None:
                rclpy.spin_once(monitor, timeout_sec=0.05)
            result = {"offset_s": args.offset, "duration_s": args.seconds,
                      "counts": monitor.counts,
                      "diagnostics": monitor.last_diagnostic,
                      "reference_comparison_not_ground_truth": compare(
                          monitor.outputs, monitor.references)}
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
        finally:
            if bag is not None:
                stop(bag)
            stop(launch)
            monitor.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
