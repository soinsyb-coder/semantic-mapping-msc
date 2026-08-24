#!/usr/bin/env python3
"""Replay the AD04-R1 motion sequence against ROS simulation time.

The ten events below were decoded from the first ten /cmd_vel messages in
20260816-121939-AD04-R1. The remaining two messages in the bag were redundant
zero-velocity messages sent 53.881 s and 126.729 s of simulation time after the
route had already ended, so they are intentionally excluded.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter


# offset in R1 simulation seconds, linear.x, angular.z, label
EVENTS = (
    (0.000, 0.00, +0.20, "yaw_left_start"),
    (1.938, 0.00, 0.00, "yaw_left_stop"),
    (6.464, 0.00, -0.20, "yaw_right_start"),
    (11.249, 0.00, 0.00, "yaw_right_stop"),
    (14.944, 0.00, +0.20, "yaw_return_start"),
    (17.692, 0.00, 0.00, "yaw_return_stop"),
    (31.221, +0.15, 0.00, "outbound_start"),
    (36.014, 0.00, 0.00, "outbound_stop"),
    (48.430, -0.15, 0.00, "return_start"),
    (52.568, 0.00, 0.00, "return_stop"),
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay the exact AD04-R1 command schedule using /clock."
    )
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument(
        "--countdown-sim",
        type=float,
        default=8.0,
        help="Simulation seconds to wait before the first command (default: 8).",
    )
    parser.add_argument(
        "--post-stop-sim",
        type=float,
        default=10.0,
        help="Stationary simulation seconds after the final stop (default: 10).",
    )
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--discovery-timeout-wall",
        type=float,
        default=30.0,
        help="Wall-clock seconds allowed for /cmd_vel subscriber discovery.",
    )
    return parser.parse_args()


def wait_until_sim(node: Node, target_ns: int):
    while rclpy.ok() and node.get_clock().now().nanoseconds < target_ns:
        rclpy.spin_once(node, timeout_sec=0.02)


def publish(node: Node, publisher, linear_x: float, angular_z: float):
    message = Twist()
    message.linear.x = linear_x
    message.angular.z = angular_z
    publisher.publish(message)


def main():
    args = parse_args()
    if args.countdown_sim < 0 or args.post_stop_sim < 0:
        raise SystemExit("countdown and post-stop durations must be non-negative")

    rclpy.init()
    node = Node(
        "ad04_r1_cmd_vel_replay",
        parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True),
        ],
    )
    publisher = node.create_publisher(Twist, args.topic, 10)
    rows = []

    try:
        deadline = time.monotonic() + args.discovery_timeout_wall
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"no subscriber discovered on {args.topic}")
            rclpy.spin_once(node, timeout_sec=0.1)

        while rclpy.ok() and node.get_clock().now().nanoseconds <= 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        start_ns = (
            node.get_clock().now().nanoseconds
            + round(args.countdown_sim * 1_000_000_000)
        )
        print(
            f"READY: subscriber found; first command in "
            f"{args.countdown_sim:.3f} simulation seconds",
            flush=True,
        )

        for index, (offset_s, linear_x, angular_z, label) in enumerate(EVENTS, 1):
            target_ns = start_ns + round(offset_s * 1_000_000_000)
            wait_until_sim(node, target_ns)
            actual_ns = node.get_clock().now().nanoseconds
            publish(node, publisher, linear_x, angular_z)
            error_ms = (actual_ns - target_ns) / 1_000_000
            rows.append(
                {
                    "event": index,
                    "label": label,
                    "planned_offset_sim_s": f"{offset_s:.3f}",
                    "actual_offset_sim_s": f"{(actual_ns - start_ns) / 1e9:.6f}",
                    "timing_error_ms": f"{error_ms:.3f}",
                    "linear_x": f"{linear_x:.2f}",
                    "angular_z": f"{angular_z:.2f}",
                    "absolute_sim_s": f"{actual_ns / 1e9:.6f}",
                }
            )
            print(
                f"EVENT {index:02d}/10 {label}: sim_offset={offset_s:.3f}s, "
                f"linear.x={linear_x:+.2f}, angular.z={angular_z:+.2f}, "
                f"timing_error={error_ms:+.3f}ms",
                flush=True,
            )

        post_stop_target_ns = (
            start_ns
            + round((EVENTS[-1][0] + args.post_stop_sim) * 1_000_000_000)
        )
        wait_until_sim(node, post_stop_target_ns)
        print(
            f"REPLAY_COMPLETE: final stop held for "
            f"{args.post_stop_sim:.3f} simulation seconds",
            flush=True,
        )
    except (KeyboardInterrupt, RuntimeError) as exc:
        # An aborted replay is not a formal repeat, but always command a stop.
        for _ in range(3):
            publish(node, publisher, 0.0, 0.0)
            time.sleep(0.05)
        print(f"REPLAY_ABORTED: {exc}", flush=True)
        raise SystemExit(1) from exc
    finally:
        if args.log_file and rows:
            args.log_file.parent.mkdir(parents=True, exist_ok=True)
            with args.log_file.open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
