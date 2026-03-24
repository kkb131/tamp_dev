#!/usr/bin/env python3
"""Send all DG5F joints to 0 degrees via ROS2 JointTrajectory topic.

Requires dg5f_driver running:
    ros2 launch dg5f_driver dg5f_right_driver.launch.py delto_ip:=169.254.186.72

Usage:
    python3 -m tesollo.tests.test_zero_ros2
    python3 -m tesollo.tests.test_zero_ros2 --hand left --duration 2.0
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

RIGHT_JOINT_NAMES = [
    "rj_dg_1_1", "rj_dg_1_2", "rj_dg_1_3", "rj_dg_1_4",
    "rj_dg_2_1", "rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4",
    "rj_dg_3_1", "rj_dg_3_2", "rj_dg_3_3", "rj_dg_3_4",
    "rj_dg_4_1", "rj_dg_4_2", "rj_dg_4_3", "rj_dg_4_4",
    "rj_dg_5_1", "rj_dg_5_2", "rj_dg_5_3", "rj_dg_5_4",
]

LEFT_JOINT_NAMES = [
    "lj_dg_1_1", "lj_dg_1_2", "lj_dg_1_3", "lj_dg_1_4",
    "lj_dg_2_1", "lj_dg_2_2", "lj_dg_2_3", "lj_dg_2_4",
    "lj_dg_3_1", "lj_dg_3_2", "lj_dg_3_3", "lj_dg_3_4",
    "lj_dg_4_1", "lj_dg_4_2", "lj_dg_4_3", "lj_dg_4_4",
    "lj_dg_5_1", "lj_dg_5_2", "lj_dg_5_3", "lj_dg_5_4",
]


class ZeroPositionNode(Node):
    def __init__(self, hand: str, duration: float):
        super().__init__("dg5f_zero_position")
        self._hand = hand
        self._duration = duration

        prefix = f"dg5f_{hand}"
        self._joint_names = RIGHT_JOINT_NAMES if hand == "right" else LEFT_JOINT_NAMES

        # Publisher for JointTrajectory commands
        traj_topic = f"/{prefix}/{prefix}_controller/joint_trajectory"
        self._pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # Subscriber for joint state feedback
        js_topic = f"/{prefix}/joint_states"
        self._latest_js = None
        self._sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        self.get_logger().info(f"Publishing to: {traj_topic}")
        self.get_logger().info(f"Listening on: {js_topic}")

    def _js_cb(self, msg: JointState):
        self._latest_js = msg

    def wait_for_joint_states(self, timeout=5.0):
        """Wait until we receive at least one joint_states message."""
        self.get_logger().info("Waiting for joint_states...")
        t0 = time.monotonic()
        while self._latest_js is None and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest_js is None:
            self.get_logger().warn("No joint_states received (timeout)")
            return False
        self.get_logger().info(
            f"Got joint_states: {len(self._latest_js.position)} joints"
        )
        return True

    def send_zero(self):
        """Send all joints to 0 radians."""
        msg = JointTrajectory()
        msg.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * 20
        point.time_from_start = Duration(seconds=self._duration).to_msg()
        msg.points = [point]

        self._pub.publish(msg)
        self.get_logger().info(
            f"Sent 0-degree command ({self._duration}s duration)"
        )

    def print_current_positions(self):
        """Print current joint positions if available."""
        if self._latest_js is None:
            return
        positions = self._latest_js.position
        print(f"\nCurrent positions ({len(positions)} joints):")
        names = self._latest_js.name
        for name, pos in zip(names, positions):
            print(f"  {name}: {pos:+.4f} rad ({pos * 180 / 3.14159:+.1f} deg)")


def main():
    parser = argparse.ArgumentParser(description="DG5F zero position via ROS2")
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--duration", type=float, default=1.0,
                        help="Trajectory duration in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = ZeroPositionNode(hand=args.hand, duration=args.duration)

    try:
        # Wait for driver feedback
        if node.wait_for_joint_states():
            node.print_current_positions()

        # Send zero command
        node.send_zero()

        # Wait for motion to complete
        print(f"\nWaiting {args.duration + 0.5}s for motion...")
        t0 = time.monotonic()
        while (time.monotonic() - t0) < args.duration + 0.5:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Show final positions
        node.print_current_positions()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Done")


if __name__ == "__main__":
    main()
