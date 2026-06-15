#!/usr/bin/env python3
"""
test_perceptual_scan.py — Test the 6-pose PERCEPTUAL_SCAN Cartesian grid.

Simulates being at handoff distance (~0.5m from patient), generates the same
6-pose grid as wbc_qp_controller._gen_cartesian_scan_grid(), sends each pose
to the IK solver, and reports convergence.

Usage:
    ros2 run spot_control test_perceptual_scan
    ros2 run spot_control test_perceptual_scan --ros-args -p target_x:=0.5 -p target_y:=0.0 -p target_z:=0.6
"""

import sys
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class PerceptualScanTester(Node):

    def __init__(self):
        super().__init__('test_perceptual_scan')

        # Virtual target — where the patient torso is relative to link00
        self._target_x = float(
            self.declare_parameter('target_x', 0.50).value)
        self._target_y = float(
            self.declare_parameter('target_y', 0.00).value)
        self._target_z = float(
            self.declare_parameter('target_z', 0.60).value)
        self._scan_timeout = float(
            self.declare_parameter('scan_timeout', 1.5).value)  # per-ik timeout
        self._step_mode = bool(
            self.declare_parameter('step_mode', True).value)

        # Publishers
        self._pub_ik = self.create_publisher(
            PoseStamped, '/wbc/ik_goal_pose', 10)
        self._pub_en = self.create_publisher(
            Bool, '/wbc/ik_enable', 10)

        # Subscriber
        self._ik_done = False
        self._sub_ik = self.create_subscription(
            Bool, '/wbc/ik_done', self._cb_ik_done, 10)

        self._nlf_active = bool(
            self.declare_parameter('nlf_active', False).value)

    def _cb_ik_done(self, msg: Bool):
        self._ik_done = msg.data

    def generate_poses(self) -> list[PoseStamped]:
        """Replicate _gen_cartesian_scan_grid from wbc_qp_controller."""
        center = np.array([self._target_x, self._target_y, self._target_z])
        wrist_step = 0.04 if self._nlf_active else 0.12
        lateral_step = 0.06 if self._nlf_active else 0.20
        grid_type = 'NLF (tight)' if self._nlf_active else 'YOLO (wide)'

        poses: list[PoseStamped] = []

        # Phase 1 — wrist sweep 2×2 = 4 poses
        labels = ["LL", "LH", "HL", "HH"]
        for wy in range(2):
            for wz in range(2):
                p = PoseStamped()
                p.header.frame_id = 'my_spot/odom'
                p.pose.position.x = float(center[0])
                p.pose.position.y = float(center[1]) + (wy - 0.5) * wrist_step
                p.pose.position.z = float(center[2]) + (wz - 0.5) * wrist_step
                p.pose.orientation.w = 1.0
                poses.append(p)

        # Phase 2 — lateral parallax ±Y
        for sign in [-1.0, 1.0]:
            p = PoseStamped()
            p.header.frame_id = 'my_spot/odom'
            p.pose.position.x = float(center[0])
            p.pose.position.y = float(center[1]) + sign * lateral_step
            p.pose.position.z = float(center[2])
            p.pose.orientation.w = 1.0
            poses.append(p)

        self.get_logger().info(
            f'{grid_type}: center=[{center[0]:.2f},{center[1]:.2f},{center[2]:.2f}] '
            f'wrist={wrist_step:.2f} lateral={lateral_step:.2f} → {len(poses)} poses')
        return poses

    def run_scan(self):
        poses = self.generate_poses()

        print(f"\n{'='*60}")
        print(f" PERCEPTUAL SCAN TEST — {len(poses)} Cartesian poses")
        print(f" Target:  x={self._target_x:.2f}  y={self._target_y:.2f}  z={self._target_z:.2f}")
        print(f" Step mode: {'ON (press ENTER)' if self._step_mode else 'OFF (automatic)'}")
        print(f"{'='*60}\n")

        results = []
        for i, pose in enumerate(poses):
            dist = math.hypot(pose.pose.position.x,
                              pose.pose.position.y,
                              pose.pose.position.z)
            label = ["wrist LL", "wrist LH", "wrist HL", "wrist HH",
                      "lateral -Y", "lateral +Y"][i]
            symbol = "🟢" if i < 4 else "🔵"

            print(f" {symbol} Pose {i+1}/6 [{label:>10}]  "
                  f"x={pose.pose.position.x:.2f}  "
                  f"y={pose.pose.position.y:+.2f}  "
                  f"z={pose.pose.position.z:+.2f}  "
                  f"dist={dist:.2f}m")

            if self._step_mode:
                input("   Press ENTER to send IK goal...")

            # Send IK goal
            self._ik_done = False
            pose.header.stamp = self.get_clock().now().to_msg()
            self._pub_en.publish(Bool(data=True))
            self._pub_ik.publish(pose)

            # Wait for IK convergence or timeout
            t0 = self.get_clock().now()
            arrived = False
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
                if self._ik_done:
                    t_arrive = elapsed
                    arrived = True
                    break
                if elapsed >= self._scan_timeout:
                    break

            if arrived:
                print(f"   ✅ IK converged in {t_arrive:.1f}s")
            else:
                print(f"   ❌ IK timeout after {self._scan_timeout:.1f}s")

            results.append((label, dist, arrived, elapsed if not arrived else t_arrive))
            self._pub_en.publish(Bool(data=False))

        # Summary
        reached = sum(1 for _, _, ok, _ in results if ok)
        print(f"\n{'='*60}")
        print(f" SCAN COMPLETE: {reached}/{len(results)} poses reached")
        for label, dist, ok, t in results:
            status = f"✅ {t:.1f}s" if ok else f"❌ timeout"
            print(f"   {label:>10}: dist={dist:.2f}m  {status}")
        print(f"{'='*60}")

        return results


def main(args=None):
    rclpy.init(args=args)
    node = PerceptualScanTester()

    try:
        # Let subscriptions connect
        time.sleep(0.5)
        node.run_scan()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
