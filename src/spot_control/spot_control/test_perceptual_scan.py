#!/usr/bin/env python3
"""
test_perceptual_scan.py — Test the 6-pose PERCEPTUAL_SCAN Cartesian grid.

Fully simulates the APPROACHING flow:
  1. Send arm to HOME (FWD-C, same as wbc_qp_controller._send_home)
  2. Wait for ik_done
  3. Send LOOKAT goal (arm points to target center)
  4. Wait for ik_done
  5. Run 6-pose Cartesian scan grid at handoff distance

Usage:
    ros2 run spot_control test_perceptual_scan
    ros2 run spot_control test_perceptual_scan --ros-args \\
      -p target_x:=0.5 -p target_y:=0.0 -p target_z:=0.6 \\
      -p step_mode:=false
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# FWD-C home pose (from wbc_qp_controller SEARCH_POSES)
HOME_POS = np.array([0.144, -0.005, 0.52])
HOME_QUAT = np.array([0.0162, 0.2376, -0.0232, 0.9709])


class PerceptualScanTester(Node):

    def __init__(self):
        super().__init__('test_perceptual_scan')

        self._target_x = float(self.declare_parameter('target_x', 0.50).value)
        self._target_y = float(self.declare_parameter('target_y', 0.00).value)
        self._target_z = float(self.declare_parameter('target_z', 0.60).value)
        self._ik_timeout = float(self.declare_parameter('ik_timeout', 3.0).value)
        self._step_mode = bool(self.declare_parameter('step_mode', True).value)
        self._nlf_active = bool(self.declare_parameter('nlf_active', False).value)

        self._pub_ik = self.create_publisher(PoseStamped, '/wbc/ik_goal_pose', 10)
        self._pub_en = self.create_publisher(Bool, '/wbc/ik_enable', 10)
        self._ik_done = False
        self._sub_ik = self.create_subscription(Bool, '/wbc/ik_done', self._cb_ik_done, 10)

    def _cb_ik_done(self, msg: Bool):
        self._ik_done = msg.data

    def _wait_ik(self, label: str) -> tuple[bool, float]:
        """Wait for IK convergence or timeout. Returns (ok, elapsed_sec)."""
        self._ik_done = False
        t0 = self.get_clock().now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if self._ik_done:
                return True, elapsed
            if elapsed >= self._ik_timeout:
                return False, elapsed

    def _send_pose(self, pose: PoseStamped, label: str) -> bool:
        pose.header.stamp = self.get_clock().now().to_msg()
        self._pub_en.publish(Bool(data=True))
        self._pub_ik.publish(pose)
        ok, t = self._wait_ik(label)
        if ok:
            print(f"   ✅ {label} — converged in {t:.1f}s")
        else:
            print(f"   ❌ {label} — timeout after {t:.1f}s")
        self._pub_en.publish(Bool(data=False))
        return ok

    # ── Phase 1: HOME ────────────────────────────────────────────────
    def _phase_home(self) -> bool:
        print("\n── Phase 1: HOME (LOCKING)\n")
        p = PoseStamped()
        p.header.frame_id = 'link00'
        p.pose.position.x = float(HOME_POS[0])
        p.pose.position.y = float(HOME_POS[1])
        p.pose.position.z = float(HOME_POS[2])
        p.pose.orientation.x = float(HOME_QUAT[0])
        p.pose.orientation.y = float(HOME_QUAT[1])
        p.pose.orientation.z = float(HOME_QUAT[2])
        p.pose.orientation.w = float(HOME_QUAT[3])

        if self._step_mode:
            input("   Press ENTER to send HOME (FWD-C)...")
        return self._send_pose(p, "HOME (FWD-C)")

    # ── Phase 2: LOOKAT ──────────────────────────────────────────────
    def _phase_lookat(self) -> bool:
        print("\n── Phase 2: LOOKAT (PRE_APPROACH / APPROACHING) ──\n")

        # LOOKAT goal: identity orientation, position = target center
        # This tells the IK solver to point the EE toward this position
        p = PoseStamped()
        p.header.frame_id = 'my_spot/odom'
        p.pose.position.x = self._target_x
        p.pose.position.y = self._target_y
        p.pose.position.z = self._target_z
        p.pose.orientation.w = 1.0

        if self._step_mode:
            input(f"   Press ENTER to send LOOKAT ({self._target_x:.2f},{self._target_y:.2f},{self._target_z:.2f})...")
        return self._send_pose(p, f"LOOKAT [{self._target_x:.2f},{self._target_y:.2f},{self._target_z:.2f}]")

    # ── Phase 3: PERCEPTUAL SCAN ─────────────────────────────────────
    def _generate_scan_poses(self) -> list[PoseStamped]:
        """Replicate _gen_cartesian_scan_grid from wbc_qp_controller."""
        center = np.array([self._target_x, self._target_y, self._target_z])
        wrist_step = 0.04 if self._nlf_active else 0.12
        lateral_step = 0.06 if self._nlf_active else 0.20
        grid_type = 'NLF (tight)' if self._nlf_active else 'YOLO (wide)'

        poses: list[PoseStamped] = []
        for wy in range(2):
            for wz in range(2):
                p = PoseStamped()
                p.header.frame_id = 'my_spot/odom'
                p.pose.position.x = float(center[0])
                p.pose.position.y = float(center[1]) + (wy - 0.5) * wrist_step
                p.pose.position.z = float(center[2]) + (wz - 0.5) * wrist_step
                p.pose.orientation.w = 1.0
                poses.append(p)
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

    def _phase_scan(self) -> list:
        print("\n── Phase 3: PERCEPTUAL SCAN (6 Cartesian poses) ──\n")
        poses = self._generate_scan_poses()
        wrist_step = 0.04 if self._nlf_active else 0.12
        lateral_step = 0.06 if self._nlf_active else 0.20
        labels = ["wrist LL", "wrist LH", "wrist HL", "wrist HH",
                   "lateral -Y", "lateral +Y"]
        symbols = ["🟢"] * 4 + ["🔵"] * 2

        results = []
        for i, pose in enumerate(poses):
            dist = math.hypot(pose.pose.position.x,
                              pose.pose.position.y,
                              pose.pose.position.z)
            print(f" {symbols[i]} Pose {i+1}/6 [{labels[i]:>10}]  "
                  f"x={pose.pose.position.x:.2f}  "
                  f"y={pose.pose.position.y:+.2f}  "
                  f"z={pose.pose.position.z:+.2f}  "
                  f"dist={dist:.2f}m")

            if self._step_mode:
                input("   Press ENTER to send IK goal...")

            ok = self._send_pose(pose, labels[i])
            results.append((labels[i], dist, ok))

        return results

    # ── Main ─────────────────────────────────────────────────────────
    def run(self):
        print(f"\n{'='*60}")
        print(f" PERCEPTUAL SCAN SIMULATION")
        print(f" Target:  x={self._target_x:.2f}  y={self._target_y:.2f}  z={self._target_z:.2f}")
        print(f" Mode:    {'NLF tight (4cm/6cm)' if self._nlf_active else 'YOLO wide (12cm/20cm)'}")
        print(f" Step:    {'ON (press ENTER)' if self._step_mode else 'OFF (automatic)'}")
        print(f" IK to:   {self._ik_timeout:.1f}s")
        print(f"{'='*60}")

        self._pub_en.publish(Bool(data=True))

        # Phase 1
        if not self._phase_home():
            print("\n❌ HOME failed — aborting")
            return

        # Phase 2
        if not self._phase_lookat():
            print("\n❌ LOOKAT failed — aborting")
            return

        # Phase 3
        results = self._phase_scan()
        self._pub_en.publish(Bool(data=False))

        # Summary
        reached = sum(1 for _, _, ok in results if ok)
        print(f"\n{'='*60}")
        print(f" SCAN COMPLETE: {reached}/{len(results)} poses reached")
        for label, dist, ok in results:
            print(f"   {label:>10}: dist={dist:.2f}m  "
                  f"{'✅' if ok else '❌ timeout'}")
        print(f"{'='*60}")


def main(args=None):
    rclpy.init(args=args)
    node = PerceptualScanTester()
    try:
        time.sleep(0.5)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
