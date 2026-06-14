#!/usr/bin/env python3
"""
Test script for 7 search poses — manual stepping.
Publishes one pose at a time to /ik_goal_pose, waits for ik_done,
then waits for ENTER before sending the next.

Usage:
    ros2 run spot_control test_search_poses
    # or: python3 scripts/test_search_poses.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Header
import numpy as np

# ============================================================
# 7 Search Poses (from wbc_qp_controller.py)
# Interleave FWD-C between behind poses to prevent IK wrist-path issues.
# Final FWD-C returns arm to center before Spot changes yaw.
# ============================================================
SEARCH_POSES = [
    # FORWARD (10° camera tilt)
    ("FWD-C",   [0.144, -0.005, 0.52], [0.0162, 0.2376, -0.0232, 0.9709]),
    ("FWD-L",   [0.067, -0.070, 0.52], [0.0556, 0.2662, -0.4040, 0.8734]),
    # BEHIND LEFT (via forward)
    ("BWD-L",   [-0.052, -0.042, 0.52], [-0.1279, -0.2016, 0.9247, -0.2966]),
    # BEHIND CENTER
    ("BWD-C",   [-0.075, -0.013, 0.52], [-0.0659, -0.1086, 0.9919, 0.0095]),
    # TRANSIT to center
    ("FWD-C⤓", [0.144, -0.005, 0.52], [0.0162, 0.2376, -0.0232, 0.9709]),
    # BEHIND RIGHT
    ("BWD-R",   [-0.077, 0.071, 0.52], [-0.0333, 0.0390, 0.9380, 0.3427]),
    # RETURN to center (before Spot changes yaw)
    ("FWD-C",   [0.144, -0.005, 0.52], [0.0162, 0.2376, -0.0232, 0.9709]),
]


class SearchPoseTester(Node):
    def __init__(self):
        super().__init__("test_search_poses")
        self._pub_goal = self.create_publisher(PoseStamped, "/ik_goal_pose", 10)
        self._pub_enable = self.create_publisher(Bool, "/ik_enable", 10)
        self.create_subscription(Bool, "/ik_done", self._cb_ik_done, 10)
        self._ik_done = False
        self._current_idx = 0
        self._running = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("SEARCH POSE TESTER — press ENTER to step through poses")
        self.get_logger().info("=" * 60)
        for i, (name, pos, quat) in enumerate(SEARCH_POSES):
            self.get_logger().info(f"  {i+1}. {name:6s}  pos={pos}  quat={[round(q,4) for q in quat]}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Commands: ENTER=next pose  p=pause  r=resume  h=home  q=quit")
        self.get_logger().info("")

    def _cb_ik_done(self, msg: Bool):
        if msg.data:
            self._ik_done = True

    def _send_pose(self, idx):
        name, pos, quat = SEARCH_POSES[idx]
        msg = PoseStamped()
        msg.header.frame_id = "link00"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        self._pub_enable.publish(Bool(data=True))
        self._pub_goal.publish(msg)
        self._ik_done = False
        self.get_logger().info(f"▶ [{idx+1}/{len(SEARCH_POSES)}] {name} — pos={[round(p,3) for p in pos]}  waiting for ik_done...")

    def _send_home(self):
        msg = PoseStamped()
        msg.header.frame_id = "link00"
        msg.pose.position.x = 0.144
        msg.pose.position.y = -0.005
        msg.pose.position.z = 0.52
        msg.pose.orientation.x = 0.0162
        msg.pose.orientation.y = 0.2376
        msg.pose.orientation.z = -0.0232
        msg.pose.orientation.w = 0.9709
        self._pub_enable.publish(Bool(data=True))
        self._pub_goal.publish(msg)
        self._ik_done = False
        self.get_logger().info(f"🏠 HOME sent [0.144, -0.005, 0.52]")

    def spin(self):
        self._running = True
        self.get_logger().info("Press ENTER to send first pose...")

        import sys, select, termios, tty
        old_settings = None
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass  # non-interactive mode

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                # Check ik_done
                if self._ik_done and self._current_idx < len(SEARCH_POSES):
                    self.get_logger().info(f"  ✅ ik_done received — press ENTER for next pose")

                # Check keyboard
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == '\n' or key == '\r':  # ENTER
                            if self._ik_done or self._current_idx == 0:
                                if self._current_idx < len(SEARCH_POSES):
                                    self._send_pose(self._current_idx)
                                    self._current_idx += 1
                                else:
                                    self.get_logger().info("✅ All poses done! Press ENTER to go HOME, q to quit")
                            else:
                                self.get_logger().warn("⚠️  ik_done not received yet — wait for arm to finish")
                        elif key == 'p':
                            self.get_logger().info("⏸  PAUSED — press r to resume")
                            self._running = False
                        elif key == 'r':
                            self._running = True
                            self.get_logger().info("▶ RESUMED")
                        elif key == 'h':
                            self._send_home()
                            self.get_logger().info("🏠 HOME — press ENTER to restart from pose 1")
                            self._current_idx = 0
                        elif key == 'q':
                            self.get_logger().info("👋 Quit")
                            self._running = False
                except Exception:
                    pass

        finally:
            if old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = SearchPoseTester()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
