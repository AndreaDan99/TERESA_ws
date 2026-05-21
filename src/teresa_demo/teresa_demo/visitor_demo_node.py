#!/usr/bin/env python3
"""
Visitor Demo Node
=================
Orchestrates simultaneous Spot body_pose grid + Z1 arm pose cycling.
No cameras, no WBC, no perception — pure coordinated motion demo.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class _ArmState:
    INIT          = 'INIT'
    SEND_HOME     = 'SEND_HOME'
    WAIT_IK_HOME  = 'WAIT_IK_HOME'
    HOME_PAUSE    = 'HOME_PAUSE'
    SEND_POSE     = 'SEND_POSE'
    WAIT_IK_POSE  = 'WAIT_IK_POSE'
    POSE_PAUSE    = 'POSE_PAUSE'


class VisitorDemoNode(Node):

    def __init__(self):
        super().__init__('visitor_demo')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('search_body_height',       -0.20)
        self.declare_parameter('search_pitch_angles',      [0.087, 0.17, 0.26])
        self.declare_parameter('search_yaw_offsets',       [0.0, 0.17, -0.17])
        self.declare_parameter('search_pause_per_point',    3.0)

        self.declare_parameter('arm_home',  [0.0767, 0.0006, 0.3131,
                                             -0.0062, 0.4107, 0.0021, 0.9118])
        self.declare_parameter('arm_poses', [])
        self.declare_parameter('arm_pause',       2.0)
        self.declare_parameter('arm_ik_timeout', 15.0)
        self.declare_parameter('ik_goal_topic',   '/ik_goal_pose')
        self.declare_parameter('ik_enable_topic', '/ik_enable')
        self.declare_parameter('ik_done_topic',   '/ik_done')

        p = lambda n: self.get_parameter(n).value

        self._search_body_height    = float(p('search_body_height'))
        self._search_pitch_angles   = [float(v) for v in p('search_pitch_angles')]
        self._search_yaw_offsets    = [float(v) for v in p('search_yaw_offsets')]
        self._search_pause_per_point = float(p('search_pause_per_point'))

        self._arm_home = self._parse_pose(p('arm_home'))
        raw_poses = p('arm_poses')
        self._arm_poses = [self._parse_pose(rp) for rp in raw_poses] if raw_poses else []
        self._arm_pause = float(p('arm_pause'))
        self._arm_timeout = float(p('arm_ik_timeout'))

        # ── Spot grid ─────────────────────────────────────────────────────
        self._grid = []
        for yaw in self._search_yaw_offsets:
            for pitch in self._search_pitch_angles:
                self._grid.append((pitch, yaw))
        self._grid_idx = 0

        # ── Arm state machine ─────────────────────────────────────────────
        self._arm_state = _ArmState.INIT
        self._arm_pose_idx = 0
        self._arm_state_t0 = None
        self._ik_done = False
        self._arm_ik_t0 = None
        self._have_js = False

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_body_pose = self.create_publisher(Pose, '/my_spot/body_pose', 10)
        self._pub_cmd_vel   = self.create_publisher(Twist, '/my_spot/cmd_vel', 10)
        self._pub_ik_goal   = self.create_publisher(
            PoseStamped, p('ik_goal_topic'), 10)
        self._pub_ik_enable = self.create_publisher(
            Bool, p('ik_enable_topic'), 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self._sub_ik_done = self.create_subscription(
            Bool, p('ik_done_topic'), self._cb_ik_done, 10)
        self._sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self._spot_timer = self.create_timer(
            self._search_pause_per_point, self._spot_tick)
        self._arm_timer = self.create_timer(0.5, self._arm_tick)

        self.get_logger().info(
            f'Visitor Demo ready.\n'
            f'  Spot: {len(self._grid)} grid points x '
            f'{self._search_pause_per_point}s = '
            f'{len(self._grid) * self._search_pause_per_point}s cycle\n'
            f'  Arm:  {len(self._arm_poses)} poses + home, '
            f'pause={self._arm_pause}s\n'
            f'  Waiting for /joint_states before starting arm...')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_ik_done(self, msg: Bool) -> None:
        if msg.data:
            self._ik_done = True

    def _cb_js(self, msg: JointState) -> None:
        if not self._have_js:
            self._have_js = True
            self.get_logger().info(
                '/joint_states received — arm ready to start')

    # ── Spot loop (timer) ─────────────────────────────────────────────────

    def _spot_tick(self) -> None:
        pitch, yaw = self._grid[self._grid_idx]
        self._set_body_pose(self._search_body_height, pitch, yaw)
        self._grid_idx = (self._grid_idx + 1) % len(self._grid)

    # ── Arm state machine (timer) ─────────────────────────────────────────

    def _arm_tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._arm_state == _ArmState.INIT:
            if self._have_js:
                self.get_logger().info('Arm state machine starting')
                self._arm_state = _ArmState.SEND_HOME

        elif self._arm_state == _ArmState.SEND_HOME:
            self._send_ik_goal(self._arm_home, self._arm_home[3:])
            self._arm_ik_t0 = now
            self._arm_state = _ArmState.WAIT_IK_HOME

        elif self._arm_state == _ArmState.WAIT_IK_HOME:
            if self._ik_done:
                self._arm_state_t0 = now
                self._arm_state = _ArmState.HOME_PAUSE
            elif now - self._arm_ik_t0 > self._arm_timeout:
                self.get_logger().warn('Home IK timeout — retrying')
                self._arm_state = _ArmState.SEND_HOME

        elif self._arm_state == _ArmState.HOME_PAUSE:
            if now - self._arm_state_t0 >= self._arm_pause:
                if self._arm_poses:
                    self._arm_state = _ArmState.SEND_POSE
                else:
                    self._arm_state = _ArmState.SEND_HOME

        elif self._arm_state == _ArmState.SEND_POSE:
            pose = self._arm_poses[self._arm_pose_idx]
            self._send_ik_goal(pose, pose[3:])
            self._arm_ik_t0 = now
            self._arm_state = _ArmState.WAIT_IK_POSE

        elif self._arm_state == _ArmState.WAIT_IK_POSE:
            if self._ik_done:
                self._arm_state_t0 = now
                self._arm_state = _ArmState.POSE_PAUSE
            elif now - self._arm_ik_t0 > self._arm_timeout:
                self.get_logger().warn(
                    f'IK timeout pose {self._arm_pose_idx} — skipping')
                self._arm_pose_idx = (self._arm_pose_idx + 1) % len(self._arm_poses)
                self._arm_state = _ArmState.SEND_HOME

        elif self._arm_state == _ArmState.POSE_PAUSE:
            if now - self._arm_state_t0 >= self._arm_pause:
                self._arm_pose_idx = (self._arm_pose_idx + 1) % len(self._arm_poses)
                self._arm_state = _ArmState.SEND_HOME

    # ── Helpers ────────────────────────────────────────────────────────────

    def _send_ik_goal(self, pose, quat) -> None:
        self._ik_done = False

        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(pose[0])
        goal.pose.position.y = float(pose[1])
        goal.pose.position.z = float(pose[2])
        goal.pose.orientation.x = float(quat[0])
        goal.pose.orientation.y = float(quat[1])
        goal.pose.orientation.z = float(quat[2])
        goal.pose.orientation.w = float(quat[3])

        self._pub_ik_goal.publish(goal)
        self._pub_ik_enable.publish(Bool(data=True))

        self.get_logger().info(
            f'Arm → [{float(pose[0]):.3f}, {float(pose[1]):.3f}, '
            f'{float(pose[2]):.3f}]')

    def _set_body_pose(self, height: float, pitch: float, yaw: float) -> None:
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, pitch, yaw)
        pose = Pose()
        pose.position.z = float(height)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self._pub_body_pose.publish(pose)
        self._pub_cmd_vel.publish(Twist())
        self.get_logger().info(
            f'Spot → height={height:.2f}m  pitch={math.degrees(pitch):.0f}°  '
            f'yaw={math.degrees(yaw):.0f}°')

    def _parse_pose(self, arr) -> list:
        return [float(v) for v in arr]


def main(args=None):
    rclpy.init(args=args)
    node = VisitorDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
