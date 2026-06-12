#!/usr/bin/env python3
"""
body_pose_optimizer.py — Optimize Spot standing position for ultrasound exposure.

Given a laying-human body frame and a set of exposure-point requests,
this node evaluates candidate Spot body poses (height × pitch × workspace grid)
and selects the best standing configuration for arm reachability.
"""

import math
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Vector3Stamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
from tf_transformations import euler_from_quaternion


class RetryState(IntEnum):
    PENDING = 0
    D2_ACTIVE = 1   # 2D (h, p) goal published, waiting for IK
    D3_ACTIVE = 2   # 3D (dy_body, h, p) goal published, waiting for IK
    DONE = 4
    SKIPPED = 5


class BodyPoseOptimizer(Node):
    """Optimize Spot body pose over a height×pitch grid to maximize arm reach.

    Subscriptions:
      /laying_human/approach_point  — human-relative entry point (PoseStamped)
      /laying_human/body_axis       — human body direction (Vector3Stamped)
      /ik_done                      — IK completion signal (Bool)
      ~/optimize_request            — exposure poses to evaluate (PoseArray)

    Publishers:
      ~/optimize_result  — best body pose per request
      ~/navigator_goal   — Spot navigator goal (PoseStamped)
      ~/body_pose        — Spot body pose (Pose)
      ~/ik_goal          — arm IK goal (PoseStamped)
      ~/ik_enable        — arm IK enable (Bool)
    """

    def __init__(self):
        super().__init__('body_pose_optimizer')

        # ── Parameters ───────────────────────────────────────────────────────
        self._body_grid_heights = (
            self.declare_parameter('body_grid_heights', [-0.20, -0.18, -0.15])
            .get_parameter_value().double_array_value)
        self._body_grid_pitches = (
            self.declare_parameter('body_grid_pitches', [0.0, 0.087, 0.17, 0.26])
            .get_parameter_value().double_array_value)
        self._body_sweet_spot = (
            self.declare_parameter('body_sweet_spot', [0.35, 0.0, 0.30])
            .get_parameter_value().double_array_value)
        self._ws_ext_dx_steps = (
            self.declare_parameter('ws_ext_dx_steps', 5)
            .get_parameter_value().integer_value)
        self._ws_ext_dx_max = (
            self.declare_parameter('ws_ext_dx_max', 0.20)
            .get_parameter_value().double_value)
        self._ws_ext_dy_fwd_max = (
            self.declare_parameter('ws_ext_dy_fwd_max', 0.20)
            .get_parameter_value().double_value)
        self._ws_ext_dy_bwd_max = (
            self.declare_parameter('ws_ext_dy_bwd_max', 0.30)
            .get_parameter_value().double_value)
        self._ik_retry_timeout = (
            self.declare_parameter('ik_retry_timeout', 2.0)
            .get_parameter_value().double_value)
        self._max_workspace_reach = (
            self.declare_parameter('max_workspace_reach', 0.60)
            .get_parameter_value().double_value)
        self._mount_x = (
            self.declare_parameter('mount_x', 0.20)
            .get_parameter_value().double_value)
        self._mount_z = (
            self.declare_parameter('mount_z', 0.20)
            .get_parameter_value().double_value)
        self._spot_y_penalty = (
            self.declare_parameter('spot_y_penalty', 0.50)  # cost = dist + penalty * |dy_body|
            .get_parameter_value().double_value)

        # ── Subscribers ───────────────────────────────────────────────────────
        self._sub_approach_point = self.create_subscription(
            PoseStamped, '/laying_human/approach_point',
            self._cb_approach_point, 10)
        self._sub_body_axis = self.create_subscription(
            Vector3Stamped, '/laying_human/body_axis',
            self._cb_body_axis, 10)
        self._sub_ik_done = self.create_subscription(
            Bool, '/ik_done',
            self._cb_ik_done, 10)
        self._sub_optimize_request = self.create_subscription(
            PoseArray, '~/optimize_request',
            self._cb_optimize_request, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_optimize_result = self.create_publisher(
            PoseArray, '~/optimize_result', 10)
        self._pub_navigator_goal = self.create_publisher(
            PoseStamped, '/wbc/spot_goal', 10)
        self._pub_body_pose = self.create_publisher(
            Pose, '~/body_pose', 10)
        self._pub_ik_goal = self.create_publisher(
            PoseStamped, '~/ik_goal', 10)
        self._pub_ik_enable = self.create_publisher(
            Bool, '~/ik_enable', 10)

        # ── TF infrastructure ─────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── State ─────────────────────────────────────────────────────────────
        self._has_approach_point: bool = False
        self._has_body_axis: bool = False
        self._current_body_height: float = 0.0  # last applied body_pose height
        self._pending_requests: list = []
        self._results: dict = {}

        # ── IK-driven retry state ──────────────────────────────────────────────
        self._point_states: dict[int, RetryState] = {}
        self._point_results: dict[int, tuple] = {}
        self._ik_done_received: bool = False
        self._attempt_starts: dict[int, rclpy.time.Time] = {}
        self._target_points_odom: list[np.ndarray] = []

        # ── Timer ──────────────────────────────────────────────────────────────
        self._retry_timer = self.create_timer(0.1, self._tick_retry)

        # ── Completion tracking ────────────────────────────────────────────────
        self._results_published: bool = False

        self.get_logger().info('BodyPoseOptimizer initialized')

    # ═══════════════════════════════════════════════════════════════════════════
    #  Callbacks
    # ═══════════════════════════════════════════════════════════════════════════

    def _cb_approach_point(self, msg: PoseStamped) -> None:
        self._has_approach_point = True

    def _cb_body_axis(self, msg: Vector3Stamped) -> None:
        self._has_body_axis = True

    def _cb_ik_done(self, msg: Bool) -> None:
        self._ik_done_received = msg.data

    def _cb_optimize_request(self, msg: PoseArray) -> None:
        points_odom = []
        for pose in msg.poses:
            points_odom.append(np.array([pose.position.x,
                                          pose.position.y,
                                          pose.position.z]))
        self._process_request(points_odom)

    def _get_body_rotation(self) -> np.ndarray | None:
        """Return 3×3 rotation matrix from my_spot/odom→patient_body TF."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'my_spot/odom', 'patient_body', rclpy.time.Time())
        except TransformException:
            return None
        qx, qy, qz, qw = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        return np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
        ])

    # ═══════════════════════════════════════════════════════════════════════════
    #  IK-driven retry loop
    # ═══════════════════════════════════════════════════════════════════════════

    def _process_request(self, points_odom: list[np.ndarray]) -> None:
        self._target_points_odom = points_odom
        self._point_states.clear()
        self._point_results.clear()
        self._attempt_starts.clear()
        self._ik_done_received = False
        self._results_published = False

        # Step 1: Run 2D on all points
        results_2d, needs_escalation = self._optimize_2d(points_odom)

        # Step 2: Publish 2D goals for all points
        for idx in range(len(points_odom)):
            h, p, dist = results_2d[idx]
            self._publish_goal(idx, h, p, dx=0.0, dy=0.0)
            self._point_states[idx] = RetryState.D2_ACTIVE

        self._ik_done_received = False

    def _tick_retry(self) -> None:
        active_idx = self._first_active_idx()
        if active_idx is None:
            if self._all_points_terminal():
                self._publish_results()
            return

        state = self._point_states[active_idx]

        if self._ik_done_received:
            self._point_states[active_idx] = RetryState.DONE
            self._ik_done_received = False
            self.get_logger().info(
                f'Point[{active_idx}]: IK done at {state.name}')
            return

        if self._attempt_timed_out(active_idx):
            self._escalate_point(active_idx)

    def _first_active_idx(self) -> int | None:
        for idx, st in sorted(self._point_states.items()):
            if st in (RetryState.D2_ACTIVE, RetryState.D3_ACTIVE):
                return idx
        return None

    def _all_points_terminal(self) -> bool:
        if not self._point_states:
            return False
        return all(st in (RetryState.DONE, RetryState.SKIPPED)
                   for st in self._point_states.values())

    def _attempt_timed_out(self, idx: int) -> bool:
        start = self._attempt_starts.get(idx)
        if start is None:
            return False
        elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
        return elapsed > self._ik_retry_timeout

    def _escalate_point(self, idx: int) -> None:
        state = self._point_states[idx]

        if state == RetryState.D2_ACTIVE:
            results_3d = self._optimize_3d(self._target_points_odom, {idx})
            if idx in results_3d:
                dy, h, p, dist = results_3d[idx]
                self._publish_goal(idx, h, p, dx=0.0, dy=dy)
                self._point_states[idx] = RetryState.D3_ACTIVE
                self.get_logger().info(
                    f'Point[{idx}]: escalated 2D→3D (dy={dy:.2f}m, '
                    f'h={h:.2f}m, p={math.degrees(p):.1f}°)')
            else:
                self._point_states[idx] = RetryState.SKIPPED
                self.get_logger().warn(
                    f'Point[{idx}]: 3D optimisation returned no result → SKIPPED')

        elif state == RetryState.D3_ACTIVE:
            self._point_states[idx] = RetryState.SKIPPED
            self.get_logger().warn(
                f'Point[{idx}]: 3D attempt timed out → SKIPPED')

        self._ik_done_received = False

    def _publish_goal(self, idx: int, h: float, p: float,
                       dx: float, dy: float) -> None:
        body_tf = self._tf_buffer.lookup_transform(
            'my_spot/odom', 'my_spot/body', rclpy.time.Time())
        quat = body_tf.transform.rotation
        _, _, body_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        body_pos = np.array([body_tf.transform.translation.x,
                              body_tf.transform.translation.y,
                              body_tf.transform.translation.z])

        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            body_rot = self._get_body_rotation()
            if body_rot is not None:
                odom_disp = body_rot @ np.array([dx, dy, 0.0])
            else:
                odom_disp = np.array([dx, dy, 0.0])
            nav_pos = body_pos + odom_disp

            nav_goal = PoseStamped()
            nav_goal.header.stamp = self.get_clock().now().to_msg()
            nav_goal.header.frame_id = 'my_spot/odom'
            nav_goal.pose.position.x = float(nav_pos[0])
            nav_goal.pose.position.y = float(nav_pos[1])
            nav_goal.pose.position.z = float(nav_pos[2])
            nav_goal.pose.orientation = quat
            self._pub_navigator_goal.publish(nav_goal)

        body_pose = Pose()
        body_pose.position.z = h
        sp = math.sin(p / 2.0)
        cp = math.cos(p / 2.0)
        body_pose.orientation.x = 0.0
        body_pose.orientation.y = sp
        body_pose.orientation.z = 0.0
        body_pose.orientation.w = cp
        self._pub_body_pose.publish(body_pose)

        target = self._target_points_odom[idx]
        ik_goal = PoseStamped()
        ik_goal.header.stamp = self.get_clock().now().to_msg()
        ik_goal.header.frame_id = 'world'
        ik_goal.pose.position.x = float(target[0])
        ik_goal.pose.position.y = float(target[1])
        ik_goal.pose.position.z = float(target[2])
        ik_goal.pose.orientation.w = 1.0
        self._pub_ik_goal.publish(ik_goal)

        self._pub_ik_enable.publish(Bool(data=True))
        self._attempt_starts[idx] = self.get_clock().now()
        self._point_results[idx] = (dx, dy, h, p)

    def _publish_results(self) -> None:
        if self._results_published:
            return
        result_msg = PoseArray()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = 'my_spot/odom'
        for idx in sorted(self._point_results.keys()):
            dx, dy, h, p = self._point_results[idx]
            pose = Pose()
            pose.position.x = float(h)
            pose.position.y = float(p)
            pose.position.z = float(dx)
            pose.orientation.w = float(dy)
            result_msg.poses.append(pose)
        self._pub_optimize_result.publish(result_msg)
        self._results_published = True
        self.get_logger().info(
            f'Results published: {len(result_msg.poses)} point(s)')

    # ═══════════════════════════════════════════════════════════════════════════
    #  2D body-pose optimisation (h, p) grid search
    # ═══════════════════════════════════════════════════════════════════════════

    def _simulate_link00(self, body_pos: np.ndarray, body_yaw: float,
                         height: float, pitch: float) -> tuple:
        """Compute link00 position + rotation in odom for a given body config.

        Returns (link00_odom_xyz, R_body_odom).
        body_pos: current body [x,y,z] in odom (at current body_pose height).
        height: desired body_pose height offset (negative = lower).
        pitch: desired body_pose pitch [rad].
        """
        body_nominal_z = (float(body_pos[2]) - self._current_body_height
                          if hasattr(self, '_current_body_height')
                          else float(body_pos[2]))
        body_new_z = body_nominal_z + height
        t_body = np.array([float(body_pos[0]), float(body_pos[1]), body_new_z])

        R_yaw = np.array([[math.cos(body_yaw), -math.sin(body_yaw), 0.0],
                          [math.sin(body_yaw),  math.cos(body_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        R_pitch = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                            [0.0, 1.0, 0.0],
                            [-math.sin(pitch), 0.0, math.cos(pitch)]])
        R_body = R_yaw @ R_pitch

        mount = np.array([self._mount_x, 0.0, self._mount_z])
        link00_odom = t_body + R_body @ mount
        return link00_odom, R_body

    def _odom_to_link00_vec(self, point_odom: np.ndarray,
                             link00_odom: np.ndarray,
                             body_yaw: float, pitch: float) -> np.ndarray:
        """Transform a point from odom to the link00 frame for given body config."""
        R_yaw = np.array([[math.cos(body_yaw), -math.sin(body_yaw), 0.0],
                          [math.sin(body_yaw),  math.cos(body_yaw), 0.0],
                          [0.0, 0.0, 1.0]])
        R_pitch = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                            [0.0, 1.0, 0.0],
                            [-math.sin(pitch), 0.0, math.cos(pitch)]])
        R_body = R_yaw @ R_pitch
        return R_body.T @ (point_odom - link00_odom)

    def _optimize_2d(self, target_points_odom: list[np.ndarray]) -> tuple:
        """Grid search: for each odom target, find optimal (h, p).

        Returns (results, needs_escalation) where:
          results: list of (h, p, dist) tuples, one per input point
          needs_escalation: set of point indices with dist > max_workspace_reach
        """
        heights = self._body_grid_heights
        pitches = self._body_grid_pitches
        sweet = np.array(self._body_sweet_spot)

        body_tf = self._tf_buffer.lookup_transform(
            'my_spot/odom', 'my_spot/body', rclpy.time.Time())
        quat = body_tf.transform.rotation
        _, _, body_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        body_pos = np.array([body_tf.transform.translation.x,
                             body_tf.transform.translation.y,
                             body_tf.transform.translation.z])

        results = []
        needs_escalation = set()

        for idx, target_odom in enumerate(target_points_odom):
            best_h, best_p, best_dist = 0.0, 0.0, float('inf')
            for h in heights:
                for p in pitches:
                    link00_odom, _ = self._simulate_link00(body_pos, body_yaw, h, p)
                    target_link00 = self._odom_to_link00_vec(
                        target_odom, link00_odom, body_yaw, p)
                    dist = float(np.linalg.norm(target_link00 - sweet))
                    if dist < best_dist:
                        best_h, best_p, best_dist = h, p, dist

            results.append((best_h, best_p, best_dist))
            self.get_logger().info(
                f'Point[{idx}]: best (h={best_h:.2f}m, p={math.degrees(best_p):.1f}°)'
                f' -> sweet_dist={best_dist:.3f}m')

            if best_dist > self._max_workspace_reach:
                needs_escalation.add(idx)

        return results, needs_escalation

    # ═══════════════════════════════════════════════════════════════════════════
    #  3D body-pose optimisation (dy_body, h, p) grid search
    # ═══════════════════════════════════════════════════════════════════════════

    def _optimize_3d(self, target_points_odom: list[np.ndarray],
                     needs_escalation: set) -> dict[int, tuple]:
        """Grid search adding body-frame lateral displacement for escalated points.

        For each point whose best (h, p) still exceeds max_workspace_reach,
        evaluates dy_body × h × p combinations. dy_body displaces Spot
        along the patient body-long axis (body-frame Y), converted to odom.

        Args:
            target_points_odom:  List of target exposure points in odom frame.
            needs_escalation:    Set of 0-based point indices needing 3D search.

        Returns:
            dict mapping idx → (dy_body, h, p, dist) for escalated points.
        """
        body_rot = self._get_body_rotation()
        if body_rot is None:
            self.get_logger().warn('_optimize_3d: body frame TF not available')
            return {}

        dy_body_values = np.arange(-0.68, 0.73, 0.10)  # 15 values, ±0.68 m
        heights = self._body_grid_heights
        pitches = self._body_grid_pitches
        sweet = np.array(self._body_sweet_spot)

        body_tf = self._tf_buffer.lookup_transform(
            'my_spot/odom', 'my_spot/body', rclpy.time.Time())
        quat = body_tf.transform.rotation
        _, _, body_yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        body_pos = np.array([body_tf.transform.translation.x,
                             body_tf.transform.translation.y,
                             body_tf.transform.translation.z])

        results: dict[int, tuple] = {}

        for idx in needs_escalation:
            target_odom = target_points_odom[idx]
            best_dy, best_h, best_p, best_dist = 0.0, 0.0, 0.0, float('inf')
            best_cost = float('inf')

            for dy_body in dy_body_values:
                # Convert body-frame Y displacement to odom-frame vector
                odom_disp = body_rot @ np.array([0.0, dy_body, 0.0])
                shifted_pos = body_pos + odom_disp

                for h in heights:
                    for p in pitches:
                        link00_odom, _ = self._simulate_link00(
                            shifted_pos, body_yaw, h, p)
                        target_link00 = self._odom_to_link00_vec(
                            target_odom, link00_odom, body_yaw, p)
                        dist = float(np.linalg.norm(target_link00 - sweet))
                        cost = dist + self._spot_y_penalty * abs(dy_body)
                        if cost < best_cost:
                            best_cost = cost
                            best_dist = dist
                            best_dy = float(dy_body)
                            best_h = float(h)
                            best_p = float(p)
                            best_dist = dist

            results[idx] = (best_dy, best_h, best_p, best_dist)
            self.get_logger().info(
                f'3D[{idx}]: dy_body={best_dy:.2f}m, h={best_h:.2f}m, '
                f'p={math.degrees(best_p):.1f}°, dist={best_dist:.3f}m')

        return results
def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
