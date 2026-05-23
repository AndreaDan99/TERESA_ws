#!/usr/bin/env python3
"""
WBC Approach Scanner — multi-view body scan + WBC look-at during motion.

During APPROACHING:
  Phase 1: home position with wrist sweep (±8°) — find torso center
  Phase 2: arc grid (±4cm) with wrist — multi-view 3D mapping
  Each point oriented with WBC look-at toward torso estimate.

During SCANNING (only if keypoints have low confidence):
  Phase 3: adaptive refinement grid (±5cm lateral or ±10cm body-axis).

Subscribes to /torso_scan_point for BodySearchScanner data feed.
Uses real BodySearchScanner + ScanManager (same logic as FSM).
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_matrix, quaternion_from_matrix

from teresa_utils.orientation import compute_ee_orientation

from z1_vision.body_search_scanner import BodySearchScanner, ScanAction, ScanTick
from z1_vision.z1_scan_manager import ScanManager


# ── Reduced motion parameters (Spot is moving during scan) ────────────
WRIST_NY = 2             # wrist grid Y steps
WRIST_NZ = 2             # wrist grid Z steps
WRIST_ANG_DEG = 8.0      # ± wrist angle [deg]

ARC_EXT_Y = 0.04          # arc Y offset [m]  (head→feet axis)
ARC_EXT_X = 0.04          # arc X offset [m]  (lateral)
ARC_NY = 2                # arc grid Y points
ARC_NX = 2                # arc grid X points
ARC_WRIST_NY = 1          # wrist grid per arc point
ARC_WRIST_NZ = 1

P3_SKIP_THR   = 0.50      # min KP confidence to skip phase 3
P3_ASYM_THR   = 0.15      # asymmetry threshold
P3_EXT_X      = 0.05      # lateral offset [m]
P3_EXT_Y      = 0.10      # body-axis offset [m]
P3_FAR_HIP_Z  = 0.03      # vertical offset [m]

SCAN_POINT_TIMEOUT = 4.0  # [s]
SCAN_MIN_FRAMES    = 5
SCAN_EARLY_STOP    = 0.95
SCAN_STABILITY_K   = 10.0

HOME_POS = np.array([-0.09, 0.0, 0.44])
HOME_ORI = np.array([-0.0062, 0.4107, 0.0021, 0.9118])


def _make_pose_stamped(pos: np.ndarray, orientation: np.ndarray,
                       frame_id: str = 'world') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x    = float(pos[0])
    p.pose.position.y    = float(pos[1])
    p.pose.position.z    = float(pos[2])
    p.pose.orientation.x = float(orientation[0])
    p.pose.orientation.y = float(orientation[1])
    p.pose.orientation.z = float(orientation[2])
    p.pose.orientation.w = float(orientation[3])
    return p


def _wrist_poses_at(pos: np.ndarray, R_base: np.ndarray,
                    ny: int, nz: int, ang_deg: float) -> list:
    """Generate wrist-angle grid poses at given position with base rotation."""
    ang = math.radians(ang_deg)
    alphas = np.linspace(-ang, ang, ny) if ny > 1 else np.array([0.0])
    betas  = np.linspace(-ang, ang, nz) if nz > 1 else np.array([0.0])
    poses = []
    for alpha in alphas:
        ca, sa = math.cos(alpha), math.sin(alpha)
        Ry = np.array([[ca, 0, sa], [0, 1, 0], [-sa, 0, ca]])
        for beta in betas:
            cb, sb = math.cos(beta), math.sin(beta)
            Rz = np.array([[cb, -sb, 0], [sb, cb, 0], [0, 0, 1]])
            R_new = R_base @ Ry @ Rz
            T = np.eye(4); T[:3, :3] = R_new
            q = quaternion_from_matrix(T)
            poses.append(_make_pose_stamped(pos, q))
    return poses


class WBCApproachScanner(Node):

    def __init__(self):
        super().__init__('wbc_approach_scanner')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('z1_base_frame', 'world')
        self.declare_parameter('home_orientation', HOME_ORI.tolist())
        self.declare_parameter('home_position', HOME_POS.tolist())

        p = lambda n: self.get_parameter(n).value
        self._scan_rate   = float(p('scan_rate'))
        self._z1_base_frame = p('z1_base_frame')
        self._home_pos    = np.array([float(x) for x in p('home_position')])
        self._home_ori    = np.array([float(x) for x in p('home_orientation')])

        # ── TF ─────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── Subscriptions ──────────────────────────────────────────────
        self.create_subscription(Bool, '/wbc/enable', self._cb_enable, 10)
        self.create_subscription(PoseStamped, '/torso_target_ee',
                                 self._cb_torso, 10)
        self.create_subscription(Bool, '/ik_done', self._cb_ik_done, 10)
        self.create_subscription(String, '/wbc/state', self._cb_wbc_state, 10)
        self.create_subscription(Float32MultiArray, '/torso_scan_point',
                                 self._cb_scan_data, 10)

        # ── Publishers ─────────────────────────────────────────────────
        self._pub_ik     = self.create_publisher(PoseStamped, '/wbc/ik_goal_pose', 10)
        self._pub_enable = self.create_publisher(Bool, '/wbc/ik_enable', 10)
        self._pub_fast   = self.create_publisher(PoseArray, '/z1/fast_points', 10)
        self._pub_ready  = self.create_publisher(Bool, '/z1/fast_ready', 10)

        # ── State ──────────────────────────────────────────────────────
        self._enabled   = False
        self._phase     = 'IDLE'   # IDLE | ARC_GRID | WAIT_PHASE3 | PHASE_3 | DONE
        self._torso_pos = None     # latest torso in world frame
        self._ik_done   = False

        self._scanner: BodySearchScanner | None = None
        self._scan_mgr: ScanManager | None = None
        self._scan_torso_estimate: np.ndarray | None = None
        self._kp_stats: dict = {}
        self._pending_data: list[float] = []
        self._phase3_needed = False

        self.get_logger().info('WBC Approach Scanner ready.')

    # ── Callbacks ──────────────────────────────────────────────────────

    def _cb_enable(self, msg: Bool) -> None:
        if msg.data and not self._enabled:
            self._enabled = True
            self._start_arc_grid()
        elif not msg.data:
            self._enabled = False

    def _cb_torso(self, msg: PoseStamped) -> None:
        try:
            tf = self._tf.lookup_transform(
                self._z1_base_frame, msg.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            import tf2_geometry_msgs
            t = tf2_geometry_msgs.do_transform(msg, tf)
            self._torso_pos = np.array([
                t.pose.position.x, t.pose.position.y, t.pose.position.z])
        except TransformException:
            pass

    def _cb_ik_done(self, msg: Bool) -> None:
        self._ik_done = msg.data

    def _cb_wbc_state(self, msg: String) -> None:
        if msg.data == 'SCANNING' and self._phase == 'WAIT_PHASE3':
            if self._phase3_needed:
                self._start_phase3()

    def _cb_scan_data(self, msg: Float32MultiArray) -> None:
        self._pending_data.append(list(msg.data))

    # ── Phase control ──────────────────────────────────────────────────

    def _start_arc_grid(self) -> None:
        """Build scan poses: home wrist sweep + arc grid with wrist."""
        self._phase = 'ARC_GRID'
        self._ik_done = False
        self._phase3_needed = False
        self._kp_stats = {}
        self._scan_torso_estimate = None

        poses = self._gen_arc_grid_poses()
        if not poses:
            self.get_logger().warn('No scan poses — DONE')
            self._phase = 'DONE'
            return

        self._scanner = BodySearchScanner(
            scan_poses=poses,
            scan_point_timeout=SCAN_POINT_TIMEOUT,
            scan_min_frames=SCAN_MIN_FRAMES,
            early_stop_score=SCAN_EARLY_STOP,
            logger=self.get_logger(),
            stability_k=SCAN_STABILITY_K,
        )
        self._scanner.reset()
        self._scan_mgr = ScanManager.from_params(self)
        self.get_logger().info(f'ARC_GRID: {len(poses)} scan poses')

    def _start_phase3(self) -> None:
        if self._scan_torso_estimate is None:
            self._publish_fast_points()
            return

        poses_p3, transit = self._gen_phase3_poses()
        if poses_p3 is None:
            self.get_logger().info('All keypoints visible — skipping phase 3')
            self._publish_fast_points()
            return

        self._phase = 'PHASE_3'
        self._ik_done = False
        self._scanner = BodySearchScanner(
            scan_poses=poses_p3,
            scan_point_timeout=SCAN_POINT_TIMEOUT,
            scan_min_frames=SCAN_MIN_FRAMES,
            early_stop_score=SCAN_EARLY_STOP,
            logger=self.get_logger(),
            transit_indices=transit if transit else None,
            stability_k=SCAN_STABILITY_K,
        )
        self._scanner.reset()
        self.get_logger().info(
            f'PHASE_3: {len(poses_p3)} poses ({len(transit or set())} transit)')

    # ── Pose generation ────────────────────────────────────────────────

    def _gen_arc_grid_poses(self) -> list:
        """Phase 1: home wrist sweep, Phase 2: arc grid with wrist."""
        center = self._home_pos
        torso = (self._torso_pos if self._torso_pos is not None
                 else center + np.array([0.35, 0, 0]))
        R_home = quaternion_matrix(self._home_ori)[:3, :3]

        poses: list = []

        # Phase 1: home position with wrist sweep (±8°, 2×2 = 4 poses)
        poses.extend(_wrist_poses_at(center, R_home,
                     WRIST_NY, WRIST_NZ, WRIST_ANG_DEG))

        # Phase 2: arc grid (±4cm) with look-at + wrist per point
        ys = np.linspace(center[1] - ARC_EXT_Y, center[1] + ARC_EXT_Y, ARC_NY)
        xs = np.linspace(center[0], center[0] + ARC_EXT_X, ARC_NX)
        for x in xs:
            for y in ys:
                pos = np.array([float(x), float(y), float(center[2])])
                d = torso - pos
                norm = float(np.linalg.norm(d))
                if norm < 1e-6:
                    R_base = R_home
                else:
                    q_base = compute_ee_orientation(d / norm,
                                                    self._home_ori.tolist())
                    R_base = quaternion_matrix(q_base)[:3, :3]
                poses.extend(_wrist_poses_at(
                    pos, R_base, ARC_WRIST_NY, ARC_WRIST_NZ, WRIST_ANG_DEG))
        return poses

    def _gen_phase3_poses(self) -> tuple[list | None, set | None]:
        """Adaptive refinement for unobserved keypoints."""
        if self._scan_torso_estimate is None:
            return None, None
        stats = self._kp_stats
        if not stats:
            return None, None

        shoulders = stats.get('shoulders', 1.0)
        hips      = stats.get('hips', 1.0)
        per_kp    = stats.get('per_kp', np.zeros(4))
        kp5  = float(per_kp[0])
        kp6  = float(per_kp[1])
        kp11 = float(per_kp[2])
        kp12 = float(per_kp[3])

        if shoulders > P3_SKIP_THR and hips > P3_SKIP_THR:
            return None, None

        torso = self._scan_torso_estimate

        if hips <= P3_SKIP_THR:
            far_hidden = kp11 < kp12 - P3_ASYM_THR
            offset_z = P3_FAR_HIP_Z if far_hidden else 0.0
            pos = np.array([
                float(self._home_pos[0]),
                float(self._home_pos[1]) + P3_EXT_Y,
                float(self._home_pos[2]) + offset_z,
            ])
            lookat = torso + np.array([0.0, P3_EXT_Y, 0.0])
        else:
            asym_x = kp6 - kp5
            offset_x = np.sign(asym_x) * P3_EXT_X if abs(asym_x) >= P3_ASYM_THR else 0.0
            pos = np.array([
                float(self._home_pos[0]) + offset_x,
                float(self._home_pos[1]),
                float(self._home_pos[2]),
            ])
            lookat = torso

        d = lookat - pos
        norm = float(np.linalg.norm(d))
        q = (self._home_ori if norm < 1e-6
             else compute_ee_orientation(d / norm, self._home_ori.tolist()))

        home_p = _make_pose_stamped(self._home_pos, self._home_ori)
        scan_p = _make_pose_stamped(pos, q)
        transit = {0}
        return [home_p, scan_p], transit

    # ── Tick ───────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if not self._enabled or self._phase not in ('ARC_GRID', 'PHASE_3'):
            return
        if self._scanner is None:
            return

        for data in self._pending_data:
            self._scanner.feed_scan_data(data)
        self._pending_data.clear()

        now = self.get_clock().now().nanoseconds * 1e-9
        st: ScanTick = self._scanner.tick(ik_done=self._ik_done, now=now)

        if st.action == ScanAction.SEND_IK and st.goal is not None:
            self._ik_done = False
            self._pub_ik.publish(st.goal)
            self._pub_enable.publish(Bool(data=True))

        elif st.action in (ScanAction.EXIT_SCAN_MODE, ScanAction.DONE):
            self._pub_enable.publish(Bool(data=False))
            self._finish_phase()

        elif st.action == ScanAction.FAILED:
            self.get_logger().warn('Scan FAILED')
            self._pub_enable.publish(Bool(data=False))
            self._finish_phase()

        # Follow torso movement — update remaining poses' look-at
        if (self._torso_pos is not None
                and self._scan_torso_estimate is not None
                and self._phase in ('ARC_GRID', 'PHASE_3')):
            drift = float(np.linalg.norm(
                self._torso_pos - self._scan_torso_estimate))
            if drift > 0.03:
                try:
                    self._scanner.update_remaining_lookat(
                        self._torso_pos,
                        lambda x_ee: compute_ee_orientation(
                            x_ee, self._home_ori.tolist()))
                except Exception:
                    pass
                self._scan_torso_estimate = self._torso_pos.copy()

    # ── Phase completion ───────────────────────────────────────────────

    def _finish_phase(self) -> None:
        if self._scanner is None:
            return

        if self._phase == 'ARC_GRID':
            torso = self._scanner.fused_torso_xyz()
            if torso is not None:
                self._scan_torso_estimate = torso
                self._kp_stats = self._scanner.kp_visibility_stats()
                self._phase3_needed = (
                    self._kp_stats.get('shoulders', 1.0) <= P3_SKIP_THR or
                    self._kp_stats.get('hips', 1.0) <= P3_SKIP_THR
                )
            self._scanner = None
            self._phase = 'WAIT_PHASE3'
            self.get_logger().info(
                f'ARC_GRID done.  Needs phase 3: {self._phase3_needed}.')

        elif self._phase == 'PHASE_3':
            self._scanner = None
            self._publish_fast_points()

    def _publish_fast_points(self) -> None:
        self._phase = 'DONE'
        self._enabled = False

        if self._scan_torso_estimate is None:
            self.get_logger().warn('No scan data — empty FAST')
            self._pub_fast.publish(PoseArray())
            self._pub_ready.publish(Bool(data=True))
            return

        torso = self._scan_torso_estimate

        fast = PoseArray()
        fast.header.frame_id = self._z1_base_frame
        offsets = [
            (0.00, 0.00),          # Hub
            (0.00, -0.08),         # Subxiphoid
            (-0.05, -0.04),        # RUQ
            (0.05, -0.04),         # LUQ
            (0.00, 0.10),          # Suprapubic
        ]
        for dx, dy in offsets:
            p = Pose()
            p.position.x = float(torso[0] + dx)
            p.position.y = float(torso[1] + dy)
            p.position.z = float(torso[2])
            p.orientation.w = 1.0
            fast.poses.append(p)

        self._pub_fast.publish(fast)
        self._pub_ready.publish(Bool(data=True))
        self.get_logger().info(
            f'FAST points ({len(fast.poses)} pts) — /z1/fast_ready=True')

    # ── Timer ──────────────────────────────────────────────────────────

    def start_timer(self) -> None:
        period = 1.0 / self._scan_rate if self._scan_rate > 0 else 0.1
        self.create_timer(period, self._tick)


def main(args=None):
    rclpy.init(args=args)
    node = WBCApproachScanner()
    node.start_timer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
