#!/usr/bin/env python3
"""
test_exposure_poses.py — Test exposure scan arm poses using virtual body keypoints.

Generates the same 23-point exposure grid as exposure_scanner.py, but uses
hardcoded virtual SMPL-24 keypoints instead of real body detection.
Interactive stepping: press ENTER to send each point to the IK solver,
with h=home, p=pause, r=resume, q=quit.

link00 frame (matches IK solver):
  X = UP (red, vertical)             — camera above (+X), EE points down (-X)
  Y = left (green, head→feet)        — grid spans head→feet along Y
  Z = forward (blue, toward patient) — camera slightly behind (-Z), EE points forward (+Z)

Spot is beside the body, near the torso (Y≈0 in link00 frame).
Lying body: on ground (X≈0), extends along Y (head→feet).

Two modes:
  arm-only (enable_spot_body_pose=False): Direct IK goals. Current behavior.
  arm+spot (enable_spot_body_pose=True, default): Per-point body pose optimization
    over height×pitch grid, followed by settle wait, then IK goal. Simulates the
    full exposure scan protocol (Spot reconfiguration + arm positioning).

Key 'b' bypasses Spot body pose for the current point when Spot mode is active.

Two body orientations:
  lying (default):  Body on ground (Z≈0), extends along Y (head→feet, ~1.70m span).
                    Head/feet require Spot body pose optimization (height+pitch)
                    to reach. Best for testing full exposure scan with Spot reconfig.
  standing:         Body vertical along Y (head at Y=1.60m). Requires Spot
                    body pose optimization (height/pitch) to reach upper body.
                    Simulates real exposure scan for STANDING patients.

For lying: realistic 1.70m body along Y axis (head→feet). Body pose optimization
           uses pitch to reach head (Y=+0.85) and feet (Y=-0.85).
For standing: set virtual_body_x=0.60 for lower body; upper body needs Spot.

Usage:
    ros2 run spot_control test_exposure_poses
    ros2 run spot_control test_exposure_poses --ros-args -p body_orientation:=standing -p virtual_body_x:=0.60
    ros2 run spot_control test_exposure_poses --ros-args -p enable_spot_body_pose:=false
"""

import sys
import select
import termios
import tty
import math
import time
from dataclasses import dataclass
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray

from teresa_utils.orientation import compute_ee_orientation_minrot
from spot_perception.sml_pose_indices import (
    PELVIS, HIP_LEFT, HIP_RIGHT, SPINE1, KNEE_LEFT, KNEE_RIGHT,
    SPINE2, ANKLE_LEFT, ANKLE_RIGHT, SPINE3, FOOT_LEFT, FOOT_RIGHT,
    NECK, COLLAR_LEFT, COLLAR_RIGHT, HEAD,
    SHOULDER_LEFT, SHOULDER_RIGHT, ELBOW_LEFT, ELBOW_RIGHT,
    WRIST_LEFT, WRIST_RIGHT, HAND_LEFT, HAND_RIGHT,
    NUM_JOINTS,
)

# ── Home orientation (from exposure_scanner.py) ──────────────────────────
HOME_ORI = np.array([-0.0062, 0.4107, 0.0021, 0.9118])

# ── Home pose position (arm stowed, from wbc_qp_controller FWD-C) ─────────
HOME_POS = np.array([0.144, -0.005, 0.52])
HOME_QUAT = [0.0182, 0.1521, -0.0217, 0.9880]


# ═══════════════════════════════════════════════════════════════════════════
#  Replicated from exposure_scanner.py
# ═══════════════════════════════════════════════════════════════════════════

def lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + t * (b - a)


class BodyRegion(Enum):
    HEAD      = 'head'
    TORSO     = 'torso'
    LEFT_ARM  = 'left_arm'
    RIGHT_ARM = 'right_arm'
    LEFT_LEG  = 'left_leg'
    RIGHT_LEG = 'right_leg'
    FEET      = 'feet'


REGION_ORDER = [
    BodyRegion.HEAD,
    BodyRegion.TORSO,
    BodyRegion.LEFT_ARM,
    BodyRegion.RIGHT_ARM,
    BodyRegion.LEFT_LEG,
    BodyRegion.RIGHT_LEG,
    BodyRegion.FEET,
]

REGION_COLORS = {
    BodyRegion.HEAD:      (1.0, 0.9, 0.0),  # yellow
    BodyRegion.TORSO:     (0.2, 0.4, 1.0),  # blue
    BodyRegion.LEFT_ARM:  (1.0, 0.2, 0.2),  # red
    BodyRegion.RIGHT_ARM: (1.0, 0.5, 0.0),  # orange
    BodyRegion.LEFT_LEG:  (0.2, 0.8, 0.2),  # green
    BodyRegion.RIGHT_LEG: (0.4, 0.9, 0.4),  # light green
    BodyRegion.FEET:      (0.7, 0.2, 1.0),  # purple
}

POINTS_PER_REGION = {
    BodyRegion.HEAD:      3,
    BodyRegion.TORSO:     6,
    BodyRegion.LEFT_ARM:  3,
    BodyRegion.RIGHT_ARM: 3,
    BodyRegion.LEFT_LEG:  3,
    BodyRegion.RIGHT_LEG: 3,
    BodyRegion.FEET:      2,
}


@dataclass
class ExposurePoint:
    camera_xyz: np.ndarray
    surface_xyz: np.ndarray
    look_dir: np.ndarray
    region: BodyRegion
    region_index: int
    global_index: int = -1


# ═══════════════════════════════════════════════════════════════════════════
#  Virtual body generation
# ═══════════════════════════════════════════════════════════════════════════

# ── Standing orientation (body vertical, Y=up) ──────────────────────────
# link00 frame: X=UP, Y=left, Z=forward.
# Head at Y=1.60m, feet at Y=-0.05m. Requires Spot body pose to reach
# upper body. Use for real exposure scan simulation with STANDING patients.
_VIRTUAL_BODY_STANDING: dict[int, tuple[float, float, float]] = {
    HEAD:           (0.0, 1.60,  0.00),
    NECK:           (0.0, 1.45,  0.00),
    SHOULDER_LEFT:  (0.0, 1.35, -0.20),
    SHOULDER_RIGHT: (0.0, 1.35,  0.20),
    ELBOW_LEFT:     (0.0, 1.20, -0.32),
    ELBOW_RIGHT:    (0.0, 1.20,  0.32),
    WRIST_LEFT:     (0.0, 1.00, -0.32),
    WRIST_RIGHT:    (0.0, 1.00,  0.32),
    HAND_LEFT:      (0.0, 0.90, -0.32),
    HAND_RIGHT:     (0.0, 0.90,  0.32),
    HIP_LEFT:       (0.0, 0.90, -0.15),
    HIP_RIGHT:      (0.0, 0.90,  0.15),
    KNEE_LEFT:      (0.0, 0.45, -0.15),
    KNEE_RIGHT:     (0.0, 0.45,  0.15),
    ANKLE_LEFT:     (0.0, 0.05, -0.12),
    ANKLE_RIGHT:    (0.0, 0.05,  0.12),
    FOOT_LEFT:      (0.0, -0.05, -0.12),
    FOOT_RIGHT:     (0.0, -0.05,  0.12),
    SPINE1:         (0.0, 1.15,  0.00),
    SPINE2:         (0.0, 1.05,  0.00),
    SPINE3:         (0.0, 0.95,  0.00),
    PELVIS:         (0.0, 0.80,  0.00),
    COLLAR_LEFT:    (0.0, 1.40, -0.15),
    COLLAR_RIGHT:   (0.0, 1.40,  0.15),
}

# ── Lying orientation (link00 frame: X=UP, Y=left/head→feet, Z=forward) ─
# link00 frame: X=UP (red, vertical), Y=left (green, head→feet), Z=forward (blue, toward patient).
# Body on ground (X≈0), extends along Y (head→feet).
# Body width along Z (across body). Spot beside body at Y≈0, Z≈0.
_VIRTUAL_BODY_LYING: dict[int, tuple[float, float, float]] = {
    # link00 frame: X=UP, Y=left(head→feet), Z=forward(toward patient)
    HEAD:           (0.05, 0.85, 0.00),
    NECK:           (0.05, 0.70, 0.00),
    SHOULDER_LEFT:  (0.05, 0.60, -0.20),
    SHOULDER_RIGHT: (0.05, 0.60, 0.20),
    ELBOW_LEFT:     (0.00, 0.35, -0.22),
    ELBOW_RIGHT:    (0.00, 0.35, 0.22),
    WRIST_LEFT:     (0.00, 0.10, -0.22),
    WRIST_RIGHT:    (0.00, 0.10, 0.22),
    HAND_LEFT:      (0.00, -0.05, -0.22),
    HAND_RIGHT:     (0.00, -0.05, 0.22),
    HIP_LEFT:       (0.05, -0.15, -0.15),
    HIP_RIGHT:      (0.05, -0.15, 0.15),
    KNEE_LEFT:      (0.00, -0.45, -0.15),
    KNEE_RIGHT:     (0.00, -0.45, 0.15),
    ANKLE_LEFT:     (0.00, -0.75, -0.12),
    ANKLE_RIGHT:    (0.00, -0.75, 0.12),
    FOOT_LEFT:      (0.00, -0.85, -0.12),
    FOOT_RIGHT:     (0.00, -0.85, 0.12),
    SPINE1:         (0.03, 0.40, 0.00),
    SPINE2:         (0.03, 0.20, 0.00),
    SPINE3:         (0.03, 0.00, 0.00),
    PELVIS:         (0.03, -0.15, 0.00),
    COLLAR_LEFT:    (0.04, 0.63, -0.15),
    COLLAR_RIGHT:   (0.04, 0.63, 0.15),
}


def make_virtual_body(offset_x: float, offset_y: float,
                      offset_z: float,
                      orientation: str = 'lying',
                      body_scale: float = 1.0) -> dict[int, np.ndarray]:
    """Return dict {SMPL_index: world_xyz} for a virtual body at given offset.

    link00 frame: X=UP (vertical), Y=left (head→feet), Z=forward (toward patient).

    Args:
        offset_x: Vertical offset (X axis, up/down) in link00 frame.
        offset_y: Left/right offset (Y axis, head→feet) in link00 frame.
        offset_z: Forward/backward offset (Z axis, toward patient) in link00 frame.
        orientation: 'lying' (body on ground, X≈0) or 'standing' (body vertical).
        body_scale: Scale factor for body span (1.0=full size, 0.35=fit Z1 workspace).
    """
    if orientation == 'standing':
        body = _VIRTUAL_BODY_STANDING
    else:
        body = _VIRTUAL_BODY_LYING
    kp: dict[int, np.ndarray] = {}
    # link00 frame: base = [up, left, forward]; body centered at X=0 (ground), Y≈0, Z≈0
    base = np.array([offset_x, offset_y, offset_z], dtype=float)
    for idx, rel in body.items():
        kp[idx] = base + np.array(rel, dtype=float) * body_scale
    return kp


# ═══════════════════════════════════════════════════════════════════════════
#  Grid generation  (exact replica of exposure_scanner._gen_exposure_grid)
# ═══════════════════════════════════════════════════════════════════════════

def _gen_exposure_grid(kp: dict[int, np.ndarray],
                       standoff: float,
                       standoff_vertical: bool = True,
                       regions: str = 'all') -> list[ExposurePoint]:
    if standoff_vertical:
        # Camera ABOVE (+X) and slightly BEHIND (-Z) surface.
        # link00 frame: X=UP, Y=left, Z=forward.
        # z_off = [standoff, 0, -standoff*0.20] → camera = surface + [0.35 high, 0, -0.07 back]
        # look_dir = [-0.35, 0, 0.07] normalized = [-0.98, 0, 0.20]
        # X_ee = [-0.98, 0, 0.20] → mostly DOWN (-X), slightly FORWARD (+Z)
        z_off = np.array([standoff, 0.0, -standoff * 0.20])
    else:
        # Camera BESIDE body (offset in -Y, looking along +Y)
        z_off = np.array([0.0, -standoff, 0.0])

    if regions != 'all':
        allowed = set(r.strip() for r in regions.split(','))
        region_order = [r for r in REGION_ORDER if r.value in allowed]
    else:
        region_order = REGION_ORDER

    points: list[ExposurePoint] = []

    for region in region_order:
        region_points = _gen_region(region, kp, z_off)
        if region_points:
            points.extend(region_points)

    for i, ep in enumerate(points):
        ep.global_index = i
    return points


def _gen_region(region: BodyRegion, kp: dict,
                z_off: np.ndarray) -> list[ExposurePoint]:
    if region == BodyRegion.HEAD:
        return _gen_head(kp, z_off)
    elif region == BodyRegion.TORSO:
        return _gen_torso(kp, z_off)
    elif region == BodyRegion.LEFT_ARM:
        return _gen_arm(kp, [SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT, HAND_LEFT],
                        z_off, region)
    elif region == BodyRegion.RIGHT_ARM:
        return _gen_arm(kp, [SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT, HAND_RIGHT],
                        z_off, region)
    elif region == BodyRegion.LEFT_LEG:
        return _gen_leg(kp, [HIP_LEFT, KNEE_LEFT, ANKLE_LEFT, FOOT_LEFT],
                        z_off, region)
    elif region == BodyRegion.RIGHT_LEG:
        return _gen_leg(kp, [HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT, FOOT_RIGHT],
                        z_off, region)
    elif region == BodyRegion.FEET:
        return _gen_feet(kp, z_off)
    return []


def _gen_head(kp: dict, z_off: np.ndarray) -> list[ExposurePoint]:
    neck = kp.get(NECK)
    if neck is None:
        sl = kp.get(SHOULDER_LEFT)
        sr = kp.get(SHOULDER_RIGHT)
        if sl is not None and sr is not None:
            neck = (sl + sr) / 2.0 + np.array([0.0, -0.25, 0.0])
        else:
            return []

    head = kp.get(HEAD)
    center = head if head is not None else neck

    shoulder_width = 0.15
    sl = kp.get(SHOULDER_LEFT)
    sr = kp.get(SHOULDER_RIGHT)
    if sl is not None and sr is not None:
        shoulder_width = float(np.linalg.norm(sr - sl)) * 0.7

    points = []
    n = POINTS_PER_REGION[BodyRegion.HEAD]
    for i in range(n):
        offset_y = (i / max(n - 1, 1) - 0.5) * shoulder_width
        surface = center + np.array([0.0, offset_y, 0.0])
        camera = surface + z_off
        look_dir = surface - camera
        look_dir /= float(np.linalg.norm(look_dir))
        points.append(ExposurePoint(
            camera_xyz=camera, surface_xyz=surface,
            look_dir=look_dir, region=BodyRegion.HEAD,
            region_index=i))
    return points


def _gen_torso(kp: dict, z_off: np.ndarray) -> list[ExposurePoint]:
    torso_rows = 3
    torso_cols = 2
    tl = kp.get(SHOULDER_LEFT)
    tr = kp.get(SHOULDER_RIGHT)
    bl = kp.get(HIP_LEFT)
    br = kp.get(HIP_RIGHT)
    if any(x is None for x in [tl, tr, bl, br]):
        available = [kp[i] for i in [SHOULDER_LEFT, SHOULDER_RIGHT,
                                      HIP_LEFT, HIP_RIGHT]
                     if kp.get(i) is not None]
        if len(available) < 3:
            return []
        tl = tr = bl = br = np.mean(available, axis=0)

    points = []
    for r in range(torso_rows):
        for c in range(torso_cols):
            u = c / max(torso_cols - 1, 1)
            v = r / max(torso_rows - 1, 1)
            surface = lerp(lerp(tl, tr, u), lerp(bl, br, u), v)
            camera = surface + z_off
            look_dir = surface - camera
            look_dir /= float(np.linalg.norm(look_dir))
            idx = r * torso_cols + c
            points.append(ExposurePoint(
                camera_xyz=camera, surface_xyz=surface,
                look_dir=look_dir, region=BodyRegion.TORSO,
                region_index=idx))
    return points


def _gen_arm(kp: dict, bone_indices: list, z_off: np.ndarray,
             region: BodyRegion) -> list[ExposurePoint]:
    positions = [kp.get(i) for i in bone_indices]
    positions = [p for p in positions if p is not None]
    if len(positions) < 2:
        return []

    points = []
    n = POINTS_PER_REGION[region]
    for i in range(n):
        t = i / max(n - 1, 1)
        surface = _sample_polyline(positions, t)
        camera = surface + z_off
        look_dir = surface - camera
        look_dir /= float(np.linalg.norm(look_dir))
        points.append(ExposurePoint(
            camera_xyz=camera, surface_xyz=surface,
            look_dir=look_dir, region=region, region_index=i))
    return points


def _gen_leg(kp: dict, bone_indices: list, z_off: np.ndarray,
             region: BodyRegion) -> list[ExposurePoint]:
    return _gen_arm(kp, bone_indices, z_off, region)


def _gen_feet(kp: dict, z_off: np.ndarray) -> list[ExposurePoint]:
    feet = []
    for side, knee_idx, ankle_idx in [
        ('left', KNEE_LEFT, ANKLE_LEFT),
        ('right', KNEE_RIGHT, ANKLE_RIGHT),
    ]:
        ankle = kp.get(ankle_idx)
        if ankle is None:
            continue
        knee = kp.get(knee_idx)
        dir_vec = np.array([0.0, 0.0, -1.0])
        if knee is not None:
            d = ankle - knee
            d_norm = float(np.linalg.norm(d))
            if d_norm > 0.01:
                dir_vec = d / d_norm

        n = max(POINTS_PER_REGION[BodyRegion.FEET] // 2, 1)
        for i in range(n):
            offset = (i + 1) * 0.10
            surface = ankle + dir_vec * offset
            camera = surface + z_off
            look_dir = surface - camera
            look_dir /= float(np.linalg.norm(look_dir))
            feet.append(ExposurePoint(
                camera_xyz=camera, surface_xyz=surface,
                look_dir=look_dir, region=BodyRegion.FEET,
                region_index=len(feet)))
    return feet


def _sample_polyline(positions: list, t: float) -> np.ndarray:
    segments = len(positions) - 1
    if segments == 0:
        return positions[0].copy()
    local_t = t * segments
    seg_idx = min(int(local_t), segments - 1)
    seg_frac = local_t - seg_idx
    return lerp(positions[seg_idx], positions[seg_idx + 1], seg_frac)


# ═══════════════════════════════════════════════════════════════════════════
#  ROS2 Test Node
# ═══════════════════════════════════════════════════════════════════════════

class ExposurePoseTester(Node):
    """Standalone node to test exposure scan poses with virtual body keypoints."""

    def __init__(self):
        super().__init__('test_exposure_poses')

        # Parameters
        self._orientation = str(
            self.declare_parameter('body_orientation', 'lying')
            .get_parameter_value().string_value)
        self._offset_x = float(
            self.declare_parameter('virtual_body_x', 0.60)
            .get_parameter_value().double_value)
        self._offset_y = float(
            self.declare_parameter('virtual_body_y', 0.0)
            .get_parameter_value().double_value)
        self._offset_z = float(
            self.declare_parameter('virtual_body_z', 0.0)
            .get_parameter_value().double_value)
        self._standoff = float(
            self.declare_parameter('standoff', 0.50)
            .get_parameter_value().double_value)
        self._standoff_vertical = bool(
            self.declare_parameter('standoff_vertical', True)
            .get_parameter_value().bool_value)
        self._regions = str(
            self.declare_parameter('regions', 'all')
            .get_parameter_value().string_value)

        # Spot body pose parameters
        self._spot_enabled = bool(
            self.declare_parameter('enable_spot_body_pose', True)
            .get_parameter_value().bool_value)
        self._z1_mount_x = float(
            self.declare_parameter('z1_mount_x', 0.30)
            .get_parameter_value().double_value)
        self._z1_mount_z = float(
            self.declare_parameter('z1_mount_z', 0.15)
            .get_parameter_value().double_value)
        self._body_settle_s = float(
            self.declare_parameter('body_settle_s', 1.5)
            .get_parameter_value().double_value)

        # Publishers
        self._pub_goal = self.create_publisher(
            PoseStamped, '/ik_goal_pose', 10)
        self._pub_enable = self.create_publisher(
            Bool, '/ik_enable', 10)
        self._pub_grid = self.create_publisher(
            MarkerArray, '/exposure/grid_markers', 10)
        self._pub_body_pose = self.create_publisher(
            Pose, '/my_spot/body_pose', 10)
        self._pub_cmd_vel = self.create_publisher(
            Twist, '/my_spot/cmd_vel', 10)
        self._pub_refined = self.create_publisher(
            PoseArray, '/exposure/refined_skeleton', 10)

        # Subscribers
        self._sub_ik_done = self.create_subscription(
            Bool, '/ik_done', self._cb_ik_done, 10)
        self._sub_goto = self.create_subscription(
            Int32, '/exposure/goto_point', self._cb_goto_point, 10)

        self._ik_done = False
        self._current_idx = 0
        self._running = False
        self._paused = False
        self._points: list[ExposurePoint] = []
        self._goto_idx: int | None = None
        self._point_body_pose: dict[int, tuple[float, float, np.ndarray]] = {}

        # Auto-scale body for arm-only mode (link00 frame: X=UP, Y=left, Z=forward)
        if self._spot_enabled:
            self._body_scale = 1.0
        else:
            self._body_scale = 0.30
            self._offset_z = 0.30   # body 30cm forward (closer to Spot)
            self._offset_y = 0.0    # centered on Y (left/right)
            self._offset_x = 0.0    # on ground (X=UP, X≈0)
            self._standoff = 0.25   # reduced standoff
            self.get_logger().info(
                f'  Body scale: {self._body_scale:.2f}, offset_z: {self._offset_z:.2f}, offset_y: {self._offset_y:.2f}, standoff: {self._standoff:.2f} (arm-only)')

        # Spot body pose state
        self._settling = False
        self._settle_deadline = None
        self._best_h: float | None = None
        self._best_p: float | None = None
        self._camera_link00: np.ndarray | None = None

        # Generate virtual body and exposure grid
        kp = make_virtual_body(self._offset_x, self._offset_y, self._offset_z,
                                self._orientation, self._body_scale)
        self._virtual_kp = kp
        self._points = _gen_exposure_grid(kp, self._standoff, self._standoff_vertical, self._regions)

        # Timers
        self._grid_timer = self.create_timer(0.2, self._publish_grid_markers)
        self._skeleton_timer = self.create_timer(0.5, self._publish_virtual_skeleton)

        # Print header
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'EXPOSURE POSE TESTER — virtual body at '
            f'({self._offset_x:.1f}, {self._offset_y:.1f}, {self._offset_z:.1f}) '
            f'(link00 frame: X=UP, Y=left, Z=forward)'
        )
        self.get_logger().info(f'  Orientation: {self._orientation}')
        if self._standoff_vertical:
            self.get_logger().info(f'  Standoff: VERTICAL ({self._standoff:.2f}m, camera above +X, EE down -X)')
        else:
            self.get_logger().info(f'  Standoff: HORIZONTAL ({self._standoff:.2f}m, camera beside body in -Y)')
        if self._spot_enabled:
            self.get_logger().info(
                f'  Spot body pose: ENABLED '
                f'(h×p grid search, settle {self._body_settle_s:.1f}s)'
            )
        else:
            self.get_logger().info(
                '  Spot body pose: DISABLED (arm-only, press b to enable per-point)'
            )
        self.get_logger().info(f'  Total points: {len(self._points)}')
        regs = sorted(set(ep.region for ep in self._points),
                      key=lambda r: REGION_ORDER.index(r))
        self.get_logger().info(f'  Regions: {len(regs)}')
        for r in regs:
            pts = [ep for ep in self._points if ep.region == r]
            self.get_logger().info(
                f'    {r.value}: {len(pts)} points')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            'Commands: ENTER=next  b=bypass spot  g=grid  h=home  p=pause  r=resume  q=quit')
        self.get_logger().info(
            '         Web UI: click grid marker on camera_view.html → goto point')
        self.get_logger().info('')

    # ── Callbacks ──────────────────────────────────────────────────────

    def _cb_ik_done(self, msg: Bool):
        if msg.data:
            self._ik_done = True

    def _cb_goto_point(self, msg: Int32):
        idx = msg.data
        if idx < 0 or idx >= len(self._points):
            self.get_logger().warn(f'Goto point {idx} out of range (0-{len(self._points)-1})')
            return
        ep = self._points[idx]
        self.get_logger().info(f'🖱️  Web UI: goto point {idx} ({ep.region.value}[{ep.region_index}])')

        # Cancel any pending settle
        self._settling = False

        # If spot enabled, optimize and apply body pose for this point
        if self._spot_enabled:
            self.get_logger().info(f'🔍 Re-optimizing Spot body pose for point {idx}...')
            h, p = self._optimize_body_pose(ep.camera_xyz, idx)
            self._apply_body_pose(h, p)
            self._settling = True
            self._settle_deadline = (
                self.get_clock().now()
                + rclpy.duration.Duration(seconds=self._body_settle_s))
            self._goto_idx = idx  # send IK goal after settle, but don't advance
            self.get_logger().info(f'⏳ Settling for {self._body_settle_s:.1f}s...')
        else:
            # Arm-only mode: send IK goal directly
            self._send_ik_goal(idx)
            self.get_logger().info('📸 Camera positioned — take screenshot!')

    # ── Spot body pose optimization ─────────────────────────────────────

    def _optimize_body_pose(self, camera_odom: np.ndarray, idx: int) -> tuple[float, float]:
        """Grid search over height × pitch to minimize distance to Z1 sweet spot.

        The Z1 dexterous workspace center is ~[0.35, 0.0, 0.30] in link00 frame
        (link00: X=UP, Y=left, Z=forward).
        For each (height, pitch) combination, computes the camera position in
        the link00 frame and measures distance to the sweet spot.

        Args:
            camera_odom: Camera position in odom frame [x, y, z].

        Returns:
            (best_height, best_pitch) in meters and radians.
        """
        heights = [-0.20, -0.18, -0.15, -0.10, -0.05, 0.0]
        pitches = [0.0, 0.087, 0.17, 0.26]  # 0°, 5°, 10°, 15° in rad
        sweet_spot = np.array([0.35, 0.0, 0.30])
        mx, mz = self._z1_mount_x, self._z1_mount_z

        best_h, best_p = 0.0, 0.0
        best_dist = float('inf')
        best_cam_link00: np.ndarray | None = None

        for h in heights:
            for p in pitches:
                c = math.cos(p)
                s = math.sin(p)

                # link00 position in odom frame
                link00_x = c * mx + s * mz
                link00_z = h - s * mx + c * mz

                # Camera position in link00 frame: R_y(-p) @ (cam_odom - link00_odom)
                dx = camera_odom[0] - link00_x
                dz = camera_odom[2] - link00_z
                cam_link00 = np.array([
                    c * dx - s * dz,
                    camera_odom[1],
                    s * dx + c * dz,
                ])

                dist = float(np.linalg.norm(cam_link00 - sweet_spot))
                if dist < best_dist:
                    best_dist = dist
                    best_h = h
                    best_p = p
                    best_cam_link00 = cam_link00

        self._best_h = best_h
        self._best_p = best_p
        self._camera_link00 = best_cam_link00

        self.get_logger().info(
            f'  Body pose optimized: h={best_h:.3f}m, '
            f'pitch={best_p * 57.3:.1f}°, dist_to_sweet={best_dist:.3f}m'
        )
        self._point_body_pose[idx] = (best_h, best_p, best_cam_link00)
        self.get_logger().info(f'  💾 Saved state for point {idx}: h={best_h:.3f}, p={best_p*57.3:.1f}°')
        return best_h, best_p

    def _apply_body_pose(self, height: float, pitch: float):
        """Publish body_pose and zero Twist to flush to spot_driver.

        Spot's body_pose is lazy: spot_driver saves the parameters and applies
        them on the next cmd_vel. The zero Twist flushes the stored body_pose.
        """
        half = pitch / 2.0
        qx = 0.0
        qy = math.sin(half)
        qz = 0.0
        qw = math.cos(half)

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = height
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self._pub_body_pose.publish(pose)

        twist = Twist()
        self._pub_cmd_vel.publish(twist)

    # ── IK goal sending ────────────────────────────────────────────────

    def _send_ik_goal(self, idx: int):
        ep = self._points[idx]
        x_ee = ep.look_dir
        quat = compute_ee_orientation_minrot(x_ee, HOME_ORI.tolist())

        if self._spot_enabled and self._best_h is not None and self._camera_link00 is not None:
            cx, cy, cz = (
                float(self._camera_link00[0]),
                float(self._camera_link00[1]),
                float(self._camera_link00[2]),
            )
        else:
            cx = float(ep.camera_xyz[0])
            cy = float(ep.camera_xyz[1])
            cz = float(ep.camera_xyz[2])

        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = cx
        goal.pose.position.y = cy
        goal.pose.position.z = cz
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self._pub_enable.publish(Bool(data=True))
        self._pub_goal.publish(goal)
        self._ik_done = False

        self.get_logger().info(
            f'▶ [{idx + 1}/{len(self._points)}] '
            f'{ep.region.value}[{ep.region_index}] '
            f'pos=({cx:.3f}, {cy:.3f}, {cz:.3f}) '
            f'quat=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})'
        )

    def _send_home(self):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(HOME_POS[0])
        msg.pose.position.y = float(HOME_POS[1])
        msg.pose.position.z = float(HOME_POS[2])
        msg.pose.orientation.x = HOME_QUAT[0]
        msg.pose.orientation.y = HOME_QUAT[1]
        msg.pose.orientation.z = HOME_QUAT[2]
        msg.pose.orientation.w = HOME_QUAT[3]

        self._pub_enable.publish(Bool(data=True))
        self._pub_goal.publish(msg)
        self._ik_done = False
        self._current_idx = 0
        self._paused = False
        self._settling = False
        self.get_logger().info(
            f'🏠 HOME sent ({HOME_POS[0]:.3f}, {HOME_POS[1]:.3f}, {HOME_POS[2]:.3f}) — reset to point 0'
        )

    # ── Markers ────────────────────────────────────────────────────────

    def _publish_virtual_skeleton(self):
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            pose = Pose()
            if i in self._virtual_kp:
                kp = self._virtual_kp[i]
                pose.position.x = float(kp[0])
                pose.position.y = float(kp[1])
                pose.position.z = float(kp[2])
                pose.orientation.w = 1.0
            else:
                pose.position.x = float('nan')
                pose.position.y = float('nan')
                pose.position.z = float('nan')
            pa.poses.append(pose)
        self._pub_refined.publish(pa)

    def _publish_grid_markers(self):
        if not self._points:
            return
        markers = MarkerArray()
        for i, ep in enumerate(self._points):
            cr, cg, cb = REGION_COLORS.get(ep.region, (0.5, 0.5, 0.5))
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f'exposure_grid_{ep.region.value}'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(ep.camera_xyz[0])
            m.pose.position.y = float(ep.camera_xyz[1])
            m.pose.position.z = float(ep.camera_xyz[2])
            if i == self._current_idx and self._running and not self._paused:
                m.scale.x = 0.025
                m.scale.y = 0.025
                m.scale.z = 0.025
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 1.0
            elif i < self._current_idx:
                m.scale.x = 0.015
                m.scale.y = 0.015
                m.scale.z = 0.015
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 0.4
            else:
                m.scale.x = 0.015
                m.scale.y = 0.015
                m.scale.z = 0.015
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 0.7
            markers.markers.append(m)
        self._pub_grid.publish(markers)

    # ── Main interactive loop ──────────────────────────────────────────

    def spin(self):
        self._running = True
        self.get_logger().info('Press ENTER to send first pose...')

        old_settings = None
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass  # non-interactive mode

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                # Check settle completion (Spot body pose applied → now send IK goal)
                if self._settling:
                    if self.get_clock().now() >= self._settle_deadline:
                        self._settling = False
                        if self._goto_idx is not None:
                            self.get_logger().info(
                                f'  ✅ Settle complete — sending IK goal for point {self._goto_idx}')
                            self._send_ik_goal(self._goto_idx)
                            self._goto_idx = None
                        else:
                            self.get_logger().info(
                                '  ✅ Settle complete — sending IK goal')
                            self._send_ik_goal(self._current_idx)
                            self._current_idx += 1

                # Check ik_done (only when not settling)
                if not self._settling:
                    if self._ik_done and self._current_idx < len(self._points):
                        self.get_logger().info(
                            '  ✅ ik_done received — press ENTER for next pose')

                # Check keyboard
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key in ('\n', '\r'):  # ENTER
                            if self._paused:
                                self.get_logger().warn(
                                    '⏸  Currently paused — press r to resume')
                                continue
                            if self._settling:
                                self.get_logger().warn(
                                    '⏳ Still settling — wait for settle to complete')
                                continue
                            if self._ik_done or self._current_idx == 0:
                                if self._current_idx < len(self._points):
                                    if self._spot_enabled:
                                        ep = self._points[self._current_idx]
                                        self.get_logger().info(
                                            f'🔍 Optimizing Spot body pose for point '
                                            f'{self._current_idx + 1}...')
                                        h, p = self._optimize_body_pose(ep.camera_xyz, self._current_idx)
                                        self._apply_body_pose(h, p)
                                        self._settling = True
                                        self._settle_deadline = (
                                            self.get_clock().now()
                                            + rclpy.duration.Duration(
                                                seconds=self._body_settle_s))
                                        self.get_logger().info(
                                            f'⏳ Settling for '
                                            f'{self._body_settle_s:.1f}s...')
                                    else:
                                        self._send_ik_goal(self._current_idx)
                                        self._current_idx += 1
                                else:
                                    self.get_logger().info(
                                        '✅ All points done! '
                                        'Press ENTER to go HOME, q to quit')
                            else:
                                self.get_logger().warn(
                                    '⚠️  ik_done not received yet — '
                                    'wait for arm to finish')
                        elif key == 'b':
                            if self._paused:
                                self.get_logger().warn(
                                    '⏸  Currently paused — press r to resume')
                                continue
                            if self._settling:
                                self.get_logger().warn(
                                    '⏳ Still settling — wait for settle to complete')
                                continue
                            if self._ik_done or self._current_idx == 0:
                                if self._current_idx < len(self._points):
                                    self.get_logger().info(
                                        '🔀 Bypassing Spot body pose')
                                    self._send_ik_goal(self._current_idx)
                                    self._current_idx += 1
                                else:
                                    self.get_logger().info(
                                        '✅ All points done! '
                                        'Press ENTER to go HOME, q to quit')
                            else:
                                self.get_logger().warn(
                                    '⚠️  ik_done not received yet — '
                                    'wait for arm to finish')
                        elif key == 'p':
                            self._paused = True
                            self.get_logger().info(
                                '⏸  PAUSED — press r to resume')
                        elif key == 'r':
                            self._paused = False
                            self.get_logger().info('▶ RESUMED')
                        elif key == 'g':
                            self.get_logger().info('─' * 50)
                            self.get_logger().info(f'GRID SUMMARY — {len(self._points)} points:')
                            for i, ep in enumerate(self._points):
                                marker = '🟢' if i < self._current_idx else ('🔵' if i == self._current_idx else '⚪')
                                self.get_logger().info(
                                    f'  {marker} [{i:2d}] {ep.region.value:10s}[{ep.region_index}] '
                                    f'cam=({ep.camera_xyz[0]:.2f}, {ep.camera_xyz[1]:.2f}, {ep.camera_xyz[2]:.2f})'
                                )
                            self.get_logger().info('─' * 50)
                        elif key == 'h':
                            self._send_home()
                            self.get_logger().info(
                                '🏠 HOME — press ENTER to restart from point 1')
                        elif key == 'q':
                            self.get_logger().info('👋 Quit')
                            self._running = False
                except Exception:
                    pass

        finally:
            if old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = ExposurePoseTester()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
