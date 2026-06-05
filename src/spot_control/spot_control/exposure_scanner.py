#!/usr/bin/env python3
"""
exposure_scanner.py — Full-body scanning node for TERESA exposure assessment.

Moves the Z1 arm with the RealSense camera over a grid of points
on the patient's body, coordinating with the WBC coordinator for
Spot body posture changes via the same per-point protocol as FAST.

Regions (14 points total):
  HEAD (2) → TORSO (4) → LEFT_ARM (2) → RIGHT_ARM (2) →
  LEFT_LEG (2) → RIGHT_LEG (2) → FEET (2)

Protocol:
  exposure_scanner          wbc_coordinator
       |                         |
       |── next_point_idx ──────→|  coordinator optimises Spot posture
       |                         |  _set_body_pose(h*,p*) → settle 1.5s
       |←── /wbc/body_ready ────|
       |                         |
       |── IK goal (pos + look-at) →|  via ik_goal_mux → z1_ik_to_jtc
       |←── /ik_done ───────────|
       |                         |
       |── dwell 2s ────────────→|  RealSense observes body point
       |   (accumulates /exposure/body_keypoints)
       |                         |
       |── next_point_idx ──────→|  advance
"""

import json
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from teresa_utils.orientation import compute_ee_orientation

HOME_ORI = np.array([-0.0062, 0.4107, 0.0021, 0.9118])


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
    BodyRegion.HEAD:      2,
    BodyRegion.TORSO:     4,
    BodyRegion.LEFT_ARM:  2,
    BodyRegion.RIGHT_ARM: 2,
    BodyRegion.LEFT_LEG:  2,
    BodyRegion.RIGHT_LEG: 2,
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


class ExposureScanner(Node):

    def __init__(self):
        super().__init__('exposure_scanner')

        self._standoff = float(
            self.declare_parameter('exposure_standoff', 0.50)
            .get_parameter_value().double_value
        )
        self._dwell = float(
            self.declare_parameter('exposure_dwell', 2.0)
            .get_parameter_value().double_value
        )
        self._torso_rows = int(
            self.declare_parameter('exposure_torso_rows', 2)
            .get_parameter_value().integer_value
        )
        self._torso_cols = int(
            self.declare_parameter('exposure_torso_cols', 2)
            .get_parameter_value().integer_value
        )
        self._min_keypoints = int(
            self.declare_parameter('exposure_min_keypoints', 3)
            .get_parameter_value().integer_value
        )
        self._output_dir = str(
            self.declare_parameter('exposure_output_dir', '/tmp')
            .get_parameter_value().string_value
        )

        self._active = False
        self._points: list[ExposurePoint] = []
        self._idx = 0
        self._phase = 'idle'
        self._dwell_start = None
        self._body_ready = False
        self._ik_done = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._keypoints: dict[int, np.ndarray | None] = {}
        self._scan_data: dict[int, list] = {}
        self._scan_buffer: list = []
        self._refined_kp: dict[int, np.ndarray] = {}
        self._kp_buffer: dict[int, list] = {}

        self._sub_state = self.create_subscription(
            String, '/wbc/state', self._cb_wbc_state, 10
        )
        self._sub_body_ready = self.create_subscription(
            Bool, '/wbc/body_ready', self._cb_body_ready, 10
        )
        self._sub_ik_done = self.create_subscription(
            Bool, '/ik_done', self._cb_ik_done, 10
        )
        self._sub_goto = self.create_subscription(
            Int32, '/exposure/goto_point', self._cb_goto_point, 10
        )
        self._sub_skeleton = self.create_subscription(
            PoseArray, '/human_pose/points_3d', self._cb_skeleton, 10
        )
        self._sub_scan = self.create_subscription(
            Float32MultiArray, '/torso_scan_point', self._cb_scan_point, 10
        )
        self._sub_body_kp = self.create_subscription(
            PoseArray, '/exposure/body_keypoints', self._cb_body_keypoints, 10
        )

        self._pub_goal = self.create_publisher(
            PoseStamped, '/z1/ik_goal_pose', 10
        )
        self._pub_next = self.create_publisher(
            Int32, '/z1/next_point_idx', 10
        )
        self._pub_ready = self.create_publisher(
            Bool, '/exposure/ready', 10
        )
        self._pub_grid = self.create_publisher(
            MarkerArray, '/exposure/grid_markers', 10
        )
        self._pub_refined = self.create_publisher(
            PoseArray, '/exposure/refined_skeleton', 10
        )

        self._timer = self.create_timer(0.1, self._tick)
        self._grid_timer = self.create_timer(0.2, self._publish_grid_markers)

        self.get_logger().info('Exposure scanner ready')

    # ── callbacks ────────────────────────────────────────────────

    def _cb_wbc_state(self, msg):
        if msg.data == 'EXPOSURE_SCANNING' and not self._active:
            self._start()
        elif msg.data != 'EXPOSURE_SCANNING' and self._active:
            self._active = False
            self.get_logger().info('Exposure scan stopped by FSM')

    def _cb_body_ready(self, msg):
        if msg.data:
            self._body_ready = True

    def _cb_ik_done(self, msg):
        if msg.data:
            self._ik_done = True

    def _cb_goto_point(self, msg):
        idx = msg.data
        if idx < 0 or idx >= len(self._points):
            return
        self._active = True
        self._idx = idx
        self._phase = 'request_body_pose'
        self._body_ready = False
        self._ik_done = False
        self._scan_buffer = []
        self._pub_next.publish(Int32(data=idx))
        self.get_logger().info(f'Review: goto point {idx}')

    def _cb_skeleton(self, msg: PoseArray):
        if len(msg.poses) < 17:
            return
        frame_id = msg.header.frame_id or 'orbbec_color_optical_frame'
        for i, pose in enumerate(msg.poses):
            p = pose.position
            if np.isnan(p.x) or np.isnan(p.y) or np.isnan(p.z):
                self._keypoints[i] = None
                continue
            pt = PointStamped()
            pt.header.frame_id = frame_id
            pt.header.stamp = msg.header.stamp
            pt.point.x = p.x
            pt.point.y = p.y
            pt.point.z = p.z
            try:
                transform = self._tf_buffer.lookup_transform(
                    'world', frame_id, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                world_pt = do_transform_point(pt, transform)
                self._keypoints[i] = np.array([
                    world_pt.point.x, world_pt.point.y, world_pt.point.z])
            except TransformException:
                self._keypoints[i] = None

    def _cb_scan_point(self, msg: Float32MultiArray):
        if self._active and self._phase == 'dwell':
            self._scan_buffer.append(list(msg.data))

    def _cb_body_keypoints(self, msg: PoseArray):
        if not self._active or self._phase != 'dwell':
            return
        if len(msg.poses) < 17:
            return
        for i, pose in enumerate(msg.poses):
            p = pose.position
            if np.isnan(p.x) or np.isnan(p.y) or np.isnan(p.z):
                continue
            if i not in self._kp_buffer:
                self._kp_buffer[i] = []
            self._kp_buffer[i].append(np.array([p.x, p.y, p.z]))

    # ── lifecycle ────────────────────────────────────────────────

    def _start(self):
        self._active = True
        self._points = self._gen_exposure_grid()
        self._idx = 0
        self._scan_data = {}
        self._scan_buffer = []
        self._refined_kp = {}
        self._kp_buffer = {}
        if not self._points:
            self.get_logger().warn(
                'No exposure points generated — skipping'
            )
            self._pub_ready.publish(Bool(data=True))
            self._active = False
            return
        self._phase = 'request_body_pose'
        self._body_ready = False
        self._ik_done = False
        self.get_logger().info(
            f'Exposure scan started: {len(self._points)} points, '
            f'{len(set(ep.region for ep in self._points))} regions'
        )
        self._pub_next.publish(Int32(data=0))

    # ── grid generation ──────────────────────────────────────────

    def _gen_exposure_grid(self) -> list[ExposurePoint]:
        kp = self._keypoints
        z_off = np.array([-self._standoff, 0.0, 0.0])
        points = []

        for region in REGION_ORDER:
            region_points = self._gen_region(region, kp, z_off)
            if region_points:
                self.get_logger().info(
                    f'  {region.value}: {len(region_points)} points'
                )
                points.extend(region_points)
            else:
                self.get_logger().warn(
                    f'  {region.value}: skipped (insufficient keypoints)'
                )

        for i, ep in enumerate(points):
            ep.global_index = i
        return points

    def _gen_region(self, region: BodyRegion, kp: dict,
                    z_off: np.ndarray) -> list[ExposurePoint]:
        if region == BodyRegion.HEAD:
            return self._gen_head(kp, z_off)
        elif region == BodyRegion.TORSO:
            return self._gen_torso(kp, z_off)
        elif region == BodyRegion.LEFT_ARM:
            return self._gen_arm(kp, [5, 7, 9], z_off, region)
        elif region == BodyRegion.RIGHT_ARM:
            return self._gen_arm(kp, [6, 8, 10], z_off, region)
        elif region == BodyRegion.LEFT_LEG:
            return self._gen_leg(kp, [11, 13, 15], z_off, region)
        elif region == BodyRegion.RIGHT_LEG:
            return self._gen_leg(kp, [12, 14, 16], z_off, region)
        elif region == BodyRegion.FEET:
            return self._gen_feet(kp, z_off)
        return []

    def _gen_head(self, kp: dict,
                  z_off: np.ndarray) -> list[ExposurePoint]:
        nose = kp.get(0)
        if nose is None:
            s5, s6 = kp.get(5), kp.get(6)
            if s5 is not None and s6 is not None:
                nose = (s5 + s6) / 2.0 + np.array([0.0, -0.25, 0.0])
            else:
                return []

        shoulder_width = 0.15
        s5, s6 = kp.get(5), kp.get(6)
        if s5 is not None and s6 is not None:
            shoulder_width = float(np.linalg.norm(s6 - s5)) * 0.7

        points = []
        n = POINTS_PER_REGION[BodyRegion.HEAD]
        for i in range(n):
            offset_y = (i / max(n - 1, 1) - 0.5) * shoulder_width
            surface = nose + np.array([0.0, offset_y, 0.0])
            camera = surface + z_off
            look_dir = surface - camera
            look_dir /= float(np.linalg.norm(look_dir))
            points.append(ExposurePoint(
                camera_xyz=camera, surface_xyz=surface,
                look_dir=look_dir, region=BodyRegion.HEAD,
                region_index=i))
        return points

    def _gen_torso(self, kp: dict,
                   z_off: np.ndarray) -> list[ExposurePoint]:
        tl = kp.get(5)
        tr = kp.get(6)
        bl = kp.get(11)
        br = kp.get(12)
        if any(x is None for x in [tl, tr, bl, br]):
            available = [kp[i] for i in [5, 6, 11, 12] if kp.get(i) is not None]
            if len(available) < self._min_keypoints:
                return []
            tl = tr = bl = br = np.mean(available, axis=0)

        points = []
        for r in range(self._torso_rows):
            for c in range(self._torso_cols):
                u = c / max(self._torso_cols - 1, 1)
                v = r / max(self._torso_rows - 1, 1)
                surface = lerp(lerp(tl, tr, u), lerp(bl, br, u), v)
                camera = surface + z_off
                look_dir = surface - camera
                look_dir /= float(np.linalg.norm(look_dir))
                idx = r * self._torso_cols + c
                points.append(ExposurePoint(
                    camera_xyz=camera, surface_xyz=surface,
                    look_dir=look_dir, region=BodyRegion.TORSO,
                    region_index=idx))
        return points

    def _gen_arm(self, kp: dict, bone_indices: list, z_off: np.ndarray,
                 region: BodyRegion) -> list[ExposurePoint]:
        positions = [kp.get(i) for i in bone_indices]
        positions = [p for p in positions if p is not None]
        if len(positions) < 2:
            return []

        points = []
        n = POINTS_PER_REGION[region]
        for i in range(n):
            t = i / max(n - 1, 1)
            surface = self._sample_polyline(positions, t)
            camera = surface + z_off
            look_dir = surface - camera
            look_dir /= float(np.linalg.norm(look_dir))
            points.append(ExposurePoint(
                camera_xyz=camera, surface_xyz=surface,
                look_dir=look_dir, region=region, region_index=i))
        return points

    def _gen_leg(self, kp: dict, bone_indices: list, z_off: np.ndarray,
                 region: BodyRegion) -> list[ExposurePoint]:
        return self._gen_arm(kp, bone_indices, z_off, region)

    def _gen_feet(self, kp: dict,
                  z_off: np.ndarray) -> list[ExposurePoint]:
        feet = []
        for side, knee_idx, ankle_idx in [
            ('left', 13, 15), ('right', 14, 16),
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

    def _sample_polyline(self, positions: list, t: float) -> np.ndarray:
        segments = len(positions) - 1
        if segments == 0:
            return positions[0].copy()
        local_t = t * segments
        seg_idx = min(int(local_t), segments - 1)
        seg_frac = local_t - seg_idx
        return lerp(positions[seg_idx], positions[seg_idx + 1], seg_frac)

    # ── main loop ────────────────────────────────────────────────

    def _tick(self):
        if not self._active:
            return

        if self._phase == 'request_body_pose':
            if self._body_ready:
                self._body_ready = False
                self._send_ik_goal()
                self._phase = 'wait_ik'

        elif self._phase == 'wait_ik':
            if self._ik_done:
                self._ik_done = False
                self._dwell_start = self.get_clock().now()
                self._scan_buffer = []
                self._kp_buffer = {}
                self._phase = 'dwell'
                self.get_logger().debug(
                    f'Point {self._idx}: IK done, dwelling'
                )

        elif self._phase == 'dwell':
            elapsed = (
                self.get_clock().now() - self._dwell_start
            ).nanoseconds * 1e-9
            if elapsed >= self._dwell:
                if self._scan_buffer:
                    self._scan_data[self._idx] = self._scan_buffer.copy()
                    self._scan_buffer = []
                self._update_refined_kp()
                self._publish_refined_skeleton()
                self._idx += 1
                if self._idx >= len(self._points):
                    self._finish()
                else:
                    self._phase = 'request_body_pose'
                    self._body_ready = False
                    self._pub_next.publish(
                        Int32(data=self._idx)
                    )
                    self.get_logger().info(
                        f'Advancing to point {self._idx}/'
                        f'{len(self._points)}'
                    )

    def _send_ik_goal(self):
        ep = self._points[self._idx]
        x_ee = ep.look_dir
        quat = compute_ee_orientation(x_ee, HOME_ORI.tolist())

        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(ep.camera_xyz[0])
        goal.pose.position.y = float(ep.camera_xyz[1])
        goal.pose.position.z = float(ep.camera_xyz[2])
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        self._pub_goal.publish(goal)

        self.get_logger().info(
            f'{ep.region.value}[{ep.region_index}] '
            f'({self._idx + 1}/{len(self._points)}): IK goal sent'
        )

    def _update_refined_kp(self):
        for kp_idx, positions in self._kp_buffer.items():
            if not positions:
                continue
            mean_pos = np.mean(positions, axis=0)
            if kp_idx not in self._refined_kp:
                self._refined_kp[kp_idx] = mean_pos
            else:
                alpha = 0.5
                self._refined_kp[kp_idx] = (
                    alpha * mean_pos + (1.0 - alpha) * self._refined_kp[kp_idx]
                )
        self._kp_buffer = {}

    def _publish_refined_skeleton(self):
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = self.get_clock().now().to_msg()
        for i in range(17):
            pose = Pose()
            if i in self._refined_kp:
                kp = self._refined_kp[i]
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
            if i == self._idx and self._active:
                m.scale.x = 0.05
                m.scale.y = 0.05
                m.scale.z = 0.05
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 1.0
            elif i < self._idx:
                m.scale.x = 0.03
                m.scale.y = 0.03
                m.scale.z = 0.03
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 0.4
            else:
                m.scale.x = 0.03
                m.scale.y = 0.03
                m.scale.z = 0.03
                m.color.r = cr
                m.color.g = cg
                m.color.b = cb
                m.color.a = 0.7
            markers.markers.append(m)
        self._pub_grid.publish(markers)

    def _finish(self):
        self._active = False
        self._pub_ready.publish(Bool(data=True))
        self._pub_next.publish(Int32(data=-1))

        regions_data = {}
        for ep in self._points:
            rname = ep.region.value
            if rname not in regions_data:
                regions_data[rname] = {'num_points': 0, 'points': []}
            regions_data[rname]['num_points'] += 1
            regions_data[rname]['points'].append({
                'region_index': ep.region_index,
                'global_index': ep.global_index,
                'camera_xyz': ep.camera_xyz.tolist(),
                'surface_xyz': ep.surface_xyz.tolist(),
                'look_dir': ep.look_dir.tolist(),
                'scan_data_frames': len(self._scan_data.get(ep.global_index, [])),
            })

        output = {
            'timestamp': datetime.now().isoformat(),
            'total_points': len(self._points),
            'standoff_m': self._standoff,
            'dwell_s': self._dwell,
            'regions': regions_data,
        }

        out_path = (Path(self._output_dir) /
                    f'exposure_scan_{datetime.now():%Y%m%d_%H%M%S}.json')
        with open(out_path, 'w') as f:
            json.dump(output, f, indent=2)

        self.get_logger().info(
            f'Exposure scan complete: {len(self._points)} points visited. '
            f'Data saved to {out_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExposureScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
