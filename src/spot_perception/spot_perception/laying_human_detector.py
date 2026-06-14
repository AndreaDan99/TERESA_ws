#!/usr/bin/env python3
"""
Laying Human Detector.
Rileva persona sdraiata e calcola approach point LATERALE (lato del corpo),
geometria identica a teresa_mission.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped, Vector3Stamped
from rclpy.duration import Duration
from std_msgs.msg import String, Float32
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
from visualization_msgs.msg import Marker

from spot_perception.sml_pose_indices import *


class LayingHumanDetector(Node):
    def __init__(self):
        super().__init__('laying_human_detector')

        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('approach_margin',          0.05)   # extra oltre bbox edge [m]
        self.declare_parameter('spot_front_offset',        0.50)   # body center → muso Spot [m]
        self.declare_parameter('preferred_side',           'auto') # 'auto'|'left'|'right'
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_valid_keypoints',      4)
        self.declare_parameter('test_mode',                True)
        self.declare_parameter('detection_timeout',        2.0)

        self.approach_margin    = float(self.get_parameter('approach_margin').value)
        self.spot_front_offset  = float(self.get_parameter('spot_front_offset').value)
        self.preferred_side     = str(self.get_parameter('preferred_side').value)
        self.min_conf          = float(self.get_parameter('min_detection_confidence').value)
        self.min_kp            = int(self.get_parameter('min_valid_keypoints').value)
        self.test_mode         = bool(self.get_parameter('test_mode').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)

        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        self.skeleton_sub = self.create_subscription(
            PoseArray, '/human_pose/points_3d', self.skeleton_callback, 10)
        self.posture_sub = self.create_subscription(
            String, '/human_pose/posture', self.posture_callback, 10)
        self.posture_conf_sub = self.create_subscription(
            Float32, '/human_pose/posture_confidence', self.confidence_callback, 10)

        # ============================================================
        # PUBLISHERS
        # ============================================================
        self.goal_pub = self.create_publisher(
            PoseStamped, '/laying_human/approach_point', 10)
        self.body_axis_pub = self.create_publisher(
            Vector3Stamped, '/laying_human/body_axis', 10)
        self.approach_marker_pub = self.create_publisher(
            Marker, '/laying_human/approach_marker', 10)
        self.body_center_pub = self.create_publisher(
            PoseStamped, '/laying_human/body_center', 10)

        # ── TF infrastructure ──────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ============================================================
        # STATE
        # ============================================================
        self.current_posture    = 'UNKNOWN'
        self.current_confidence = 0.0
        self.latest_skeleton    = None
        self.goal_sent          = False
        self.last_detection_time = None
        self._body_frame_trans = None
        self._body_frame_frame_id = None
        self._body_axis = None

        self.reset_timer = self.create_timer(1.0, self.check_detection_timeout)

        mode_str = 'TEST MODE (pubblica sempre)' if self.test_mode else 'ACTIVE MODE (pubblica una volta)'
        self.get_logger().info(
            f'✅ LayingHumanDetector READY — {mode_str}\n'
            f'   approach_margin={self.approach_margin}m  side={self.preferred_side}\n'
            f'   min_conf={self.min_conf}  min_kp={self.min_kp}'
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def confidence_callback(self, msg):
        self.current_confidence = msg.data

    def posture_callback(self, msg):
        prev = self.current_posture
        self.current_posture = msg.data
        if prev == 'LYING' and self.current_posture != 'LYING':
            self.goal_sent = False
            self.get_logger().info('Detection LYING persa → flag reset')

    def check_detection_timeout(self):
        if self.last_detection_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.detection_timeout:
            if self.goal_sent:
                self.goal_sent = False
                self.get_logger().info(f'Detection timeout ({elapsed:.1f}s) → flag reset')
            self.last_detection_time = None

    def skeleton_callback(self, msg: PoseArray):
        self.latest_skeleton = msg

        if self.current_posture != 'LYING':
            return
        if self.current_confidence < self.min_conf:
            self.get_logger().warn(
                f'LYING ma confidence bassa: {self.current_confidence:.2f}',
                throttle_duration_sec=2.0)
            return

        valid_points = []
        for pose in msg.poses:
            p = pose.position
            if not (math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z)):
                valid_points.append([p.x, p.y, p.z])

        if len(valid_points) < self.min_kp:
            self.get_logger().warn(
                f'LYING ma pochi keypoints: {len(valid_points)}/{self.min_kp}',
                throttle_duration_sec=2.0)
            return

        self.last_detection_time = self.get_clock().now()

        self._try_publish_body_axis(msg)

        if not self.test_mode and not self.goal_sent:
            self._publish_lateral_approach(msg)
            self.goal_sent = True
        elif self.test_mode:
            self._publish_lateral_approach(msg)

        self._broadcast_body_tf()

    # ============================================================
    # GEOMETRIA LATERALE (identica a teresa_mission)
    # ============================================================

    def _extract_kp(self, msg: PoseArray):
        """Ritorna lista di np.array o None per ogni keypoint COCO."""
        kp = []
        for p in msg.poses:
            pos = p.position
            if math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z):
                kp.append(None)
            else:
                kp.append(np.array([pos.x, pos.y, pos.z], dtype=np.float64))
        return kp

    def _publish_lateral_approach(self, msg: PoseArray):
        """
        Geometria:
        - asse corpo: piedi → testa
        - laterale: perpendicolare all'asse corpo nel piano XZ (camera optical frame)
        - approach point: torso_center ± lateral * (bbox_half + approach_margin)
        - orientamento: Spot guarda verso torso_center
        """
        kp = self._extract_kp(msg)
        if len(kp) < NUM_JOINTS:
            return

        # --- Centro torso ---
        torso_pts = [kp[i] for i in [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT] if kp[i] is not None]
        if len(torso_pts) < 2:
            return
        torso_center = np.mean(torso_pts, axis=0)

        # --- Asse corpo (testa → bacino) ---
        head_pos   = kp[HEAD]   if kp[HEAD]   is not None else None
        pelvis_pos = kp[PELVIS] if kp[PELVIS] is not None else None
        if head_pos is not None and pelvis_pos is not None:
            body_axis = head_pos - pelvis_pos
        else:
            # fallback: NECK → mean(ankles)
            feet_pts = [kp[i] for i in [ANKLE_LEFT, ANKLE_RIGHT] if kp[i] is not None]
            if len(feet_pts) == 0:
                feet_pts = [kp[i] for i in [KNEE_LEFT, KNEE_RIGHT] if kp[i] is not None]
            if kp[NECK] is None or len(feet_pts) == 0:
                return
            body_axis = kp[NECK] - np.mean(feet_pts, axis=0)
        body_len = np.linalg.norm(body_axis)
        if body_len < 0.1:
            return
        body_axis_n = body_axis / body_len

        # --- Direzione laterale nel piano XZ (camera optical: X=right, Y=down, Z=depth) ---
        up_cam = np.array([0.0, -1.0, 0.0])
        lateral = np.cross(body_axis_n, up_cam)
        lat_norm = np.linalg.norm(lateral)
        if lat_norm < 1e-6:
            lateral = np.array([1.0, 0.0, 0.0])
        else:
            lateral = lateral / lat_norm

        # --- Bbox half nella direzione laterale ---
        arm_leg_points = [kp[i] for i in ARM_JOINTS + LEG_JOINTS if kp[i] is not None]
        if len(arm_leg_points) < 2:
            arm_leg_points = [kp[i] for i in range(NUM_JOINTS) if kp[i] is not None]
        all_pts = np.array(arm_leg_points)
        bbox_min = all_pts.min(axis=0)
        bbox_max = all_pts.max(axis=0)
        bbox_size = bbox_max - bbox_min
        bbox_half = float(np.abs(np.dot(bbox_size * 0.5, np.abs(lateral))))
        bbox_half = max(bbox_half, 0.3)

        dist = bbox_half + self.approach_margin + self.spot_front_offset

        # --- Scelta lato ---
        candidate_a = torso_center + lateral * dist
        candidate_b = torso_center - lateral * dist

        if self.preferred_side == 'auto':
            approach_pos = candidate_a if candidate_a[2] < candidate_b[2] else candidate_b
        elif self.preferred_side == 'left':
            approach_pos = candidate_a
        else:
            approach_pos = candidate_b

        # --- Orientamento: Spot guarda verso torso_center ---
        dx = torso_center[0] - approach_pos[0]
        dz = torso_center[2] - approach_pos[2]
        yaw = math.atan2(dx, dz)
        qz  = math.sin(yaw / 2.0)
        qw  = math.cos(yaw / 2.0)

        self._body_frame_trans = approach_pos.copy()
        self._body_frame_frame_id = msg.header.frame_id

        # --- Pubblica PoseStamped ---
        goal = PoseStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = msg.header.frame_id
        goal.pose.position.x = float(approach_pos[0])
        goal.pose.position.y = float(approach_pos[1])
        goal.pose.position.z = float(approach_pos[2])
        goal.pose.orientation.z = float(qz)
        goal.pose.orientation.w = float(qw)
        self.goal_pub.publish(goal)

        # --- Pubblica body center (torso centroid) per LOOKAT in PRE_APPROACH ---
        body_ctr = PoseStamped()
        body_ctr.header.stamp    = self.get_clock().now().to_msg()
        body_ctr.header.frame_id = msg.header.frame_id
        body_ctr.pose.position.x = float(torso_center[0])
        body_ctr.pose.position.y = float(torso_center[1])
        body_ctr.pose.position.z = float(torso_center[2])
        body_ctr.pose.orientation.w = 1.0
        self.body_center_pub.publish(body_ctr)

        # --- Marker RViz ---
        self._publish_approach_marker(approach_pos, msg.header)

        self.get_logger().info(
            f'Approach laterale: ({approach_pos[0]:.2f}, {approach_pos[1]:.2f}, {approach_pos[2]:.2f}) '
            f'[margin={self.approach_margin}m side={self.preferred_side}]',
            throttle_duration_sec=2.0
        )

    def _try_publish_body_axis(self, msg: PoseArray):
        if len(msg.poses) < 13:
            return

        def _kp(idx):
            p = msg.poses[idx].position
            if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
                return None
            return np.array([p.x, p.y, p.z])

        kp_shl = _kp(SHOULDER_LEFT)
        kp_shr = _kp(SHOULDER_RIGHT)
        kp_hl  = _kp(HIP_LEFT)
        kp_hr  = _kp(HIP_RIGHT)
        if any(x is None for x in [kp_shl, kp_shr, kp_hl, kp_hr]):
            return

        shoulder_mid = (kp_shl + kp_shr) / 2.0
        hip_mid      = (kp_hl + kp_hr) / 2.0
        body_vec     = hip_mid - shoulder_mid
        body_len     = float(np.linalg.norm(body_vec))
        if body_len < 0.1:
            return

        axis = body_vec / body_len
        v = Vector3Stamped()
        v.header.stamp    = self.get_clock().now().to_msg()
        v.header.frame_id = msg.header.frame_id
        v.vector.x = float(axis[0])
        v.vector.y = float(axis[1])
        v.vector.z = float(axis[2])
        self.body_axis_pub.publish(v)

        self._body_axis = axis.copy()

    def _broadcast_body_tf(self):
        if self._body_frame_trans is None or self._body_frame_frame_id is None \
                or self._body_axis is None:
            return

        body_vec = self._body_axis
        body_len = np.linalg.norm(body_vec)
        if body_len < 0.001 or np.any(np.isnan(body_vec)):
            self.get_logger().warn(
                'Body axis invalid for TF broadcast',
                throttle_duration_sec=2.0)
            return

        body_y = body_vec / body_len
        body_x = np.cross(body_y, np.array([0.0, 0.0, 1.0]))
        if np.linalg.norm(body_x) < 0.001:
            body_x = np.array([1.0, 0.0, 0.0])
        body_x = body_x / np.linalg.norm(body_x)
        body_z = np.cross(body_x, body_y)
        body_z = body_z / np.linalg.norm(body_z)
        R = np.column_stack([body_x, body_y, body_z])

        qw = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        if qw < 1e-9:
            qx = qy = qz = 0.0
        else:
            qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
            qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
            qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)

        try:
            pose_cam = PoseStamped()
            pose_cam.header.stamp = self.get_clock().now().to_msg()
            pose_cam.header.frame_id = self._body_frame_frame_id
            pose_cam.pose.position.x = float(self._body_frame_trans[0])
            pose_cam.pose.position.y = float(self._body_frame_trans[1])
            pose_cam.pose.position.z = float(self._body_frame_trans[2])
            pose_cam.pose.orientation.w = 1.0
            transformed = self._tf_buffer.transform(
                pose_cam, 'my_spot/odom', timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(
                f'Cannot transform body frame to odom: {e}',
                throttle_duration_sec=2.0)
            return

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'my_spot/odom'
        tf.child_frame_id = 'patient_body'
        tf.transform.translation.x = transformed.pose.position.x
        tf.transform.translation.y = transformed.pose.position.y
        tf.transform.translation.z = transformed.pose.position.z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def _publish_approach_marker(self, pos: np.ndarray, header):
        m = Marker()
        m.header.frame_id = header.frame_id
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'approach_point'
        m.id     = 1
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.approach_marker_pub.publish(m)


def main():
    rclpy.init()
    node = LayingHumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
