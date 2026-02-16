#!/usr/bin/env python3
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped

import tf2_ros

import torch
import smplx


# ============================================================
#                    Math utilities
# ============================================================

def normalize(v, eps=1e-9):
    n = float(np.linalg.norm(v))
    return v / (n + eps)

def rotmat_to_axis_angle(Rm: np.ndarray) -> np.ndarray:
    """Rotation matrix (3x3) -> axis-angle (3,)"""
    tr = float(np.trace(Rm))
    c = np.clip((tr - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-8:
        return np.zeros(3, dtype=np.float32)

    axis = np.array([
        Rm[2, 1] - Rm[1, 2],
        Rm[0, 2] - Rm[2, 0],
        Rm[1, 0] - Rm[0, 1]
    ], dtype=np.float64)
    axis = normalize(axis)
    return (axis * angle).astype(np.float32)

def axis_angle_to_quat(rotvec: np.ndarray) -> np.ndarray:
    """
    axis-angle (3,) -> quaternion (x,y,z,w)  (unit quaternion)
    """
    rv = np.asarray(rotvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    axis = rv / angle
    s = math.sin(angle * 0.5)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(angle*0.5)], dtype=np.float64)

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton product q = q1 * q2
    quaternions in (x,y,z,w)
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=np.float64)

def quat_normalize(q: np.ndarray, eps=1e-12) -> np.ndarray:
    n = float(np.linalg.norm(q))
    return q / (n + eps)

def torso_frame(pelvis, sh_l, sh_r):
    """
    Build body frame from shoulders + pelvis:
    x = right (R_sh - L_sh)
    y = up (shoulders_mid - pelvis)
    z = forward = x cross y
    """
    x = normalize(sh_r - sh_l)                     # right
    y = normalize(((sh_l + sh_r) * 0.5) - pelvis)  # up
    z = normalize(np.cross(x, y))                  # forward
    y = normalize(np.cross(z, x))                  # re-orthonormalize
    return np.column_stack([x, y, z]).astype(np.float32)


# ============================================================
#                    COCO indices
# ============================================================

L_SHOULDER, R_SHOULDER = 5, 6
L_HIP, R_HIP = 11, 12


# ============================================================
#                      Node
# ============================================================

class SmplMeshNode(Node):

    def __init__(self):
        super().__init__("smpl_mesh_from_kf_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("smpl_model_folder", "/home/andrea/smpl_models")
        self.declare_parameter("mesh_rate_hz", 10.0)
        self.declare_parameter("src_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_frame", "camera_link")

        # Mesh marker resource (COLLADA/DAE consigliato per RViz)
        self.declare_parameter("mesh_resource", "file:///home/andrea/smpl_neutral.dae")
        self.declare_parameter("mesh_scale", 1.0)
        self.declare_parameter("mesh_offset_xyz", [0.0, -0.12, 0.0])

        # Questo è IL FIX di orientamento: 180° su Y (come quando “era di spalle”)
        # Lo applichiamo al marker, non allo SMPL.
        self.declare_parameter("apply_flip_y_pi", True)

        self.model_folder = self.get_parameter("smpl_model_folder").value
        self.src_frame = self.get_parameter("src_frame").value
        self.publish_frame = self.get_parameter("publish_frame").value

        self.mesh_resource = self.get_parameter("mesh_resource").value
        self.mesh_scale = float(self.get_parameter("mesh_scale").value)
        self.mesh_offset = np.array(self.get_parameter("mesh_offset_xyz").value, dtype=np.float64)
        self.apply_flip = bool(self.get_parameter("apply_flip_y_pi").value)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._cached_T = None  # (R, p)

        # ---------------- SMPL-X (solo per ottenere orientamento globale robusto) ----------------
        # Nota: qui non stiamo generando vertici, solo calcolando global_orient.
        # Ma lasciamo smplx caricato per coerenza con la pipeline.
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.smpl = smplx.create(
            model_path=self.model_folder,
            model_type="smplx",
            gender="neutral",
            num_betas=10,
            use_pca=False,
            batch_size=1
        ).to(self.device)
        self.smpl.eval()

        # ---------------- ROS ----------------
        self.sub = self.create_subscription(PoseArray, "/human_pose/points_3d", self.cb_pose, 10)
        self.pub = self.create_publisher(Marker, "/human_pose/smpl_mesh", 10)

        self.last_pts = None
        self.last_stamp = None

        period = 1.0 / max(float(self.get_parameter("mesh_rate_hz").value), 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info("✅ SMPL mesh node ready (MESH_RESOURCE fast)")

    # ============================================================
    #                  TF utilities
    # ============================================================

    def get_transform(self):
        if self._cached_T is not None:
            return self._cached_T

        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.publish_frame,
                self.src_frame,
                rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation

            Rm = self.quat_to_rot(q.x, q.y, q.z, q.w)
            p = np.array([t.x, t.y, t.z], dtype=np.float64)

            self._cached_T = (Rm, p)
            self.get_logger().info(f"Using TF {self.src_frame} → {self.publish_frame}")
            return self._cached_T

        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return None

    def quat_to_rot(self, x, y, z, w):
        # (x,y,z,w) -> R
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
        ], dtype=np.float64)

    def transform_point(self, p_cam: np.ndarray) -> np.ndarray:
        T = self.get_transform()
        if T is None:
            return p_cam
        Rm, t = T
        return (Rm @ p_cam.reshape(3, 1)).reshape(3,) + t

    # ============================================================
    #                  Pose callback
    # ============================================================

    def cb_pose(self, msg: PoseArray):
        pts = []
        for p in msg.poses[:17]:
            x, y, z = float(p.position.x), float(p.position.y), float(p.position.z)
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                pts.append(None)
            else:
                pts.append(np.array([x, y, z], dtype=np.float64))

        self.last_pts = pts
        self.last_stamp = msg.header.stamp

    # ============================================================
    #                  Orientation from torso
    # ============================================================

    def compute_global_orient_rotvec(self, pts):
        """
        Return axis-angle (rotvec) in camera frame, using pelvis + shoulders.
        """
        if pts is None:
            return None, None

        if any(pts[i] is None for i in [L_HIP, R_HIP, L_SHOULDER, R_SHOULDER]):
            return None, None

        pelvis = 0.5 * (pts[L_HIP] + pts[R_HIP])
        R0 = torso_frame(pelvis, pts[L_SHOULDER], pts[R_SHOULDER])  # 3x3
        rotvec = rotmat_to_axis_angle(R0)  # (3,)
        return pelvis, rotvec

    # ============================================================
    #                  Timer
    # ============================================================

    def on_timer(self):
        if self.last_pts is None or self.last_stamp is None:
            return

        pelvis_cam, rotvec_cam = self.compute_global_orient_rotvec(self.last_pts)
        if pelvis_cam is None:
            return

        # Position: pelvis + offset (in camera frame), then transform to publish_frame
        pos_cam = pelvis_cam + self.mesh_offset
        pos_pub = self.transform_point(pos_cam)

        # Orientation: rotvec -> quat in camera frame
        q_cam = axis_angle_to_quat(rotvec_cam)

        # Apply 180° flip around Y IF requested (this is the “perfect orientation” fix)
        # quaternion for 180° around Y is (x=0, y=1, z=0, w=0)
        if self.apply_flip:
            q_flip = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
            q_cam = quat_mul(q_cam, q_flip)   # same effect as "R0 @ rot_y_pi" but applied in marker space
            q_cam = quat_normalize(q_cam)

        # Now rotate into publish_frame too (same transform as points)
        T = self.get_transform()
        if T is not None:
            Rm, _ = T
            # Convert Rm -> quaternion and compose: q_pub = q_tf * q_cam
            q_tf = self.rot_to_quat(Rm)
            q_pub = quat_mul(q_tf, q_cam)
            q_pub = quat_normalize(q_pub)
        else:
            q_pub = q_cam

        self.publish_mesh(pos_pub, q_pub)

    def rot_to_quat(self, Rm: np.ndarray) -> np.ndarray:
        """Rotation matrix -> quaternion (x,y,z,w)"""
        tr = float(np.trace(Rm))
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * S
            x = (Rm[2,1] - Rm[1,2]) / S
            y = (Rm[0,2] - Rm[2,0]) / S
            z = (Rm[1,0] - Rm[0,1]) / S
        else:
            # find max diagonal
            if (Rm[0,0] > Rm[1,1]) and (Rm[0,0] > Rm[2,2]):
                S = math.sqrt(1.0 + Rm[0,0] - Rm[1,1] - Rm[2,2]) * 2.0
                w = (Rm[2,1] - Rm[1,2]) / S
                x = 0.25 * S
                y = (Rm[0,1] + Rm[1,0]) / S
                z = (Rm[0,2] + Rm[2,0]) / S
            elif Rm[1,1] > Rm[2,2]:
                S = math.sqrt(1.0 + Rm[1,1] - Rm[0,0] - Rm[2,2]) * 2.0
                w = (Rm[0,2] - Rm[2,0]) / S
                x = (Rm[0,1] + Rm[1,0]) / S
                y = 0.25 * S
                z = (Rm[1,2] + Rm[2,1]) / S
            else:
                S = math.sqrt(1.0 + Rm[2,2] - Rm[0,0] - Rm[1,1]) * 2.0
                w = (Rm[1,0] - Rm[0,1]) / S
                x = (Rm[0,2] + Rm[2,0]) / S
                y = (Rm[1,2] + Rm[2,1]) / S
                z = 0.25 * S

        return quat_normalize(np.array([x, y, z, w], dtype=np.float64))

    # ============================================================
    #                  Publish
    # ============================================================

    def publish_mesh(self, position, q_xyzw):
        m = Marker()
        m.header.frame_id = self.publish_frame
        m.header.stamp = self.last_stamp
        m.ns = "smpl_mesh"
        m.id = 0

        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = self.mesh_resource
        m.mesh_use_embedded_materials = False
        m.action = Marker.ADD

        m.pose.position.x = float(position[0])
        m.pose.position.y = float(position[1])
        m.pose.position.z = float(position[2])

        m.pose.orientation.x = float(q_xyzw[0])
        m.pose.orientation.y = float(q_xyzw[1])
        m.pose.orientation.z = float(q_xyzw[2])
        m.pose.orientation.w = float(q_xyzw[3])

        m.scale.x = self.mesh_scale
        m.scale.y = self.mesh_scale
        m.scale.z = self.mesh_scale

        # If you want colors to work, keep alpha > 0 and embedded materials off
        m.color.r = 0.1
        m.color.g = 0.8
        m.color.b = 0.9
        m.color.a = 0.9

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SmplMeshNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
