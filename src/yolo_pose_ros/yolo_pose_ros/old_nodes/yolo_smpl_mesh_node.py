#!/usr/bin/env python3
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped

import tf2_ros

import torch
import smplx

# ============================================================
#                    Math utilities
# ============================================================

def normalize(v, eps=1e-9):
    n = np.linalg.norm(v)
    return v / (n + eps)

def axis_angle_from_two_vectors(v_from, v_to):
    a = normalize(v_from)
    b = normalize(v_to)
    c = np.clip(np.dot(a, b), -1.0, 1.0)

    if c > 0.9999:
        return np.zeros(3)
    if c < -0.9999:
        axis = normalize(np.cross(a, np.array([1.0, 0.0, 0.0])))
        return axis * math.pi

    axis = normalize(np.cross(a, b))
    angle = math.acos(c)
    return axis * angle

def rotmat_to_axis_angle(R):
    angle = math.acos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-8:
        return np.zeros(3)
    axis = np.array([
        R[2,1] - R[1,2],
        R[0,2] - R[2,0],
        R[1,0] - R[0,1]
    ])
    axis = normalize(axis)
    return axis * angle

def rot_y_pi():
    return np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float64)



def torso_frame(pelvis, sh_l, sh_r):
    x = normalize(sh_r - sh_l)                     # right
    y = normalize(((sh_l + sh_r) * 0.5) - pelvis)  # up
    z = normalize(np.cross(x, y))                  # forward
    y = normalize(np.cross(z, x))
    return np.column_stack([x, y, z])

# ============================================================
#                    COCO indices
# ============================================================

L_SHOULDER, R_SHOULDER = 5, 6
L_ELBOW, R_ELBOW = 7, 8
L_WRIST, R_WRIST = 9, 10
L_HIP, R_HIP = 11, 12
L_KNEE, R_KNEE = 13, 14
L_ANKLE, R_ANKLE = 15, 16

# ============================================================
#                    SMPL indices
# ============================================================

SMPL_L_HIP = 0
SMPL_R_HIP = 1
SMPL_L_KNEE = 3
SMPL_R_KNEE = 4
SMPL_L_ANKLE = 6
SMPL_R_ANKLE = 7
SMPL_SPINE1 = 2
SMPL_SPINE2 = 5
SMPL_SPINE3 = 8
SMPL_NECK = 11
SMPL_HEAD = 14
SMPL_L_SHOULDER = 15
SMPL_R_SHOULDER = 16
SMPL_L_ELBOW = 17
SMPL_R_ELBOW = 18
SMPL_L_WRIST = 19
SMPL_R_WRIST = 20

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

        self.model_folder = self.get_parameter("smpl_model_folder").value
        self.src_frame = self.get_parameter("src_frame").value
        self.publish_frame = self.get_parameter("publish_frame").value

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._cached_T = None

        # ---------------- SMPL-X ----------------
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

        self.faces = self.smpl.faces.astype(np.int32)
        self.declare_parameter("mesh_offset_xyz", [0.0, -0.12, 0.0])
        self.mesh_offset = np.array(self.get_parameter("mesh_offset_xyz").value, dtype=np.float32)
        # ---------------- ROS ----------------
        self.sub = self.create_subscription(
            PoseArray,
            "/human_pose/points_3d",
            self.cb_pose,
            10
        )

        self.pub = self.create_publisher(
            Marker,
            "/human_pose/smpl_mesh",
            10
        )

        self.last_pts = None
        self.last_stamp = None

        self.timer = self.create_timer(
            1.0 / self.get_parameter("mesh_rate_hz").value,
            self.on_timer
        )

        self.get_logger().info("✅ SMPL mesh node ready")

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

            R = self.quat_to_rot(q.x, q.y, q.z, q.w)
            p = np.array([t.x, t.y, t.z])

            self._cached_T = (R, p)
            self.get_logger().info(f"Using TF {self.src_frame} → {self.publish_frame}")
            return self._cached_T

        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return None

    def quat_to_rot(self, x, y, z, w):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
        ])

    def transform_vertices(self, V):
        T = self.get_transform()
        if T is None:
            return V
        R, p = T
        return (V @ R.T) + p

    # ============================================================
    #                  Pose callback
    # ============================================================

    def cb_pose(self, msg: PoseArray):
        pts = []
        for p in msg.poses[:17]:
            if math.isnan(p.position.x):
                pts.append(None)
            else:
                pts.append(np.array([p.position.x, p.position.y, p.position.z]))
        self.last_pts = pts
        self.last_stamp = msg.header.stamp

    # ============================================================
    #                  IK → SMPL
    # ============================================================

    def build_smpl(self, pts):
        transl = np.zeros((1,3))
        global_orient = np.zeros((1,3))
        body_pose = np.zeros((1,63))

        if any(pts[i] is None for i in [L_HIP, R_HIP, L_SHOULDER, R_SHOULDER]):
            return transl, global_orient, body_pose

        pelvis = 0.5 * (pts[L_HIP] + pts[R_HIP])
        transl[0] = pelvis

        R0 = torso_frame(pelvis, pts[L_SHOULDER], pts[R_SHOULDER])

        #SMPL guarda al contrario → flip 180° attorno all’asse UP del torso
        R0 = R0 @ rot_y_pi()

        global_orient[0] = rotmat_to_axis_angle(R0)
        Rt = R0.T


        v_down = np.array([0, -1, 0])

        def set(j, aa):
            body_pose[0, 3*j:3*j+3] = aa

        if pts[L_KNEE] is not None:
            set(SMPL_L_HIP, axis_angle_from_two_vectors(v_down, Rt@(pts[L_KNEE]-pts[L_HIP])))
        if pts[R_KNEE] is not None:
            set(SMPL_R_HIP, axis_angle_from_two_vectors(v_down, Rt@(pts[R_KNEE]-pts[R_HIP])))
        if pts[L_ANKLE] is not None:
            set(SMPL_L_KNEE, axis_angle_from_two_vectors(v_down, Rt@(pts[L_ANKLE]-pts[L_KNEE])))
        if pts[R_ANKLE] is not None:
            set(SMPL_R_KNEE, axis_angle_from_two_vectors(v_down, Rt@(pts[R_ANKLE]-pts[R_KNEE])))

        return transl, global_orient, body_pose

    # ============================================================
    #                  Timer
    # ============================================================

    def on_timer(self):
        if self.last_pts is None:
            return

        transl_np, glob_np, body_np = self.build_smpl(self.last_pts)

        with torch.no_grad():
            out = self.smpl(
                betas=torch.zeros((1, 10), dtype=torch.float32, device=self.device),
                transl=torch.tensor(transl_np, dtype=torch.float32, device=self.device),
                global_orient=torch.tensor(glob_np, dtype=torch.float32, device=self.device),
                body_pose=torch.tensor(body_np, dtype=torch.float32, device=self.device),

                left_hand_pose=torch.zeros((1, 45), dtype=torch.float32, device=self.device),
                right_hand_pose=torch.zeros((1, 45), dtype=torch.float32, device=self.device),
                jaw_pose=torch.zeros((1, 3), dtype=torch.float32, device=self.device),
                leye_pose=torch.zeros((1, 3), dtype=torch.float32, device=self.device),
                reye_pose=torch.zeros((1, 3), dtype=torch.float32, device=self.device),
                expression=torch.zeros((1, 10), dtype=torch.float32, device=self.device),
            )

            verts = out.vertices[0].cpu().numpy()


        verts = self.transform_vertices(verts)
        verts = verts + self.mesh_offset
        self.publish_mesh(verts)

    # ============================================================
    #                  Publish
    # ============================================================

    def publish_mesh(self, verts):
        m = Marker()
        m.header.frame_id = self.publish_frame
        m.header.stamp = self.last_stamp
        m.ns = "smpl_mesh"
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD

        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = 0.1
        m.color.g = 0.8
        m.color.b = 0.9
        m.color.a = 0.7

        m.pose.orientation.w = 1.0

        for f in self.faces:
            for idx in f:
                p = verts[idx]
                m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))

        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SmplMeshNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
