#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

# ============================================================
#           Simple 3D Kalman Filter (per keypoint)
#   State: [x,y,z,vx,vy,vz]
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.02, r=0.01, p0=1.0):
        self.dt = float(dt)

        self.x = np.zeros((6, 1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * float(p0)

        self.F = np.eye(6, dtype=np.float64)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt

        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.Q = np.eye(6, dtype=np.float64) * float(q)
        self.R = np.eye(3, dtype=np.float64) * float(r)

        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        self.x[3:, 0] *= vel_damping

    def update(self, z):
        z = np.asarray(z, dtype=np.float64).reshape(3, 1)

        if not self.initialized:
            self.x[0:3] = z
            self.x[3:6] = 0.0
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0:3, 0].copy()

    def set_position(self, p):
        self.x[0:3, 0] = p.reshape(3)


# ============================================================
#                 Constraint utilities
# ============================================================

def robust_median_and_mad(values):
    if len(values) == 0:
        return None, None
    v = np.asarray(values)
    med = float(np.median(v))
    mad = float(np.median(np.abs(v - med)))
    return med, 1.4826 * mad


def apply_bone_constraints(pts, edges, lengths, iters=2, stiffness=1.0):
    for _ in range(iters):
        for (a, b) in edges:
            if (a, b) not in lengths:
                continue
            if pts[a] is None and pts[b] is None:
                continue

            if pts[a] is None or pts[b] is None:
                continue

            pa, pb = pts[a], pts[b]
            L = lengths[(a, b)]

            d = pb - pa
            dist = np.linalg.norm(d)
            if dist < 1e-6:
                continue

            u = d / dist
            err = dist - L
            corr = 0.5 * stiffness * err * u

            pts[a] += corr
            pts[b] -= corr

    return pts


# ============================================================
#                       Skeleton Node
# ============================================================

class YoloSkeletonNodeKFCalib(Node):

    def __init__(self):
        super().__init__("yolo_skeleton_kf_calib_node")

        # ---------------- Params ----------------
        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("dt", 1.0 / 30.0)
        self.declare_parameter("q", 0.02)
        self.declare_parameter("r", 0.01)

        self.declare_parameter("conf_thr", 0.30)
        self.declare_parameter("calib_frames", 60)
        self.declare_parameter("vel_damping", 0.6)
        self.declare_parameter("constraint_iters", 2)
        self.declare_parameter("constraint_stiffness", 1.0)
        self.declare_parameter("max_depth_m", 8.0)

        self.model = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()

        # ---------------- Subscriptions ----------------
        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.info_callback, 10
        )

        self.depth_image = None
        self.color_info = None

        # ---------------- Publishers ----------------
        self.pub_poses = self.create_publisher(PoseArray, "/human_pose/points_3d", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/human_pose/skeleton_markers", 10)

        self.num_joints = 17
        self.kf = [Kalman3D() for _ in range(self.num_joints)]

        self.edges = [
            (0,1),(0,2),(1,3),(2,4),
            (5,6),
            (5,7),(7,9),
            (6,8),(8,10),
            (11,12),
            (11,13),(13,15),
            (12,14),(14,16),
            (0,5),(0,6),
            (5,11),(6,12),
        ]

        self.initialized = False
        self.calibrated = False
        self.calib_count = 0
        self.edge_obs = {e: [] for e in self.edges}
        self.bone_lengths = {}

        self.get_logger().info("Skeleton node ready")

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------

    def compute_root(self, pts):
        anchors = [5,6,11,12]
        valid = [pts[i] for i in anchors if pts[i] is not None]
        if len(valid) < 3:
            return None
        return np.mean(valid, axis=0)

    def freeze_limb(self, parent, child, length):
        if self.kf[child].initialized and self.kf[parent].initialized:
            p = self.kf[parent].get_position()
            c = self.kf[child].get_position()
            d = c - p
            n = np.linalg.norm(d)
            if n > 1e-6:
                new = p + length * (d / n)
                self.kf[child].set_position(new)

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------

    def info_callback(self, msg):
        self.color_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def color_callback(self, msg):
        if self.depth_image is None or self.color_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img, verbose=False)

        if len(results) == 0 or results[0].keypoints is None:
            self.predict_only(msg.header.stamp)
            return

        kp = results[0].keypoints.xy[0].cpu().numpy()
        conf = None
        if results[0].keypoints.conf is not None:
            conf = results[0].keypoints.conf[0].cpu().numpy()

        fx, fy, cx, cy = self.get_intrinsics()

        pts = [None]*self.num_joints
        valid = [False]*self.num_joints

        for i in range(self.num_joints):
            if conf is not None and conf[i] < self.get_parameter("conf_thr").value:
                continue

            u,v = int(kp[i][0]), int(kp[i][1])
            if u<0 or v<0 or u>=self.depth_image.shape[1] or v>=self.depth_image.shape[0]:
                continue

            d = self.depth_image[v,u]*0.001
            if d<=0 or d>self.get_parameter("max_depth_m").value:
                continue

            X = (u-cx)*d/fx
            Y = (v-cy)*d/fy
            Z = d

            pts[i] = np.array([X,Y,Z])
            valid[i] = True

        for i in range(self.num_joints):
            if valid[i]:
                self.kf[i].predict(1.0)
                self.kf[i].update(pts[i])
            else:
                self.kf[i].predict(self.get_parameter("vel_damping").value)

            if self.kf[i].initialized:
                pts[i] = self.kf[i].get_position()
            else:
                pts[i] = None

        root = self.compute_root(pts)

        if self.calibrated:
            for (a,b) in [(11,13),(13,15),(12,14),(14,16)]:
                self.freeze_limb(a,b,self.bone_lengths[(a,b)])

            pts = apply_bone_constraints(
                pts, self.edges, self.bone_lengths,
                self.get_parameter("constraint_iters").value,
                self.get_parameter("constraint_stiffness").value
            )

            for i in range(self.num_joints):
                if pts[i] is not None and self.kf[i].initialized:
                    self.kf[i].set_position(pts[i])

        self.publish_all(pts, msg.header.stamp)

    # ---------------------------------------------------------

    def get_intrinsics(self):
        k = self.color_info.k
        return k[0], k[4], k[2], k[5]

    def predict_only(self, stamp):
        pts = []
        for k in self.kf:
            k.predict(self.get_parameter("vel_damping").value)
            pts.append(k.get_position() if k.initialized else None)
        self.publish_all(pts, stamp)

    def publish_all(self, pts, stamp):
        pa = PoseArray()
        pa.header.frame_id = "camera_color_optical_frame"
        pa.header.stamp = stamp

        for p in pts:
            pose = Pose()
            if p is None:
                pose.position.x = pose.position.y = pose.position.z = float("nan")
            else:
                pose.position.x,pose.position.y,pose.position.z = p
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.pub_poses.publish(pa)
        self.publish_markers(pts, stamp)

    def publish_markers(self, pts, stamp):
        ma = MarkerArray()

        j = Marker()
        j.header.frame_id = "camera_color_optical_frame"
        j.header.stamp = stamp
        j.ns = "joints"
        j.id = 0
        j.type = Marker.SPHERE_LIST
        j.scale.x = j.scale.y = j.scale.z = 0.03
        j.color.r = 1.0; j.color.a = 1.0

        for p in pts:
            if p is not None:
                j.points.append(Point(x=p[0],y=p[1],z=p[2]))
        ma.markers.append(j)

        b = Marker()
        b.header.frame_id = "camera_color_optical_frame"
        b.header.stamp = stamp
        b.ns = "bones"
        b.id = 1
        b.type = Marker.LINE_LIST
        b.scale.x = 0.015
        b.color.g = 1.0; b.color.a = 1.0

        for (a,c) in self.edges:
            if pts[a] is not None and pts[c] is not None:
                b.points.append(Point(x=pts[a][0],y=pts[a][1],z=pts[a][2]))
                b.points.append(Point(x=pts[c][0],y=pts[c][1],z=pts[c][2]))

        ma.markers.append(b)
        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = YoloSkeletonNodeKFCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
