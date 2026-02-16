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
#                Kalman Filter 3D (robusto)
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.02, r=0.01, p0=1.0):
        self.dt = float(dt)

        self.x = np.zeros((6,1), dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * p0

        self.F = np.eye(6, dtype=np.float64)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = self.dt

        self.H = np.zeros((3,6), dtype=np.float64)
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0

        self.Q = np.eye(6, dtype=np.float64) * q
        self.R = np.eye(3, dtype=np.float64) * r

        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[3:,0] *= vel_damping

    def update(self, z):
        z = np.asarray(z, dtype=np.float64).reshape(3,1)

        if not self.initialized:
            self.x[0:3] = z
            self.x[3:] = 0.0
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_position(self):
        if not self.initialized:
            return None
        return self.x[0:3,0].copy()

    def set_position(self, p):
        self.x[0:3,0] = p.reshape(3)


# ============================================================
#                  Bone constraints
# ============================================================

def apply_bone_constraints(pts, edges, lengths, iters=2, stiffness=1.0):
    for _ in range(iters):
        for (a,b) in edges:
            if (a,b) not in lengths:
                continue
            if pts[a] is None or pts[b] is None:
                continue

            pa, pb = pts[a], pts[b]
            L = lengths[(a,b)]

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

class YoloSkeletonNodeStable(Node):

    def __init__(self):
        super().__init__("yolo_skeleton_node_stable")

        self.declare_parameter("model_path", "yolov8n-pose.pt")
        self.declare_parameter("conf_thr", 0.30)
        self.declare_parameter("vel_damping", 0.6)
        self.declare_parameter("max_depth_m", 3.0)

        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.model = YOLO(self.get_parameter("model_path").value)
        self.bridge = CvBridge()

        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.cb_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10
        )

        self.pub_poses = self.create_publisher(
            PoseArray, "/human_pose/points_3d", 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/human_pose/skeleton_markers", 10
        )

        self.depth_img = None
        self.cam_info = None

        self.num_joints = 17
        self.kf = [Kalman3D() for _ in range(self.num_joints)]
        self.missing = [0]*self.num_joints

        self.edges = [
            (0,1),(0,2),(1,3),(2,4),
            (5,6),
            (5,7),(7,9),
            (6,8),(8,10),
            (11,12),
            (11,13),(13,15),
            (12,14),(14,16),
            (5,11),(6,12),
        ]

        self.bone_lengths = {
            (5,11):0.45,(6,12):0.45,
            (11,13):0.42,(13,15):0.43,
            (12,14):0.42,(14,16):0.43,
            (5,7):0.30,(7,9):0.27,
            (6,8):0.30,(8,10):0.27
        }

        self.get_logger().info("✅ YOLO skeleton node (NO-PERSON SAFE) ready")

    # ============================================================

    def cb_info(self, msg):
        self.cam_info = msg

    def cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def robust_depth(self, u, v, win=5):
        h,w = self.depth_img.shape
        r = win//2
        u0,u1 = max(0,u-r), min(w,u+r+1)
        v0,v1 = max(0,v-r), min(h,v+r+1)

        patch = self.depth_img[v0:v1, u0:u1]
        patch = patch[patch>0]
        if patch.size < 6:
            return None
        return np.median(patch)*0.001

    # ============================================================

    def cb_color(self, msg):
        if self.depth_img is None or self.cam_info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.model(img, verbose=False)

        # ---------------- NO PERSON ----------------
        if len(res)==0 or res[0].keypoints is None or res[0].keypoints.xy.shape[0]==0:
            self.predict_only(msg.header.stamp)
            return

        kp = res[0].keypoints.xy[0].cpu().numpy()
        conf = res[0].keypoints.conf
        if conf is not None:
            conf = conf[0].cpu().numpy()

        fx,fy,cx,cy = self.cam_info.k[0],self.cam_info.k[4],self.cam_info.k[2],self.cam_info.k[5]

        pts = [None]*self.num_joints
        valid = [False]*self.num_joints

        for i in range(self.num_joints):
            if conf is not None:
                thr = 0.6 if i<5 else self.conf_thr
                if conf[i] < thr:
                    continue

            u,v = int(kp[i][0]), int(kp[i][1])
            if u<0 or v<0 or u>=self.depth_img.shape[1] or v>=self.depth_img.shape[0]:
                continue

            d = self.robust_depth(u,v)
            if d is None or d<=0 or d>self.max_depth_m:
                continue

            X = (u-cx)*d/fx
            Y = (v-cy)*d/fy
            Z = d

            pts[i] = np.array([X,Y,Z])
            valid[i] = True

        for i in range(self.num_joints):
            if valid[i]:
                self.missing[i]=0
                self.kf[i].predict(1.0)

                if self.kf[i].initialized:
                    if np.linalg.norm(pts[i]-self.kf[i].get_position())>0.35:
                        continue

                self.kf[i].update(pts[i])
            else:
                self.missing[i]+=1
                self.kf[i].predict(self.vel_damping)
                if self.missing[i]>8:
                    self.kf[i].x[3:,0]=0.0

            pts[i] = self.kf[i].get_position()

        pts = apply_bone_constraints(pts, self.edges, self.bone_lengths, 2, 0.9)

        for i in range(self.num_joints):
            if pts[i] is not None and self.kf[i].initialized:
                self.kf[i].set_position(pts[i])

        self.publish_all(pts, msg.header.stamp)

    # ============================================================

    def predict_only(self, stamp):
        pts=[]
        for i,k in enumerate(self.kf):
            if k.initialized:
                k.predict(self.vel_damping)
                pts.append(k.get_position())
            else:
                pts.append(None)
        self.publish_all(pts, stamp)

    # ============================================================

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

        ma = MarkerArray()

        j = Marker()
        j.header = pa.header
        j.ns="joints"; j.id=0
        j.type=Marker.SPHERE_LIST
        j.scale.x=j.scale.y=j.scale.z=0.03
        j.color.r=1.0; j.color.a=1.0
        for p in pts:
            if p is not None:
                j.points.append(Point(x=p[0],y=p[1],z=p[2]))
        ma.markers.append(j)

        b = Marker()
        b.header=pa.header
        b.ns="bones"; b.id=1
        b.type=Marker.LINE_LIST
        b.scale.x=0.015
        b.color.g=1.0; b.color.a=1.0
        for a,c in self.edges:
            if pts[a] is not None and pts[c] is not None:
                b.points.append(Point(x=pts[a][0],y=pts[a][1],z=pts[a][2]))
                b.points.append(Point(x=pts[c][0],y=pts[c][1],z=pts[c][2]))
        ma.markers.append(b)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = YoloSkeletonNodeStable()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()