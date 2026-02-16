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
#                    Kalman Filter 3D
# ============================================================

class Kalman3D:
    def __init__(self, dt=1/30, q=0.02, r=0.01, p0=1.0):
        self.dt = dt
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * p0

        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

        self.Q = np.eye(6) * q
        self.R = np.eye(3) * r
        self.initialized = False

    def predict(self, vel_damping=1.0):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[3:6] *= vel_damping

    def update(self, z):
        z = z.reshape(3, 1)
        if not self.initialized:
            self.x[0:3] = z
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0:3].flatten()

    def set_position(self, p):
        self.x[0:3, 0] = p


# ============================================================
#           Geometry Utilities
# ============================================================

def quat_from_z_axis(v):
    v = v / (np.linalg.norm(v) + 1e-9)
    z = np.array([0, 0, 1.0])
    c = np.cross(z, v)
    d = np.dot(z, v)
    if d < -0.999:
        return (1, 0, 0, 0)
    s = np.sqrt((1 + d) * 2)
    return (c[0]/s, c[1]/s, c[2]/s, s/2)


def midpoint(a, b):
    return 0.5 * (a + b)


# ============================================================
#                   Main Node
# ============================================================

class YoloSkeletonMannequinNode(Node):
    def __init__(self):
        super().__init__("yolo_skeleton_mannequin")

        self.model = YOLO("yolov8n-pose.pt")
        self.bridge = CvBridge()

        self.sub_color = self.create_subscription(Image, "/camera/camera/color/image_raw", self.cb_color, 10)
        self.sub_depth = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, 10)
        self.sub_info = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.cb_info, 10)

        self.pub_mannequin = self.create_publisher(MarkerArray, "/human_pose/mannequin_markers", 10)

        self.depth = None
        self.info = None

        self.num_joints = 17
        self.kf = [Kalman3D() for _ in range(self.num_joints)]

        self.edges = [
            (5,7),(7,9),(6,8),(8,10),
            (11,13),(13,15),(12,14),(14,16),
            (5,6),(11,12),(5,11),(6,12)
        ]

    def cb_info(self, msg): self.info = msg
    def cb_depth(self, msg): self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def cb_color(self, msg):
        if self.depth is None or self.info is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.model(img, verbose=False)

        if res[0].keypoints is None:
            return

        kp = res[0].keypoints.xy.cpu().numpy()[0]

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]

        pts = [None]*17
        for i,(u,v) in enumerate(kp):
            u,v = int(u), int(v)
            if u<0 or v<0 or v>=self.depth.shape[0] or u>=self.depth.shape[1]:
                self.kf[i].predict(0.6)
            else:
                d = self.depth[v,u]*0.001
                if d>0:
                    X = (u-cx)*d/fx
                    Y = (v-cy)*d/fy
                    Z = d
                    self.kf[i].predict()
                    self.kf[i].update(np.array([X,Y,Z]))
            if self.kf[i].initialized:
                pts[i] = self.kf[i].get_position()

        self.publish_mannequin(pts, msg.header.stamp)

    # ======================================================
    #               MANNEQUIN PUBLISH
    # ======================================================

    def publish_mannequin(self, pts, stamp):
        ma = MarkerArray()
        frame = "camera_color_optical_frame"
        mid = 0

        # -------- HEAD --------
        if pts[0] is not None:
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = "head"
            m.id = mid; mid+=1
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pts[0]
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.r = 1.0; m.color.g = 0.8; m.color.b = 0.6; m.color.a = 1.0
            ma.markers.append(m)

        # -------- TORSO --------
        if all(pts[i] is not None for i in [5,6,11,12]):
            c_sh = midpoint(pts[5], pts[6])
            c_hp = midpoint(pts[11], pts[12])
            center = midpoint(c_sh, c_hp)

            height = np.linalg.norm(c_sh - c_hp)
            width = np.linalg.norm(pts[5] - pts[6])

            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = "torso"
            m.id = mid; mid+=1
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = center
            m.scale.x = width
            m.scale.y = width * 0.6
            m.scale.z = height
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 0.9; m.color.a = 0.9
            ma.markers.append(m)

        # -------- LIMBS --------
        for a,b in self.edges:
            if pts[a] is None or pts[b] is None:
                continue
            pa, pb = pts[a], pts[b]
            v = pb - pa
            L = np.linalg.norm(v)
            q = quat_from_z_axis(v)

            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = "limbs"
            m.id = mid; mid+=1
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = midpoint(pa,pb)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x = m.scale.y = 0.08
            m.scale.z = L
            m.color.r = 0.7; m.color.g = 0.7; m.color.b = 0.7; m.color.a = 1.0
            ma.markers.append(m)

        self.pub_mannequin.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(YoloSkeletonMannequinNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
