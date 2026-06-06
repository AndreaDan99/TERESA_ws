#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32


# ============================================================
# Utils
# ============================================================

def normalize(v, eps=1e-9):
    n = float(np.linalg.norm(v))
    return v / (n + eps)

def angle_deg(a, b):
    a = normalize(a)
    b = normalize(b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(math.acos(c)))

def nan():
    return float("nan")

def percentile_height(points, up, p):
    proj = [float(np.dot(pt, up)) for pt in points]
    return float(np.percentile(proj, p))


# ============================================================
# COCO/YOLO keypoint indices
# ============================================================

L_SHOULDER = 5
R_SHOULDER = 6
L_HIP = 11
R_HIP = 12
L_KNEE = 13
R_KNEE = 14
L_ANKLE = 15
R_ANKLE = 16


# ============================================================
# Node
# ============================================================

class HumanPostureAnalyzerSpot(Node):

    def __init__(self):
        super().__init__("human_posture_analyzer_spot")

        # Frame (deve corrispondere al frame della camera Orbbec)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.frame_id = self.get_parameter("frame_id").value

        # Soglie classificazione
        self.declare_parameter("knee_angle_stand_min", 160.0)
        self.declare_parameter("knee_angle_sit_max", 120.0)
        self.declare_parameter("torso_angle_lying_min", 65.0)
        self.declare_parameter("hip_knee_ratio_sit_max", 0.8)

        self.knee_stand = float(self.get_parameter("knee_angle_stand_min").value)
        self.knee_sit = float(self.get_parameter("knee_angle_sit_max").value)
        self.torso_lying = float(self.get_parameter("torso_angle_lying_min").value)
        self.hip_knee_ratio = float(self.get_parameter("hip_knee_ratio_sit_max").value)

        # Subscriber
        self.sub = self.create_subscription(
            PoseArray,
            "/human_pose/points_3d",
            self.cb_points,
            10
        )

        # Publishers
        self.pub_posture = self.create_publisher(String, "/human_pose/posture", 10)
        self.pub_conf = self.create_publisher(Float32, "/human_pose/posture_confidence", 10)
        self.pub_height = self.create_publisher(Float32, "/human_pose/body_height_m", 10)
        self.pub_angle = self.create_publisher(Float32, "/human_pose/torso_angle_deg", 10)
        self.pub_marker = self.create_publisher(Marker, "/human_pose/torso_marker", 10)

        self.get_logger().info("✅ HumanPostureAnalyzer (Orbbec) READY")

    # ============================================================
    # Callback
    # ============================================================

    def cb_points(self, msg: PoseArray):
        pts = []
        for p in msg.poses:
            if math.isnan(p.position.x):
                pts.append(None)
            else:
                pts.append(np.array([p.position.x, p.position.y, p.position.z], dtype=np.float64))

        posture, conf, height, angle, origin, vec = self.estimate_posture(pts)

        self.pub_posture.publish(String(data=posture))
        self.pub_conf.publish(Float32(data=conf))
        self.pub_height.publish(Float32(data=height if height is not None else nan()))
        self.pub_angle.publish(Float32(data=angle if angle is not None else nan()))

        if origin is not None and vec is not None:
            self.publish_torso_marker(origin, vec, msg.header.stamp)

        if conf > 0.5:
            h_str = f"{height:.2f}m" if height is not None else "N/A"
            self.get_logger().info(
                f"Posture: {posture} (conf: {conf:.2f}, height: {h_str}, angle: {angle:.1f}°)",
                throttle_duration_sec=2.0
            )

    # ============================================================
    # Core logic
    # ============================================================

    def estimate_posture(self, pts):
        # Camera optical frame: UP = -Y (Orbbec standard)
        up = np.array([0.0, -1.0, 0.0], dtype=np.float64)

        valid = [p for p in pts if p is not None]
        quality = len(valid) / 17.0

        if len(valid) < 4:
            return "UNKNOWN", 0.0, None, None, None, None

        # --------------------------------------------------
        # ALTEZZA ANATOMICA
        # --------------------------------------------------
        height = None

        shoulders = []
        if pts[L_SHOULDER] is not None:
            shoulders.append(pts[L_SHOULDER])
        if pts[R_SHOULDER] is not None:
            shoulders.append(pts[R_SHOULDER])

        feet = []
        if pts[L_ANKLE] is not None:
            feet.append(pts[L_ANKLE])
        if pts[R_ANKLE] is not None:
            feet.append(pts[R_ANKLE])

        if len(feet) == 0:
            if pts[L_KNEE] is not None:
                feet.append(pts[L_KNEE])
            if pts[R_KNEE] is not None:
                feet.append(pts[R_KNEE])

        if len(shoulders) > 0 and len(feet) > 0:
            sh_mid = np.mean(shoulders, axis=0)
            feet_mid = np.mean(feet, axis=0)
            height = abs(np.dot(sh_mid - feet_mid, up))
        else:
            height = None

        # --------------------------------------------------
        # Hip midpoint
        # --------------------------------------------------
        hip_mid = None
        if pts[L_HIP] is not None and pts[R_HIP] is not None:
            hip_mid = 0.5 * (pts[L_HIP] + pts[R_HIP])
        elif pts[L_HIP] is not None:
            hip_mid = pts[L_HIP]
        elif pts[R_HIP] is not None:
            hip_mid = pts[R_HIP]

        if hip_mid is None:
            return "UNKNOWN", 0.0, height, None, None, None

        # --------------------------------------------------
        # Knee midpoint
        # --------------------------------------------------
        knee_mid = None
        knees = []
        if pts[L_KNEE] is not None:
            knees.append(pts[L_KNEE])
        if pts[R_KNEE] is not None:
            knees.append(pts[R_KNEE])

        if len(knees) > 0:
            knee_mid = np.mean(knees, axis=0)

        # --------------------------------------------------
        # Torso vector + angle
        # --------------------------------------------------
        if len(shoulders) == 0:
            return "UNKNOWN", 0.0, height, None, None, None

        sh_mid = np.mean(shoulders, axis=0)
        torso_vec = sh_mid - hip_mid
        torso_angle = angle_deg(torso_vec, up)

        # --------------------------------------------------
        # Knee angles
        # --------------------------------------------------
        def knee_angle(h, k, a):
            v1 = h - k
            v2 = a - k
            if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
                return None
            return angle_deg(v1, v2)

        knee_angles = []

        if pts[L_HIP] is not None and pts[L_KNEE] is not None and pts[L_ANKLE] is not None:
            ang = knee_angle(pts[L_HIP], pts[L_KNEE], pts[L_ANKLE])
            if ang is not None:
                knee_angles.append(ang)

        if pts[R_HIP] is not None and pts[R_KNEE] is not None and pts[R_ANKLE] is not None:
            ang = knee_angle(pts[R_HIP], pts[R_KNEE], pts[R_ANKLE])
            if ang is not None:
                knee_angles.append(ang)

        avg_knee_angle = float(np.mean(knee_angles)) if len(knee_angles) > 0 else None

        # --------------------------------------------------
        # Hip-to-knee vertical distance ratio
        # --------------------------------------------------
        hip_knee_dist_ratio = None
        if knee_mid is not None and height > 0.1:
            hip_knee_vertical = abs(np.dot(hip_mid - knee_mid, up))
            hip_knee_dist_ratio = hip_knee_vertical / height

        # --------------------------------------------------
        # CLASSIFICATION
        # --------------------------------------------------
        posture = "UNKNOWN"
        score = 0.0

        # Fallback: torso-only classification when knees not visible
        if avg_knee_angle is None and hip_knee_dist_ratio is None:
            if torso_angle > 65.0:
                posture = "LYING"
                score = 0.70
            elif torso_angle < 25.0:
                posture = "STANDING"
                score = 0.65
            else:
                posture = "SITTING"
                score = 0.55

        # LYING
        if torso_angle > self.torso_lying and (height is None or height < 0.50):
            posture = "LYING"
            score = 0.85
            if avg_knee_angle is not None and avg_knee_angle < 140:
                score += 0.05

        # SITTING
        elif avg_knee_angle is not None and avg_knee_angle < self.knee_sit:
            posture = "SITTING"
            score = 0.70
            if hip_knee_dist_ratio is not None and hip_knee_dist_ratio < self.hip_knee_ratio:
                score += 0.15
            if torso_angle < 45:
                score += 0.05

        # STANDING
        elif avg_knee_angle is not None and avg_knee_angle > self.knee_stand:
            posture = "STANDING"
            score = 0.70
            if hip_knee_dist_ratio is not None and hip_knee_dist_ratio > 0.25:
                score += 0.15
            if torso_angle < 30:
                score += 0.05

        # FALLBACK: hip-knee ratio
        elif hip_knee_dist_ratio is not None:
            if hip_knee_dist_ratio < 0.15:
                posture = "SITTING"
                score = 0.50
            elif hip_knee_dist_ratio > 0.30:
                posture = "STANDING"
                score = 0.50
            else:
                posture = "SITTING"
                score = 0.35

        else:
            posture = "UNKNOWN"
            score = 0.20

        conf = score + 0.15 * quality
        conf = float(np.clip(conf, 0.0, 1.0))

        return posture, conf, height, torso_angle, hip_mid, torso_vec

    # ============================================================
    # Visualization
    # ============================================================

    def publish_torso_marker(self, origin, vec, stamp):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "torso"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.scale.x = 0.03
        m.scale.y = 0.06
        m.scale.z = 0.06

        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        p0 = origin
        p1 = origin + normalize(vec) * 0.45

        m.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])))
        m.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))

        self.pub_marker.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPostureAnalyzerSpot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
