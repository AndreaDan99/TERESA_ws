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
# COCO indices
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

class HumanPostureAnalyzer(Node):

    def __init__(self):
        super().__init__("human_posture_analyzer_node")

        # Frame
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.frame_id = self.get_parameter("frame_id").value

        # Soglie (robuste, non dipendono dalla distanza)
        self.declare_parameter("hip_to_base_stand", 0.35)
        self.declare_parameter("hip_to_base_sit", 0.20)
        self.declare_parameter("lying_height_max", 0.45)
        self.declare_parameter("lying_angle_min", 65.0)

        self.th_stand = float(self.get_parameter("hip_to_base_stand").value)
        self.th_sit = float(self.get_parameter("hip_to_base_sit").value)
        self.th_lying_h = float(self.get_parameter("lying_height_max").value)
        self.th_lying_ang = float(self.get_parameter("lying_angle_min").value)

        # Subscribers
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

        self.get_logger().info("✅ HumanPostureAnalyzer READY (standing / sitting / lying)")

    # ============================================================
    # Callback
    # ============================================================

    def cb_points(self, msg: PoseArray):
        pts = []
        for p in msg.poses:
            if math.isnan(p.position.x):
                pts.append(None)
            else:
                pts.append(np.array([p.position.x, p.position.y, p.position.z]))

        posture, conf, height, angle, origin, vec = self.estimate_posture(pts)

        self.pub_posture.publish(String(data=posture))
        self.pub_conf.publish(Float32(data=conf))
        self.pub_height.publish(Float32(data=height if height is not None else nan()))
        self.pub_angle.publish(Float32(data=angle if angle is not None else nan()))

        if origin is not None and vec is not None:
            self.publish_torso_marker(origin, vec, msg.header.stamp)

    # ============================================================
    # Core logic
    # ============================================================

    def estimate_posture(self, pts):
        """
        Output:
        posture, confidence, height, torso_angle, torso_origin, torso_vec
        """

        # Camera optical frame: UP = -Y
        up = np.array([0.0, -1.0, 0.0])

        valid = [p for p in pts if p is not None]
        quality = len(valid) / 17.0

        if len(valid) < 4:
            return "UNKNOWN", 0.0, None, None, None, None

        # --------------------------------------------------
        # Robust body height (percentiles)
        # --------------------------------------------------
        h10 = percentile_height(valid, up, 10)
        h90 = percentile_height(valid, up, 90)
        height = h90 - h10

        # --------------------------------------------------
        # Hip midpoint
        # --------------------------------------------------
        hip_mid = None
        if pts[L_HIP] is not None and pts[R_HIP] is not None:
            hip_mid = 0.5 * (pts[L_HIP] + pts[R_HIP])

        # --------------------------------------------------
        # Base of support
        # --------------------------------------------------
        feet = []
        if pts[L_ANKLE] is not None:
            feet.append(pts[L_ANKLE])
        if pts[R_ANKLE] is not None:
            feet.append(pts[R_ANKLE])

        if len(feet) > 0:
            base = np.mean(feet, axis=0)
        elif hip_mid is not None:
            base = hip_mid
        else:
            return "UNKNOWN", 0.0, height, None, None, None

        # --------------------------------------------------
        # Torso vector + angle
        # --------------------------------------------------
        if pts[L_SHOULDER] is None or pts[R_SHOULDER] is None or hip_mid is None:
            return "UNKNOWN", 0.0, height, None, None, None

        sh_mid = 0.5 * (pts[L_SHOULDER] + pts[R_SHOULDER])
        torso_vec = sh_mid - hip_mid
        torso_angle = angle_deg(torso_vec, up)

        # --------------------------------------------------
        # Hip to base vertical distance
        # --------------------------------------------------
        hip_to_base = abs(np.dot(hip_mid - base, up))

        # --------------------------------------------------
        # CLASSIFICATION
        # --------------------------------------------------
        posture = "UNKNOWN"
        score = 0.0

        # ---------- LYING ----------
        if torso_angle > self.th_lying_ang and height < self.th_lying_h:
            posture = "LYING"
            score = 0.75

            # safety: se anche ancora alte → penalizza
            if hip_to_base > 0.25:
                score -= 0.2

        # ---------- STANDING ----------
        elif hip_to_base > self.th_stand:
            posture = "STANDING"
            score = 0.6

        # ---------- SITTING ----------
        elif hip_to_base < self.th_sit:
            posture = "SITTING"
            score = 0.6

        else:
            posture = "SITTING"
            score = 0.45

        # --------------------------------------------------
        # Knee refinement (confidence only)
        # --------------------------------------------------
        def knee_angle(h, k, a):
            v1 = h - k
            v2 = a - k
            if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
                return None
            return angle_deg(v1, v2)

        angles = []

        if pts[L_HIP] is not None and pts[L_KNEE] is not None and pts[L_ANKLE] is not None:
            ang = knee_angle(pts[L_HIP], pts[L_KNEE], pts[L_ANKLE])
            if ang is not None:
                angles.append(ang)

        if pts[R_HIP] is not None and pts[R_KNEE] is not None and pts[R_ANKLE] is not None:
            ang = knee_angle(pts[R_HIP], pts[R_KNEE], pts[R_ANKLE])
            if ang is not None:
                angles.append(ang)

        if len(angles) > 0:
            avg = float(np.mean(angles))
            if posture == "STANDING" and avg > 160.0:
                score += 0.1
            if posture == "SITTING" and avg < 140.0:
                score += 0.1

        # --------------------------------------------------
        # Final confidence
        # --------------------------------------------------
        conf = score + 0.25 * quality
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
    node = HumanPostureAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()