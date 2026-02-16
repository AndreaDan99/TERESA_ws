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
#   Stato: [x,y,z,vx,vy,vz]
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

    def predict(self, vel_damping: float = 1.0):
        """Predict step. If no measurement arrives, use vel_damping<1 to avoid drift to infinity."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Damping velocities (only meaningful when measurement is missing)
        vel_damping = float(vel_damping)
        self.x[3, 0] *= vel_damping
        self.x[4, 0] *= vel_damping
        self.x[5, 0] *= vel_damping

    def update(self, z_xyz: np.ndarray):
        z = np.asarray(z_xyz, dtype=np.float64).reshape(3, 1)

        if not self.initialized:
            self.x[0:3] = z
            self.x[3:6] = 0.0
            self.initialized = True
            return

        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0:3, 0].copy()

    def set_position(self, p_xyz: np.ndarray):
        """Hard-set position (used by constraint projection)."""
        p = np.asarray(p_xyz, dtype=np.float64).reshape(3)
        self.x[0, 0] = p[0]
        self.x[1, 0] = p[1]
        self.x[2, 0] = p[2]


# ============================================================
#                 Constraint Projection Utilities
# ============================================================

def robust_median_and_mad(values: list[float]):
    """Return (median, mad_sigma) where mad_sigma approximates std (1.4826*MAD)."""
    if len(values) == 0:
        return None, None
    v = np.asarray(values, dtype=np.float64)
    med = float(np.median(v))
    mad = float(np.median(np.abs(v - med)))
    sigma = 1.4826 * mad
    return med, sigma


def apply_bone_constraints(points_xyz, edges, lengths, iters=2, stiffness=1.0):
    """
    points_xyz: list of np.array(3,) or None, len=17
    lengths: dict {(a,b): L} in meters
    iters: number of passes
    stiffness: 1.0 full correction, <1 softer
    """
    stiffness = float(stiffness)
    if stiffness <= 0.0:
        return points_xyz

    pts = points_xyz  # in-place edits on arrays
    for _ in range(int(iters)):
        for (a, b) in edges:
            key = (a, b)
            if key not in lengths:
                continue
            if pts[a] is None or pts[b] is None:
                continue

            L = float(lengths[key])
            pa = pts[a]
            pb = pts[b]
            d = pb - pa
            dist = float(np.linalg.norm(d))
            if dist < 1e-6:
                continue

            u = d / dist
            err = dist - L  # positive if too long
            corr = 0.5 * stiffness * err * u

            pts[a] = pa + corr
            pts[b] = pb - corr

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
        self.declare_parameter("calib_frames", 60)         # how many frames to collect
        self.declare_parameter("vel_damping", 0.60)         # applied when measurement missing
        self.declare_parameter("constraint_iters", 2)       # projection passes per frame
        self.declare_parameter("constraint_stiffness", 1.0) # 1 full, <1 softer
        self.declare_parameter("max_depth_m", 8.0)          # reject crazy depth

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.dt = self.get_parameter("dt").value
        self.q = self.get_parameter("q").value
        self.r = self.get_parameter("r").value

        self.conf_thr = float(self.get_parameter("conf_thr").value)
        self.calib_frames_target = int(self.get_parameter("calib_frames").value)
        self.vel_damping = float(self.get_parameter("vel_damping").value)
        self.constraint_iters = int(self.get_parameter("constraint_iters").value)
        self.constraint_stiffness = float(self.get_parameter("constraint_stiffness").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        # ---------------- YOLO ----------------
        self.get_logger().info(f"Loading YOLO Pose model: {self.model_path} ...")
        self.model = YOLO(self.model_path)

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
        self.kf = [Kalman3D(dt=self.dt, q=self.q, r=self.r) for _ in range(self.num_joints)]

        # ✅ Skeleton edges (torso connected like your “good” version)
        self.edges = [
            # Face
            (0, 1), (0, 2),
            (1, 3), (2, 4),

            # Shoulders
            (5, 6),

            # Arms
            (5, 7), (7, 9),
            (6, 8), (8, 10),

            # Hips
            (11, 12),

            # Legs
            (11, 13), (13, 15),
            (12, 14), (14, 16),

            # Upper torso links (nose->shoulders)
            (0, 5), (0, 6),

            # Torso sides (shoulders->hips)  ✅ fixes “arms detached from legs”
            (5, 11), (6, 12),
        ]

        # -------------- Calibration buffers --------------
        self.initialized = False           # first person ever seen
        self.calibrated = False
        self.calib_count = 0

        # store per-edge observed lengths during calibration
        self.edge_obs = {e: [] for e in self.edges}  # list of float
        self.bone_lengths = {}   # {(a,b): L}
        self.bone_sigmas = {}    # {(a,b): sigma}

        self.get_logger().info(
            f"Node ready. Calibration will collect {self.calib_frames_target} good frames."
        )

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------

    def info_callback(self, msg: CameraInfo):
        self.color_info = msg

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  # uint16 mm

    def _get_intrinsics(self):
        fx = float(self.color_info.k[0])
        fy = float(self.color_info.k[4])
        cx = float(self.color_info.k[2])
        cy = float(self.color_info.k[5])
        return fx, fy, cx, cy

    def _safe_parse_keypoints(self, results):
        """Return kp_xy (K,2), kp_conf (K,) or (None,None)"""
        if results is None or len(results) == 0:
            return None, None

        r0 = results[0]
        if r0.keypoints is None:
            return None, None

        kp_xy_all = r0.keypoints.xy
        if kp_xy_all is None:
            return None, None

        kp_xy_all = kp_xy_all.cpu().numpy()  # (N,17,2) typically
        if kp_xy_all.shape[0] == 0:
            return None, None

        kp_xy = kp_xy_all[0]  # first person

        # conf optional
        kp_conf = None
        try:
            conf = r0.keypoints.conf
            if conf is not None:
                conf = conf.cpu().numpy()
                # conf can be (N,17) or (N,17,1)
                if conf.ndim == 3:
                    kp_conf = conf[0, :, 0]
                elif conf.ndim == 2:
                    kp_conf = conf[0, :]
        except Exception:
            kp_conf = None

        return kp_xy, kp_conf

    def _keypoint_to_3d(self, u, v, fx, fy, cx, cy):
        """Return np.array([X,Y,Z]) in meters or None."""
        u = int(u); v = int(v)
        if u < 0 or v < 0 or v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            return None

        d_mm = float(self.depth_image[v, u])
        if d_mm <= 0.0:
            return None

        z = d_mm * 0.001
        if z <= 0.0 or z > self.max_depth_m:
            return None

        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z
        return np.array([X, Y, Z], dtype=np.float64)

    def color_callback(self, msg: Image):
        if self.depth_image is None or self.color_info is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(color_img, verbose=False)

        kp_xy, kp_conf = self._safe_parse_keypoints(results)

        # ----------------- NO PERSON -----------------
        if kp_xy is None:
            if not self.initialized:
                self.get_logger().warn("Waiting for first person...")
                return

            # already initialized: predict all with damping, then publish constrained skeleton
            pts = []
            for j in range(self.num_joints):
                self.kf[j].predict(vel_damping=self.vel_damping)
                if self.kf[j].initialized:
                    pts.append(self.kf[j].get_position())
                else:
                    pts.append(None)

            # apply constraints if calibrated
            if self.calibrated:
                pts = apply_bone_constraints(
                    pts, self.edges, self.bone_lengths,
                    iters=self.constraint_iters,
                    stiffness=self.constraint_stiffness,
                )
                # write back to kf positions (keeps visual stable)
                for j in range(self.num_joints):
                    if pts[j] is not None and self.kf[j].initialized:
                        self.kf[j].set_position(pts[j])

            self._publish_all(pts, msg.header.stamp)
            return

        # ----------------- PERSON FOUND -----------------
        if not self.initialized:
            self.initialized = True
            self.get_logger().info("First person detected. Starting calibration...")

        fx, fy, cx, cy = self._get_intrinsics()

        # Build raw 3D measurements for this frame
        meas_3d = [None] * self.num_joints
        valid_meas = [False] * self.num_joints

        for j in range(self.num_joints):
            u, v = kp_xy[j]
            conf_ok = True
            if kp_conf is not None:
                conf_ok = (float(kp_conf[j]) >= self.conf_thr)

            if not conf_ok:
                continue

            p3 = self._keypoint_to_3d(u, v, fx, fy, cx, cy)
            if p3 is None:
                continue

            meas_3d[j] = p3
            valid_meas[j] = True

        # ----------------- KALMAN UPDATE/PREDICT -----------------
        pts = [None] * self.num_joints
        for j in range(self.num_joints):
            if valid_meas[j]:
                self.kf[j].predict(vel_damping=1.0)     # no damping when we have measurement
                self.kf[j].update(meas_3d[j])
            else:
                self.kf[j].predict(vel_damping=self.vel_damping)

            if self.kf[j].initialized:
                pts[j] = self.kf[j].get_position()
            else:
                pts[j] = None

        # ----------------- CALIBRATION PHASE -----------------
        if (not self.calibrated) and (self.calib_count < self.calib_frames_target):
            # For calibration, require that a reasonable subset is present:
            # at least torso anchors: shoulders + hips (5,6,11,12)
            anchors = [5, 6, 11, 12]
            if all(valid_meas[a] for a in anchors):
                # collect bone lengths only when both endpoints have valid measurement
                for (a, b) in self.edges:
                    if valid_meas[a] and valid_meas[b]:
                        L = float(np.linalg.norm(meas_3d[a] - meas_3d[b]))
                        if 0.05 < L < 2.0:  # sanity
                            self.edge_obs[(a, b)].append(L)

                self.calib_count += 1
                if (self.calib_count % 10) == 0:
                    self.get_logger().info(f"Calibration progress: {self.calib_count}/{self.calib_frames_target}")
            else:
                self.get_logger().warn("Calibration pose not stable (missing torso anchors). Hold the pose...")

            # Once reached target, finalize
            if self.calib_count >= self.calib_frames_target:
                self._finalize_calibration()

        # ----------------- TRACKING (apply constraints) -----------------
        if self.calibrated:
            pts = apply_bone_constraints(
                pts, self.edges, self.bone_lengths,
                iters=self.constraint_iters,
                stiffness=self.constraint_stiffness,
            )
            # write back to KF positions (prevents drift on missing frames)
            for j in range(self.num_joints):
                if pts[j] is not None and self.kf[j].initialized:
                    self.kf[j].set_position(pts[j])

        # ----------------- Publish -----------------
        self._publish_all(pts, msg.header.stamp)

    # ---------------------------------------------------------

    def _finalize_calibration(self):
        good = 0
        for e in self.edges:
            med, sig = robust_median_and_mad(self.edge_obs[e])
            if med is None:
                continue
            self.bone_lengths[e] = med
            self.bone_sigmas[e] = sig if sig is not None else 0.0
            good += 1

        self.calibrated = True
        self.get_logger().info(f"✅ Calibration done. Bones calibrated: {good}/{len(self.edges)}")
        # Optional: print a few lengths
        for e in [(5,11), (6,12), (5,7), (7,9), (11,13), (13,15)]:
            if e in self.bone_lengths:
                self.get_logger().info(f"Bone {e}: L={self.bone_lengths[e]:.3f} m")

    # ---------------------------------------------------------

    def _publish_all(self, pts, stamp):
        # PoseArray (always 17 poses; NaN for missing)
        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_color_optical_frame"
        pose_array.header.stamp = stamp

        nan = float("nan")
        for j in range(self.num_joints):
            p = Pose()
            if pts[j] is None:
                p.position.x = nan
                p.position.y = nan
                p.position.z = nan
            else:
                p.position.x = float(pts[j][0])
                p.position.y = float(pts[j][1])
                p.position.z = float(pts[j][2])
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.pub_poses.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} 3D skeleton points")

        # Markers
        self.publish_skeleton_markers(pts, stamp)

    def publish_skeleton_markers(self, pts, stamp):
        markers = MarkerArray()

        # Joints (only valid pts)
        joint = Marker()
        joint.header.frame_id = "camera_color_optical_frame"
        joint.header.stamp = stamp
        joint.ns = "joints"
        joint.id = 0
        joint.type = Marker.SPHERE_LIST
        joint.action = Marker.ADD
        joint.scale.x = joint.scale.y = joint.scale.z = 0.03
        joint.color.r = 1.0
        joint.color.g = 0.4
        joint.color.b = 0.1
        joint.color.a = 1.0

        for p in pts:
            if p is None:
                continue
            joint.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))

        markers.markers.append(joint)

        # Bones (skip if endpoints missing)
        bones = Marker()
        bones.header.frame_id = "camera_color_optical_frame"
        bones.header.stamp = stamp
        bones.ns = "bones"
        bones.id = 1
        bones.type = Marker.LINE_LIST
        bones.action = Marker.ADD
        bones.scale.x = 0.015
        bones.color.r = 0.0
        bones.color.g = 0.9
        bones.color.b = 0.9
        bones.color.a = 1.0

        for (a, b) in self.edges:
            if a >= len(pts) or b >= len(pts):
                continue
            if pts[a] is None or pts[b] is None:
                continue
            pa = pts[a]
            pb = pts[b]
            bones.points.append(Point(x=float(pa[0]), y=float(pa[1]), z=float(pa[2])))
            bones.points.append(Point(x=float(pb[0]), y=float(pb[1]), z=float(pb[2])))

        markers.markers.append(bones)

        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSkeletonNodeKFCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
