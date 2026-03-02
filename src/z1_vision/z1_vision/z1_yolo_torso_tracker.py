#!/usr/bin/env python3
"""
Z1 YOLO Torso Tracker
Versione con FSM interna stabile:
States: SEARCHING → ESTIMATING → LOCKED → LOST

La FSM generale usa SOLO:
- /torso_target_ee_locked
- /target_lock_valid
"""

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Bool, ColorRGBA, String, Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
import cv2
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from .kalman_filter import Kalman3D


TORSO_KEYPOINTS = [5, 6, 11, 12]


LOCK_COLORS = {
    'SEARCHING':  ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),
    'ESTIMATING': ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9),
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
    'LOST':       ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9),
}


class Z1YoloTorsoTracker(Node):

    def __init__(self):
        super().__init__('z1_yolo_torso_tracker')

        # ───────── PARAMETRI ─────────
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('conf_thr', 0.3)
        self.declare_parameter('imgsz', 416)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_depth', 2.5)

        self.declare_parameter('tracking_speed', 0.05)
        self.declare_parameter('lock_stable_frames', 20)
        self.declare_parameter('lock_variance_thr', 0.005)
        self.declare_parameter('lock_alpha', 0.02)

        self.conf_thr = self.get_parameter('conf_thr').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value
        self.max_depth = self.get_parameter('max_depth').value
        self.tracking_speed = self.get_parameter('tracking_speed').value
        self.lock_stable_frames = self.get_parameter('lock_stable_frames').value
        self.lock_variance_thr = self.get_parameter('lock_variance_thr').value
        self.lock_alpha = self.get_parameter('lock_alpha').value

        # ───────── YOLO ─────────
        self.model = YOLO(self.get_parameter('model_path').value)
        self.model.to(self.device)

        # ───────── KALMAN ─────────
        self.kf = Kalman3D(dt=0.033, process_noise=5e-5, measurement_noise=1.0)

        # ───────── TF ─────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ───────── SUBSCRIBERS ─────────
        self.bridge = CvBridge()
        self.sub_rgb = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], 5, 0.05)
        self.sync.registerCallback(self.cb_sync)

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 1)

        # ───────── PUBLISHERS ─────────
        self.pub_locked = self.create_publisher(PoseStamped, '/torso_target_ee_locked', 10)
        self.pub_lock_valid = self.create_publisher(Bool, '/target_lock_valid', 10)
        self.pub_lock_state = self.create_publisher(String, '/target_lock_state', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/torso_markers', 10)

        # ───────── STATO FSM INTERNA ─────────
        self.state = 'SEARCHING'
        self.position_history = []
        self.locked_world = None
        self.tracking_current_pos = None

        self.cam_info = None

        self.get_logger().info("✅ Torso Tracker con FSM interna pronto")

    # ──────────────────────────────────────────────

    def cb_info(self, msg):
        self.cam_info = msg

    # ──────────────────────────────────────────────

    def cb_sync(self, rgb_msg, depth_msg):
        if self.cam_info is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        results = self.model.predict(
            rgb, conf=self.conf_thr, classes=[0],
            imgsz=self.imgsz, device=self.device, verbose=False)

        torso_raw = self._extract_torso(results, depth)
        if torso_raw is None:
            self._handle_lost()
            return

        if not self.kf.initialized:
            self.kf.initialize(torso_raw)
        else:
            self.kf.predict(0.9)
            self.kf.update(torso_raw)

        est_cam = self.kf.get_position()
        est_world = self._camera_to_world(est_cam)

        if est_world is None:
            return

        self._update_fsm(est_world)

    # ──────────────────────────────────────────────

    def _update_fsm(self, pos):

        if self.state == 'SEARCHING':
            self.position_history.append(pos)
            if len(self.position_history) > self.lock_stable_frames:
                self.position_history.pop(0)

                var = np.var(self.position_history, axis=0).mean()
                if var < self.lock_variance_thr:
                    self.state = 'LOCKED'
                    self.locked_world = pos.copy()

        elif self.state == 'LOCKED':
            self.locked_world = (
                (1.0 - self.lock_alpha) * self.locked_world
                + self.lock_alpha * pos
            )

        self._publish_outputs()

    # ──────────────────────────────────────────────

    def _handle_lost(self):
        if self.state != 'LOCKED':
            self.state = 'SEARCHING'
        self._publish_outputs()

    # ──────────────────────────────────────────────

    def _publish_outputs(self):
        valid = self.state == 'LOCKED'

        if valid:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(self.locked_world[0])
            pose.pose.position.y = float(self.locked_world[1])
            pose.pose.position.z = float(self.locked_world[2])
            pose.pose.orientation.w = 1.0
            self.pub_locked.publish(pose)

        self.pub_lock_valid.publish(Bool(data=valid))
        self.pub_lock_state.publish(String(data=self.state))

        self._publish_marker()

    # ──────────────────────────────────────────────

    def _publish_marker(self):
        if self.locked_world is None:
            return

        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'torso_lock'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.locked_world[0])
        m.pose.position.y = float(self.locked_world[1])
        m.pose.position.z = float(self.locked_world[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color = LOCK_COLORS[self.state]

        ma = MarkerArray()
        ma.markers.append(m)
        self.pub_markers.publish(ma)

    # ──────────────────────────────────────────────

    def _camera_to_world(self, point):
        pt = PointStamped()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(point[0])
        pt.point.y = float(point[1])
        pt.point.z = float(point[2])

        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                'camera_depth_optical_frame',
                rclpy.time.Time()
            )
            out = do_transform_point(pt, tf)
            return np.array([out.point.x, out.point.y, out.point.z])
        except TransformException:
            return None

    # ──────────────────────────────────────────────

    def _extract_torso(self, results, depth):
        if len(results) == 0 or results[0].keypoints is None:
            return None

        kp = results[0].keypoints
        if kp.xy is None or kp.xy.shape[0] == 0:
            return None

        kp_xy = kp.xy.cpu().numpy()[0]
        kp_conf = kp.conf.cpu().numpy()[0]

        K = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        pts = []
        for idx in TORSO_KEYPOINTS:
            if kp_conf[idx] < self.conf_thr:
                continue
            u, v = int(kp_xy[idx][0]), int(kp_xy[idx][1])
            d = depth[v, u]
            if d <= 0.05 or d > self.max_depth:
                continue
            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            pts.append([X, Y, d])

        if len(pts) < 2:
            return None

        return np.mean(pts, axis=0)


def main():
    rclpy.init()
    node = Z1YoloTorsoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
