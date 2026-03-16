#!/usr/bin/env python3
"""
Z1 YOLO Torso Tracker 
Stati interni: IDLE → ESTIMATING → LOCKED

"""

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
import cv2
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from .kalman_filter import Kalman3D


# COCO Keypoints: spalle + fianchi
TORSO_KEYPOINTS = [5, 6, 11, 12]

TORSO_EDGES = [
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
]

STATE_COLORS = {
    'IDLE':       ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),  # grigio
    'ESTIMATING': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),  # arancione
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),  # verde
}


class Z1YoloTorsoTracker(Node):

    def __init__(self):
        super().__init__('z1_yolo_torso_tracker')

        # ── Parametri base ─────────────────────────────────────────
        self.declare_parameter('model_path',         'yolo11n-pose.pt')
        self.declare_parameter('conf_thr',           0.3)
        self.declare_parameter('max_depth',          2.5)
        self.declare_parameter('device',             'cpu')
        self.declare_parameter('imgsz',              416)

        # ── Parametri Kalman ───────────────────────────────────────
        self.declare_parameter('kf_process_noise',   0.00005)
        self.declare_parameter('kf_meas_noise',      1.0)
        self.declare_parameter('kf_vel_damping',     0.9)

        # ── Parametri macchina a stati (lock) ──────────────────────
        self.declare_parameter('min_detection_conf', 0.6)
        self.declare_parameter('min_keypoints',      3)
        self.declare_parameter('lock_stable_frames', 20)
        self.declare_parameter('lock_variance_thr',  0.005)
        self.declare_parameter('lock_stable_checks', 5)
        self.declare_parameter('lock_drift_thr',     0.25)   
        self.declare_parameter('lock_drift_frames',  5)   
        self.declare_parameter('recovery_frames',    10)     

        # ── Parametri velocità ─────────────────────────────────────
        self.declare_parameter('tracking_speed',     0.05)   # invariato (interpolazione verso target)

        # ── Parametri sync RGB+Depth ───────────────────────────────
        self.declare_parameter('sync_slop',          0.10)   # secondi tolleranza sync timestamp
        self.declare_parameter('sync_queue_size',    10)

        # ── Leggi parametri ────────────────────────────────────────
        self.conf_thr           = float(self.get_parameter('conf_thr').value)
        self.max_depth          = float(self.get_parameter('max_depth').value)
        self.imgsz              = int(self.get_parameter('imgsz').value)
        self.vel_damping        = float(self.get_parameter('kf_vel_damping').value)

        self.min_det_conf       = float(self.get_parameter('min_detection_conf').value)
        self.min_keypoints      = int(self.get_parameter('min_keypoints').value)
        self.lock_stable_frames = int(self.get_parameter('lock_stable_frames').value)
        self.lock_variance_thr  = float(self.get_parameter('lock_variance_thr').value)
        self.lock_stable_checks = int(self.get_parameter('lock_stable_checks').value)

        self.lock_drift_thr     = float(self.get_parameter('lock_drift_thr').value)
        self.lock_drift_frames  = int(self.get_parameter('lock_drift_frames').value)

        self.recovery_frames    = int(self.get_parameter('recovery_frames').value)
        self.tracking_speed     = float(self.get_parameter('tracking_speed').value)
        sync_slop               = float(self.get_parameter('sync_slop').value)
        sync_queue_size         = int(self.get_parameter('sync_queue_size').value)

        device = self.get_parameter('device').value

        # ── YOLO ──────────────────────────────────────────────────
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        try:
            self.model.to(device)
            self.get_logger().info(f'✅YOLO su device: {device}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Fallback CPU: {e}')
            device = 'cpu'
            self.model.to('cpu')
        self.device = device

        # ── Kalman ────────────────────────────────────────────────
        self.kf = Kalman3D(
            dt=0.033,
            process_noise=float(self.get_parameter('kf_process_noise').value),
            measurement_noise=float(self.get_parameter('kf_meas_noise').value)
        )

        # ── Bridge + TF ───────────────────────────────────────────
        self.bridge      = CvBridge()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribers (NUOVA FSM) ────────────────────────────────
        self.sub_rgb   = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], queue_size=sync_queue_size, slop=sync_slop)
        self.sync.registerCallback(self.cb_synchronized)

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 1)

        # FSM esterna (solo per label/colore se vuoi, qui lo pubblichiamo solo come testo marker)
        self.sub_fsm_state = self.create_subscription(
            String, '/torso_sm_state', self.cb_fsm_state, 10)

        # ── Publishers (NUOVA FSM) ─────────────────────────────────
        self.pub_torso_ee          = self.create_publisher(PoseStamped,  '/torso_target_ee',     10)
        self.pub_torso_ee_locked   = self.create_publisher(PoseStamped,  '/torso_target_ee_locked', 10)

        self.pub_markers           = self.create_publisher(MarkerArray, '/torso_markers',        10)
        self.pub_tracker_state     = self.create_publisher(String,      '/torso_tracker_state',  10)

        # ── Stato interno (invariato) ──────────────────────────────
        self.state            = 'IDLE'
        self.stable_counter   = 0
        self.recovery_counter = 0
        self.drift_counter    = 0
        self.position_history = []
        self.locked_target    = None  # world frame

        # ── Interpolazione tracking (invariata) ────────────────────
        self.tracking_current_pos = None  # world frame

        # ── Stato generico ─────────────────────────────────────────
        self.cam_info   = None
        self.last_kp_3d = {}
        self.last_stamp = None

        # FSM esterna (solo label)
        self.fsm_state_external = 'WAITING'

        self.get_logger().info('🚀 Z1 YOLO Torso Tracker pronto (I/O nuova FSM, no RETURNING).')

    # ──────────────────────────────────────────────────────────────
    def cb_info(self, msg):
        self.cam_info = msg

    def cb_fsm_state(self, msg: String):
        self.fsm_state_external = msg.data

    # ──────────────────────────────────────────────────────────────
    def _camera_to_world(self, point_camera):
        pt = PointStamped()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x = float(point_camera[0])
        pt.point.y = float(point_camera[1])
        pt.point.z = float(point_camera[2])
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            transformed = do_transform_point(pt, transform)
            return np.array([transformed.point.x, transformed.point.y, transformed.point.z])
        except TransformException as e:
            self.get_logger().warn(f'TF camera→world fallita: {e}', throttle_duration_sec=2.0)
            return None

    # ──────────────────────────────────────────────────────────────
    def _interpolate_to_target(self, target_world):
        if self.tracking_current_pos is None:
            self.tracking_current_pos = target_world.copy()

        direction = target_world - self.tracking_current_pos
        distance  = np.linalg.norm(direction)

        dt       = self.kf.dt if hasattr(self.kf, 'dt') else 0.033
        max_step = self.tracking_speed * dt

        if distance <= max_step:
            self.tracking_current_pos = target_world.copy()
        else:
            self.tracking_current_pos = (self.tracking_current_pos + (direction / distance) * max_step)
        return self.tracking_current_pos

    # ──────────────────────────────────────────────────────────────
    def cb_synchronized(self, rgb_msg, depth_msg):
        if self.cam_info is None:
            self.get_logger().warn('CameraInfo non ancora ricevuta', throttle_duration_sec=2.0)
            return

        # ── Misura dt reale ────────────────────────────────────────
        now_sec = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None:
            real_dt = now_sec - self.last_stamp
            if 0.01 < real_dt < 0.5:
                self.kf.dt = real_dt
        self.last_stamp = now_sec

        # ── Conversione immagini ───────────────────────────────────
        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            h_rgb, w_rgb = rgb.shape[:2]
            if depth.shape[:2] != (h_rgb, w_rgb):
                depth = cv2.resize(depth, (w_rgb, h_rgb), interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            self.get_logger().error(f'Errore immagini: {e}')
            return

        # ── YOLO inference ─────────────────────────────────────────
        try:
            results = self.model.predict(
                rgb, conf=self.conf_thr, classes=[0],
                verbose=False, imgsz=self.imgsz, device=self.device)
        except Exception as e:
            self.get_logger().error(f'YOLO fallita: {e}')
            return

        # ── Estrai misura torso (camera frame) ────────────────────
        torso_raw, kp_3d, n_valid, avg_conf = self._extract_torso(results, depth)

        # ── Macchina a stati (identica) ────────────────────────────
        self._update_state(torso_raw, n_valid, avg_conf, rgb_msg.header)

        # ── Markers ───────────────────────────────────────────────
        target = self.locked_target if self.locked_target is not None \
                 else (self._camera_to_world(self.kf.get_position())
                       if self.kf.initialized else None)
        if target is not None:
            self._publish_markers(target, kp_3d, rgb_msg.header)

    # ──────────────────────────────────────────────────────────────
    def _extract_torso(self, results, depth):
        if len(results) == 0 or results[0].keypoints is None:
            return None, {}, 0, 0.0
        kp_data = results[0].keypoints
        if kp_data.xy is None or kp_data.xy.shape[0] == 0:
            return None, {}, 0, 0.0

        kp_xy   = kp_data.xy.cpu().numpy()[0]
        kp_conf = kp_data.conf.cpu().numpy()[0]

        K      = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        torso_pts, kp_3d, confs = [], {}, []
        for idx in TORSO_KEYPOINTS:
            if kp_conf[idx] < self.conf_thr:
                continue
            u, v = int(kp_xy[idx, 0]), int(kp_xy[idx, 1])
            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue
            d = self._get_depth_robust(depth, u, v, window=5)
            if d is None:
                continue
            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            torso_pts.append([X, Y, d])
            kp_3d[idx] = [X, Y, d]
            confs.append(kp_conf[idx])

        if len(torso_pts) < 2:
            return None, {}, 0, 0.0

        return np.mean(torso_pts, axis=0), kp_3d, len(torso_pts), float(np.mean(confs))

    # ──────────────────────────────────────────────────────────────
    def _update_state(self, torso_raw, n_valid, avg_conf, header):
        prev_state = self.state

        # ── IDLE ──────────────────────────────────────────────────
        if self.state == 'IDLE':
            if (torso_raw is not None
                    and n_valid >= self.min_keypoints
                    and avg_conf >= self.min_det_conf):
                self.state            = 'ESTIMATING'
                self.kf.reset()
                self.kf.initialize(torso_raw)
                self.position_history = [torso_raw.copy()]
                self.stable_counter   = 0
                self.tracking_current_pos = None

        # ── ESTIMATING ────────────────────────────────────────────
        elif self.state == 'ESTIMATING':
            if (torso_raw is not None
                    and n_valid >= self.min_keypoints
                    and avg_conf >= self.min_det_conf):

                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)
                estimated_cam = self.kf.get_position()

                self.position_history.append(estimated_cam.copy())
                if len(self.position_history) > self.lock_stable_frames:
                    self.position_history.pop(0)

                target_world = self._camera_to_world(estimated_cam)
                if target_world is not None:
                    interpolated = self._interpolate_to_target(target_world)
                    # self._publish_target_world(interpolated)
                    # self._publish_visible(True)

                if len(self.position_history) >= self.lock_stable_frames:
                    variance = np.var(self.position_history, axis=0).sum()
                    if variance < self.lock_variance_thr:
                        self.stable_counter += 1
                        if self.stable_counter >= self.lock_stable_checks:
                            locked_world = self._camera_to_world(estimated_cam)
                            if locked_world is not None:
                                self.locked_target        = locked_world
                                self.tracking_current_pos = None
                                self.state                = 'LOCKED'
                                self.drift_counter        = 0
                    else:
                        self.stable_counter = 0
            else:
                self.get_logger().warn('⚠️ Torso perso durante stima → IDLE')
                self.state = 'IDLE'
                self.kf.reset()
                self.position_history     = []
                self.tracking_current_pos = None
                # self._publish_visible(False)

        elif self.state == 'LOCKED':
            if self.locked_target is None:
                self.get_logger().warn("Locked ma locked_target è None", throttle_duration_sec=1.0)
                return
            interpolated = self._interpolate_to_target(self.locked_target)
            self._publish_target_world(interpolated)
            self._publish_target_world_locked(self.locked_target)
            # self._publish_visible(True)

            ok_det = (torso_raw is not None and n_valid >= self.min_keypoints and avg_conf >= self.min_det_conf)
            if ok_det and self.locked_target is not None:
                torso_world = self._camera_to_world(torso_raw)
                if torso_world is not None:
                    dist = float(np.linalg.norm(torso_world - self.locked_target))

                    if dist > self.lock_drift_thr:
                        self.drift_counter += 1
                    else:
                        self.drift_counter = 0

                    if self.drift_counter >= self.lock_drift_frames:
                        self.get_logger().info(
                            f'🔓 UNLOCK: torso moved {dist:.3f}m (> {self.lock_drift_thr:.3f}m) → ESTIMATING'
                        )
                        # riparti da ESTIMATING sul nuovo punto
                        self.state = 'ESTIMATING'
                        self.locked_target = None
                        self.tracking_current_pos = None
                        self.position_history = [torso_raw.copy()]
                        self.stable_counter = 0
                        self.drift_counter = 0

                        # re-init KF dal measurement corrente (come in IDLE→ESTIMATING)
                        self.kf.reset()
                        self.kf.initialize(torso_raw)

        # ── Log cambio stato ──────────────────────────────────────
        if self.state != prev_state:
            self.get_logger().info(
                f'🔄 {prev_state} → {self.state} (kp={n_valid} conf={avg_conf:.2f})'
            )

        # Pubblica stato tracker
        self.pub_tracker_state.publish(String(data=self.state))

    # ──────────────────────────────────────────────────────────────
    def _publish_target_world(self, point_world):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee.publish(pose)

    def _publish_target_world_locked(self, point_world):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee_locked.publish(pose)

    # def _publish_visible(self, visible: bool):
    #     self.pub_visible.publish(Bool(data=bool(visible)))

    def _get_depth_robust(self, depth, u, v, window=5):
        h, w  = depth.shape
        half  = window // 2
        patch = depth[max(v-half, 0):min(v+half+1, h),
                      max(u-half, 0):min(u+half+1, w)]
        valid = patch[(patch > 0.05) & (patch < self.max_depth)]
        if valid.size < 3:
            return None
        return float(np.median(valid))

    # ──────────────────────────────────────────────────────────────
    def _publish_markers(self, torso_center_world, kp_3d, header):
        kp_3d_w = {idx: self._camera_to_world(pt) for idx, pt in kp_3d.items()}
        kp_3d_w = {idx: w for idx, w in kp_3d_w.items() if w is not None}

        frame   = 'world'
        stamp   = self.get_clock().now().to_msg()
        markers = MarkerArray()
        color   = STATE_COLORS.get(self.state, STATE_COLORS['IDLE'])

        # 1. Pallino centrale
        cm = Marker()
        cm.header.frame_id    = frame
        cm.header.stamp       = stamp
        cm.ns                 = 'torso_center'
        cm.id                 = 0
        cm.type               = Marker.SPHERE
        cm.action             = Marker.ADD
        cm.pose.position.x    = float(torso_center_world[0])
        cm.pose.position.y    = float(torso_center_world[1])
        cm.pose.position.z    = float(torso_center_world[2])
        cm.pose.orientation.w = 1.0
        cm.scale.x = cm.scale.y = cm.scale.z = 0.08
        cm.color              = color
        cm.lifetime.nanosec   = 200_000_000
        markers.markers.append(cm)

        # 2. Sfere keypoint
        for i, idx in enumerate(TORSO_KEYPOINTS):
            if idx not in kp_3d_w:
                continue
            kp = kp_3d_w[idx]
            m = Marker()
            m.header.frame_id    = frame
            m.header.stamp       = stamp
            m.ns                 = 'torso_keypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(kp[0])
            m.pose.position.y    = float(kp[1])
            m.pose.position.z    = float(kp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color   = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            m.lifetime.nanosec   = 200_000_000
            markers.markers.append(m)

        # 3. Linee tra keypoint
        lm = Marker()
        lm.header.frame_id    = frame
        lm.header.stamp       = stamp
        lm.ns                 = 'torso_edges'
        lm.id                 = 10
        lm.type               = Marker.LINE_LIST
        lm.action             = Marker.ADD
        lm.scale.x            = 0.015
        lm.color              = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        lm.pose.orientation.w = 1.0
        lm.lifetime.nanosec   = 200_000_000
        for (a, b) in TORSO_EDGES:
            if a in kp_3d_w and b in kp_3d_w:
                lm.points.append(Point(x=float(kp_3d_w[a][0]),
                                       y=float(kp_3d_w[a][1]),
                                       z=float(kp_3d_w[a][2])))
                lm.points.append(Point(x=float(kp_3d_w[b][0]),
                                       y=float(kp_3d_w[b][1]),
                                       z=float(kp_3d_w[b][2])))
        if lm.points:
            markers.markers.append(lm)

        # 4. Linee centro → keypoint
        sm = Marker()
        sm.header.frame_id    = frame
        sm.header.stamp       = stamp
        sm.ns                 = 'torso_spokes'
        sm.id                 = 11
        sm.type               = Marker.LINE_LIST
        sm.action             = Marker.ADD
        sm.scale.x            = 0.008
        sm.color              = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.6)
        sm.pose.orientation.w = 1.0
        sm.lifetime.nanosec   = 200_000_000
        pc = Point(x=float(torso_center_world[0]),
                   y=float(torso_center_world[1]),
                   z=float(torso_center_world[2]))
        for idx in TORSO_KEYPOINTS:
            if idx in kp_3d_w:
                sm.points.append(pc)
                sm.points.append(Point(x=float(kp_3d_w[idx][0]),
                                       y=float(kp_3d_w[idx][1]),
                                       z=float(kp_3d_w[idx][2])))
        if sm.points:
            markers.markers.append(sm)

        # 5. Testo stato (aggiungo anche FSM esterna solo come label)
        tm = Marker()
        tm.header.frame_id    = frame
        tm.header.stamp       = stamp
        tm.ns                 = 'torso_state'
        tm.id                 = 20
        tm.type               = Marker.TEXT_VIEW_FACING
        tm.action             = Marker.ADD
        tm.pose.position.x    = float(torso_center_world[0])
        tm.pose.position.y    = float(torso_center_world[1])
        tm.pose.position.z    = float(torso_center_world[2]) + 0.15
        tm.pose.orientation.w = 1.0
        tm.scale.z            = 0.05
        tm.color              = color
        tm.text               = f"INT:{self.state} | EXT:{self.fsm_state_external}"
        tm.lifetime.nanosec   = 200_000_000
        markers.markers.append(tm)

        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = Z1YoloTorsoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()