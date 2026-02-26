#!/usr/bin/env python3
"""
Z1 YOLO Torso Tracker + Impedance Control
- Macchina a stati: IDLE → ESTIMATING → LOCKED → RECOVERY
- YOLO → Torso center (camera_depth_optical_frame)
- Kalman Filter 3D per convergenza stima
- Target si congela quando stima è stabile
- Visualizzazione RViz in world frame
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

import tf2_ros
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

# Colori marker per stato
STATE_COLORS = {
    'IDLE':       ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),  # grigio
    'ESTIMATING': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),  # arancione
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),  # verde
    'RECOVERY':   ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),  # rosso
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
        self.declare_parameter('kf_process_noise',   0.0001)
        self.declare_parameter('kf_meas_noise',      0.5)
        self.declare_parameter('kf_vel_damping',     0.9)

        # ── Parametri macchina a stati ─────────────────────────────
        self.declare_parameter('min_detection_conf', 0.6)   # conf minima per iniziare
        self.declare_parameter('min_keypoints',      3)     # keypoint minimi validi
        self.declare_parameter('lock_stable_frames', 20)    # frame stabili per lock
        self.declare_parameter('lock_variance_thr',  0.005) # varianza max per lock (m²)
        self.declare_parameter('recovery_frames',    10)    # frame recovery prima di ri-lock
        self.declare_parameter('lock_stable_checks', 5)

        self.declare_parameter('lock_drift_thr',    0.15)  # 15cm — distanza max tollerata
        self.declare_parameter('lock_drift_frames', 10)    # frame consecutivi prima di recovery


        # Leggi parametri
        self.conf_thr          = float(self.get_parameter('conf_thr').value)
        self.max_depth         = float(self.get_parameter('max_depth').value)
        self.imgsz             = int(self.get_parameter('imgsz').value)
        self.vel_damping       = float(self.get_parameter('kf_vel_damping').value)
        self.min_det_conf      = float(self.get_parameter('min_detection_conf').value)
        self.min_keypoints     = int(self.get_parameter('min_keypoints').value)
        self.lock_stable_frames = int(self.get_parameter('lock_stable_frames').value)
        self.lock_variance_thr = float(self.get_parameter('lock_variance_thr').value)
        self.recovery_frames   = int(self.get_parameter('recovery_frames').value)
        self.lock_stable_checks = int(self.get_parameter('lock_stable_checks').value)
        self.lock_drift_thr    = float(self.get_parameter('lock_drift_thr').value)
        self.lock_drift_frames = int(self.get_parameter('lock_drift_frames').value)
        
        device = self.get_parameter('device').value

        # ── YOLO ──────────────────────────────────────────────────
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        try:
            self.model.to(device)
            self.get_logger().info(f'✅ YOLO su device: {device}')
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

        # ── Subscribers ───────────────────────────────────────────
        self.sub_rgb   = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.cb_synchronized)

        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 1)

        # ── Publishers ────────────────────────────────────────────
        self.pub_torso_camera = self.create_publisher(PointStamped, '/torso_target_camera', 10)
        self.pub_torso_ee     = self.create_publisher(PoseStamped,  '/torso_target_ee',     10)
        self.pub_visible      = self.create_publisher(Bool,         '/torso_visible',        10)
        self.pub_markers      = self.create_publisher(MarkerArray,  '/torso_markers',        10)
        self.pub_state        = self.create_publisher(String,       '/torso_tracker_state',  10)

        # ── Macchina a stati ──────────────────────────────────────
        self.state           = 'IDLE'
        self.stable_counter  = 0
        self.recovery_counter = 0
        self.drift_counter    = 0 
        self.position_history = []   # lista di np.array [x,y,z]
        self.locked_target   = None  # target congelato in camera frame

        # ── Stato generico ────────────────────────────────────────
        self.cam_info    = None
        self.last_kp_3d  = {}

        self.get_logger().info('🚀 Z1 YOLO Torso Tracker (state machine) pronto!')
        self.get_logger().info(
            f'   lock_stable_frames={self.lock_stable_frames} '
            f'lock_variance_thr={self.lock_variance_thr} '
            f'min_keypoints={self.min_keypoints}'
        )

    # ──────────────────────────────────────────────────────────────
    def cb_info(self, msg):
        self.cam_info = msg

    # ──────────────────────────────────────────────────────────────
    def cb_synchronized(self, rgb_msg, depth_msg):
        if self.cam_info is None:
            self.get_logger().warn('CameraInfo non ancora ricevuta',
                                   throttle_duration_sec=2.0)
            return

        # ── Conversione immagini ───────────────────────────────────
        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            h_rgb, w_rgb = rgb.shape[:2]
            if depth.shape[:2] != (h_rgb, w_rgb):
                depth = cv2.resize(depth, (w_rgb, h_rgb),
                                   interpolation=cv2.INTER_NEAREST)
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

        # ── Estrai misura torso ────────────────────────────────────
        torso_raw, kp_3d, n_valid, avg_conf = self._extract_torso(
            results, depth)

        # ── Macchina a stati ──────────────────────────────────────
        self._update_state(torso_raw, n_valid, avg_conf, rgb_msg.header)

        # ── Pubblica markers ──────────────────────────────────────
        target = self.locked_target if self.locked_target is not None \
                 else (self.kf.get_position() if self.kf.initialized else None)

        if target is not None and kp_3d:
            self._publish_markers(target, kp_3d, rgb_msg.header)

    # ──────────────────────────────────────────────────────────────
    def _extract_torso(self, results, depth):
        """Estrae centro torso 3D da risultati YOLO.
        Ritorna: (torso_raw, kp_3d, n_valid, avg_conf)
        """
        if len(results) == 0 or results[0].keypoints is None:
            return None, {}, 0, 0.0

        kp_data = results[0].keypoints
        if kp_data.xy is None or kp_data.xy.shape[0] == 0:
            return None, {}, 0, 0.0

        kp_xy   = kp_data.xy.cpu().numpy()[0]
        kp_conf = kp_data.conf.cpu().numpy()[0]

        K  = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        torso_pts = []
        kp_3d     = {}
        confs     = []

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
            Z = d
            torso_pts.append([X, Y, Z])
            kp_3d[idx] = [X, Y, Z]
            confs.append(kp_conf[idx])

        if len(torso_pts) < 2:
            return None, {}, 0, 0.0

        torso_raw = np.mean(torso_pts, axis=0)
        avg_conf  = float(np.mean(confs))
        return torso_raw, kp_3d, len(torso_pts), avg_conf

    # ──────────────────────────────────────────────────────────────
    def _update_state(self, torso_raw, n_valid, avg_conf, header):
        """Macchina a stati principale."""

        prev_state = self.state

        # ── IDLE ──────────────────────────────────────────────────
        if self.state == 'IDLE':
            if torso_raw is not None \
                    and n_valid >= self.min_keypoints \
                    and avg_conf >= self.min_det_conf:
                self.state = 'ESTIMATING'
                self.kf.reset()
                self.kf.initialize(torso_raw)
                self.position_history = [torso_raw.copy()]
                self.stable_counter   = 0
                self.get_logger().info('🟠 IDLE → ESTIMATING')

        # ── ESTIMATING ────────────────────────────────────────────
        elif self.state == 'ESTIMATING':
            if torso_raw is not None \
                    and n_valid >= self.min_keypoints \
                    and avg_conf >= self.min_det_conf:

                # Aggiorna Kalman
                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)
                estimated = self.kf.get_position()

                # Accumula storia posizioni
                self.position_history.append(estimated.copy())
                if len(self.position_history) > self.lock_stable_frames:
                    self.position_history.pop(0)

                # Pubblica stima corrente
                self._publish_target(estimated, header)
                self._publish_visible(True)

                # Controlla se stima è stabile → LOCK
                if len(self.position_history) >= self.lock_stable_frames:
                    variance = np.var(self.position_history, axis=0).sum()
                    if variance < self.lock_variance_thr:
                        self.stable_counter += 1
                        if self.stable_counter >= self.lock_stable_checks:  # 3 check consecutivi stabili
                            self.locked_target = estimated.copy()
                            self.state = 'LOCKED'
                    else:
                        self.stable_counter = 0  # reset se torna instabile

            else:
                # YOLO ha perso il torso in ESTIMATING → torna IDLE
                self.get_logger().warn('⚠️ Torso perso durante stima → IDLE')
                self.state = 'IDLE'
                self.kf.reset()
                self.position_history = []
                self._publish_visible(False)

        # ── LOCKED ────────────────────────────────────────────────
        elif self.state == 'LOCKED':
            self._publish_target(self.locked_target, header)
            self._publish_visible(True)

            # ── Controllo deriva: se il torso si è spostato → RECOVERY ──
            if torso_raw is not None \
                    and n_valid >= self.min_keypoints \
                    and avg_conf >= self.min_det_conf:

                drift = np.linalg.norm(torso_raw - self.locked_target)

                if drift > self.lock_drift_thr:
                    self.drift_counter += 1
                    if self.drift_counter >= self.lock_drift_frames:
                        self.get_logger().info(
                            f'⚠️ Deriva rilevata: {drift:.3f}m → RECOVERY')
                        self.state = 'RECOVERY'
                        self.drift_counter = 0
                        self.recovery_counter = 0
                        self.position_history = []
                        self.kf.reset()
                else:
                    self.drift_counter = 0  # reset se torna vicino


        # ── RECOVERY ──────────────────────────────────────────────
        elif self.state == 'RECOVERY':
            self.recovery_counter += 1

            if torso_raw is not None \
                    and n_valid >= self.min_keypoints \
                    and avg_conf >= self.min_det_conf:
                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)
                estimated = self.kf.get_position()
                # Usa la media della storia:
                if len(self.position_history) >= 5:
                    smoothed = np.mean(self.position_history[-10:], axis=0)
                else:
                    smoothed = estimated
                self._publish_target(smoothed, header)

                self.position_history.append(estimated.copy())
                if len(self.position_history) > self.lock_stable_frames:
                    self.position_history.pop(0)

            # Dopo recovery_frames → ri-lock
            if self.recovery_counter >= self.recovery_frames:
                if self.kf.initialized:
                    self.locked_target = self.kf.get_position().copy()
                self.state = 'LOCKED'
                self.recovery_counter = 0
                self.get_logger().info('🟢 RECOVERY → LOCKED (ri-lock)')

        # ── Log cambio stato ──────────────────────────────────────
        if self.state != prev_state:
            self.get_logger().info(
                f'🔄 Stato: {prev_state} → {self.state}  '
                f'(n_kp={n_valid} conf={avg_conf:.2f})'
            )

        # Pubblica stato
        state_msg = String()
        state_msg.data = self.state
        self.pub_state.publish(state_msg)

    # ──────────────────────────────────────────────────────────────
    def _publish_target(self, point_camera, header):
        """Pubblica target in camera frame e trasforma in link06."""
        pt = PointStamped()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = point_camera
        self.pub_torso_camera.publish(pt)
        self._publish_ee(point_camera)

    # ──────────────────────────────────────────────────────────────
    def _publish_ee(self, point_camera):
        pt_stamped = PointStamped()
        pt_stamped.header.frame_id = 'camera_depth_optical_frame'
        pt_stamped.header.stamp    = self.get_clock().now().to_msg()
        pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = point_camera

        try:
            transform = self.tf_buffer.lookup_transform(
                'link06', 'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            transformed = do_transform_point(pt_stamped, transform)
            pose = PoseStamped()
            pose.header             = transformed.header
            pose.pose.position      = transformed.point
            pose.pose.orientation.w = 1.0
            self.pub_torso_ee.publish(pose)
        except TransformException as e:
            self.get_logger().warn(f'TF fallita: {e}', throttle_duration_sec=2.0)

    # ──────────────────────────────────────────────────────────────
    def _publish_visible(self, visible: bool):
        msg = Bool()
        msg.data = visible
        self.pub_visible.publish(msg)

    # ──────────────────────────────────────────────────────────────
    def _get_depth_robust(self, depth, u, v, window=5):
        h, w = depth.shape
        half  = window // 2
        patch = depth[max(v-half,0):min(v+half+1,h),
                      max(u-half,0):min(u+half+1,w)]
        valid = patch[(patch > 0.05) & (patch < self.max_depth)]
        if valid.size < 3:
            return None
        return float(np.median(valid))

    # ──────────────────────────────────────────────────────────────
    def _publish_markers(self, torso_center, kp_3d, header):

        def cam_to_world(pt):
            ps = PointStamped()
            ps.header.frame_id = 'camera_depth_optical_frame'
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.point.x, ps.point.y, ps.point.z = float(pt[0]), float(pt[1]), float(pt[2])
            try:
                t = self.tf_buffer.lookup_transform(
                    'world', 'camera_depth_optical_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                tr = do_transform_point(ps, t)
                return [tr.point.x, tr.point.y, tr.point.z]
            except:
                return None

        center_w = cam_to_world(torso_center)
        if center_w is None:
            return

        kp_3d_w = {idx: cam_to_world(pt) for idx, pt in kp_3d.items()}
        kp_3d_w = {idx: w for idx, w in kp_3d_w.items() if w is not None}

        frame   = 'world'
        stamp   = self.get_clock().now().to_msg()
        markers = MarkerArray()
        color   = STATE_COLORS.get(self.state, STATE_COLORS['IDLE'])

        # 1. Pallino centrale (colore = stato)
        cm = Marker()
        cm.header.frame_id    = frame
        cm.header.stamp       = stamp
        cm.ns                 = 'torso_center'
        cm.id                 = 0
        cm.type               = Marker.SPHERE
        cm.action             = Marker.ADD
        cm.pose.position.x    = float(center_w[0])
        cm.pose.position.y    = float(center_w[1])
        cm.pose.position.z    = float(center_w[2])
        cm.pose.orientation.w = 1.0
        cm.scale.x = cm.scale.y = cm.scale.z = 0.08
        cm.color              = color
        cm.lifetime.nanosec   = 200_000_000
        markers.markers.append(cm)

        # 2. Sfere BLU keypoint
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
            m.color              = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            m.lifetime.nanosec   = 200_000_000
            markers.markers.append(m)

        # 3. Linee GIALLE tra keypoint
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

        # 4. Linee ROSSE centro → keypoint
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
        pc = Point(x=float(center_w[0]),
                   y=float(center_w[1]),
                   z=float(center_w[2]))
        for idx in TORSO_KEYPOINTS:
            if idx in kp_3d_w:
                sm.points.append(pc)
                sm.points.append(Point(x=float(kp_3d_w[idx][0]),
                                       y=float(kp_3d_w[idx][1]),
                                       z=float(kp_3d_w[idx][2])))
        if sm.points:
            markers.markers.append(sm)

        # 5. Testo stato in RViz
        tm = Marker()
        tm.header.frame_id    = frame
        tm.header.stamp       = stamp
        tm.ns                 = 'torso_state'
        tm.id                 = 20
        tm.type               = Marker.TEXT_VIEW_FACING
        tm.action             = Marker.ADD
        tm.pose.position.x    = float(center_w[0])
        tm.pose.position.y    = float(center_w[1])
        tm.pose.position.z    = float(center_w[2]) + 0.15
        tm.pose.orientation.w = 1.0
        tm.scale.z            = 0.05
        tm.color              = color
        tm.text               = self.state
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
