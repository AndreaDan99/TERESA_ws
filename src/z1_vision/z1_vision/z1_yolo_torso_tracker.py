#!/usr/bin/env python3
"""
Z1 YOLO Torso Tracker + Impedance Control
- YOLO → Torso center (camera_depth_optical_frame)
- Kalman Filter 3D per smoothing e predizione
- Debounce sul lost
- Visualizzazione RViz in world frame
"""

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Bool, ColorRGBA
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
    (5, 6),   # spalla sx → spalla dx
    (5, 11),  # spalla sx → fianco sx
    (6, 12),  # spalla dx → fianco dx
    (11, 12), # fianco sx → fianco dx
]


class Z1YoloTorsoTracker(Node):

    def __init__(self):
        super().__init__('z1_yolo_torso_tracker')

        # ── Parametri ──────────────────────────────────────────────
        self.declare_parameter('model_path',        'yolo11n-pose.pt')
        self.declare_parameter('conf_thr',          0.3)
        self.declare_parameter('max_depth',         2.5)
        self.declare_parameter('lost_timeout',      1.0)
        self.declare_parameter('lost_debounce',     0.3)
        self.declare_parameter('device',            'cpu')
        self.declare_parameter('imgsz',             416)
        self.declare_parameter('kf_process_noise',  0.005)  # Q — smoothing
        self.declare_parameter('kf_meas_noise',     0.05)   # R — reattività
        self.declare_parameter('kf_vel_damping',    0.9)    # smorzamento velocità

        self.conf_thr       = float(self.get_parameter('conf_thr').value)
        self.max_depth      = float(self.get_parameter('max_depth').value)
        self.lost_timeout   = float(self.get_parameter('lost_timeout').value)
        self.lost_debounce  = float(self.get_parameter('lost_debounce').value)
        self.imgsz          = int(self.get_parameter('imgsz').value)
        self.vel_damping    = float(self.get_parameter('kf_vel_damping').value)
        device              = self.get_parameter('device').value

        # ── YOLO con fallback CPU ──────────────────────────────────
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)

        try:
            self.model.to(device)
            self.get_logger().info(f'✅ YOLO su device: {device}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Device {device} non disponibile, uso CPU: {e}')
            device = 'cpu'
            self.model.to('cpu')

        self.device = device

        # ── Kalman Filter sul centro torso ─────────────────────────
        self.kf = Kalman3D(
            dt=0.033,
            process_noise=float(self.get_parameter('kf_process_noise').value),
            measurement_noise=float(self.get_parameter('kf_meas_noise').value)
        )

        # ── Bridge + TF ────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribers sincronizzati (RealSense) ──────────────────
        self.sub_rgb   = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=5,
            slop=0.05
        )
        self.sync.registerCallback(self.cb_synchronized)

        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.cb_info,
            1
        )

        # ── Publishers ─────────────────────────────────────────────
        self.pub_torso_camera = self.create_publisher(PointStamped, '/torso_target_camera', 10)
        self.pub_torso_ee     = self.create_publisher(PoseStamped,  '/torso_target_ee',     10)
        self.pub_visible      = self.create_publisher(Bool,         '/torso_visible',        10)
        self.pub_markers      = self.create_publisher(MarkerArray,  '/torso_markers',        10)

        # ── Stato ──────────────────────────────────────────────────
        self.cam_info       = None
        self.last_torso     = None
        self.target_saved   = None
        self.lost_timer     = None
        self.torso_visible  = False
        self.debounce_timer = None
        self.last_kp_3d     = {}

        self.get_logger().info(
            f'🚀 Z1 YOLO Torso Tracker pronto!\n'
            f'   KF process_noise={self.get_parameter("kf_process_noise").value} '
            f'meas_noise={self.get_parameter("kf_meas_noise").value} '
            f'vel_damping={self.vel_damping}'
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
            h_d,   w_d   = depth.shape[:2]
            if (h_d != h_rgb) or (w_d != w_rgb):
                depth = cv2.resize(depth, (w_rgb, h_rgb),
                                   interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            self.get_logger().error(f'Errore conversione immagini: {e}')
            return

        # ── YOLO inference ─────────────────────────────────────────
        try:
            results = self.model.predict(
                rgb,
                conf=self.conf_thr,
                classes=[0],
                verbose=False,
                imgsz=self.imgsz,
                device=self.device
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference fallita: {e}')
            return

        # ── Validazione output ─────────────────────────────────────
        if len(results) == 0 or results[0].keypoints is None:
            self._handle_lost()
            return

        kp_data = results[0].keypoints
        if kp_data.xy is None or kp_data.xy.shape[0] == 0:
            self._handle_lost()
            return

        kp_xy   = kp_data.xy.cpu().numpy()[0]
        kp_conf = kp_data.conf.cpu().numpy()[0]

        # ── Calcolo torso center 3D ────────────────────────────────
        K  = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        torso_pts = []
        kp_3d     = {}

        for idx in TORSO_KEYPOINTS:
            if kp_conf[idx] < self.conf_thr:
                continue

            u, v = int(kp_xy[idx, 0]), int(kp_xy[idx, 1])

            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue

            d = float(depth[v, u])
            if d <= 0.0 or d > self.max_depth:
                continue

            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            Z = d
            torso_pts.append([X, Y, Z])
            kp_3d[idx] = [X, Y, Z]

        # ── Almeno 2 punti validi ──────────────────────────────────
        if len(torso_pts) >= 2:
            torso_raw = np.mean(torso_pts, axis=0)
            self.last_kp_3d     = kp_3d
            self.debounce_timer = None

            # ── Kalman: predict + update con misura reale ──────────
            self.kf.predict(self.vel_damping)
            self.kf.update(torso_raw)
            torso_filtered = self.kf.get_position()

            self._handle_detected(torso_filtered, rgb_msg.header)
            self._publish_markers(torso_filtered, kp_3d, rgb_msg.header)
        else:
            self._handle_lost()

    # ──────────────────────────────────────────────────────────────
    def _handle_detected(self, torso_center, header):
        """Torso visibile: pubblica posizione filtrata da Kalman."""
        self.torso_visible = True
        self.last_torso    = torso_center.copy()
        self.target_saved  = None
        self.lost_timer    = self.get_clock().now()

        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = torso_center
        self.pub_torso_camera.publish(pt)

        self._publish_ee(torso_center, header)
        self._publish_visible(True)

    # ──────────────────────────────────────────────────────────────
    def _handle_lost(self):
        """Torso perso: debounce + Kalman predice senza misura."""
        now = self.get_clock().now()

        # ── Debounce: aspetta lost_debounce sec prima di dichiarare perso
        if self.debounce_timer is None:
            self.debounce_timer = now
            # Durante il debounce: Kalman predice e continua a pubblicare
            if self.kf.initialized:
                self.kf.predict(self.vel_damping)
                predicted = self.kf.get_position()
                self._publish_ee(predicted, header=None)
            return

        elapsed_debounce = (now - self.debounce_timer).nanoseconds / 1e9

        if elapsed_debounce < self.lost_debounce:
            # Ancora in debounce → Kalman predice senza misura
            if self.kf.initialized:
                self.kf.predict(self.vel_damping)
                predicted = self.kf.get_position()
                self._publish_ee(predicted, header=None)
            return

        # ── Torso davvero perso ────────────────────────────────────
        if self.torso_visible:
            self.get_logger().info('👤 Torso perso (confermato dopo debounce)')

        self.torso_visible = False
        self._publish_visible(False)

        # Kalman continua a predire (utile per target_saved)
        if self.kf.initialized:
            self.kf.predict(self.vel_damping * 0.95)  # smorzamento extra quando perso

        if self.lost_timer is None:
            self.lost_timer = now
            return

        elapsed_lost = (now - self.lost_timer).nanoseconds / 1e9

        # Salva target dopo lost_timeout
        if (elapsed_lost > self.lost_timeout
                and self.last_torso is not None
                and self.target_saved is None):
            self.target_saved = self.last_torso.copy()
            self.get_logger().info(f'🎯 TARGET SALVATO: {self.target_saved}')

        # Pubblica target salvato (posizione congelata)
        if self.target_saved is not None:
            self._publish_ee(self.target_saved, header=None)

    # ──────────────────────────────────────────────────────────────
    def _publish_ee(self, point_camera, header):
        pt_stamped = PointStamped()
        pt_stamped.header.frame_id = 'camera_depth_optical_frame'
        pt_stamped.header.stamp    = self.get_clock().now().to_msg()
        pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = point_camera

        try:
            transform = self.tf_buffer.lookup_transform(
                'link06',
                'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            transformed = do_transform_point(pt_stamped, transform)

            pose = PoseStamped()
            pose.header             = transformed.header
            pose.pose.position      = transformed.point
            pose.pose.orientation.w = 1.0
            self.pub_torso_ee.publish(pose)

        except TransformException as e:
            self.get_logger().warn(
                f'TF camera→link06 fallita: {e}',
                throttle_duration_sec=2.0
            )

    # ──────────────────────────────────────────────────────────────
    def _publish_visible(self, visible: bool):
        msg = Bool()
        msg.data = visible
        self.pub_visible.publish(msg)

    # ──────────────────────────────────────────────────────────────
    def _publish_markers(self, torso_center, kp_3d, header):
        """Pubblica MarkerArray in world frame per RViz."""

        def cam_to_world(pt):
            ps = PointStamped()
            ps.header.frame_id = 'camera_depth_optical_frame'
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.point.x = float(pt[0])
            ps.point.y = float(pt[1])
            ps.point.z = float(pt[2])
            try:
                t = self.tf_buffer.lookup_transform(
                    'world', 'camera_depth_optical_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                tr = do_transform_point(ps, t)
                return [tr.point.x, tr.point.y, tr.point.z]
            except:
                return None

        center_w = cam_to_world(torso_center)
        if center_w is None:
            return

        kp_3d_w = {}
        for idx, pt in kp_3d.items():
            w = cam_to_world(pt)
            if w is not None:
                kp_3d_w[idx] = w

        frame   = 'world'
        stamp   = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # 1. Pallino VERDE sul centro filtrato da Kalman
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
        cm.color   = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        cm.lifetime.nanosec = 200_000_000
        markers.markers.append(cm)

        # 2. Sfere BLU sui keypoint raw (non filtrati)
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
            m.lifetime.nanosec = 200_000_000
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

        # 4. Linee ROSSE dal centro ai keypoint
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

        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = Z1YoloTorsoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
