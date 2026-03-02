#!/usr/bin/env python3
""" 
Z1 YOLO Torso Tracker (solo percezione + filtro)

Pubblica:
- /torso_target_camera  (PointStamped, camera_depth_optical_frame)
- /torso_target_ee      (PoseStamped, world)  [stima torso in world]
- /torso_target_ee_locked (PoseStamped, world)  [target LOCKED]
- /target_lock_valid      (Bool)
- /target_lock_state      (String: SEARCHING/CANDIDATE/LOCKED)
- /torso_visible        (Bool)
- /torso_avg_conf       (Float32)
- /torso_n_valid_kp     (Int32)
- /torso_tracker_state  (String: TRACKING/LOST)
- /torso_markers        (MarkerArray)
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

STATE_COLORS = {
    # Tracker-only
    'TRACKING':        ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),  # verde
    'LOST':            ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.9),  # arancione/rosso

    # FSM states (per RViz)
    'WAITING':         ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),  # grigio
    'APPROACHING_JTC': ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9),  # azzurro
    'WAIT_JTC':        ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9),  # blu
    'SWITCH_TO_TORQUE':        ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),  # arancione
    'WAIT_SWITCH_TO_TORQUE':   ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),  # giallo
    'IMPEDANCE_CONTACT': ColorRGBA(r=0.8, g=0.0, b=1.0, a=0.9),  # viola
    'HOLD_CONTACT':      ColorRGBA(r=1.0, g=0.0, b=0.6, a=0.9),  # magenta
    'SWITCH_TO_JTC':     ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),  # ciano
    'WAIT_SWITCH_TO_JTC':ColorRGBA(r=0.0, g=0.7, b=0.7, a=0.9),  # ciano scuro
    'RETURN_HOME_JTC':   ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9),  # verde scuro
    'WAIT_RETURN':       ColorRGBA(r=0.0, g=0.5, b=0.0, a=0.9),  # verde molto scuro
    'FAULT':             ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.95), # rosso
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

        # ── Parametri posizione marker ─────────────────────────────
        self.declare_parameter('starting_position',  [0.0411, 0.0103, 0.5133])
        # ── Target Lock (NO temporal hold) ───────────────────────
        self.declare_parameter('min_lock_frames', 8)
        self.declare_parameter('relock_translation_thresh', 0.05)  # [m]

        # ── Leggi parametri ────────────────────────────────────────
        self.conf_thr      = float(self.get_parameter('conf_thr').value)
        self.max_depth     = float(self.get_parameter('max_depth').value)
        self.imgsz         = int(self.get_parameter('imgsz').value)
        self.vel_damping   = float(self.get_parameter('kf_vel_damping').value)
        raw_sp             = self.get_parameter('starting_position').value
        self.starting_position = np.array(raw_sp, dtype=np.float64)
        self.device        = self.get_parameter('device').value
        self.min_lock_frames = int(self.get_parameter('min_lock_frames').value)
        self.relock_translation_thresh = float(self.get_parameter('relock_translation_thresh').value)
        # ── YOLO ──────────────────────────────────────────────────
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        try:
            self.model.to(self.device)
            self.get_logger().info(f'✅ YOLO su device: {self.device}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Fallback CPU: {e}')
            self.device = 'cpu'
            self.model.to('cpu')

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

        # Stato FSM esterna (per colorare marker in RViz)
        self.sub_fsm_state = self.create_subscription(
            String, '/torso_sm_state', self.cb_fsm_state, 10)

        # (Removed: /current_ee_pose and /target_unreachable subscriptions)

        # ── Publishers ────────────────────────────────────────────
        self.pub_torso_camera = self.create_publisher(PointStamped, '/torso_target_camera', 10)
        self.pub_torso_ee     = self.create_publisher(PoseStamped,  '/torso_target_ee',     10)
        self.pub_visible      = self.create_publisher(Bool,         '/torso_visible',        10)
        self.pub_markers      = self.create_publisher(MarkerArray,  '/torso_markers',        10)
        self.pub_state        = self.create_publisher(String,       '/torso_tracker_state',  10)
        self.pub_avg_conf     = self.create_publisher(Float32, '/torso_avg_conf', 10)
        self.pub_n_valid_kp   = self.create_publisher(Int32,   '/torso_n_valid_kp', 10)
        self.pub_torso_ee_locked = self.create_publisher(PoseStamped, '/torso_target_ee_locked', 10)
        self.pub_lock_valid      = self.create_publisher(Bool,        '/target_lock_valid', 10)
        self.pub_lock_state      = self.create_publisher(String,      '/target_lock_state', 10)
        # ── Stato generico ────────────────────────────────────────
        self.cam_info   = None
        self.last_kp_3d = {}
        self.last_stamp = None

        # Stato tracker (per marker/testo)
        self.tracker_state = 'LOST'
        self.fsm_state = 'WAITING'

        #stato locked 
        self.lock_state = 'SEARCHING'
        self.lock_candidate_count = 0
        self.locked_world = None  # np.array([x,y,z])

        self.get_logger().info(
            '🚀 Z1 YOLO Torso Tracker pronto! (solo percezione + filtro)\n'
            f'   starting_position (solo marker): {self.starting_position}\n'
            f'   YOLO device: {self.device}'
        )

    # ──────────────────────────────────────────────────────────────
    def cb_info(self, msg):
        self.cam_info = msg
    # ──────────────────────────────────────────────────────────────

    def cb_fsm_state(self, msg: String):
        prev = self.fsm_state
        self.fsm_state = msg.data

        # reset lock solo a fine ciclo
        if self.fsm_state == 'WAITING' and prev != 'WAITING':
            self.lock_state = 'SEARCHING'
            self.lock_candidate_count = 0
            self.locked_world = None
            self._publish_lock_valid(False)
            self._publish_lock_state(self.lock_state)

    # ──────────────────────────────────────────────────────────────

    def _camera_to_world(self, point_camera, stamp_msg=None):
        pt = PointStamped()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.header.stamp = stamp_msg if stamp_msg is not None else self.get_clock().now().to_msg()
        pt.point.x = float(point_camera[0])
        pt.point.y = float(point_camera[1])
        pt.point.z = float(point_camera[2])

        # 1) prova col timestamp del frame
        if stamp_msg is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'world',
                    'camera_depth_optical_frame',
                    rclpy.time.Time.from_msg(stamp_msg),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                out = do_transform_point(pt, tf)
                return np.array([out.point.x, out.point.y, out.point.z], dtype=np.float64)
            except TransformException as e:
                self.get_logger().warn(
                    f"TF camera→world (stamped) fallita, provo latest: {e}",
                    throttle_duration_sec=2.0
                )

        # 2) fallback: latest
        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                'camera_depth_optical_frame',
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            out = do_transform_point(pt, tf)
            return np.array([out.point.x, out.point.y, out.point.z], dtype=np.float64)
        except TransformException as e:
            self.get_logger().warn(
                f"TF camera→world (latest) fallita: {e}",
                throttle_duration_sec=2.0
            )
            return None
    # ──────────────────────────────────────────────────────────────
    def cb_synchronized(self, rgb_msg, depth_msg):
        if self.cam_info is None:
            self.get_logger().warn('CameraInfo non ancora ricevuta',throttle_duration_sec=2.0)
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

        # ── Estrai misura torso (camera frame) ────────────────────
        torso_raw, kp_3d, n_valid, avg_conf = self._extract_torso(results, depth)

        # ── Filtro + pubblicazione osservazioni ───────────────────
        if torso_raw is not None and n_valid >= 2:
            if not self.kf.initialized:
                self.kf.reset()
                self.kf.initialize(torso_raw)
            else:
                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)

            est_cam = self.kf.get_position()  # camera frame

            # Pubblica in camera frame
            cam_msg = PointStamped()
            cam_msg.header.stamp    = self.get_clock().now().to_msg()
            cam_msg.header.frame_id = 'camera_depth_optical_frame'
            cam_msg.point.x = float(est_cam[0])
            cam_msg.point.y = float(est_cam[1])
            cam_msg.point.z = float(est_cam[2])
            self.pub_torso_camera.publish(cam_msg)

            # Pubblica in world frame (su /torso_target_ee per compatibilità)
            est_world = self._camera_to_world(est_cam, rgb_msg.header.stamp)
            if est_world is not None:
                self._publish_target_world(est_world)
                self._publish_visible(True)
                est_w = np.array(est_world, dtype=np.float64)

                if self.lock_state in ('SEARCHING', 'CANDIDATE'):
                    self.lock_candidate_count += 1
                    self.lock_state = 'CANDIDATE'
                    if self.lock_candidate_count >= self.min_lock_frames:
                        self.lock_state = 'LOCKED'
                        self.locked_world = est_w.copy()
                        self.lock_candidate_count = 0

                elif self.lock_state == 'LOCKED':
                    if self.locked_world is None:
                        self.locked_world = est_w.copy()
                        self.lock_candidate_count = 0
                    else:
                        diff = float(np.linalg.norm(est_w - self.locked_world))
                        if diff > self.relock_translation_thresh:
                            self.lock_candidate_count += 1
                            if self.lock_candidate_count >= self.min_lock_frames:
                                self.locked_world = est_w.copy()
                                self.lock_candidate_count = 0
                        else:
                            self.lock_candidate_count = 0

                # publish lock (se esiste)
                if self.locked_world is not None:
                    self._publish_target_world_locked(self.locked_world)
                    self._publish_lock_valid(True)
                    self._publish_lock_state(self.lock_state)
                else:
                    self._publish_lock_valid(False)
                    self._publish_lock_state(self.lock_state)
            else:
                self._publish_visible(False)
                if self.locked_world is not None:
                    self._publish_target_world_locked(self.locked_world)
                    self._publish_lock_valid(True)
                    self._publish_lock_state(self.lock_state)
                else:
                    self._publish_lock_valid(False)
                    self._publish_lock_state(self.lock_state)

            # Diagnostica
            conf_msg = Float32(); conf_msg.data = float(avg_conf)
            kp_msg   = Int32();   kp_msg.data   = int(n_valid)
            self.pub_avg_conf.publish(conf_msg)
            self.pub_n_valid_kp.publish(kp_msg)

            self.tracker_state = 'TRACKING'
            state_msg = String(); state_msg.data = 'TRACKING'
            self.pub_state.publish(state_msg)

        else:
            # Nessuna detection valida
            self._publish_visible(False)
            # Continua a pubblicare il target LOCKED anche se YOLO perde il torso
            if self.locked_world is not None:
                self._publish_target_world_locked(self.locked_world)
                self._publish_lock_valid(True)
                self._publish_lock_state(self.lock_state)
            else:
                self._publish_lock_valid(False)
                self._publish_lock_state(self.lock_state)
            conf_msg = Float32(); conf_msg.data = float(avg_conf)
            kp_msg   = Int32();   kp_msg.data   = int(n_valid)
            self.pub_avg_conf.publish(conf_msg)
            self.pub_n_valid_kp.publish(kp_msg)

            self.tracker_state = 'LOST'
            state_msg = String(); state_msg.data = 'LOST'
            self.pub_state.publish(state_msg)

        # ── Markers ───────────────────────────────────────────────
        marker_target = self.locked_world if self.locked_world is not None else (
            self._camera_to_world(self.kf.get_position(), rgb_msg.header.stamp) if self.kf.initialized else None
        )
        if marker_target is not None:
            self._publish_markers(marker_target, kp_3d, rgb_msg.header)
    # ──────────────────────────────────────────────────────────────
    def _extract_torso(self, results, depth):
        """Estrae centro torso in camera frame."""
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


    def _publish_target_world(self, point_world):
        """
        Pubblica la stima del torso in world frame su /torso_target_ee (compatibilità).
        """
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee.publish(pose)

    # ──────────────────────────────────────────────────────────────
    def _publish_visible(self, visible: bool):
        msg = Bool()
        msg.data = visible
        self.pub_visible.publish(msg)
    # ──────────────────────────────────────────────────────────────
    def _publish_lock_valid(self, valid: bool):
        m = Bool()
        m.data = bool(valid)
        self.pub_lock_valid.publish(m)
    # ──────────────────────────────────────────────────────────────
    def _publish_lock_state(self, state: str):
        s = String()
        s.data = str(state)
        self.pub_lock_state.publish(s)
    # ──────────────────────────────────────────────────────────────
    def _publish_target_world_locked(self, point_world_locked):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world_locked[0])
        pose.pose.position.y = float(point_world_locked[1])
        pose.pose.position.z = float(point_world_locked[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee_locked.publish(pose)
    # ──────────────────────────────────────────────────────────────
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
        """Tutti i marker sono in world frame."""

        kp_3d_w = {idx: self._camera_to_world(pt, header.stamp) for idx, pt in kp_3d.items()}
        kp_3d_w = {idx: w for idx, w in kp_3d_w.items() if w is not None}

        frame   = 'world'
        stamp   = header.stamp
        # Preferisci colore FSM se noto, altrimenti usa tracker_state
        color = STATE_COLORS.get(self.fsm_state, STATE_COLORS.get(self.tracker_state, STATE_COLORS['LOST']))
        markers = MarkerArray()

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

        # 5. Testo stato
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
        tm.text               = f"{self.fsm_state} | {self.tracker_state} | {self.lock_state}"
        tm.lifetime.nanosec   = 200_000_000
        markers.markers.append(tm)

        # 6. Pallino bianco fisso = starting_position
        sp = Marker()
        sp.header.frame_id    = frame
        sp.header.stamp       = stamp
        sp.ns                 = 'starting_position'
        sp.id                 = 30
        sp.type               = Marker.SPHERE
        sp.action             = Marker.ADD
        sp.pose.position.x    = float(self.starting_position[0])
        sp.pose.position.y    = float(self.starting_position[1])
        sp.pose.position.z    = float(self.starting_position[2])
        sp.pose.orientation.w = 1.0
        sp.scale.x = sp.scale.y = sp.scale.z = 0.06
        sp.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.6)
        sp.lifetime.nanosec   = 200_000_000
        markers.markers.append(sp)

        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = Z1YoloTorsoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
