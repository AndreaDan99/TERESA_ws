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
from std_msgs.msg import Bool, ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import numpy as np
import cv2
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from .kalman_filter import Kalman3D


# COCO Keypoints (indici standard)
SHOULDER_KEYPOINTS = [5, 6]             # spalla sx, spalla dx
HIP_KEYPOINTS      = [11, 12]           # fianco sx, fianco dx
FACE_KEYPOINTS     = [0, 1, 2, 3, 4]   # naso, occhio sx, occhio dx, orecchio sx, orecchio dx
TORSO_KEYPOINTS    = SHOULDER_KEYPOINTS + HIP_KEYPOINTS   # spalle + fianchi

TORSO_EDGES = [
    (5, 6),   # spalle
    (5, 11),  # spalla sx → fianco sx
    (6, 12),  # spalla dx → fianco dx
    (11, 12), # fianchi
]

STATE_COLORS = {
    'IDLE':       ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),  # grigio
    'ESTIMATING': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),  # arancione
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),  # verde
    'SCANNING':   ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),  # ciano (scan mode)
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

        # ── Stima centro torso ─────────────────────────────────────
        # Metodo primario:  media spalle + fianchi (se fianchi rilevati)
        # Metodo fallback:  shoulder_mid + chest_offset lungo asse corpo (direzione opposta al viso)
        self.declare_parameter('use_face_fallback',           True)
        self.declare_parameter('chest_offset_from_shoulder',  0.15)  # [m]

        # ── Scan mode (body search scanner) ────────────────────────
        # Numero minimo di frame validi per dichiarare SCAN_POINT_LOCKED
        self.declare_parameter('scan_min_frames', 8)

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
        self.use_face_fallback  = bool(self.get_parameter('use_face_fallback').value)
        self.chest_offset_m     = float(self.get_parameter('chest_offset_from_shoulder').value)
        sync_slop               = float(self.get_parameter('sync_slop').value)
        sync_queue_size         = int(self.get_parameter('sync_queue_size').value)
        self._scan_min_frames   = int(self.get_parameter('scan_min_frames').value)

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

        # Reset tracker: riceve True dalla FSM quando torna in HOME → forza IDLE
        self.sub_tracker_reset = self.create_subscription(
            Bool, '/tracker_reset', self.cb_tracker_reset, 10)

        # ── Scan mode: comandi dalla FSM ────────────────────────────
        # /tracker_scan_mode True  → attiva scan mode
        # /tracker_scan_mode False → disattiva scan mode (reset a IDLE normale)
        self.sub_scan_mode = self.create_subscription(
            Bool, '/tracker_scan_mode', self._cb_scan_mode, 10)

        # /tracker_scan_next True → reset punto corrente (prossima posa del braccio)
        self.sub_scan_next = self.create_subscription(
            Bool, '/tracker_scan_next', self._cb_scan_next, 10)

        # /torso_scan_seed: stima fusa multi-vista dalla body scan.
        # Quando ricevuto, il tracker salta direttamente a LOCKED con quella
        # posizione invece di ricominciare da IDLE → ESTIMATING → LOCKED.
        self.sub_scan_seed = self.create_subscription(
            PointStamped, '/torso_scan_seed', self._cb_scan_seed, 10)

        # ── Publishers (NUOVA FSM) ─────────────────────────────────
        self.pub_torso_ee          = self.create_publisher(PoseStamped,  '/torso_target_ee',        10)
        self.pub_torso_ee_locked   = self.create_publisher(PoseStamped,  '/torso_target_ee_locked', 10)
        self.pub_markers           = self.create_publisher(MarkerArray,  '/torso_markers',          10)
        self.pub_tracker_state     = self.create_publisher(String,       '/torso_tracker_state',    10)

        # ── Publisher scan mode ─────────────────────────────────────
        # Pubblica ogni frame mentre scan mode è attivo:
        #   Float32MultiArray data = [score, n_kp, conf, x_world, y_world, z_world]
        # score = (n_kp / 4) * conf  per frame validi; 0.0 per frame non validi.
        self.pub_scan_point = self.create_publisher(
            Float32MultiArray, '/torso_scan_point', 10)

        # ── Stato interno normale ──────────────────────────────────
        self.state            = 'IDLE'
        self.stable_counter   = 0
        self.recovery_counter = 0
        self.drift_counter    = 0
        self.position_history = []
        self.locked_target    = None  # world frame

        # ── Stato scan mode ────────────────────────────────────────
        # _scan_mode:  True quando la FSM ha attivato la modalità scan
        # _scan_state: stato interno della scan  (IDLE / COLLECTING / POINT_LOCKED)
        # _scan_valid: contatore frame validi accumulati nel punto corrente
        self._scan_mode        = False
        self._scan_state       = 'IDLE'   # IDLE | COLLECTING | POINT_LOCKED
        self._scan_valid       = 0
        self._scan_torso_world = None     # ultima posizione torso rilevata in scan mode
        # Confidence per-keypoint torso dell'ultimo frame processato.
        # Inizializzato vuoto: se _update_scan viene chiamata prima di
        # _extract_torso (non dovrebbe, ma per sicurezza), tutti i kp = 0.0.
        self._last_kp_conf: dict = {}

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

    def cb_tracker_reset(self, msg: Bool):
        if not msg.data:
            return
        prev = self.state
        self.state            = 'IDLE'
        self.locked_target    = None
        self.tracking_current_pos = None
        self.position_history = []
        self.stable_counter   = 0
        self.drift_counter    = 0
        self.recovery_counter = 0
        self.kf.reset()
        self.get_logger().info(f'🔄 Tracker reset: {prev} → IDLE (richiesto da FSM)')

    def _cb_scan_mode(self, msg: Bool):
        """
        /tracker_scan_mode True  → entra in scan mode (congela LOCKED normale)
        /tracker_scan_mode False → esce da scan mode, resettta a IDLE normale
        """
        if msg.data == self._scan_mode:
            return
        self._scan_mode = msg.data
        if msg.data:
            # Attivazione: reset stato scan
            self._scan_state = 'IDLE'
            self._scan_valid = 0
            self.get_logger().info('🔍 Scan mode ATTIVATO')
        else:
            # Disattivazione: reset tracker normale → il tracker rileverà
            # da solo il torso dalla posizione corrente (migliore) e andrà in LOCKED
            self._scan_state = 'IDLE'
            self._scan_valid = 0
            self.state            = 'IDLE'
            self.locked_target    = None
            self.tracking_current_pos = None
            self.position_history = []
            self.stable_counter   = 0
            self.drift_counter    = 0
            self.recovery_counter = 0
            self.kf.reset()
            self.get_logger().info('🔍 Scan mode DISATTIVATO → IDLE normale')

    def _cb_scan_next(self, msg: Bool):
        """
        /tracker_scan_next True → resettta il punto corrente della scan.
        Chiamato dalla FSM quando il braccio arriva alla posa successiva.
        """
        if not msg.data or not self._scan_mode:
            return
        self._scan_state = 'IDLE'
        self._scan_valid = 0
        self.get_logger().info('🔄 Scan next: reset punto corrente')

    def _cb_scan_seed(self, msg: PointStamped):
        """
        /torso_scan_seed: stima fusa multi-vista dal body scan.
        Inizializza direttamente LOCKED con la posizione ricevuta, evitando
        il ciclo IDLE → ESTIMATING → LOCKED. Il tracker verificherà la
        posizione con le prime detections reali e si adatterà se necessario.
        """
        if self._scan_mode:
            return  # ignora se ancora in scan mode
        seed = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.locked_target        = seed
        self.state                = 'LOCKED'
        self.drift_counter        = 0
        self.tracking_current_pos = None
        self.get_logger().info(
            f'🎯 Seed scan ricevuto → LOCKED diretto '
            f'[{seed[0]:.3f}, {seed[1]:.3f}, {seed[2]:.3f}]'
        )

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

        # ── Macchina a stati: normale o scan mode ──────────────────
        if self._scan_mode:
            self._update_scan(torso_raw, n_valid, avg_conf)
        else:
            self._update_state(torso_raw, n_valid, avg_conf, rgb_msg.header)

        # ── Markers ───────────────────────────────────────────────
        if self._scan_mode:
            # Scan mode: usa ultima posizione torso rilevata (ciano)
            target = self._scan_torso_world
        else:
            # Normale: usa locked_target o stima Kalman (colore per stato)
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

        # ── Estrai in 3D tutti i keypoint rilevanti (torso + viso) ────
        kp_3d, kp_3d_conf = {}, {}
        for idx in (TORSO_KEYPOINTS + FACE_KEYPOINTS):
            if idx >= len(kp_conf) or kp_conf[idx] < self.conf_thr:
                continue
            u, v = int(kp_xy[idx, 0]), int(kp_xy[idx, 1])
            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue
            d = self._get_depth_robust(depth, u, v, window=5)
            if d is None:
                continue
            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            kp_3d[idx]      = [X, Y, d]
            kp_3d_conf[idx] = float(kp_conf[idx])

        shoulder_pts  = [kp_3d[i] for i in SHOULDER_KEYPOINTS if i in kp_3d]
        hip_pts       = [kp_3d[i] for i in HIP_KEYPOINTS      if i in kp_3d]
        shoulder_conf = [kp_3d_conf[i] for i in SHOULDER_KEYPOINTS if i in kp_3d]
        hip_conf      = [kp_3d_conf[i] for i in HIP_KEYPOINTS      if i in kp_3d]

        # Salva confidence per-keypoint torso per _update_scan (scan mode)
        self._last_kp_conf = {idx: kp_3d_conf[idx] for idx in (5, 6, 11, 12)
                              if idx in kp_3d_conf}

        # ── Metodo primario: spalle + fianchi ─────────────────────────
        if len(shoulder_pts) >= 1 and len(hip_pts) >= 1:
            torso_pts = shoulder_pts + hip_pts
            all_confs = shoulder_conf + hip_conf
            return np.mean(torso_pts, axis=0), kp_3d, len(torso_pts), float(np.mean(all_confs))

        # ── Metodo fallback: spalle + direzione viso ──────────────────
        # Il viso è "sopra" le spalle lungo l'asse del corpo.
        # Il centro del petto = shoulder_mid + chest_offset * (-direzione_viso)
        if self.use_face_fallback and len(shoulder_pts) >= 1:
            face_pts   = [kp_3d[i] for i in FACE_KEYPOINTS if i in kp_3d]
            face_confs = [kp_3d_conf[i] for i in FACE_KEYPOINTS if i in kp_3d]
            if len(face_pts) >= 1:
                shoulder_mid = np.mean(shoulder_pts, axis=0)
                face_center  = np.mean(face_pts,     axis=0)
                up_dir       = face_center - shoulder_mid
                up_norm      = np.linalg.norm(up_dir)
                if up_norm > 0.01:
                    up_dir /= up_norm
                    torso_raw = shoulder_mid + self.chest_offset_m * (-up_dir)
                else:
                    torso_raw = shoulder_mid  # direzione non stimabile
                n_valid  = len(shoulder_pts) + len(face_pts)
                avg_conf = float(np.mean(shoulder_conf + face_confs))
                self.get_logger().debug(
                    f'[tracker] fallback face: shoulder_mid→chest offset {self.chest_offset_m:.2f}m',
                    throttle_duration_sec=2.0)
                return torso_raw, kp_3d, n_valid, avg_conf

        return None, kp_3d, 0, 0.0

    # ──────────────────────────────────────────────────────────────
    def _update_scan(self, torso_raw, n_valid, avg_conf):
        """
        Logica di update in scan mode.
        Pubblica ogni frame su /torso_scan_point:
          [score, n_kp, conf, x_world, y_world, z_world,
           kp5_conf, kp6_conf, kp11_conf, kp12_conf]
        - Indici 0-5 : come prima (retrocompatibili)
        - Indici 6-9 : confidence individuale dei 4 keypoint torso.
                       0.0 se il keypoint non è stato rilevato.
        score = (n_kp / 4) * conf  per frame validi, 0.0 altrimenti.

        Stato interno:
          IDLE         → aspetta prima detection valida
          COLLECTING   → accumula frame validi
          POINT_LOCKED → ha raggiunto scan_min_frames (continua a pubblicare)
        """
        TORSO_KP_MAX = 4  # spalle + fianchi (max keypoint torso)

        valid = (torso_raw is not None
                 and n_valid >= self.min_keypoints
                 and avg_conf >= self.min_det_conf)

        if valid:
            per_frame_score = (min(n_valid, TORSO_KP_MAX) / TORSO_KP_MAX) * avg_conf
            torso_world     = self._camera_to_world(torso_raw)
        else:
            per_frame_score = 0.0
            torso_world     = None

        # Aggiorna ultima posizione nota in scan mode (usata dai marker)
        if torso_world is not None:
            self._scan_torso_world = torso_world

        # ── Confidence per-keypoint (kp5, kp6, kp11, kp12) ──────────────
        # Recuperata da _extract_torso via kp_data.conf già disponibile
        # nella callback. Usiamo self._last_kp_conf aggiornato in cb_synchronized.
        kp5_c  = float(getattr(self, '_last_kp_conf', {}).get(5,  0.0))
        kp6_c  = float(getattr(self, '_last_kp_conf', {}).get(6,  0.0))
        kp11_c = float(getattr(self, '_last_kp_conf', {}).get(11, 0.0))
        kp12_c = float(getattr(self, '_last_kp_conf', {}).get(12, 0.0))

        # ── Pubblica dato per-frame ──────────────────────────────────────
        if torso_world is not None:
            data = [per_frame_score, float(n_valid), avg_conf,
                    float(torso_world[0]), float(torso_world[1]), float(torso_world[2]),
                    kp5_c, kp6_c, kp11_c, kp12_c]
        else:
            data = [0.0] * 10
        msg = Float32MultiArray()
        msg.data = data
        self.pub_scan_point.publish(msg)

        # ── Aggiorna stato scan ──
        prev_scan = self._scan_state
        if self._scan_state == 'IDLE':
            if valid:
                self._scan_state = 'COLLECTING'
                self._scan_valid = 1
        elif self._scan_state == 'COLLECTING':
            if valid:
                self._scan_valid += 1
                if self._scan_valid >= self._scan_min_frames:
                    self._scan_state = 'POINT_LOCKED'
                    self.get_logger().info(
                        f'🔒 SCAN_POINT_LOCKED ({self._scan_valid} frame validi)')
            else:
                # Frame non valido: decrementa (tollera qualche frame perso)
                self._scan_valid = max(0, self._scan_valid - 1)
                if self._scan_valid == 0:
                    self._scan_state = 'IDLE'
        # In POINT_LOCKED: continua a pubblicare, aspetta scan_next dalla FSM

        # Pubblica stato (con prefisso SCAN_ per distinguerlo dal normale)
        scan_label = f'SCAN_{self._scan_state}'
        if self._scan_state != prev_scan:
            self.get_logger().info(f'🔄 Scan state: {prev_scan} → {self._scan_state}')
        self.pub_tracker_state.publish(String(data=scan_label))

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

                self.recovery_counter = 0   # buona rilevazione: azzera contatore recovery

                self.kf.predict(self.vel_damping)
                self.kf.update(torso_raw)
                estimated_cam = self.kf.get_position()

                self.position_history.append(estimated_cam.copy())
                if len(self.position_history) > self.lock_stable_frames:
                    self.position_history.pop(0)

                target_world = self._camera_to_world(estimated_cam)
                if target_world is not None:
                    interpolated = self._interpolate_to_target(target_world)

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
                # Frame non valido: non tornare subito a IDLE, usa recovery_frames
                self.recovery_counter += 1
                if self.recovery_counter >= self.recovery_frames:
                    self.get_logger().warn(
                        f'⚠️ Torso perso durante stima '
                        f'({self.recovery_counter} frame consecutivi) → IDLE'
                    )
                    self.state            = 'IDLE'
                    self.kf.reset()
                    self.position_history     = []
                    self.tracking_current_pos = None
                    self.recovery_counter     = 0
                else:
                    # Predict-only: mantiene stima KF senza aggiornamento
                    self.kf.predict(self.vel_damping)
                    self.get_logger().debug(
                        f'[ESTIMATING] recovery {self.recovery_counter}/{self.recovery_frames}',
                        throttle_duration_sec=0.5
                    )

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
        color   = (STATE_COLORS['SCANNING'] if self._scan_mode
                   else STATE_COLORS.get(self.state, STATE_COLORS['IDLE']))

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

        # 2a. Sfere keypoint torso (spalle + fianchi) — blu
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

        # 2b. Sfere keypoint viso (naso/occhi/orecchie) — giallo
        for i, idx in enumerate(FACE_KEYPOINTS):
            if idx not in kp_3d_w:
                continue
            kp = kp_3d_w[idx]
            m = Marker()
            m.header.frame_id    = frame
            m.header.stamp       = stamp
            m.ns                 = 'face_keypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(kp[0])
            m.pose.position.y    = float(kp[1])
            m.pose.position.z    = float(kp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.03
            m.color   = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.8)  # giallo
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