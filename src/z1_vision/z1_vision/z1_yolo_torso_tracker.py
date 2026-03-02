#!/usr/bin/env python3
""" 
Z1 YOLO Torso Tracker (percezione + filtro + LOCK interno) — versione "reattiva" come il vecchio tracker.

Obiettivo
- Ripristinare il comportamento RViz "di prima": marker stabili, colore che cambia subito con lo stato,
  niente inseguimento lento del pallino.
- Adattare I/O alla NUOVA FSM esterna (che gestisce approaching/controllo).

Pubblica
- /torso_target_camera        (PointStamped, camera_depth_optical_frame)  # stima filtrata in camera frame
- /torso_target_ee            (PoseStamped, world)                        # stima filtrata in world
- /torso_target_ee_locked     (PoseStamped, world)                        # target LOCKED (world)
- /target_lock_valid          (Bool)                                      # True solo quando LOCKED
- /target_lock_state          (String)                                    # IDLE/ESTIMATING/LOCKED
- /torso_visible              (Bool)
- /torso_avg_conf             (Float32)
- /torso_n_valid_kp           (Int32)
- /torso_tracker_state        (String)                                    # TRACKING/LOST
- /torso_markers              (MarkerArray)

Sottoscrive
- /camera/camera/color/image_raw
- /camera/camera/depth/image_rect_raw
- /camera/camera/color/camera_info
- /torso_sm_state             (String)  # stato FSM esterna (solo per label/colore RViz)

FSM interna (solo lock, simile alla vecchia)
- IDLE -> ESTIMATING -> LOCKED
- Se detection valida manca: IDLE (ma il LOCKED resta pubblicato finché la FSM esterna non resetta in WAITING)

Nota
- La FSM esterna decide quando usare /torso_target_ee (tracking) oppure /torso_target_ee_locked (lock) e quando
  resettare (pubblicando WAITING su /torso_sm_state).
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


# COCO Keypoints: spalle + fianchi
TORSO_KEYPOINTS = [5, 6, 11, 12]
TORSO_EDGES = [
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
]

# Colori stile "vecchio" (stati interni)
STATE_COLORS_INTERNAL = {
    'IDLE':       ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),  # grigio
    'ESTIMATING': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),  # arancione
    'LOCKED':     ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),  # verde
}

# (opzionale) colori stati FSM esterna
STATE_COLORS_EXTERNAL = {
    'WAITING':         ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8),
    'APPROACHING_JTC': ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9),
    'WAIT_JTC':        ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9),
    'SWITCH_TO_TORQUE':      ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
    'WAIT_SWITCH_TO_TORQUE': ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),
    'IMPEDANCE_CONTACT': ColorRGBA(r=0.8, g=0.0, b=1.0, a=0.9),
    'HOLD_CONTACT':      ColorRGBA(r=1.0, g=0.0, b=0.6, a=0.9),
    'SWITCH_TO_JTC':      ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),
    'WAIT_SWITCH_TO_JTC': ColorRGBA(r=0.0, g=0.7, b=0.7, a=0.9),
    'RETURN_HOME_JTC':    ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9),
    'WAIT_RETURN':        ColorRGBA(r=0.0, g=0.5, b=0.0, a=0.9),
    'FAULT':              ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.95),
}


def _median_depth(depth: np.ndarray, u: int, v: int, window: int, max_depth: float):
    h, w = depth.shape[:2]
    half = window // 2
    patch = depth[max(v - half, 0):min(v + half + 1, h),
                  max(u - half, 0):min(u + half + 1, w)]
    valid = patch[(patch > 0.05) & (patch < max_depth)]
    if valid.size < 3:
        return None
    return float(np.median(valid))


class Z1YoloTorsoTracker(Node):

    def __init__(self):
        super().__init__('z1_yolo_torso_tracker')

        # ── Parametri base
        self.declare_parameter('model_path', 'yolo11n-pose.pt')
        self.declare_parameter('conf_thr', 0.3)
        self.declare_parameter('max_depth', 2.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('imgsz', 416)

        # ── Kalman
        self.declare_parameter('kf_process_noise', 5e-5)
        self.declare_parameter('kf_meas_noise', 1.0)
        self.declare_parameter('kf_vel_damping', 0.9)

        # ── Lock (come vecchio)
        self.declare_parameter('min_detection_conf', 0.6)
        self.declare_parameter('min_keypoints', 3)
        self.declare_parameter('lock_stable_frames', 20)
        self.declare_parameter('lock_variance_thr', 0.005)
        self.declare_parameter('lock_stable_checks', 5)
        self.declare_parameter('recovery_frames', 10)

        # ── Marker
        self.declare_parameter('starting_position', [0.0411, 0.0103, 0.5133])

        # Se True: colore marker segue FSM esterna quando presente
        self.declare_parameter('rviz_color_use_external_fsm', False)

        # ── Lettura parametri
        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_thr = float(self.get_parameter('conf_thr').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.device = str(self.get_parameter('device').value)
        self.imgsz = int(self.get_parameter('imgsz').value)

        self.kf_vel_damping = float(self.get_parameter('kf_vel_damping').value)
        self.kf_process_noise = float(self.get_parameter('kf_process_noise').value)
        self.kf_meas_noise = float(self.get_parameter('kf_meas_noise').value)

        self.min_det_conf = float(self.get_parameter('min_detection_conf').value)
        self.min_keypoints = int(self.get_parameter('min_keypoints').value)
        self.lock_stable_frames = int(self.get_parameter('lock_stable_frames').value)
        self.lock_variance_thr = float(self.get_parameter('lock_variance_thr').value)
        self.lock_stable_checks = int(self.get_parameter('lock_stable_checks').value)
        self.recovery_frames = int(self.get_parameter('recovery_frames').value)

        self.rviz_color_use_external_fsm = bool(self.get_parameter('rviz_color_use_external_fsm').value)

        raw_sp = self.get_parameter('starting_position').value
        self.starting_position = np.array(raw_sp, dtype=np.float64)

        # ── YOLO
        self.model = YOLO(self.model_path)
        try:
            self.model.to(self.device)
            self.get_logger().info(f'✅ YOLO su device: {self.device}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ YOLO fallback CPU: {e}')
            self.device = 'cpu'
            self.model.to('cpu')

        # ── Kalman
        self.kf = Kalman3D(dt=0.033, process_noise=self.kf_process_noise, measurement_noise=self.kf_meas_noise)

        # ── TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── ROS I/O
        self.bridge = CvBridge()

        self.sub_rgb = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.sub_depth = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.sync = ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.cb_synchronized)

        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 1)

        # FSM esterna
        self.sub_fsm_state = self.create_subscription(String, '/torso_sm_state', self.cb_fsm_state, 10)

        # Publishers nuovi
        self.pub_torso_camera = self.create_publisher(PointStamped, '/torso_target_camera', 10)
        self.pub_torso_ee = self.create_publisher(PoseStamped, '/torso_target_ee', 10)
        self.pub_torso_ee_locked = self.create_publisher(PoseStamped, '/torso_target_ee_locked', 10)

        self.pub_lock_valid = self.create_publisher(Bool, '/target_lock_valid', 10)
        self.pub_lock_state = self.create_publisher(String, '/target_lock_state', 10)

        self.pub_visible = self.create_publisher(Bool, '/torso_visible', 10)
        self.pub_avg_conf = self.create_publisher(Float32, '/torso_avg_conf', 10)
        self.pub_n_valid_kp = self.create_publisher(Int32, '/torso_n_valid_kp', 10)
        self.pub_tracker_state = self.create_publisher(String, '/torso_tracker_state', 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/torso_markers', 10)

        # ── Stato
        self.cam_info: CameraInfo | None = None
        self.fsm_state_external: str = 'WAITING'

        self.state: str = 'IDLE'  # IDLE / ESTIMATING / LOCKED
        self.locked_target: np.ndarray | None = None  # world

        self.position_history: list[np.ndarray] = []
        self.stable_counter: int = 0
        self.lost_counter: int = 0

        self.last_stamp_sec: float | None = None

        self._publish_lock_outputs()

        self.get_logger().info(
            '🚀 Torso tracker pronto (logica vecchia + I/O nuova)\n'
            f'   starting_position (marker): {self.starting_position.tolist()}\n'
            f'   rviz_color_use_external_fsm: {self.rviz_color_use_external_fsm}'
        )

    # ──────────────────────────────────────────────
    def cb_info(self, msg: CameraInfo):
        self.cam_info = msg

    def cb_fsm_state(self, msg: String):
        prev = self.fsm_state_external
        self.fsm_state_external = msg.data

        # Reset ciclo quando la FSM esterna torna a WAITING
        if self.fsm_state_external == 'WAITING' and prev != 'WAITING':
            self._reset_internal(reset_lock=True)

    # ──────────────────────────────────────────────
    def cb_synchronized(self, rgb_msg: Image, depth_msg: Image):
        if self.cam_info is None:
            self.get_logger().warn('CameraInfo non ancora ricevuta', throttle_duration_sec=2.0)
            return

        # dt reale per Kalman
        now_sec = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        if self.last_stamp_sec is not None:
            real_dt = now_sec - self.last_stamp_sec
            if 0.01 < real_dt < 0.5:
                self.kf.dt = float(real_dt)
        self.last_stamp_sec = now_sec

        # immagini
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0

            h_rgb, w_rgb = rgb.shape[:2]
            if depth.shape[:2] != (h_rgb, w_rgb):
                depth = cv2.resize(depth, (w_rgb, h_rgb), interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            self.get_logger().error(f'Errore immagini: {e}')
            return

        # YOLO
        try:
            results = self.model.predict(
                rgb,
                conf=self.conf_thr,
                classes=[0],
                verbose=False,
                imgsz=self.imgsz,
                device=self.device,
            )
        except Exception as e:
            self.get_logger().error(f'YOLO fallita: {e}')
            return

        torso_raw, kp_3d_cam, n_valid, avg_conf = self._extract_torso(results, depth)

        # diagnostica
        self.pub_avg_conf.publish(Float32(data=float(avg_conf)))
        self.pub_n_valid_kp.publish(Int32(data=int(n_valid)))

        ok_det = (torso_raw is not None and n_valid >= self.min_keypoints and avg_conf >= self.min_det_conf)

        if not ok_det:
            self._handle_lost()
            # marker: se lock esiste, rimane visibile
            if self.locked_target is not None:
                self._publish_markers(self.locked_target, kp_3d_cam, rgb_msg.header)
            return

        # detection ok
        self.lost_counter = 0

        # Kalman update
        if not self.kf.initialized:
            self.kf.reset()
            self.kf.initialize(torso_raw)
        else:
            self.kf.predict(self.kf_vel_damping)
            self.kf.update(torso_raw)

        est_cam = self.kf.get_position()

        # publish camera target
        cam_msg = PointStamped()
        cam_msg.header.stamp = self.get_clock().now().to_msg()
        cam_msg.header.frame_id = 'camera_depth_optical_frame'
        cam_msg.point.x = float(est_cam[0])
        cam_msg.point.y = float(est_cam[1])
        cam_msg.point.z = float(est_cam[2])
        self.pub_torso_camera.publish(cam_msg)

        # camera -> world
        est_world = self._camera_to_world(est_cam)
        if est_world is None:
            self.get_logger().warn('TF camera->world non disponibile', throttle_duration_sec=2.0)
            self.pub_visible.publish(Bool(data=False))
            self.pub_tracker_state.publish(String(data='LOST'))
            self._publish_lock_outputs()
            if self.locked_target is not None:
                self._publish_markers(self.locked_target, kp_3d_cam, rgb_msg.header)
            return

        # publish base outputs
        self.pub_visible.publish(Bool(data=True))
        self.pub_tracker_state.publish(String(data='TRACKING'))

        # FSM interna (vecchia logica)
        self._update_state(est_world)

        # publish stima world (NO interpolazione)
        self._publish_target_world(est_world)

        # marker: se lock esiste mostra lock, altrimenti mostra stima
        marker_target = self.locked_target if self.locked_target is not None else est_world
        self._publish_markers(marker_target, kp_3d_cam, rgb_msg.header)

    # ──────────────────────────────────────────────
    def _reset_internal(self, reset_lock: bool = True):
        self.state = 'IDLE'
        self.position_history = []
        self.stable_counter = 0
        self.lost_counter = 0
        self.kf.reset()
        if reset_lock:
            self.locked_target = None
        self._publish_lock_outputs()

    def _handle_lost(self):
        self.pub_visible.publish(Bool(data=False))
        self.pub_tracker_state.publish(String(data='LOST'))

        self.lost_counter += 1

        # Se non siamo LOCKED, torna IDLE subito (come nel vecchio comportamento)
        if self.state != 'LOCKED':
            self.state = 'IDLE'
            self.position_history = []
            self.stable_counter = 0
            self.kf.reset()
        # Se siamo LOCKED, teniamo il lock finché la FSM esterna non resetta in WAITING

        self._publish_lock_outputs()

    def _update_state(self, est_world: np.ndarray):
        prev_state = self.state

        # IDLE -> ESTIMATING
        if self.state == 'IDLE':
            self.state = 'ESTIMATING'
            self.position_history = []
            self.stable_counter = 0

        # ESTIMATING: accumula history e lock se stabile
        if self.state == 'ESTIMATING':
            self.position_history.append(est_world.copy())
            if len(self.position_history) > self.lock_stable_frames:
                self.position_history.pop(0)

            if len(self.position_history) >= self.lock_stable_frames:
                var = float(np.var(np.asarray(self.position_history), axis=0).sum())
                if var < self.lock_variance_thr:
                    self.stable_counter += 1
                else:
                    self.stable_counter = 0

                if self.stable_counter >= self.lock_stable_checks:
                    self.state = 'LOCKED'
                    self.locked_target = est_world.copy()  # LOCK FISSO (NO inseguimento)

        # LOCKED: resta fisso
        elif self.state == 'LOCKED':
            if self.locked_target is None:
                self.locked_target = est_world.copy()

        if self.state != prev_state:
            self.get_logger().info(f'🔄 {prev_state} → {self.state}')

        self._publish_lock_outputs()

    def _publish_lock_outputs(self):
        valid = (self.state == 'LOCKED' and self.locked_target is not None)

        self.pub_lock_valid.publish(Bool(data=bool(valid)))
        self.pub_lock_state.publish(String(data=str(self.state)))

        if valid:
            self._publish_target_world_locked(self.locked_target)

    # ──────────────────────────────────────────────
    def _publish_target_world(self, point_world: np.ndarray):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee.publish(pose)

    def _publish_target_world_locked(self, point_world: np.ndarray):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x = float(point_world[0])
        pose.pose.position.y = float(point_world[1])
        pose.pose.position.z = float(point_world[2])
        pose.pose.orientation.w = 1.0
        self.pub_torso_ee_locked.publish(pose)

    # ──────────────────────────────────────────────
    def _camera_to_world(self, point_cam: np.ndarray):
        pt = PointStamped()
        pt.header.frame_id = 'camera_depth_optical_frame'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(point_cam[0])
        pt.point.y = float(point_cam[1])
        pt.point.z = float(point_cam[2])

        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            out = do_transform_point(pt, tf)
            return np.array([out.point.x, out.point.y, out.point.z], dtype=np.float64)
        except TransformException as e:
            self.get_logger().warn(f'TF camera→world fallita: {e}', throttle_duration_sec=2.0)
            return None

    # ──────────────────────────────────────────────
    def _extract_torso(self, results, depth: np.ndarray):
        if len(results) == 0 or results[0].keypoints is None:
            return None, {}, 0, 0.0

        kp_data = results[0].keypoints
        if kp_data.xy is None or kp_data.xy.shape[0] == 0:
            return None, {}, 0, 0.0

        kp_xy = kp_data.xy.cpu().numpy()[0]
        kp_conf = kp_data.conf.cpu().numpy()[0]

        K = np.array(self.cam_info.k).reshape(3, 3)
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])

        pts = []
        kp_3d = {}
        confs = []

        for idx in TORSO_KEYPOINTS:
            if float(kp_conf[idx]) < self.conf_thr:
                continue

            u, v = int(kp_xy[idx][0]), int(kp_xy[idx][1])
            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue

            d = _median_depth(depth, u, v, window=5, max_depth=self.max_depth)
            if d is None:
                continue

            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy

            pts.append([X, Y, d])
            kp_3d[idx] = np.array([X, Y, d], dtype=np.float64)
            confs.append(float(kp_conf[idx]))

        if len(pts) < 2:
            return None, {}, len(pts), float(np.mean(confs)) if confs else 0.0

        torso = np.mean(np.asarray(pts, dtype=np.float64), axis=0)
        avg_conf = float(np.mean(confs)) if confs else 0.0
        return torso, kp_3d, len(pts), avg_conf

    # ──────────────────────────────────────────────
    def _pick_marker_color(self) -> ColorRGBA:
        if self.rviz_color_use_external_fsm:
            return STATE_COLORS_EXTERNAL.get(
                self.fsm_state_external,
                STATE_COLORS_INTERNAL.get(self.state, STATE_COLORS_INTERNAL['IDLE'])
            )
        return STATE_COLORS_INTERNAL.get(self.state, STATE_COLORS_INTERNAL['IDLE'])

    def _publish_markers(self, torso_center_world: np.ndarray, kp_3d_cam: dict, header):
        if torso_center_world is None:
            return

        frame = 'world'
        stamp = self.get_clock().now().to_msg()  # come nel tuo file "buono"
        color = self._pick_marker_color()

        markers = MarkerArray()

        # keypoints -> world
        kp_3d_w = {}
        for idx, pt_cam in kp_3d_cam.items():
            w = self._camera_to_world(pt_cam)
            if w is not None:
                kp_3d_w[idx] = w

        # 1) centro torso
        cm = Marker()
        cm.header.frame_id = frame
        cm.header.stamp = stamp
        cm.ns = 'torso_center'
        cm.id = 0
        cm.type = Marker.SPHERE
        cm.action = Marker.ADD
        cm.pose.position.x = float(torso_center_world[0])
        cm.pose.position.y = float(torso_center_world[1])
        cm.pose.position.z = float(torso_center_world[2])
        cm.pose.orientation.w = 1.0
        cm.scale.x = cm.scale.y = cm.scale.z = 0.08
        cm.color = color
        cm.lifetime.nanosec = 200_000_000
        markers.markers.append(cm)

        # 2) keypoints spheres
        for i, idx in enumerate(TORSO_KEYPOINTS):
            if idx not in kp_3d_w:
                continue
            kp = kp_3d_w[idx]
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns = 'torso_keypoints'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(kp[0])
            m.pose.position.y = float(kp[1])
            m.pose.position.z = float(kp[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.04
            m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            m.lifetime.nanosec = 200_000_000
            markers.markers.append(m)

        # 3) edges
        lm = Marker()
        lm.header.frame_id = frame
        lm.header.stamp = stamp
        lm.ns = 'torso_edges'
        lm.id = 10
        lm.type = Marker.LINE_LIST
        lm.action = Marker.ADD
        lm.scale.x = 0.015
        lm.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        lm.pose.orientation.w = 1.0
        lm.lifetime.nanosec = 200_000_000
        for a, b in TORSO_EDGES:
            if a in kp_3d_w and b in kp_3d_w:
                lm.points.append(Point(x=float(kp_3d_w[a][0]), y=float(kp_3d_w[a][1]), z=float(kp_3d_w[a][2])))
                lm.points.append(Point(x=float(kp_3d_w[b][0]), y=float(kp_3d_w[b][1]), z=float(kp_3d_w[b][2])))
        if lm.points:
            markers.markers.append(lm)

        # 4) spokes
        sm = Marker()
        sm.header.frame_id = frame
        sm.header.stamp = stamp
        sm.ns = 'torso_spokes'
        sm.id = 11
        sm.type = Marker.LINE_LIST
        sm.action = Marker.ADD
        sm.scale.x = 0.008
        sm.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.6)
        sm.pose.orientation.w = 1.0
        sm.lifetime.nanosec = 200_000_000
        pc = Point(x=float(torso_center_world[0]), y=float(torso_center_world[1]), z=float(torso_center_world[2]))
        for idx in TORSO_KEYPOINTS:
            if idx in kp_3d_w:
                sm.points.append(pc)
                sm.points.append(Point(x=float(kp_3d_w[idx][0]), y=float(kp_3d_w[idx][1]), z=float(kp_3d_w[idx][2])))
        if sm.points:
            markers.markers.append(sm)

        # 5) testo
        tm = Marker()
        tm.header.frame_id = frame
        tm.header.stamp = stamp
        tm.ns = 'torso_state'
        tm.id = 20
        tm.type = Marker.TEXT_VIEW_FACING
        tm.action = Marker.ADD
        tm.pose.position.x = float(torso_center_world[0])
        tm.pose.position.y = float(torso_center_world[1])
        tm.pose.position.z = float(torso_center_world[2]) + 0.15
        tm.pose.orientation.w = 1.0
        tm.scale.z = 0.05
        tm.color = color
        tm.text = f"EXT:{self.fsm_state_external} | INT:{self.state}"
        tm.lifetime.nanosec = 200_000_000
        markers.markers.append(tm)

        # 6) starting_position marker
        sp = Marker()
        sp.header.frame_id = frame
        sp.header.stamp = stamp
        sp.ns = 'starting_position'
        sp.id = 30
        sp.type = Marker.SPHERE
        sp.action = Marker.ADD
        sp.pose.position.x = float(self.starting_position[0])
        sp.pose.position.y = float(self.starting_position[1])
        sp.pose.position.z = float(self.starting_position[2])
        sp.pose.orientation.w = 1.0
        sp.scale.x = sp.scale.y = sp.scale.z = 0.06
        sp.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.6)
        sp.lifetime.nanosec = 200_000_000
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