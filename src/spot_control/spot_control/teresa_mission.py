#!/usr/bin/env python3
"""
TERESA Mission Node
FSM: IDLE → COMPUTING_GOAL → NAVIGATING → ARRIVED → CROUCHING → READY_FOR_SCAN

Gestisce la navigazione di Spot verso un paziente sdraiato:
- Calcola approach point laterale al centro del torso
- Rispetta il bounding box (non lo supera)
- Sceglie automaticamente il lato migliore
- Crouches all'arrivo per avvicinare il braccio Z1

dry_run=True  → calcola geometria e pubblica RViz, Spot non si muove
dry_run=False → esegue Trajectory action + SetStandHeight

PREREQUISITI:
  - spot_perception.launch.py in esecuzione
  - spot_ros2 su SpotCore (solo se dry_run=False)
"""

import math
import os
import sys
import termios
import threading
import tty

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 — registra il transform per PoseStamped

# COCO keypoint indices
NOSE       = 0
L_SHOULDER = 5
R_SHOULDER = 6
L_HIP      = 11
R_HIP      = 12
L_KNEE     = 13
R_KNEE     = 14
L_ANKLE    = 15
R_ANKLE    = 16

# FSM states
IDLE           = 'IDLE'
COMPUTING_GOAL = 'COMPUTING_GOAL'
NAVIGATING     = 'NAVIGATING'
ARRIVED        = 'ARRIVED'
CROUCHING      = 'CROUCHING'
READY_FOR_SCAN = 'READY_FOR_SCAN'
STANDING_UP    = 'STANDING_UP'
RETURNING      = 'RETURNING'


class TeresaMission(Node):

    def __init__(self):
        super().__init__('teresa_mission')

        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('approach_margin',   0.05)
        self.declare_parameter('spot_front_offset', 0.50)
        self.declare_parameter('min_confidence', 0.6)
        self.declare_parameter('nav_timeout', 30.0)
        self.declare_parameter('crouch_height', -0.10)
        self.declare_parameter('preferred_side', 'auto')   # 'auto' | 'left' | 'right'
        self.declare_parameter('goal_frame', 'my_spot/odom')
        self.declare_parameter('dry_run', False)

        self.approach_margin    = float(self.get_parameter('approach_margin').value)
        self.spot_front_offset  = float(self.get_parameter('spot_front_offset').value)
        self.min_conf           = float(self.get_parameter('min_confidence').value)
        self.nav_timeout        = float(self.get_parameter('nav_timeout').value)
        self.crouch_height      = float(self.get_parameter('crouch_height').value)
        self.preferred_side     = str(self.get_parameter('preferred_side').value)
        self.goal_frame         = str(self.get_parameter('goal_frame').value)
        self.dry_run            = bool(self.get_parameter('dry_run').value)

        # ============================================================
        # TF
        # ============================================================
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_available = False   # aggiornato al primo lookup riuscito

        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        self.sub_keypoints = self.create_subscription(
            PoseArray, '/human_pose/points_3d', self._cb_keypoints, 10
        )
        self.sub_bbox = self.create_subscription(
            Marker, '/human_pose/bounding_box', self._cb_bbox, 10
        )
        self.sub_posture = self.create_subscription(
            String, '/human_pose/posture', self._cb_posture, 10
        )
        self.sub_conf = self.create_subscription(
            Float32, '/human_pose/posture_confidence', self._cb_conf, 10
        )

        # ============================================================
        # PUBLISHERS — RViz markers
        # ============================================================
        self.pub_body_axis    = self.create_publisher(Marker,      '/teresa/body_axis',    10)
        self.pub_lateral_dir  = self.create_publisher(Marker,      '/teresa/lateral_dir',  10)
        self.pub_approach_goal = self.create_publisher(Marker,     '/teresa/approach_goal', 10)
        self.pub_safe_zone    = self.create_publisher(Marker,      '/teresa/safe_zone',     10)
        self.pub_fsm_state    = self.create_publisher(Marker,      '/teresa/fsm_state',     10)

        # ============================================================
        # ACTION CLIENTS (solo se dry_run=False)
        # ============================================================
        self._traj_client    = None
        self._traj_goal_handle = None
        self._stand_client   = None

        if not self.dry_run:
            try:
                from spot_msgs.action import Trajectory
                from spot_msgs.srv import SetStandHeight
                self._traj_client  = ActionClient(self, Trajectory,      '/spot/trajectory')
                self._stand_client = self.create_client(SetStandHeight,  '/spot/stand_height')
                self.get_logger().info('Action clients inizializzati (dry_run=False)')
            except ImportError:
                self.get_logger().error(
                    'spot_msgs non trovato — imposta dry_run:=true se Spot non è disponibile'
                )
                self.dry_run = True

        # ============================================================
        # STATO INTERNO
        # ============================================================
        self._state          = IDLE
        self._posture        = 'UNKNOWN'
        self._confidence     = 0.0
        self._keypoints      = None   # lista di np.array o None per keypoint invalidi
        self._bbox_center    = None   # np.array [x,y,z] in camera frame
        self._bbox_size      = None   # np.array [sx,sy,sz]
        self._bbox_frame     = None   # frame_id della bbox
        self._locked_goal    = None   # PoseStamped in goal_frame, bloccato durante nav
        self._nav_start_time = None

        # Flag: 's' sblocca l'invio goal, ESC ferma tutto
        self._go_authorized = False
        self._estop          = False

        # Return-to-start
        self._start_pose     = None   # PoseStamped odom, salvata prima di navigare
        self._standing_done  = False  # True quando SetStandHeight(0) completato
        self._standing_sent  = False  # evita invii multipli
        self._returning_sent = False  # evita invii multipli

        # Keyboard thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # Timer FSM a 5 Hz
        self.create_timer(0.2, self._fsm_tick)

        mode = 'DRY RUN (Spot non si muove)' if self.dry_run else 'ATTIVO (Spot si muove)'
        self.get_logger().info(
            f'✅ TeresaMission READY — {mode}\n'
            f'   approach_margin={self.approach_margin}m  '
            f'min_conf={self.min_conf}  '
            f'crouch={self.crouch_height}m  '
            f'side={self.preferred_side}\n'
            f'   Tasti: "s" = avvia | "a" = alza Spot | "b" = torna a start | ESC = stop | Ctrl+C = quit'
        )

    # ============================================================
    # CALLBACKS SUBSCRIBERS
    # ============================================================

    def _cb_keypoints(self, msg):
        pts = []
        for p in msg.poses:
            if math.isnan(p.position.x):
                pts.append(None)
            else:
                pts.append(np.array([p.position.x, p.position.y, p.position.z],
                                    dtype=np.float64))
        self._keypoints = pts
        self._kp_frame  = msg.header.frame_id

    def _cb_bbox(self, msg: Marker):
        if msg.action == Marker.DELETE:
            self._bbox_center = None
            self._bbox_size   = None
            self._bbox_frame  = None
            return
        self._bbox_center = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float64)
        self._bbox_size  = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float64)
        self._bbox_frame = msg.header.frame_id

    def _cb_posture(self, msg: String):
        self._posture = msg.data

    def _cb_conf(self, msg: Float32):
        self._confidence = float(msg.data)

    # ============================================================
    # FSM TICK
    # ============================================================

    def _fsm_tick(self):
        if self._state == IDLE:
            self._tick_idle()
        elif self._state == COMPUTING_GOAL:
            self._tick_computing()
        elif self._state == NAVIGATING:
            self._tick_navigating()
        elif self._state == ARRIVED:
            self._tick_arrived()
        elif self._state == CROUCHING:
            self._tick_crouching()
        elif self._state == READY_FOR_SCAN:
            self._tick_ready()
        elif self._state == STANDING_UP:
            self._tick_standing_up()
        elif self._state == RETURNING:
            self._tick_returning()

        # Pubblica sempre stato FSM in RViz
        self._publish_fsm_label()

    def _set_state(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'FSM: {self._state} → {new_state}')
            self._state = new_state

    # ============================================================
    # STATI FSM
    # ============================================================

    def _tick_idle(self):
        if self._estop:
            return
        ready = (self._posture == 'LYING'
                 and self._confidence >= self.min_conf
                 and self._keypoints is not None
                 and self._bbox_center is not None)
        if ready and not self._go_authorized:
            self.get_logger().info(
                'Paziente rilevato — premi "s" per avviare navigazione, ESC per stop.',
                throttle_duration_sec=3.0
            )
        if ready and self._go_authorized:
            self._go_authorized = False
            self._set_state(COMPUTING_GOAL)

    def _tick_computing(self):
        goal = self._compute_approach_goal()

        if goal is None:
            self.get_logger().warn(
                'COMPUTING_GOAL: impossibile calcolare goal, keypoints insufficienti',
                throttle_duration_sec=2.0
            )
            self._set_state(IDLE)
            return

        self._locked_goal = goal
        self.get_logger().info(
            f'Goal locked: ({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f}) '
            f'frame={goal.header.frame_id}'
        )

        if self.dry_run:
            self.get_logger().info('DRY RUN: goal calcolato, Spot non si muove')
            self._set_state(READY_FOR_SCAN)
        else:
            self._save_start_pose()
            self._send_trajectory(goal)
            self._nav_start_time = self.get_clock().now()
            self._set_state(NAVIGATING)

    def _tick_navigating(self):
        if self._estop:
            self._cancel_trajectory()
            self._set_state(IDLE)
            return

        # Timeout
        elapsed = (self.get_clock().now() - self._nav_start_time).nanoseconds * 1e-9
        if elapsed > self.nav_timeout:
            self.get_logger().warn(f'NAVIGATING: timeout {self.nav_timeout}s → IDLE')
            self._cancel_trajectory()
            self._set_state(IDLE)
            return

        # Confidence bassa (Orbbec perde il corpo da vicino) → freeze goal, no relock
        if self._confidence < self.min_conf:
            self.get_logger().info(
                f'Confidence bassa ({self._confidence:.2f}) durante navigazione — goal congelato.',
                throttle_duration_sec=3.0
            )
            return


    def _tick_arrived(self):
        self._set_state(CROUCHING)

    def _tick_crouching(self):
        if self.dry_run:
            self._set_state(READY_FOR_SCAN)
            return

        if self._stand_client is None:
            self._set_state(READY_FOR_SCAN)
            return

        if not self._stand_client.service_is_ready():
            self.get_logger().warn('SetStandHeight service non disponibile',
                                   throttle_duration_sec=2.0)
            return

        try:
            from spot_msgs.srv import SetStandHeight
            req = SetStandHeight.Request()
            req.height = float(self.crouch_height)
            future = self._stand_client.call_async(req)
            future.add_done_callback(self._cb_crouch_done)
            # Transizione avverrà nel callback
        except Exception as e:
            self.get_logger().error(f'Errore SetStandHeight: {e}')
            self._set_state(READY_FOR_SCAN)

    def _cb_crouch_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Crouch OK (height={self.crouch_height}m)')
            else:
                self.get_logger().warn(f'Crouch fallito: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Crouch exception: {e}')
        self._set_state(READY_FOR_SCAN)

    def _tick_ready(self):
        self.get_logger().info(
            '🟢 READY_FOR_SCAN — Spot in posizione, pronto per Z1 FAST scan',
            throttle_duration_sec=5.0
        )
        # Qui in futuro: trigger z1_FSM

    def _tick_standing_up(self):
        if self.dry_run or self._stand_client is None:
            self._standing_done = True
            self.get_logger().info('DRY RUN: Spot in piedi — premi B per tornare')
            return
        if not self._stand_client.service_is_ready():
            self.get_logger().warn('SetStandHeight non disponibile', throttle_duration_sec=2.0)
            return
        if not self._standing_sent:
            self._standing_sent = True
            try:
                from spot_msgs.srv import SetStandHeight
                req = SetStandHeight.Request()
                req.height = 0.0  # altezza neutra (in piedi)
                future = self._stand_client.call_async(req)
                future.add_done_callback(self._cb_stand_done)
            except Exception as e:
                self.get_logger().error(f'Errore SetStandHeight stand-up: {e}')
                self._standing_done = True
                self._standing_sent = False

    def _cb_stand_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('✅ Spot in piedi — premi B per tornare alla posizione iniziale')
            else:
                self.get_logger().warn(f'Stand-up fallito: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Stand-up exception: {e}')
        self._standing_done = True
        self._standing_sent = False

    def _tick_returning(self):
        if self._estop:
            self._cancel_trajectory()
            self._set_state(IDLE)
            return
        if self._start_pose is None:
            self.get_logger().warn('Nessuna start pose salvata — torno a IDLE')
            self._set_state(IDLE)
            return
        if not self._returning_sent:
            self._returning_sent = True
            self._nav_start_time = self.get_clock().now()
            self._send_trajectory(self._start_pose)
            return
        elapsed = (self.get_clock().now() - self._nav_start_time).nanoseconds * 1e-9
        if elapsed > self.nav_timeout:
            self.get_logger().warn(f'RETURNING: timeout {self.nav_timeout}s → IDLE')
            self._cancel_trajectory()
            self._returning_sent = False
            self._set_state(IDLE)

    def _save_start_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame, 'my_spot/body',
                rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            ps = PoseStamped()
            ps.header.frame_id = self.goal_frame
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = t.transform.rotation
            self._start_pose = ps
            self.get_logger().info(
                f'Start pose salvata: ({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f}) [{self.goal_frame}]'
            )
        except TransformException as e:
            self.get_logger().warn(f'Impossibile salvare start pose: {e}')

    # ============================================================
    # KEYBOARD
    # ============================================================

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self.get_logger().warn(
                'Keyboard non disponibile (stdin non è un TTY — lanciato via ros2 launch?).\n'
                '   Usa: ros2 run spot_control teresa_mission per il controllo da tastiera.'
            )
            return
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == 's':
                    self._go_authorized = True
                    self.get_logger().info('✅ Navigazione autorizzata da tastiera.')
                elif ch == 'a':
                    if self._state == READY_FOR_SCAN:
                        self._standing_done = False
                        self._standing_sent = False
                        self._set_state(STANDING_UP)
                        self.get_logger().info('A: Spot si alza...')
                    else:
                        self.get_logger().warn(
                            f'A ignorato — valido solo da READY_FOR_SCAN (stato attuale: {self._state})'
                        )
                elif ch == 'b':
                    if self._state == STANDING_UP and self._standing_done:
                        self._returning_sent = False
                        self._set_state(RETURNING)
                        self.get_logger().info('B: Spot torna alla posizione iniziale...')
                    elif self._state == STANDING_UP:
                        self.get_logger().warn('B: attendi che Spot finisca di alzarsi (A ancora in corso)...')
                    else:
                        self.get_logger().warn(
                            f'B ignorato — premi prima A da READY_FOR_SCAN (stato attuale: {self._state})'
                        )
                elif ch == '\x1b':  # ESC
                    self._estop = True
                    self._cancel_trajectory()
                    self.get_logger().warn('🛑 EMERGENCY STOP — missione annullata.')
                    self._set_state(IDLE)
                elif ch == '\x03':  # Ctrl+C
                    rclpy.shutdown()
                    break
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ============================================================
    # GEOMETRIA — calcolo approach point
    # ============================================================

    def _compute_approach_goal(self):
        """
        Calcola la PoseStamped di approccio per Spot.

        Geometria:
        1. Asse corpo: da piedi (ankles) a testa (nose/shoulders)
        2. Centro torso: media di shoulders + hips
        3. Direzione laterale: perpendicolare all'asse corpo sul piano orizzontale
        4. Approach point: torso_center + lateral * (bbox_half + margin)
        5. Orientamento: Spot guarda verso torso_center
        6. Lato: auto (minor rotazione da posizione Spot) o forzato

        Ritorna PoseStamped nel frame disponibile (odom se TF ok, camera frame altrimenti).
        """
        kp = self._keypoints
        if kp is None or len(kp) < 17:
            return None

        # --- Centro torso ---
        torso_pts = []
        for idx in [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]:
            if kp[idx] is not None:
                torso_pts.append(kp[idx])
        if len(torso_pts) < 2:
            return None
        torso_center = np.mean(torso_pts, axis=0)

        # --- Asse corpo (piedi → testa) ---
        head_pts = []
        if kp[NOSE] is not None:
            head_pts.append(kp[NOSE])
        for idx in [L_SHOULDER, R_SHOULDER]:
            if kp[idx] is not None:
                head_pts.append(kp[idx])

        feet_pts = []
        for idx in [L_ANKLE, R_ANKLE]:
            if kp[idx] is not None:
                feet_pts.append(kp[idx])
        if len(feet_pts) == 0:
            for idx in [L_KNEE, R_KNEE]:
                if kp[idx] is not None:
                    feet_pts.append(kp[idx])

        if len(head_pts) == 0 or len(feet_pts) == 0:
            return None

        head_center = np.mean(head_pts, axis=0)
        feet_center = np.mean(feet_pts, axis=0)
        body_axis   = head_center - feet_center
        body_len    = np.linalg.norm(body_axis)
        if body_len < 0.1:
            return None
        body_axis_n = body_axis / body_len

        # --- Direzione laterale (perpendicolare nel piano XZ della camera optical) ---
        # In camera_color_optical_frame: X=destra, Y=giù, Z=profondità
        # Il piano del pavimento è approssimativamente XZ → normale = Y
        # Perpendicolare all'asse corpo nel piano XZ:
        up_cam = np.array([0.0, -1.0, 0.0])   # su nel frame ottico = -Y
        lateral = np.cross(body_axis_n, up_cam)
        lat_norm = np.linalg.norm(lateral)
        if lat_norm < 1e-6:
            # Corpo verticale rispetto alla camera (raro per paziente sdraiato)
            lateral = np.array([1.0, 0.0, 0.0])
        else:
            lateral = lateral / lat_norm

        # --- Bbox half nel piano laterale ---
        if self._bbox_size is not None:
            # Proietta le dimensioni bbox sulla direzione laterale
            bbox_half = float(np.abs(np.dot(self._bbox_size * 0.5, np.abs(lateral))))
            bbox_half = max(bbox_half, 0.3)   # minimo 30cm
        else:
            bbox_half = 0.4   # fallback

        dist = bbox_half + self.approach_margin + self.spot_front_offset

        # --- Scelta del lato ---
        candidate_a = torso_center + lateral * dist   # lato +lateral
        candidate_b = torso_center - lateral * dist   # lato -lateral

        if self.preferred_side == 'auto':
            # Scegli il lato più vicino a Spot (approssimazione: Z minore = più vicino in camera)
            if candidate_a[2] < candidate_b[2]:
                approach_pos = candidate_a
            else:
                approach_pos = candidate_b
        elif self.preferred_side == 'left':
            approach_pos = candidate_a
        else:
            approach_pos = candidate_b

        # --- Orientamento: Spot guarda verso torso_center ---
        dx = torso_center[0] - approach_pos[0]
        dz = torso_center[2] - approach_pos[2]
        yaw = math.atan2(dx, dz)   # in camera frame XZ

        # --- Pubblica marker RViz (sempre, anche in dry_run) ---
        src_frame = self._bbox_frame if self._bbox_frame else 'camera_color_optical_frame'
        self._publish_geometry_markers(
            torso_center, feet_center, head_center,
            approach_pos, body_axis_n, lateral,
            src_frame
        )

        # --- Costruisci PoseStamped nel frame sorgente ---
        pose_cam = PoseStamped()
        pose_cam.header.stamp    = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = src_frame
        pose_cam.pose.position.x = float(approach_pos[0])
        pose_cam.pose.position.y = float(approach_pos[1])
        pose_cam.pose.position.z = float(approach_pos[2])
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose_cam.pose.orientation.z = float(qz)
        pose_cam.pose.orientation.w = float(qw)

        # --- Trasforma in goal_frame se TF disponibile ---
        if not self.dry_run:
            try:
                pose_world = self.tf_buffer.transform(
                    pose_cam, self.goal_frame,
                    timeout=Duration(seconds=0.5)
                )
                pose_world.pose.position.z = 0.0   # proietta sul piano (Spot è mobile base)
                self.tf_available = True
                return pose_world
            except TransformException as e:
                self.get_logger().warn(
                    f'TF {src_frame}→{self.goal_frame} fallita: {e} — uso camera frame',
                    throttle_duration_sec=5.0
                )

        # dry_run o TF fallita → ritorna in camera frame (solo per visualizzazione)
        return pose_cam

    # ============================================================
    # TRAJECTORY ACTION
    # ============================================================

    def _send_trajectory(self, goal_pose: PoseStamped):
        if self._traj_client is None:
            return
        try:
            from spot_msgs.action import Trajectory
            goal_msg = Trajectory.Goal()
            goal_msg.target_pose         = goal_pose
            goal_msg.duration.sec        = int(self.nav_timeout)
            goal_msg.precise_positioning = False

            if not self._traj_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error('Trajectory action server non disponibile')
                self._set_state(IDLE)
                return

            send_future = self._traj_client.send_goal_async(
                goal_msg,
                feedback_callback=self._cb_traj_feedback
            )
            send_future.add_done_callback(self._cb_traj_goal_response)
        except Exception as e:
            self.get_logger().error(f'Errore invio Trajectory: {e}')
            self._set_state(IDLE)

    def _cb_traj_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal RIFIUTATO da Spot')
            self._set_state(IDLE)
            return
        self._traj_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._cb_traj_result)

    def _cb_traj_feedback(self, feedback_msg):
        self.get_logger().info(
            f'Spot navigating: {feedback_msg.feedback.feedback}',
            throttle_duration_sec=2.0
        )

    def _cb_traj_result(self, future):
        result = future.result().result
        if self._state == RETURNING:
            self._returning_sent = False
            if result.success:
                self.get_logger().info('✅ Spot tornato alla posizione iniziale')
            else:
                self.get_logger().warn(f'Return trajectory fallita: {result.message}')
            self._set_state(IDLE)
        else:
            if result.success:
                self.get_logger().info('✅ Spot ARRIVATO al goal')
                self._set_state(ARRIVED)
            else:
                self.get_logger().warn(f'Trajectory fallita: {result.message} → IDLE')
                self._set_state(IDLE)

    def _cancel_trajectory(self):
        if self._traj_goal_handle is not None:
            self._traj_goal_handle.cancel_goal_async()
            self._traj_goal_handle = None

    # ============================================================
    # VISUALIZZAZIONE RVIZ
    # ============================================================

    def _publish_geometry_markers(self, torso_center, feet, head,
                                   approach_pos, body_axis_n, lateral,
                                   frame_id):
        stamp = self.get_clock().now().to_msg()

        # 1) Asse corpo: freccia da piedi a testa
        m = self._arrow_marker(
            frame_id, stamp, 'body_axis', 0,
            feet, head,
            r=0.0, g=0.8, b=1.0, a=0.9
        )
        self.pub_body_axis.publish(m)

        # 2) Direzione laterale: freccia da torso_center ad approach_pos
        m = self._arrow_marker(
            frame_id, stamp, 'lateral_dir', 1,
            torso_center, approach_pos,
            r=1.0, g=0.8, b=0.0, a=0.9
        )
        self.pub_lateral_dir.publish(m)

        # 3) Goal Spot: freccia grande nella direzione in cui guarderà
        dx = torso_center[0] - approach_pos[0]
        dz = torso_center[2] - approach_pos[2]
        look_len = 0.5
        norm = math.sqrt(dx**2 + dz**2) + 1e-9
        goal_tip = approach_pos + np.array([dx / norm * look_len, 0.0, dz / norm * look_len])
        m = self._arrow_marker(
            frame_id, stamp, 'approach_goal', 2,
            approach_pos, goal_tip,
            r=0.0, g=1.0, b=0.0, a=1.0,
            shaft_d=0.06, head_d=0.12
        )
        self.pub_approach_goal.publish(m)

        # 4) Safe zone: bbox + approach_margin (cubo semitrasparente rosso)
        if self._bbox_center is not None and self._bbox_size is not None:
            mz = Marker()
            mz.header.frame_id = frame_id
            mz.header.stamp    = stamp
            mz.ns     = 'safe_zone'
            mz.id     = 3
            mz.type   = Marker.CUBE
            mz.action = Marker.ADD
            mz.pose.position.x = float(self._bbox_center[0])
            mz.pose.position.y = float(self._bbox_center[1])
            mz.pose.position.z = float(self._bbox_center[2])
            mz.pose.orientation.w = 1.0
            mz.scale.x = float(self._bbox_size[0]) + 2 * self.approach_margin
            mz.scale.y = float(self._bbox_size[1]) + 2 * self.approach_margin
            mz.scale.z = float(self._bbox_size[2]) + 2 * self.approach_margin
            mz.color.r = 1.0
            mz.color.g = 0.2
            mz.color.b = 0.2
            mz.color.a = 0.15
            self.pub_safe_zone.publish(mz)

    def _publish_fsm_label(self):
        if self._bbox_center is None:
            return
        frame_id = self._bbox_frame if self._bbox_frame else 'camera_color_optical_frame'
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'fsm_state'
        m.id     = 10
        m.type   = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(self._bbox_center[0])
        m.pose.position.y = float(self._bbox_center[1]) - 0.6   # sopra in camera (-Y = su)
        m.pose.position.z = float(self._bbox_center[2])
        m.pose.orientation.w = 1.0
        m.scale.z  = 0.12
        m.color.r  = 1.0
        m.color.g  = 1.0
        m.color.b  = 1.0
        m.color.a  = 1.0
        dry_tag = ' [DRY]' if self.dry_run else ''
        m.text = f'TERESA: {self._state}{dry_tag}'
        self.pub_fsm_state.publish(m)

    @staticmethod
    def _arrow_marker(frame_id, stamp, ns, mid,
                      p0, p1, r, g, b, a,
                      shaft_d=0.03, head_d=0.06):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp    = stamp
        m.ns     = ns
        m.id     = mid
        m.type   = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = shaft_d
        m.scale.y = head_d
        m.scale.z = head_d
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a
        m.pose.orientation.w = 1.0
        m.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])))
        m.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
        return m


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = TeresaMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
