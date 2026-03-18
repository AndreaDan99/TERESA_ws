#!/usr/bin/env python3
"""
Impedance Controller for Unitree Z1 using Pinocchio
Full Cartesian Control: Position + Orientation (Cartesian space via Jacobian transpose)
- Orientamento latched al primo tick APPROACH: mantiene EE perpendicolare alla superficie
  anche quando spalla/gomito si muovono (controllo joint-space non garantisce R_ee fisso)
- Safe startup 3s: blocca posizione E orientamento via log3(R_err)
"""

import sys
import time
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray, Float32, Bool
from tf_transformations import quaternion_matrix

import signal


class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller_realsense')

        signal.signal(signal.SIGINT, self.shutdown_handler)

        # Parametri del controller
        self.declare_parameters(
            namespace='',
            parameters=[
                ('urdf_path', ''),
                ('end_effector_frame', 'link06'),
                ('control_rate', 500.0),
                ('log_rate', 2.0),

                # Controllo posizione (cartesiano)
                ('K_p_translation', [250.0, 250.0, 300.0]),
                ('K_d_translation', [15.0, 15.0, 15.0]),
                ('K_i_translation', [0.0, 0.0, 0.0]),

                # Rotazioni cartesiane — controllo orientamento EE via Jacobian transpose
                # K_p_rotation: rigidezza [Nm/rad], K_d_rotation: smorzamento [Nm·s/rad]
                # Latched al primo tick APPROACH: mantiene EE perpendicolare alla superficie
                ('K_p_rotation', [10.0, 10.0, 10.0]),
                ('K_d_rotation', [1.0,  1.0,  1.0]),
                ('K_i_rotation', [0.0,  0.0,  0.0]),

                # Parametri generali
                ('integral_limit', 0.05),
                ('torque_limit', 70.0),
                ('safe_startup_duration', 3.0),
                ('max_step_distance', 0.40),

                # Compensazione dinamica
                ('gravity_scale_factor_j2', 1.3),

                # Integrazione RealSense/superficie
                ('approach_mode',          'normal'), # "normal" | "vertical"
                ('desired_normal_offset', -0.005),   # [m] offset fisso lungo normale
                ('max_approach_distance', 0.20),      # [m] 20 cm di avanzamento
                ('approach_speed', 0.01),             # [m/s]
                ('retract_speed',  0.05),             # [m/s] velocità ritorno (default più veloce)

                # Tempo di hold al contatto
                ('hold_time', 10.0),              # [s]

                # Topic interface FSM
                ('impedance_enable_topic', '/impedance_enable'),
                ('impedance_done_topic',   '/impedance_done'),

                # Robustezza: singolarità e limiti articolari
                ('manip_threshold',         0.05),   # sotto: scala F → 0 (rolloff quadratico)
                ('joint_limit_margin_frac', 0.10),   # margine dai limiti (10% del range)
                ('joint_limit_k',          20.0),    # rigidezza repulsione limiti [Nm/rad]
            ]
        )

        # Carica parametri
        urdf_path             = self.get_parameter('urdf_path').value
        self.ee_frame_name    = self.get_parameter('end_effector_frame').value
        control_rate          = self.get_parameter('control_rate').value
        self.log_rate         = self.get_parameter('log_rate').value
        self.torque_limit     = self.get_parameter('torque_limit').value
        self.safe_startup_duration = self.get_parameter('safe_startup_duration').value
        self.max_step_distance = self.get_parameter('max_step_distance').value
        self.gravity_scale_j2 = self.get_parameter('gravity_scale_factor_j2').value

        self.approach_mode          = self.get_parameter('approach_mode').value
        self.desired_normal_offset  = self.get_parameter('desired_normal_offset').value
        self.max_approach_distance  = self.get_parameter('max_approach_distance').value
        self.approach_speed         = self.get_parameter('approach_speed').value
        self.retract_speed          = self.get_parameter('retract_speed').value
        self.hold_time              = float(self.get_parameter('hold_time').value)

        self.manip_threshold = float(self.get_parameter('manip_threshold').value)
        self.jl_margin_frac  = float(self.get_parameter('joint_limit_margin_frac').value)
        self.jl_k            = float(self.get_parameter('joint_limit_k').value)

        impedance_enable_topic = self.get_parameter('impedance_enable_topic').value
        impedance_done_topic   = self.get_parameter('impedance_done_topic').value

        # Carica modello Pinocchio
        self.get_logger().info(f'Caricamento URDF da: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()

        if self.model.existFrame(self.ee_frame_name):
            self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
            self.get_logger().info(f'Frame EE: {self.ee_frame_name} (id: {self.ee_frame_id})')
        else:
            raise ValueError(f'Frame {self.ee_frame_name} non esiste')

        # Matrici PID cartesiane (6x6)
        K_p_trans = np.array(self.get_parameter('K_p_translation').value)
        K_p_rot   = np.array(self.get_parameter('K_p_rotation').value)
        self.K_p  = np.diag(np.concatenate([K_p_trans, K_p_rot]))

        K_d_trans = np.array(self.get_parameter('K_d_translation').value)
        K_d_rot   = np.array(self.get_parameter('K_d_rotation').value)
        self.K_d  = np.diag(np.concatenate([K_d_trans, K_d_rot]))

        K_i_trans = np.array(self.get_parameter('K_i_translation').value)
        K_i_rot   = np.array(self.get_parameter('K_i_rotation').value)
        self.K_i  = np.diag(np.concatenate([K_i_trans, K_i_rot]))

        self.integral_limit = self.get_parameter('integral_limit').value

        # Guadagni polso (joint-space)

        # Stato robot
        self.n_joints         = 6
        self.q                = np.zeros(self.model.nq)
        self.dq               = np.zeros(self.model.nv)
        self.state_received   = False

        # Posa desiderata (cartesiana)
        self.x_desired             = None
        self.x_desired_initialized = False

        # Rotazione EE desiderata — latched al primo tick di APPROACH
        # Usata per controllo orientamento Cartesiano (non joint-space)
        self._R_latched: np.ndarray | None = None

        # Controllo PID
        self.control_dt       = 1.0 / control_rate
        self.error_integral   = np.zeros(6)
        self.scale_j2_filtered = 1.0

        # Safe startup
        self.safe_startup_mode    = True
        self.safe_startup_counter = 0

        # ── Interfaccia FSM ────────────────────────────────────────
        # Fasi interne: IDLE → APPROACH → HOLD → RETRACT → DONE
        self.impedance_enabled       = False
        self._phase                  = 'IDLE'   # fase interna
        self.approach_distance_accum = 0.0
        self._hold_start_time        = None     # rclpy.Time quando inizia HOLD
        self._done_published         = False
        self._R_latched              = None   # rotazione EE latched alla transizione APPROACH→HOLD

        # Latch superficie: congelata al primo tick di APPROACH
        # evita salti del target quando la persona si muove durante l'avanzamento
        self._surface_latched  = False
        self._p_surf_latched   = None   # np.array [3]
        self._normal_latched   = None   # np.array [3]
        # Direzione di approccio latched (= -normal, verso il torso)
        # usata per il feedforward di velocità in APPROACH/RETRACT
        self._approach_dir     = None   # np.array [3], verso torso

        # Stato superficie (RealSense)
        self.surface_frame = None
        self.surface_signed_distance = 0.0

        # Statistiche
        self.iteration_count  = 0
        self.error_norm_pos   = 0.0
        self.error_norm_wrist = 0.0
        self.vel_norm         = 0.0
        self.force_norm       = 0.0
        self.max_torque       = 0.0
        self.sum_error_pos    = 0.0
        self.max_error_pos    = 0.0
        self.manipulability   = 0.0

        # ── Subscribers ───────────────────────────────────────────
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.sub_enable = self.create_subscription(
            Bool, impedance_enable_topic, self.enable_callback, 10
        )
        self.impedance_params_sub = self.create_subscription(
            Float64MultiArray, '/set_impedance', self.impedance_callback, 10
        )
        self.surface_sub = self.create_subscription(
            PoseStamped, '/torso_surface_frame', self.surface_callback, 10
        )
        self.surface_dist_sub = self.create_subscription(
            Float32, '/surface_signed_distance', self.surface_dist_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────
        self.torque_pub       = self.create_publisher(Float64MultiArray, '/torque_controller/commands', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped,       '/current_ee_pose',            10)
        self.wrench_pub       = self.create_publisher(WrenchStamped,     '/cartesian_wrench',           10)
        self.pub_done         = self.create_publisher(Bool,              impedance_done_topic,          10)

        # ── Timers ────────────────────────────────────────────────
        self.timer     = self.create_timer(self.control_dt,          self.control_loop)
        self.log_timer = self.create_timer(1.0 / self.log_rate,      self.print_status)

        self.get_logger().info('='*70)
        self.get_logger().info('IMPEDANCE CONTROLLER INIZIALIZZATO')
        self.get_logger().info(f'Control: {control_rate} Hz | Log: {self.log_rate} Hz')
        self.get_logger().info(f'K_p_translation: {K_p_trans}')
        self.get_logger().info(f'K_d_translation: {K_d_trans}')
        K_p_rot = np.array(self.get_parameter('K_p_rotation').value)
        self.get_logger().info(f'K_p_rotation (Cartesian): {K_p_rot}')
        self.get_logger().info(f'Gravity scale J2: {self.gravity_scale_j2}')
        self.get_logger().info(f'Torque limit: {self.torque_limit} Nm')
        self.get_logger().info(f'Safe startup: {self.safe_startup_duration}s')
        self.get_logger().info(f'Approach mode: {self.approach_mode}')
        self.get_logger().info(f'Max approach: {self.max_approach_distance*100:.0f} cm')
        self.get_logger().info(f'Enable topic: {impedance_enable_topic}')
        self.get_logger().info(f'Done topic:   {impedance_done_topic}')
        self.get_logger().info('='*70)

    # ──────────────────────────────────────────────────────────────
    def shutdown_handler(self, signum, frame):
        self.get_logger().info('🛑 Shutdown richiesto...')
        zero_torque = Float64MultiArray()
        zero_torque.data = [0.0] * 6
        for _ in range(20):
            self.torque_pub.publish(zero_torque)
            time.sleep(0.005)
        self.get_logger().info('✅ Coppia azzerata!')
        sys.exit(0)

    # ──────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────
    def joint_state_callback(self, msg):
        if len(msg.position) < self.n_joints:
            return
        self.q[:self.n_joints]  = np.array(msg.position[:self.n_joints])
        self.dq[:self.n_joints] = (
            np.array(msg.velocity[:self.n_joints])
            if len(msg.velocity) >= self.n_joints
            else np.zeros(self.n_joints)
        )
        self.state_received = True

    def enable_callback(self, msg: Bool):
        prev = self.impedance_enabled
        self.impedance_enabled = bool(msg.data)

        if not self.impedance_enabled and prev:
            # Disabilitato: reset completo
            self._phase                  = 'IDLE'
            self.approach_distance_accum = 0.0
            self._hold_start_time        = None
            self._done_published         = False
            self._R_latched              = None
            self._surface_latched        = False
            self._p_surf_latched         = None
            self._normal_latched         = None
            self._approach_dir           = None
            self.x_desired_initialized   = False
            self.get_logger().info('🛑 Impedance disabled — reset')

        if self.impedance_enabled and not prev:
            self._phase                  = 'APPROACH'
            self.approach_distance_accum = 0.0
            self._hold_start_time        = None
            self._done_published         = False
            self._R_latched              = None   # verrà latched alla transizione APPROACH→HOLD
            self._surface_latched        = False   # verrà latched al primo tick valido
            self._p_surf_latched         = None
            self._normal_latched         = None
            self._approach_dir           = None
            self.get_logger().info('✅ Impedance enabled — fase APPROACH')

    def impedance_callback(self, msg):
        if len(msg.data) == 12:
            self.K_p = np.diag(msg.data[:6])
            self.K_d = np.diag(msg.data[6:])

    def surface_callback(self, msg: PoseStamped):
        self.surface_frame = msg

    def surface_dist_callback(self, msg: Float32):
        self.surface_signed_distance = msg.data

    # ──────────────────────────────────────────────────────────────
    # Control loop
    # ──────────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.state_received:
            return

        pin.forwardKinematics(self.model, self.data, self.q, self.dq)
        pin.updateFramePlacements(self.model, self.data)

        # ========== SAFE STARTUP MODE ==========
        if self.safe_startup_mode:
            self.safe_startup_counter += 1
            elapsed = self.safe_startup_counter * self.control_dt

            if elapsed < self.safe_startup_duration:
                if self.safe_startup_counter == 1:
                    x_current           = self.data.oMf[self.ee_frame_id]
                    self.x_startup      = x_current.copy()
                    self._R_startup     = x_current.rotation.copy()   # latch orientamento
                    pos = self.x_startup.translation
                    self.get_logger().info('📍 SAFE STARTUP: Posizione + orientamento bloccati a:')
                    self.get_logger().info(f'   [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

                x_current = self.data.oMf[self.ee_frame_id]
                x_error   = np.zeros(6)
                x_error[:3] = self.x_startup.translation - x_current.translation
                # Errore rotazionale: vettore asse-angolo dalla rotazione corrente
                # alla rotazione desiderata (frame mondo → LOCAL_WORLD_ALIGNED)
                R_err = self._R_startup @ x_current.rotation.T
                x_error[3:6] = pin.log3(R_err)

                J  = pin.computeFrameJacobian(
                    self.model, self.data, self.q, self.ee_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )
                dx = J @ self.dq

                K_p_s = np.diag([200.0, 200.0, 200.0, 20.0, 20.0, 20.0])
                K_d_s = np.diag([20.0,  20.0,  20.0,  2.0,  2.0,  2.0])

                F_cartesian  = K_p_s @ x_error - K_d_s @ dx
                tau_total    = (J.T @ F_cartesian)[:self.n_joints] + self.compute_compensation()
                tau_total    = np.clip(tau_total, -self.torque_limit, self.torque_limit)

                self.publish_torque(tau_total)
                self.iteration_count += 1
                return
            else:
                self.x_desired             = self.data.oMf[self.ee_frame_id].copy()
                self.x_desired_initialized = True
                self.safe_startup_mode     = False
                pos = self.x_desired.translation
                self.get_logger().info('='*70)
                self.get_logger().info('✅ SAFE STARTUP COMPLETATO')
                self.get_logger().info(f'   Target: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')
                self.get_logger().info('='*70)

        # ========== CONTROLLO NORMALE ==========
        x_current = self.data.oMf[self.ee_frame_id]
        self.publish_current_pose(x_current)

        # ── Se non abilitato: tieni posizione corrente, azzera stato ──
        if not self.impedance_enabled:
            self.x_desired             = x_current.copy()
            self.approach_distance_accum = 0.0
            self._done_published         = False
            # Mantieni ancora la coppia per non cadere
            self._run_impedance(x_current)
            return

        # ── Inizializza x_desired se necessario ───────────────────────
        if not self.x_desired_initialized:
            self.x_desired             = x_current.copy()
            self.x_desired_initialized = True

        # ── Safety: serve la superficie ───────────────────────────────
        if self.surface_frame is None:
            self.get_logger().warn(
                '⚠️ /torso_surface_frame non ancora disponibile',
                throttle_duration_sec=2.0
            )
            self._run_impedance(x_current)
            return

        # ── Calcola normale superficie (comune a tutte le fasi) ───────
        surf_pose = self.surface_frame.pose
        T_surf    = quaternion_matrix([
            surf_pose.orientation.x, surf_pose.orientation.y,
            surf_pose.orientation.z, surf_pose.orientation.w,
        ])
        R_surf = T_surf[:3, :3]
        p_surf = np.array([surf_pose.position.x, surf_pose.position.y, surf_pose.position.z])
        normal = R_surf[:, 2]  # asse Z del frame superficie = normale

        # ── Latch superficie al primo tick di APPROACH ────────────────
        # Congela p_surf e normal per tutta la sequenza approach/hold/retract
        # evitando salti del target se la persona si sposta durante l'avanzamento.
        #
        # FORMULA target: p_surf + (desired_normal_offset − accum) * normal
        #   desired_normal_offset = standoff JTC (es. 0.200 m)
        #   accum = 0       → target = standoff (= EE al momento del latch, zero errore)
        #   accum = max     → target = standoff − max (es. 5 mm dentro la superficie)
        #
        # accum_init = clip(desired_normal_offset − proj, 0, max)
        #   se EE è esattamente al standoff: accum_init = 0 → zero errore iniziale ✓
        #   se EE è più vicino (imprecisione JTC): accum_init > 0 ma comunque senza jerk
        if self._phase == 'APPROACH' and not self._surface_latched:
            self._p_surf_latched  = p_surf.copy()
            self._surface_latched = True

            if self.approach_mode == 'vertical':
                # Approccio verticale: ignora la normale reale, scende lungo -Z world.
                # La normale "efficace" è [0,0,1] (world Z up, dal torso verso robot),
                # così target = [p_surf.x, p_surf.y, p_surf.z + (0.2-accum)] → scende dritto.
                self._normal_latched = np.array([0.0, 0.0, 1.0])
                self.get_logger().info('🔒 Superficie latched — modalità VERTICAL ↓ (normal=[0,0,1])')
            else:
                self._normal_latched = normal.copy()
                self.get_logger().info(
                    f'🔒 Superficie latched — modalità NORMAL '
                    f'n=[{normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f}]'
                )

            # Direzione di approccio: -normal_eff = verso il torso
            # Usata come feedforward di velocità per seguire la direzione corretta.
            self._approach_dir = -self._normal_latched.copy()   # verso il torso

            # _R_latched NON viene latched qui: durante APPROACH si usa solo
            # damping angolare per non interferire con la direzione della normale.
            # Il latch avviene alla transizione APPROACH→HOLD (vedi sotto).

            # Proiezione EE sulla normale effettiva (reale o [0,0,1] in modalità vertical)
            # per calcolare l'accum iniziale senza jerk.
            ee_pos = x_current.translation
            proj   = float(np.dot(ee_pos - p_surf, self._normal_latched))
            self.approach_distance_accum = float(np.clip(
                self.desired_normal_offset - proj,
                0.0,
                self.max_approach_distance
            ))
            self.get_logger().info(
                f'  p=[{p_surf[0]:.3f},{p_surf[1]:.3f},{p_surf[2]:.3f}] '
                f'proj={proj*100:.1f}cm accum_init={self.approach_distance_accum*100:.1f}cm'
            )

        # Usa valori latched se disponibili, altrimenti quelli correnti
        if self._surface_latched:
            p_surf = self._p_surf_latched
            normal = self._normal_latched

        step = self.approach_speed * self.control_dt

        # ── Fase APPROACH: avanza fino a max_approach_distance ────────
        if self._phase == 'APPROACH':
            self.approach_distance_accum = min(
                self.approach_distance_accum + step,
                self.max_approach_distance
            )
            if self.approach_distance_accum >= self.max_approach_distance:
                self._phase           = 'HOLD'
                self._hold_start_time = self.get_clock().now()
                # Latch orientamento EE al termine dell'APPROACH:
                # si mantiene la rotazione reale raggiunta (non quella iniziale)
                # così il controllo orientamento non deve "combattere" la traiettoria.
                self._R_latched = x_current.rotation.copy()
                self.get_logger().info(
                    f'📍 APPROACH completato ({self.max_approach_distance*100:.0f} cm) → HOLD {self.hold_time:.0f}s'
                    f' | Rot latched: X=[{x_current.rotation[0,0]:.2f},{x_current.rotation[1,0]:.2f},{x_current.rotation[2,0]:.2f}]'
                )

        # ── Fase HOLD: rimani fermo per hold_time secondi ─────────────
        elif self._phase == 'HOLD':
            elapsed = (self.get_clock().now() - self._hold_start_time).nanoseconds * 1e-9
            if elapsed >= self.hold_time:
                # Snap accum a desired_normal_offset: target parte esattamente
                # dalla superficie (non da dentro), così il primo tick di RETRACT
                # ha forza = 0 e cresce immediatamente nella direzione di uscita.
                self.approach_distance_accum = self.desired_normal_offset
                self._phase = 'RETRACT'
                self.get_logger().info(
                    f'↩️  HOLD completato ({elapsed:.1f}s) → RETRACT '
                    f'(accum snap → {self.desired_normal_offset*100:.1f}cm)'
                )

        # ── Fase RETRACT: torna al standoff lungo la stessa normale ───
        elif self._phase == 'RETRACT':
            retract_step = self.retract_speed * self.control_dt
            self.approach_distance_accum = max(
                self.approach_distance_accum - retract_step,
                0.0
            )
            if self.approach_distance_accum <= 0.0:
                self._phase = 'DONE'
                self.get_logger().info('✅ RETRACT completato → DONE')

        # ── Fase DONE: pubblica done (una volta) e attendi disable ────
        elif self._phase == 'DONE':
            if not self._done_published:
                self.pub_done.publish(Bool(data=True))
                self._done_published = True
                self.get_logger().info('🏁 /impedance_done pubblicato')

        # ── Aggiorna target in base alla distanza accumulata ──────────
        # target = p_surf + (offset − accum) * normal
        #   accum=0   → target al standoff (lontano dal torso)
        #   accum=max → target al standoff−max (lieve contatto nella superficie)
        target_pos     = p_surf + (self.desired_normal_offset - self.approach_distance_accum) * normal
        self.x_desired = pin.SE3(R_surf, target_pos)

        self._run_impedance(x_current)

    # ──────────────────────────────────────────────────────────────
    def _run_impedance(self, x_current):
        """Calcola e pubblica la coppia impedenza con compensazione dinamica."""
        if not self.x_desired_initialized or self.x_desired is None:
            return

        x_error = np.zeros(6)
        x_error[:3] = self.x_desired.translation - x_current.translation

        J  = pin.computeFrameJacobian(
            self.model, self.data, self.q, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        dx = J @ self.dq

        # Anti-windup
        error_norm = np.linalg.norm(x_error[:3])
        if error_norm > 0.020:
            self.error_integral *= 0.90
        else:
            self.error_integral += x_error * self.control_dt
            self.error_integral  = np.clip(self.error_integral, -self.integral_limit, self.integral_limit)

        # Scaling adattivo Kp/Kd in funzione dell'estensione
        reach_xy   = np.linalg.norm(x_current.translation[:2])
        reach_norm = np.clip(reach_xy / 0.6, 0.0, 1.0)
        scale_kp   = 1.0 - 0.65 * reach_norm
        scale_kd   = 1.0 + 1.00 * reach_norm

        K_p_eff = self.K_p * scale_kp
        K_d_eff = self.K_d * scale_kd
        F_max   = 80.0

        # ── Velocity feedforward per APPROACH e RETRACT ───────────────
        # Senza feedforward il controller dipende solo da K_p * errore_posizione,
        # che è insufficiente per seguire una normale inclinata (componente Z vs gravità).
        # Con feedforward: K_d smorzua l'errore di velocità (dx - v_des) invece di dx,
        # così il braccio segue la direzione della normale anche in diagonale.
        v_ff = np.zeros(6)   # [m/s, m/s, m/s, rad/s, rad/s, rad/s]
        if self._phase == 'APPROACH' and self._approach_dir is not None:
            v_ff[:3] = self.approach_speed * self._approach_dir   # verso torso
        elif self._phase == 'RETRACT' and self._approach_dir is not None:
            v_ff[:3] = -self.retract_speed * self._approach_dir   # via dal torso

        F_cartesian    = K_p_eff @ x_error - K_d_eff @ (dx - v_ff) + self.K_i @ self.error_integral

        # ── 1. Monitor manipolabilità + damping vicino a singolarità ──────
        # w = sqrt(det(J_pos · J_pos^T)): → 0 vicino a singolarità
        J_pos = J[:3, :]
        manip = float(np.sqrt(max(0.0, np.linalg.det(J_pos @ J_pos.T))))
        self.manipulability = manip
        if manip < self.manip_threshold:
            # Rolloff quadratico: forza → 0 man mano che ci si avvicina alla singolarità
            manip_scale = (manip / self.manip_threshold) ** 2
            F_cartesian[:3] *= manip_scale
            if manip < self.manip_threshold * 0.5:
                self.get_logger().warn(
                    f'⚠️  Singolarità vicina! manip={manip:.4f} '
                    f'(soglia={self.manip_threshold:.3f}) → F scalata a {manip_scale*100:.0f}%',
                    throttle_duration_sec=1.0,
                )

        F_cartesian[:3] = np.clip(F_cartesian[:3], -F_max, F_max)

        # ── Controllo orientamento Cartesiano ───────────────────────────
        # Durante APPROACH si usa SOLO damping angolare (da K_p_eff @ x_error già
        # calcolato sopra con x_error[3:6]=0): questo evita che le coppie correttive
        # dell'orientamento interferiscano con il moto lungo la normale superficiale.
        #
        # Durante HOLD / RETRACT / DONE il latch _R_latched (acquisito alla fine
        # dell'APPROACH) garantisce che l'EE mantenga la perpendicolarità al torso
        # contro le forze di contatto.
        if self._phase in ('HOLD', 'RETRACT', 'DONE') and self._R_latched is not None:
            R_err   = self._R_latched @ x_current.rotation.T   # frame mondo
            e_omega = pin.log3(R_err)                           # asse-angolo 3D
            K_p_rot = np.diag(self.K_p)[3:6]
            K_d_rot = np.diag(self.K_d)[3:6]
            F_rot   = K_p_rot * e_omega - K_d_rot * dx[3:6]
            F_max_rot = 15.0                                     # [Nm] limite sicurezza
            F_cartesian[3:6] = np.clip(F_rot, -F_max_rot, F_max_rot)
        # else (APPROACH o _R_latched non ancora disponibile):
        #   F_cartesian[3:6] rimane = -K_d_eff[3:6,3:6] @ dx[3:6]  (solo smorzamento)

        tau_impedance = (J.T @ F_cartesian)[:self.n_joints]

        # ── 2. Repulsione soft dai limiti articolari ───────────────────────
        # Aggiunge coppia che respinge il giunto dal limite quando entra
        # nel margine (jl_margin_frac * range). Proporzionale alla penetrazione.
        q_lo   = self.model.lowerPositionLimit[:self.n_joints]
        q_hi   = self.model.upperPositionLimit[:self.n_joints]
        margin = self.jl_margin_frac * (q_hi - q_lo)
        q_cur  = self.q[:self.n_joints]
        tau_jl = np.zeros(self.n_joints)
        mask_hi = q_cur > (q_hi - margin)
        mask_lo = q_cur < (q_lo + margin)
        tau_jl[mask_hi] = -self.jl_k * (q_cur[mask_hi] - (q_hi - margin)[mask_hi])
        tau_jl[mask_lo] =  self.jl_k * ((q_lo + margin)[mask_lo] - q_cur[mask_lo])
        if np.any(mask_hi | mask_lo):
            active = np.where(mask_hi | mask_lo)[0]
            self.get_logger().warn(
                f'⚠️  Limite giunto attivo: J{active + 1} | '
                f'τ_jl={np.round(tau_jl[active], 2)}',
                throttle_duration_sec=1.0,
            )
        tau_impedance += tau_jl

        tau_total = tau_impedance + self.compute_compensation()
        tau_total = np.clip(tau_total, -self.torque_limit, self.torque_limit)

        self.publish_torque(tau_total)
        self.publish_wrench(F_cartesian)

        # Statistiche
        self.iteration_count += 1
        self.error_norm_pos   = float(np.linalg.norm(x_error[:3]))
        self.error_norm_wrist = float(np.rad2deg(np.linalg.norm(x_error[3:6])))   # [deg]
        self.vel_norm         = float(np.linalg.norm(dx))
        self.force_norm       = float(np.linalg.norm(F_cartesian))
        self.max_torque       = float(np.max(np.abs(tau_total)))
        self.sum_error_pos   += self.error_norm_pos
        self.max_error_pos    = max(self.max_error_pos, self.error_norm_pos)

    # ──────────────────────────────────────────────────────────────
    def compute_compensation(self):
        tau_gravity  = pin.computeGeneralizedGravity(self.model, self.data, self.q)
        aq_zero      = np.zeros(self.model.nv)
        tau_rnea     = pin.rnea(self.model, self.data, self.q, self.dq, aq_zero)
        tau_coriolis = tau_rnea - tau_gravity
        tau_comp     = tau_gravity + tau_coriolis

        # Scale adattivo J2 in funzione dell'estensione XY
        ee_pos       = self.data.oMf[self.ee_frame_id].translation
        reach_xy     = np.linalg.norm(ee_pos[:2])
        reach_norm   = np.clip(reach_xy / 0.3, 0.0, 1.0)
        scale_j2_raw = self.gravity_scale_j2 - (self.gravity_scale_j2 - 1.0) * reach_norm

        alpha                  = 0.02
        self.scale_j2_filtered = alpha * scale_j2_raw + (1.0 - alpha) * self.scale_j2_filtered
        tau_comp[1]            = tau_gravity[1] * self.scale_j2_filtered + tau_coriolis[1]

        return tau_comp[:self.n_joints]

    # ──────────────────────────────────────────────────────────────
    def publish_torque(self, tau):
        msg      = Float64MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)

    def publish_current_pose(self, x_current):
        msg                  = PoseStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'world'
        msg.pose.position.x  = x_current.translation[0]
        msg.pose.position.y  = x_current.translation[1]
        msg.pose.position.z  = x_current.translation[2]
        quat                 = pin.Quaternion(x_current.rotation)
        msg.pose.orientation.w = quat.w
        msg.pose.orientation.x = quat.x
        msg.pose.orientation.y = quat.y
        msg.pose.orientation.z = quat.z
        self.current_pose_pub.publish(msg)

    def publish_wrench(self, F_cartesian):
        msg                  = WrenchStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = self.ee_frame_name
        msg.wrench.force.x   = F_cartesian[0]
        msg.wrench.force.y   = F_cartesian[1]
        msg.wrench.force.z   = F_cartesian[2]
        msg.wrench.torque.x  = F_cartesian[3]
        msg.wrench.torque.y  = F_cartesian[4]
        msg.wrench.torque.z  = F_cartesian[5]
        self.wrench_pub.publish(msg)

    def print_status(self):
        if self.iteration_count == 0:
            return
        avg_pos = self.sum_error_pos / self.iteration_count
        phase_str = self._phase if self.impedance_enabled else 'off'
        log_msg = (
            f'[{phase_str:8s}] '
            f'Iter: {self.iteration_count:6d} | '
            f'Err_pos: {self.error_norm_pos*1000:6.2f}mm (avg: {avg_pos*1000:6.2f}mm) | '
            f'dist: {self.approach_distance_accum*100:5.1f}/{self.max_approach_distance*100:.0f}cm | '
            f'Vel: {self.vel_norm:5.3f} | F: {self.force_norm:6.1f}N | τ: {self.max_torque:5.2f}Nm | '
            f'manip: {self.manipulability:.4f}'
        )
        if self._R_latched is not None:
            log_msg += f' | Rot_err: {self.error_norm_wrist:5.2f}°'
        self.get_logger().info(log_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ImpedanceController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if controller.iteration_count > 0:
            avg_pos = controller.sum_error_pos / controller.iteration_count
            controller.get_logger().info('='*70)
            controller.get_logger().info('STATISTICHE FINALI')
            controller.get_logger().info(f'Iterazioni: {controller.iteration_count}')
            controller.get_logger().info(
                f'Errore pos: {avg_pos*1000:.2f}mm (max: {controller.max_error_pos*1000:.2f}mm)'
            )
            controller.get_logger().info('='*70)
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
