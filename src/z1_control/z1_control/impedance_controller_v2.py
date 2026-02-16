#!/usr/bin/env python3
"""
Impedance Controller for Unitree Z1 using Pinocchio
Hybrid Control: Cartesian Position + Joint-Space Orientation
With Safe Startup Mode and Enhanced Error Handling
"""

import sys
import time
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
import signal


class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller_v2')

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
                
                # Rotazioni cartesiane (non usate, mantenute per compatibilità)
                ('K_p_rotation', [0.0, 0.0, 0.0]),
                ('K_d_rotation', [0.0, 0.0, 0.0]),
                ('K_i_rotation', [0.0, 0.0, 0.0]),
                
                # Controllo orientamento (joint-space su polso J4-J5-J6)
                ('control_wrist_orientation', True),
                ('K_p_wrist_joints', [40.0, 40.0, 40.0]),
                ('K_d_wrist_joints', [4.0, 4.0, 4.0]),
                
                # Parametri generali
                ('integral_limit', 0.05),
                ('torque_limit', 70.0),
                ('safe_startup_duration', 3.0),
                ('max_step_distance', 0.40),
                
                # Compensazione dinamica
                ('gravity_scale_factor_j2', 1.3),
            ]
        )
        
        # Carica parametri
        urdf_path = self.get_parameter('urdf_path').value
        self.ee_frame_name = self.get_parameter('end_effector_frame').value
        control_rate = self.get_parameter('control_rate').value
        self.log_rate = self.get_parameter('log_rate').value
        self.torque_limit = self.get_parameter('torque_limit').value
        self.safe_startup_duration = self.get_parameter('safe_startup_duration').value
        self.max_step_distance = self.get_parameter('max_step_distance').value
        self.gravity_scale_j2 = self.get_parameter('gravity_scale_factor_j2').value
        
        # Carica modello Pinocchio
        self.get_logger().info(f'Caricamento URDF da: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Trova frame end-effector
        if self.model.existFrame(self.ee_frame_name):
            self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
            self.get_logger().info(f'Frame EE: {self.ee_frame_name} (id: {self.ee_frame_id})')
        else:
            raise ValueError(f'Frame {self.ee_frame_name} non esiste')
        
        # Matrici PID cartesiane (6x6)
        K_p_trans = np.array(self.get_parameter('K_p_translation').value)
        K_p_rot = np.array(self.get_parameter('K_p_rotation').value)
        self.K_p = np.diag(np.concatenate([K_p_trans, K_p_rot]))
        
        K_d_trans = np.array(self.get_parameter('K_d_translation').value)
        K_d_rot = np.array(self.get_parameter('K_d_rotation').value)
        self.K_d = np.diag(np.concatenate([K_d_trans, K_d_rot]))
        
        K_i_trans = np.array(self.get_parameter('K_i_translation').value)
        K_i_rot = np.array(self.get_parameter('K_i_rotation').value)
        self.K_i = np.diag(np.concatenate([K_i_trans, K_i_rot]))
        
        self.integral_limit = self.get_parameter('integral_limit').value
        
        # Guadagni polso (joint-space)
        self.control_wrist = self.get_parameter('control_wrist_orientation').value
        self.K_p_wrist = np.array(self.get_parameter('K_p_wrist_joints').value)
        self.K_d_wrist = np.array(self.get_parameter('K_d_wrist_joints').value)
        
        # Stato robot
        self.n_joints = 6
        self.q = np.zeros(self.model.nq)
        self.dq = np.zeros(self.model.nv)
        self.state_received = False
        
        # Posa desiderata (cartesiana)
        self.x_desired = None
        self.x_desired_initialized = False
        
        # Target orientamento polso (joint-space)
        self.q_desired_wrist = None
        
        # Controllo PID
        self.control_dt = 1.0 / control_rate
        self.error_integral = np.zeros(6)
        
        # Safe startup mode
        self.safe_startup_mode = True
        self.safe_startup_counter = 0
        
        # Statistiche
        self.iteration_count = 0
        self.error_norm_pos = 0.0
        self.error_norm_rot = 0.0
        self.error_norm_wrist = 0.0
        self.vel_norm = 0.0
        self.force_norm = 0.0
        self.max_torque = 0.0
        self.sum_error_pos = 0.0
        self.sum_error_rot = 0.0
        self.max_error_pos = 0.0
        self.max_error_rot = 0.0
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.desired_pose_sub = self.create_subscription(
            PoseStamped, '/desired_pose', self.desired_pose_callback, 10
        )
        self.impedance_params_sub = self.create_subscription(
            Float64MultiArray, '/set_impedance', self.impedance_callback, 10
        )
        
        # Publishers
        self.torque_pub = self.create_publisher(Float64MultiArray, '/torque_controller/commands', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_ee_pose', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, '/cartesian_wrench', 10)
        
        # Timers
        self.timer = self.create_timer(self.control_dt, self.control_loop)
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.print_status)
        
        # Log inizializzazione
        self.get_logger().info('='*70)
        self.get_logger().info('IMPEDANCE CONTROLLER V3 INIZIALIZZATO')
        self.get_logger().info('Controllo Ibrido: Posizione Cartesiana + Orientamento Joint-Space')
        self.get_logger().info(f'Control: {control_rate} Hz | Log: {self.log_rate} Hz')
        self.get_logger().info(f'K_p_translation: {K_p_trans}')
        self.get_logger().info(f'K_d_translation: {K_d_trans}')
        self.get_logger().info(f'K_i_translation: {K_i_trans}')
        self.get_logger().info(f'Controllo polso: {self.control_wrist}')
        if self.control_wrist:
            self.get_logger().info(f'K_p_wrist: {self.K_p_wrist}')
            self.get_logger().info(f'K_d_wrist: {self.K_d_wrist}')
        self.get_logger().info(f'Gravity scale J2: {self.gravity_scale_j2}')
        self.get_logger().info(f'Torque limit: {self.torque_limit} Nm')
        self.get_logger().info(f'Safe startup: {self.safe_startup_duration}s')
        self.get_logger().info(f'Max step distance: {self.max_step_distance*1000:.0f}mm')
        self.get_logger().info('='*70)

    def shutdown_handler(self, signum, frame):
        self.get_logger().info('🛑 Shutdown richiesto...')
        
        # Invia coppia zero
        zero_torque = Float64MultiArray()
        zero_torque.data = [0.0] * 6
        
        for _ in range(20):  # 0.1s @ 200Hz
            self.torque_pub.publish(zero_torque)
            time.sleep(0.005)
        
        self.get_logger().info('✅ Coppia azzerata!')
        sys.exit(0)

    def joint_state_callback(self, msg):
        """Callback per aggiornare lo stato dei giunti"""
        if len(msg.position) < self.n_joints:
            return
        
        self.q[:self.n_joints] = np.array(msg.position[:self.n_joints])
        self.dq[:self.n_joints] = (
            np.array(msg.velocity[:self.n_joints]) 
            if len(msg.velocity) >= self.n_joints 
            else np.zeros(self.n_joints)
        )
        self.state_received = True
    
    def desired_pose_callback(self, msg):
        """Callback per aggiornare la posa desiderata con validazione"""
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # Verifica workspace
        reach = np.linalg.norm(position[:2])
        if not (0.05 < reach < 0.70 and -0.35 < position[1] < 0.35 and 0.05 < position[2] < 0.85):
            self.get_logger().error(
                f'❌ Target FUORI workspace: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
            )
            self.get_logger().error('   Limiti: reach=0.05-0.70m, y=-0.35-0.35m, z=0.05-0.85m')
            return
        
        # Verifica distanza da posizione corrente
        if hasattr(self, 'data') and self.x_desired_initialized:
            x_current = self.data.oMf[self.ee_frame_id]
            distance = np.linalg.norm(position - x_current.translation)
            
            if distance > self.max_step_distance:
                self.get_logger().error(f'⚠️ Target troppo lontano: {distance*1000:.0f}mm!')
                self.get_logger().error(
                    f'   Da: [{x_current.translation[0]:.3f}, {x_current.translation[1]:.3f}, '
                    f'{x_current.translation[2]:.3f}]'
                )
                self.get_logger().error(f'   A:  [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]')
                self.get_logger().info(f'💡 Usa step < {self.max_step_distance*1000:.0f}mm')
                return
        
        # Imposta target posizione
        quaternion = pin.Quaternion(
            msg.pose.orientation.w, msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z
        )
        quaternion.normalize()
        
        self.x_desired = pin.SE3(quaternion.toRotationMatrix(), position)
        self.x_desired_initialized = True
        self.error_integral = np.zeros(6)
        
        # ========================================
        # CALCOLA TARGET ORIENTAMENTO POLSO (Joint-Space)
        # ========================================
        if self.control_wrist:
            # Opzione semplice: mantieni orientamento corrente dei joint del polso
            # (puoi sostituire con IK completo se necessario)
            self.q_desired_wrist = self.q[3:6].copy()
            
            self.get_logger().info(
                f'🎯 Target polso [j4,j5,j6]: [{np.rad2deg(self.q_desired_wrist[0]):.1f}°, '
                f'{np.rad2deg(self.q_desired_wrist[1]):.1f}°, {np.rad2deg(self.q_desired_wrist[2]):.1f}°]'
            )
        
        # Log posizione
        if hasattr(self, 'data'):
            x_current = self.data.oMf[self.ee_frame_id]
            distance = np.linalg.norm(position - x_current.translation)
            self.get_logger().info('-'*70)
            self.get_logger().info(
                f'✅ NUOVO TARGET POS: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] '
                f'(distanza: {distance*1000:.0f}mm)'
            )
            self.get_logger().info('-'*70)
        else:
            self.get_logger().info(f'Nuovo target: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]')
    
    def impedance_callback(self, msg):
        """Callback per modificare parametri di impedenza online"""
        if len(msg.data) == 12:
            self.K_p = np.diag(msg.data[:6])
            self.K_d = np.diag(msg.data[6:])
            self.get_logger().info(f'Impedenza aggiornata: K_p={msg.data[:3]}, K_d={msg.data[6:9]}')
        else:
            self.get_logger().warn(f'Formato impedance non valido: attesi 12 valori, ricevuti {len(msg.data)}')
    
    def control_loop(self):
        """Loop principale di controllo"""
        if not self.state_received:
            return
        
        # Aggiorna cinematica e dinamica
        pin.forwardKinematics(self.model, self.data, self.q, self.dq)
        pin.updateFramePlacements(self.model, self.data)
        
        # ========== SAFE STARTUP MODE ==========
        if self.safe_startup_mode:
            self.safe_startup_counter += 1
            elapsed = self.safe_startup_counter * self.control_dt
            
            if elapsed < self.safe_startup_duration:
                if self.safe_startup_counter == 1:
                    x_current = self.data.oMf[self.ee_frame_id]
                    self.x_startup = x_current.copy()
                    pos = self.x_startup.translation
                    self.get_logger().info('📍 SAFE STARTUP: Posizione bloccata a:')
                    self.get_logger().info(f'   [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')
                
                x_current = self.data.oMf[self.ee_frame_id]
                x_error = np.zeros(6)
                x_error[:3] = self.x_startup.translation - x_current.translation
                J = pin.computeFrameJacobian(
                    self.model, self.data, self.q, self.ee_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )
                dx = J @ self.dq
                # ✅ AGGIUNGI QUESTO (solo 1 volta all'avvio):
                if self.iteration_count == 500:  # Dopo 1 secondo
                    self.get_logger().info('='*70)
                    self.get_logger().info('🔍 VERIFICA FRAME JACOBIANO:')
                    self.get_logger().info(f'Frame reference usato: LOCAL_WORLD_ALIGNED (BASE FRAME)')
                    self.get_logger().info(f'End-effector frame: {self.ee_frame_name}')
                    self.get_logger().info('')
                    self.get_logger().info('Jacobiano (prima riga = movimento X):')
                    self.get_logger().info(f'  J[X,:] = {J[0,:3]}')
                    self.get_logger().info('Jacobiano (terza riga = movimento Z):')
                    self.get_logger().info(f'  J[Z,:] = {J[2,:3]}')
                    self.get_logger().info('')
                    self.get_logger().info('Se J[Z,1] (colonna J2) è il valore più grande in J[Z,:],')
                    self.get_logger().info('allora il frame è CORRETTO (base frame).')
                    self.get_logger().info('='*70)
                K_p_startup = np.diag([200.0, 200.0, 200.0, 20.0, 20.0, 20.0])
                K_d_startup = np.diag([20.0, 20.0, 20.0, 2.0, 2.0, 2.0])
                
                F_cartesian = K_p_startup @ x_error - K_d_startup @ dx
                tau_impedance = (J.T @ F_cartesian)[:self.n_joints]
                
                tau_compensation = self.compute_compensation()
                
                tau_total = tau_impedance + tau_compensation
                #tau_total = tau_impedance
                tau_total = np.clip(tau_total, -self.torque_limit, self.torque_limit)
                
                self.publish_torque(tau_total)
                
                if self.iteration_count % 250 == 0:
                    pos = x_current.translation
                    drift_mm = np.linalg.norm(x_error[:3]) * 1000
                    self.get_logger().info(
                        f'🔶 SAFE STARTUP: {elapsed:.1f}s/{self.safe_startup_duration:.1f}s | '
                        f'Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | '
                        f'Drift: {drift_mm:.1f}mm | τ_max: {np.max(np.abs(tau_total)):.2f}Nm'
                    )
                
                self.iteration_count += 1
                return
            else:
                self.x_desired = self.x_startup.copy()
                self.x_desired_initialized = True
                
                # Inizializza target polso
                if self.control_wrist:
                    self.q_desired_wrist = self.q[3:6].copy()
                
                self.safe_startup_mode = False
                
                pos = self.x_desired.translation
                self.get_logger().info('='*70)
                self.get_logger().info('✅ SAFE STARTUP COMPLETATO')
                self.get_logger().info(
                    f'   Target posizione: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]'
                )
                if self.control_wrist:
                    self.get_logger().info(
                        f'   Target polso: [{np.rad2deg(self.q_desired_wrist[0]):.1f}°, '
                        f'{np.rad2deg(self.q_desired_wrist[1]):.1f}°, {np.rad2deg(self.q_desired_wrist[2]):.1f}°]'
                    )
                self.get_logger().info('   Impedance control ATTIVO - Pronto per comandi!')
                self.get_logger().info('='*70)

        # ========== CONTROLLO NORMALE ==========

        x_current = self.data.oMf[self.ee_frame_id]
        self.publish_current_pose(x_current)

        if not self.x_desired_initialized:
            self.x_desired = x_current.copy()
            self.x_desired_initialized = True
            if self.control_wrist:
                self.q_desired_wrist = self.q[3:6].copy()
            pos = self.x_desired.translation
            self.get_logger().info(f'Target iniziale: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

        # ========================================
        # CALCOLO ERRORE (METODO SEMPLICE)
        # ========================================
        x_error = np.zeros(6)  # ✅ INIZIALIZZA PRIMA!
        x_error[:3] = self.x_desired.translation - x_current.translation  # [X, Y, Z]
        # x_error[3:6] rimane a zero (nessun controllo rotazionale cartesiano)

        # Calcola Jacobiano e velocità cartesiana
        J = pin.computeFrameJacobian(
            self.model, self.data, self.q, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        dx = J @ self.dq  

        # Anti-windup intelligente
        error_norm_total = np.linalg.norm(x_error[:3])  # ✅ Usa solo errore posizione
        if error_norm_total > 0.025:  # ✅ Abbassato da 0.15m (150mm) a 25mm
            self.error_integral *= 0.90
        else:
            self.error_integral += x_error * self.control_dt
            self.error_integral = np.clip(self.error_integral, -self.integral_limit, self.integral_limit)

        # Legge di controllo PID cartesiana
        F_cartesian = self.K_p @ x_error - self.K_d @ dx + self.K_i @ self.error_integral

        # Disabilita controllo rotazionale cartesiano (già zero, ma per sicurezza)
        F_cartesian[3:6] = 0.0

        # ===== DEBUG: LOG DETTAGLIATO =====
        if self.iteration_count % 500 == 0:
            x_curr = x_current.translation
            x_targ = self.x_desired.translation
            
            kp_x = self.K_p[0, 0]
            kp_y = self.K_p[1, 1]
            kp_z = self.K_p[2, 2]
            
            self.get_logger().info(
                f'\n{"="*70}\n'
                f'DEBUG IMPEDANCE - Iter {self.iteration_count}\n'
                f'{"="*70}\n'
                f'POSIZIONI:\n'
                f'  Corrente: [{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}]\n'
                f'  Target:   [{x_targ[0]:.4f}, {x_targ[1]:.4f}, {x_targ[2]:.4f}]\n'
                f'\n'
                f'ERRORE (target - corrente):\n'
                f'  X: {x_error[0]*1000:+7.2f} mm\n'
                f'  Y: {x_error[1]*1000:+7.2f} mm\n'
                f'  Z: {x_error[2]*1000:+7.2f} mm\n'
                f'\n'
                f'FORZA CARTESIANA:\n'
                f'  F_x: {F_cartesian[0]:+7.2f} N\n'
                f'  F_y: {F_cartesian[1]:+7.2f} N\n'
                f'  F_z: {F_cartesian[2]:+7.2f} N\n'
                f'\n'
                f'FORZA ATTESA (K_p * errore):\n'
                f'  F_x: {kp_x*x_error[0]:+7.2f} N\n'
                f'  F_y: {kp_y*x_error[1]:+7.2f} N\n'
                f'  F_z: {kp_z*x_error[2]:+7.2f} N\n'
                f'{"="*70}'
            )

        # Proietta forza cartesiana in coppie giunti
        tau_impedance = (J.T @ F_cartesian)[:self.n_joints]

        # ========== CONTROLLO ORIENTAMENTO (Joint-Space) ==========
        if self.control_wrist and self.q_desired_wrist is not None:
            q_error_wrist = self.q_desired_wrist - self.q[3:6]
            dq_wrist = self.dq[3:6]
            tau_wrist = self.K_p_wrist * q_error_wrist - self.K_d_wrist * dq_wrist
            tau_impedance[3:6] += tau_wrist
            self.error_norm_wrist = np.linalg.norm(q_error_wrist)

        # Compensazione dinamica
        tau_compensation = self.compute_compensation()

        # ===== DEBUG LOG SEGNI =====
        if self.iteration_count % 500 == 0:
            self.get_logger().info(
                f'🔍 DEBUG SEGNI:\n'
                f'  x_error[2]: {x_error[2]*1000:+.2f} mm\n'
                f'  F_z: {F_cartesian[2]:+.2f} N\n'
                f'  tau_imp[J2]: {tau_impedance[1]:+.2f} Nm\n'
                f'  tau_comp[J2]: {tau_compensation[1]:+.2f} Nm\n'
                f'  tau_total[J2]: {(tau_impedance[1] + tau_compensation[1]):+.2f} Nm'
            )

        # Coppia totale
        tau_total = tau_impedance + tau_compensation
        tau_total = np.clip(tau_total, -self.torque_limit, self.torque_limit)

        # Pubblica comandi
        self.publish_torque(tau_total)
        self.publish_wrench(F_cartesian)

        # Aggiorna statistiche
        self.iteration_count += 1
        self.error_norm_pos = np.linalg.norm(x_error[:3])
        self.error_norm_rot = 0.0  # ✅ Sempre zero (nessun controllo rot cartesiano)
        self.vel_norm = np.linalg.norm(dx)
        self.force_norm = np.linalg.norm(F_cartesian)
        self.max_torque = np.max(np.abs(tau_total))
        self.sum_error_pos += self.error_norm_pos
        self.sum_error_rot += self.error_norm_rot
        self.max_error_pos = max(self.max_error_pos, self.error_norm_pos)
        self.max_error_rot = max(self.max_error_rot, self.error_norm_rot)

    def compute_compensation(self):
        """
        Compensazione ibrida: usa Pinocchio per la forma matematica,
        ma scala il risultato per correggere parametri URDF errati
        """
        # Calcola gravità con Pinocchio (forma corretta ma ampiezza sbagliata)
        tau_gravity = pin.computeGeneralizedGravity(self.model, self.data, self.q)
        
        # Calcola Coriolis
        aq_zero = np.zeros(self.model.nv)
        tau_rnea = pin.rnea(self.model, self.data, self.q, self.dq, aq_zero)
        tau_coriolis = tau_rnea - tau_gravity
        
        # Compensa con scaling su J2
        tau_comp = tau_gravity + tau_coriolis
        tau_comp[1] = tau_gravity[1] * self.gravity_scale_j2 + tau_coriolis[1]
        
        # ===== DEBUG LOG =====
        if self.iteration_count % 500 == 0:  # Ogni secondo
            self.get_logger().info(
                f'GRAVITY J2: {tau_gravity[1]:.2f} Nm | '
                f'SCALED: {tau_gravity[1] * self.gravity_scale_j2:.2f} Nm | '
                f'TOTAL COMP: {tau_comp[1]:.2f} Nm'
            )
        # =====================
        return tau_comp[:self.n_joints]

    def publish_torque(self, tau):
        """Pubblica comandi di coppia"""
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)
    
    def publish_current_pose(self, x_current):
        """Pubblica posa corrente end-effector"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x_current.translation[0]
        msg.pose.position.y = x_current.translation[1]
        msg.pose.position.z = x_current.translation[2]
        quat = pin.Quaternion(x_current.rotation)
        msg.pose.orientation.w = quat.w
        msg.pose.orientation.x = quat.x
        msg.pose.orientation.y = quat.y
        msg.pose.orientation.z = quat.z
        self.current_pose_pub.publish(msg)
    
    def publish_wrench(self, F_cartesian):
        """Pubblica wrench cartesiano"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.ee_frame_name
        msg.wrench.force.x = F_cartesian[0]
        msg.wrench.force.y = F_cartesian[1]
        msg.wrench.force.z = F_cartesian[2]
        msg.wrench.torque.x = F_cartesian[3]
        msg.wrench.torque.y = F_cartesian[4]
        msg.wrench.torque.z = F_cartesian[5]
        self.wrench_pub.publish(msg)
    
    def print_status(self):
        """Stampa status periodico"""
        if self.iteration_count == 0:
            return
        
        avg_pos = self.sum_error_pos / self.iteration_count
        avg_rot = self.sum_error_rot / self.iteration_count
        
        log_msg = (
            f'Iter: {self.iteration_count:6d} | '
            f'Err_pos: {self.error_norm_pos*1000:6.2f}mm (avg: {avg_pos*1000:6.2f}mm) | '
            f'Vel: {self.vel_norm:5.3f} | F: {self.force_norm:6.1f}N | τ: {self.max_torque:5.2f}Nm'
        )
        
        if self.control_wrist and hasattr(self, 'error_norm_wrist'):
            log_msg += f' | Err_wrist: {np.rad2deg(self.error_norm_wrist):5.2f}°'
        
        self.get_logger().info(log_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ImpedanceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if controller.iteration_count > 0:
            avg_pos = controller.sum_error_pos / controller.iteration_count
            avg_rot = controller.sum_error_rot / controller.iteration_count
            controller.get_logger().info('='*70)
            controller.get_logger().info('STATISTICHE FINALI')
            controller.get_logger().info(f'Iterazioni: {controller.iteration_count}')
            controller.get_logger().info(
                f'Errore pos: {avg_pos*1000:.2f}mm (max: {controller.max_error_pos*1000:.2f}mm)'
            )
            controller.get_logger().info(
                f'Errore rot: {np.rad2deg(avg_rot):.2f}° (max: {np.rad2deg(controller.max_error_rot):.2f}°)'
            )
            controller.get_logger().info('='*70)
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
