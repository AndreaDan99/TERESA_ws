#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from concurrent.futures import ThreadPoolExecutor

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from tf_transformations import quaternion_matrix, quaternion_from_matrix

from z1_vision.workspace_checker import WorkspaceChecker


class Z1FSM(Node):
    # ── State constants ────────────────────────────────────────────────
    WAITING              = "WAITING"
    CHECKING_WORKSPACE   = "CHECKING_WORKSPACE"
    APPROACHING          = "APPROACHING"
    WAIT_IK_DONE         = "WAIT_IK_DONE"
    SWITCHING_TO_TORQUE  = "SWITCHING_TO_TORQUE"
    IMPEDANCE_RUNNING    = "IMPEDANCE_RUNNING"
    SWITCHING_TO_JTC     = "SWITCHING_TO_JTC"
    HOMING               = "HOMING"
    EMERGENCY_SWITCHING  = "EMERGENCY_SWITCHING"
    EMERGENCY            = "EMERGENCY"
    FAULT                = "FAULT"

    # States where the torque_controller may be active.
    # Any keyboard command arriving in these states must first switch to JTC.
    _TORQUE_STATES = frozenset([
        "SWITCHING_TO_TORQUE",
        "IMPEDANCE_RUNNING",
        "SWITCHING_TO_JTC",
    ])

    def __init__(self):
        super().__init__("z1_fsm")

        # ── Topic params ────────────────────────────────────────────────
        self.declare_parameter("torso_locked_topic",  "/torso_target_ee_locked")
        self.declare_parameter("ik_enable_topic",     "/ik_enable")
        self.declare_parameter("ik_goal_topic",       "/ik_goal_pose")
        self.declare_parameter("ik_done_topic",       "/ik_done")
        self.declare_parameter("state_topic",         "/z1_fsm/state")
        self.declare_parameter("target_max_age_s",    0.5)
        self.declare_parameter("keyboard_cmd_topic",  "/z1_keyboard_cmd")

        self.torso_locked_topic = self.get_parameter("torso_locked_topic").value
        self.ik_enable_topic    = self.get_parameter("ik_enable_topic").value
        self.ik_goal_topic      = self.get_parameter("ik_goal_topic").value
        self.ik_done_topic      = self.get_parameter("ik_done_topic").value
        self.state_topic        = self.get_parameter("state_topic").value
        self.target_max_age_s   = float(self.get_parameter("target_max_age_s").value)
        self.keyboard_cmd_topic = self.get_parameter("keyboard_cmd_topic").value

        # ── Impedance interface params ──────────────────────────────────
        self.declare_parameter("impedance_enable_topic", "/impedance_enable")
        self.declare_parameter("impedance_done_topic",   "/impedance_done")
        self.impedance_enable_topic = self.get_parameter("impedance_enable_topic").value
        self.impedance_done_topic   = self.get_parameter("impedance_done_topic").value

        # ── Approccio perpendicolare alla superficie (surface frame) ───
        # Il JTC goal viene calcolato come:
        #   p_goal = p_surf + ik_approach_standoff * normal
        # dove normal punta dalla superficie verso il robot (source: surface node).
        # L'orientamento impone l'asse X dell'EE perpendicolare alla superficie (verso il torso).
        self.declare_parameter("surface_frame_topic",  "/torso_surface_frame")
        self.declare_parameter("ik_approach_standoff", 0.200)   # [m] distanza standoff dal torso
        self._surface_frame_topic  = self.get_parameter("surface_frame_topic").value
        self._ik_approach_standoff = float(self.get_parameter("ik_approach_standoff").value)

        # ── Debug: skip impedance per verificare solo allineamento JTC ──
        # Se True: dopo WAIT_IK_DONE torna in WAITING senza avviare impedance.
        # Utile per verificare visivamente se il JTC allinea l'EE alla normale
        # prima di abilitare il torque controller.
        self.declare_parameter("skip_impedance", False)
        self._skip_impedance = bool(self.get_parameter("skip_impedance").value)

        # ── Timeout WAIT_IK_DONE ────────────────────────────────────────
        self.declare_parameter("wait_ik_timeout_s", 15.0)
        self._wait_ik_timeout = float(self.get_parameter("wait_ik_timeout_s").value)
        self._wait_ik_start: float | None = None

        # ── Workspace out-of-range topic ────────────────────────────────
        self.declare_parameter("target_out_of_workspace_topic", "/target_out_of_workspace")
        self.target_out_of_workspace_topic = self.get_parameter(
            "target_out_of_workspace_topic"
        ).value

        # ── Workspace checker params ────────────────────────────────────
        self.declare_parameter(
            "urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/"
            "share/z1_description/urdf/z1.urdf",
        )
        self.declare_parameter("ee_frame",                "link06")
        self.declare_parameter("workspace_safety_margin", 0.30)
        self.declare_parameter("arm_base_pos",            [0.0, 0.0, 0.0])

        urdf_path     = self.get_parameter("urdf_path").value
        ee_frame      = self.get_parameter("ee_frame").value
        safety_margin = float(self.get_parameter("workspace_safety_margin").value)
        self._arm_base = np.array(
            self.get_parameter("arm_base_pos").value, dtype=float
        )

        # ── Home position params ────────────────────────────────────────
        self.declare_parameter("home_position",    [0.3, 0.0, 0.3])
        self.declare_parameter("home_orientation", [0.0, 0.0, 0.0, 1.0])
        self._home_position = np.array(
            self.get_parameter("home_position").value, dtype=float
        )
        self._home_orientation = np.array(
            self.get_parameter("home_orientation").value, dtype=float
        )

        # ── WorkspaceChecker (Pinocchio, bloccante → thread) ────────────
        try:
            self._checker = WorkspaceChecker(
                urdf_path     = urdf_path,
                ee_frame      = ee_frame,
                safety_margin = safety_margin,
            )
            self.get_logger().info(
                f"✅ WorkspaceChecker pronto | safety_margin={safety_margin:.2f}m"
            )
        except Exception as e:
            self.get_logger().error(f"❌ WorkspaceChecker init fallita: {e}")
            self._checker = None

        self._ws_executor          = ThreadPoolExecutor(max_workers=1)
        self._workspace_future     = None
        self._clipped_target: PoseStamped | None = None
        self._checker_input_pose: PoseStamped | None = None
        self._target_out_of_ws     = False

        # ── Subscribers ─────────────────────────────────────────────────
        self.last_torso_pose: PoseStamped | None           = None
        self.last_torso_time                               = None
        self.ik_done                                       = False
        self.impedance_done                                = False
        self._pending_keyboard_cmd: str | None             = None
        self._latest_surface_frame: PoseStamped | None     = None
        self._skip_impedance_hold: bool                    = False  # blocca retry in skip_impedance mode
        self._post_impedance_hold: bool                    = False  # blocca retry dopo ciclo impedance completo

        self.create_subscription(
            PoseStamped, self.torso_locked_topic, self.on_torso_locked, 10
        )
        self.create_subscription(
            PoseStamped, self._surface_frame_topic, self._on_surface_frame, 10
        )
        self.create_subscription(Bool,   self.ik_done_topic,        self.on_ik_done,        10)
        self.create_subscription(Bool,   self.impedance_done_topic, self.on_impedance_done, 10)
        self.create_subscription(String, self.keyboard_cmd_topic,   self._on_keyboard_cmd,  10)

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_ik_enable        = self.create_publisher(Bool,        self.ik_enable_topic,              10)
        self.pub_ik_goal          = self.create_publisher(PoseStamped, self.ik_goal_topic,                 10)
        self.pub_state            = self.create_publisher(String,      self.state_topic,                   10)
        self.pub_impedance_enable = self.create_publisher(Bool,        self.impedance_enable_topic,        10)
        self.pub_out_of_workspace = self.create_publisher(Bool,        self.target_out_of_workspace_topic, 10)
        self.pub_ik_goal_marker   = self.create_publisher(Marker,      '/ik_goal_marker',                  10)
        self.pub_tracker_reset    = self.create_publisher(Bool,        '/tracker_reset',                   10)

        # ── Service clients: switch controller ──────────────────────────
        self.switch_to_torque_client = self.create_client(Trigger, '/safe_switch/to_torque')
        self.switch_to_jtc_client    = self.create_client(Trigger, '/safe_switch/to_jtc')
        self._switch_future = None

        # ── FSM state variables ─────────────────────────────────────────
        self.state                   = None
        self._approach_command_sent  = False
        self._impedance_command_sent = False
        self._homing_command_sent    = False
        self._homing_next_state      = self.WAITING   # where to go after HOMING

        # ── Start timer then go immediately to HOMING ───────────────────
        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self._homing_next_state = self.WAITING
        self.set_state(self.HOMING)
        # NOTA: NON pubblicare ik_enable=False qui — il nodo IK parte già disabilitato
        # e pubblicare False in __init__ può arrivare DOPO ik_enable=True del primo tick
        # causando un race condition che impedisce l'homing.

        self.get_logger().info("🧠 z1_FSM ready → avvio in HOMING")
        self.get_logger().info(f"  torso_locked:      {self.torso_locked_topic}")
        self.get_logger().info(f"  ik_goal:           {self.ik_goal_topic}")
        self.get_logger().info(f"  ik_enable:         {self.ik_enable_topic}")
        self.get_logger().info(f"  ik_done:           {self.ik_done_topic}")
        self.get_logger().info(f"  keyboard_cmd:      {self.keyboard_cmd_topic}")
        self.get_logger().info(f"  impedance_enable:  {self.impedance_enable_topic}")
        self.get_logger().info(f"  impedance_done:    {self.impedance_done_topic}")
        self.get_logger().info(f"  out_of_workspace:  {self.target_out_of_workspace_topic}")
        self.get_logger().info(
            f"  home_position:     "
            f"[{self._home_position[0]:.3f}, "
            f"{self._home_position[1]:.3f}, "
            f"{self._home_position[2]:.3f}]"
        )

    # =================================================================== #
    #  CALLBACKS                                                            #
    # =================================================================== #

    def on_torso_locked(self, msg: PoseStamped):
        self.last_torso_pose = msg
        self.last_torso_time = self.get_clock().now()

    def _on_surface_frame(self, msg: PoseStamped):
        self._latest_surface_frame = msg

    def on_ik_done(self, msg: Bool):
        if msg.data:
            self.ik_done = True

    def on_impedance_done(self, msg: Bool):
        if msg.data:
            self.impedance_done = True

    def _on_keyboard_cmd(self, msg: String):
        """Riceve comandi da z1_keyboard_safety via /z1_keyboard_cmd."""
        cmd = msg.data.strip().lower()
        if cmd in ("home", "emergency", "reset"):
            self._pending_keyboard_cmd = cmd

    # =================================================================== #
    #  HELPERS                                                              #
    # =================================================================== #

    def set_state(self, s: str):
        if s == self.state:
            return

        self.state = s
        self.pub_state.publish(String(data=s))
        self.get_logger().info(f"➡️  FSM state = {s}")

        if s == self.WAITING:
            self.ik_done                = False
            self._approach_command_sent = False

        if s == self.CHECKING_WORKSPACE:
            self._workspace_future   = None
            self._clipped_target     = None
            self._checker_input_pose = None
            self._target_out_of_ws   = False

        if s == self.APPROACHING:
            self._approach_command_sent = False

        if s == self.SWITCHING_TO_TORQUE:
            self._switch_future = None

        if s == self.SWITCHING_TO_JTC:
            self._switch_future = None

        if s == self.IMPEDANCE_RUNNING:
            self.impedance_done          = False
            self._impedance_command_sent = False

        if s == self.WAIT_IK_DONE:
            self._wait_ik_start = self.get_clock().now().nanoseconds * 1e-9

        if s == self.HOMING:
            self.ik_done              = False
            self._homing_command_sent = False

        if s == self.EMERGENCY_SWITCHING:
            self._switch_future = None
            self.get_logger().warn(
                f"🚨 EMERGENCY_SWITCHING: switch → JTC, poi HOMING → {self._homing_next_state}"
            )

        if s == self.EMERGENCY:
            self.get_logger().warn(
                "🚨 FSM in EMERGENCY — braccio fermo. Premere R per tornare in HOME."
            )

    def torso_target_fresh(self) -> bool:
        if self.last_torso_pose is None or self.last_torso_time is None:
            return False
        age = (self.get_clock().now() - self.last_torso_time).nanoseconds * 1e-9
        return age <= self.target_max_age_s

    def _pose_to_np(self, pose: PoseStamped) -> np.ndarray:
        p = pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def _make_clipped_pose(
        self, original: PoseStamped, clipped_pos: np.ndarray
    ) -> PoseStamped:
        """Costruisce un PoseStamped con posizione clippata ma orientazione invariata."""
        msg = PoseStamped()
        msg.header           = original.header
        msg.pose.orientation = original.pose.orientation
        msg.pose.position.x  = float(clipped_pos[0])
        msg.pose.position.y  = float(clipped_pos[1])
        msg.pose.position.z  = float(clipped_pos[2])
        return msg

    def _make_approach_pose(self) -> PoseStamped | None:
        """
        Posa di approccio perpendicolare alla superficie del torso:
        - Posizione:    p_surf + standoff * normal   (standoff m davanti al torso)
        - Orientamento: X_ee = -normal (asse approccio lungo la normale),
                        Y e Z: NESSUN vincolo — si ottengono con la rotazione
                        MINIMA dall'orientamento home che porta X su -normal.
                        Questo evita qualsiasi rotazione "inutile" del polso.

        Algoritmo "minimal rotation" (formula di Rodrigues):
          Ruota R_home del minimo angolo necessario per portare X_home → -normal.
          Y e Z ruotano solidali con X, senza roll aggiuntivo attorno a X.

        Fallback al target torso grezzo se il surface frame non è disponibile.
        """
        if self._latest_surface_frame is None:
            self.get_logger().warn(
                '⚠️  surface_frame non disponibile → uso target grezzo (no orientamento)',
                throttle_duration_sec=2.0,
            )
            return self._clipped_target if self._clipped_target is not None else self.last_torso_pose

        sf     = self._latest_surface_frame
        q_surf = [sf.pose.orientation.x, sf.pose.orientation.y,
                  sf.pose.orientation.z, sf.pose.orientation.w]
        R      = quaternion_matrix(q_surf)[:3, :3]
        normal = R[:, 2]   # asse Z del surface frame = normale, punta dal torso verso il robot

        p_surf = np.array([sf.pose.position.x, sf.pose.position.y, sf.pose.position.z])

        # La normale punta DAL torso VERSO il robot → standoff davanti al torso
        p_approach = p_surf + self._ik_approach_standoff * normal

        # ── Orientamento: rotazione minima da home per allineare X → -normal ──
        # R_home = orientamento di home (nessun vincolo su Y e Z)
        R_home = quaternion_matrix(self._home_orientation)[:3, :3]
        x_home = R_home[:, 0]          # asse X in home
        x_ee   = -normal               # asse X desiderato (verso il torso)

        cos_a = float(np.clip(np.dot(x_home, x_ee), -1.0, 1.0))
        axis  = np.cross(x_home, x_ee)
        sin_a = np.linalg.norm(axis)

        if sin_a < 1e-6:
            # x_home già allineato (o antiparallelo) a x_ee
            if cos_a > 0:
                R_approach = R_home                    # già allineato
            else:
                # 180°: ruota attorno a Z world (o Y se necessario)
                perp = np.array([0.0, 0.0, 1.0])
                if abs(np.dot(perp, x_home)) > 0.9:
                    perp = np.array([0.0, 1.0, 0.0])
                perp -= np.dot(perp, x_home) * x_home
                perp /= np.linalg.norm(perp)
                K = np.array([[     0, -perp[2],  perp[1]],
                              [ perp[2],      0, -perp[0]],
                              [-perp[1],  perp[0],      0]])
                R_approach = (2.0 * np.outer(perp, perp) - np.eye(3)) @ R_home
        else:
            axis  /= sin_a
            K      = np.array([[    0, -axis[2],  axis[1]],
                               [ axis[2],     0, -axis[0]],
                               [-axis[1],  axis[0],     0]])
            # Rodrigues: R_align = I + sin(θ)K + (1-cos(θ))K²
            R_align    = np.eye(3) + sin_a * K + (1.0 - cos_a) * (K @ K)
            R_approach = R_align @ R_home

        T = np.eye(4)
        T[:3, :3]  = R_approach
        q_approach = quaternion_from_matrix(T)

        goal = PoseStamped()
        goal.header.frame_id    = 'world'
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position.x    = float(p_approach[0])
        goal.pose.position.y    = float(p_approach[1])
        goal.pose.position.z    = float(p_approach[2])
        goal.pose.orientation.x = float(q_approach[0])
        goal.pose.orientation.y = float(q_approach[1])
        goal.pose.orientation.z = float(q_approach[2])
        goal.pose.orientation.w = float(q_approach[3])

        self.get_logger().info(
            f'🎯 IK goal: pos=[{p_approach[0]:.3f},{p_approach[1]:.3f},{p_approach[2]:.3f}] '
            f'n=[{normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f}] '
            f'standoff={self._ik_approach_standoff:.3f}m'
        )

        return goal

    def _make_home_pose(self) -> PoseStamped:
        """Costruisce un PoseStamped con la posizione home definita dai parametri YAML."""
        msg = PoseStamped()
        msg.header.frame_id    = "world"
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.position.x    = float(self._home_position[0])
        msg.pose.position.y    = float(self._home_position[1])
        msg.pose.position.z    = float(self._home_position[2])
        msg.pose.orientation.x = float(self._home_orientation[0])
        msg.pose.orientation.y = float(self._home_orientation[1])
        msg.pose.orientation.z = float(self._home_orientation[2])
        msg.pose.orientation.w = float(self._home_orientation[3])
        return msg

    def _handle_keyboard_cmd(self, cmd: str):
        """
        Processa un comando tastiera.

        Comandi accettati:
          "emergency"  → ferma tutto, porta in HOMING poi EMERGENCY
          "home"       → interrompe il task corrente, porta in HOMING poi WAITING
          "reset"      → da EMERGENCY: torna in HOMING poi WAITING
        """
        if cmd == "emergency":
            self.get_logger().warn(
                f"🚨 EMERGENCY ricevuto (stato: {self.state})"
            )
            self.pub_ik_enable.publish(Bool(data=False))
            self.pub_impedance_enable.publish(Bool(data=False))
            self._homing_next_state = self.EMERGENCY
            if self.state in self._TORQUE_STATES:
                self.set_state(self.EMERGENCY_SWITCHING)
            else:
                self.set_state(self.HOMING)

        elif cmd == "home":
            if self.state == self.EMERGENCY:
                self.get_logger().warn(
                    "⚠️  'home' ignorato in EMERGENCY — usare 'reset'"
                )
                return
            self.get_logger().info(f"🏠 HOME ricevuto (stato: {self.state})")
            self.pub_ik_enable.publish(Bool(data=False))
            self.pub_impedance_enable.publish(Bool(data=False))
            self._homing_next_state = self.WAITING
            if self.state in self._TORQUE_STATES:
                self.set_state(self.EMERGENCY_SWITCHING)
            else:
                self.set_state(self.HOMING)

        elif cmd == "reset":
            if self.state != self.EMERGENCY:
                self.get_logger().warn(
                    f"⚠️  'reset' ignorato — richiesto solo in EMERGENCY "
                    f"(stato attuale: {self.state})"
                )
                return
            self.get_logger().info("🔄 RESET → HOMING → WAITING")
            self._homing_next_state = self.WAITING
            self.set_state(self.HOMING)

    # =================================================================== #
    #  FSM TICK  (20 Hz)                                                    #
    # =================================================================== #

    def tick(self):

        # ── Keyboard commands (priorità massima, processati prima del tick) ──
        if self._pending_keyboard_cmd:
            cmd = self._pending_keyboard_cmd
            self._pending_keyboard_cmd = None
            self._handle_keyboard_cmd(cmd)
            # Nota: non si fa return — il tick prosegue sul nuovo stato corrente

        # ── WAITING ───────────────────────────────────────────────────────
        if self.state == self.WAITING:
            # In skip_impedance mode: non riprovare finché il lock non si perde
            if self._skip_impedance_hold:
                if not self.torso_target_fresh():
                    self._skip_impedance_hold = False
                    self.get_logger().info("🔓 Lock perso → skip_impedance hold rilasciato")
                return
            # Dopo ciclo impedance completo: non riprovare finché il lock non si perde
            if self._post_impedance_hold:
                if not self.torso_target_fresh():
                    self._post_impedance_hold = False
                    self.get_logger().info("🔓 Lock perso → post_impedance hold rilasciato")
                else:
                    return
            if self.torso_target_fresh():
                self.set_state(self.CHECKING_WORKSPACE)

        # ── CHECKING_WORKSPACE ────────────────────────────────────────────
        elif self.state == self.CHECKING_WORKSPACE:

            # Avvia il calcolo: serve un target fresco per fare il latch
            if self._workspace_future is None:
                if not self.torso_target_fresh():
                    self.get_logger().warn(
                        "⚠️  Nessun target fresco per workspace check → WAITING"
                    )
                    self.set_state(self.WAITING)
                    return

                # Se il checker non è disponibile, salta il controllo
                if self._checker is None:
                    self.get_logger().warn(
                        "⚠️  WorkspaceChecker non disponibile → skip check",
                        throttle_duration_sec=5.0,
                    )
                    self._clipped_target   = self.last_torso_pose
                    self._target_out_of_ws = False
                    self.pub_out_of_workspace.publish(Bool(data=False))
                    self.set_state(self.APPROACHING)
                    return

                # Latch del target corrente → da qui il robot si impegna su questa posa
                target_pos = self._pose_to_np(self.last_torso_pose)
                self._checker_input_pose = self.last_torso_pose
                self._workspace_future   = self._ws_executor.submit(
                    self._checker.clip_target, target_pos, self._arm_base
                )
                self.get_logger().info(
                    f"🔍 Workspace check avviato — target latchato a "
                    f"[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]"
                )
                return

            # Aspetta che il thread finisca
            if not self._workspace_future.done():
                return

            # Leggi il risultato
            try:
                clipped_pos, was_clipped, max_safe = self._workspace_future.result()
            except Exception as e:
                self.get_logger().error(f"❌ WorkspaceChecker errore: {e} → WAITING")
                self.set_state(self.WAITING)
                return

            self._target_out_of_ws = was_clipped
            self.pub_out_of_workspace.publish(Bool(data=was_clipped))

            if was_clipped:
                self.get_logger().warn(
                    f"⚠️  Target fuori workspace → clippato a {max_safe:.3f} m dalla base "
                    f"(pubblicato /target_out_of_workspace=True)"
                )
            else:
                self.get_logger().info(
                    f"✅ Target nel workspace (max_safe = {max_safe:.3f} m)"
                )

            self._clipped_target = self._make_clipped_pose(
                self._checker_input_pose, clipped_pos
            )
            self.set_state(self.APPROACHING)

        # ── APPROACHING ───────────────────────────────────────────────────
        elif self.state == self.APPROACHING:
            # Nessun abort per target non fresco: il robot si impegna
            # sul target latchato in CHECKING_WORKSPACE

            if not self._approach_command_sent:
                self.ik_done = False

                target = self._make_approach_pose()
                if target is None:
                    return

                self.pub_ik_goal.publish(target)
                self.pub_ik_enable.publish(Bool(data=True))
                self._approach_command_sent = True

                # Marker blu una-tantum al momento dell'invio del goal
                from builtin_interfaces.msg import Duration as BuiltinDuration
                m = Marker()
                m.header.frame_id = 'world'
                m.header.stamp    = self.get_clock().now().to_msg()
                m.ns = 'ik_goal'; m.id = 0
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose = target.pose
                m.scale.x = m.scale.y = m.scale.z = 0.08
                m.color.r = 0.0; m.color.g = 0.4; m.color.b = 1.0; m.color.a = 0.9
                m.lifetime = BuiltinDuration(sec=30, nanosec=0)
                self.pub_ik_goal_marker.publish(m)

                self.set_state(self.WAIT_IK_DONE)

        # ── WAIT_IK_DONE ──────────────────────────────────────────────────
        elif self.state == self.WAIT_IK_DONE:
            # Timeout: se l'IK non converge entro N secondi torna in WAITING
            if self._wait_ik_start is not None:
                elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._wait_ik_start
                if elapsed > self._wait_ik_timeout:
                    self.get_logger().warn(
                        f'⏱️  WAIT_IK_DONE timeout ({elapsed:.1f}s) → WAITING'
                    )
                    self.pub_ik_enable.publish(Bool(data=False))
                    self.set_state(self.WAITING)
                    return

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                if self._skip_impedance:
                    # Modalità debug: salta impedance, torna in attesa
                    # → non riprovare finché il lock non si perde (evita loop)
                    self.get_logger().warn(
                        "⚠️  skip_impedance=True → IK completato, "
                        "impedance SALTATA → WAITING (hold fino a nuovo lock)"
                    )
                    self._skip_impedance_hold = True
                    self.set_state(self.WAITING)
                else:
                    self.set_state(self.SWITCHING_TO_TORQUE)

        # ── SWITCHING_TO_TORQUE ───────────────────────────────────────────
        elif self.state == self.SWITCHING_TO_TORQUE:
            if self._switch_future is None:
                if not self.switch_to_torque_client.service_is_ready():
                    self.get_logger().warn(
                        "⏳ /safe_switch/to_torque non ancora disponibile...",
                        throttle_duration_sec=2.0,
                    )
                    return
                self.get_logger().info("📞 Chiamata /safe_switch/to_torque")
                self._switch_future = self.switch_to_torque_client.call_async(
                    Trigger.Request()
                )
                return

            if not self._switch_future.done():
                return

            result = self._switch_future.result()
            if result.success:
                self.get_logger().info("✅ Switch JTC → torque_controller riuscito")
                self.set_state(self.IMPEDANCE_RUNNING)
            else:
                self.get_logger().error(f"❌ Switch fallito: {result.message}")
                self.set_state(self.FAULT)

        # ── IMPEDANCE_RUNNING ─────────────────────────────────────────────
        elif self.state == self.IMPEDANCE_RUNNING:
            if not self._impedance_command_sent:
                self.pub_impedance_enable.publish(Bool(data=True))
                self._impedance_command_sent = True
                self.get_logger().info("🦾 Impedance controller avviato")
                return

            if self.impedance_done:
                self.pub_impedance_enable.publish(Bool(data=False))
                self.get_logger().info("✅ Impedance done → SWITCHING_TO_JTC")
                self.set_state(self.SWITCHING_TO_JTC)

        # ── SWITCHING_TO_JTC ──────────────────────────────────────────────
        elif self.state == self.SWITCHING_TO_JTC:
            if self._switch_future is None:
                if not self.switch_to_jtc_client.service_is_ready():
                    self.get_logger().warn(
                        "⏳ /safe_switch/to_jtc non ancora disponibile...",
                        throttle_duration_sec=2.0,
                    )
                    return
                self.get_logger().info("📞 Chiamata /safe_switch/to_jtc")
                self._switch_future = self.switch_to_jtc_client.call_async(
                    Trigger.Request()
                )
                return

            if not self._switch_future.done():
                return

            result = self._switch_future.result()
            if result.success:
                self.get_logger().info(
                    "✅ Switch torque_controller → JTC riuscito → HOMING (poi WAITING)"
                )
                self._post_impedance_hold = True   # non ripartire subito al prossimo lock
                self._homing_next_state   = self.WAITING
                # Reset del tracker: forza IDLE così può ri-acquisire il torso al ciclo successivo
                self.pub_tracker_reset.publish(Bool(data=True))
                self.get_logger().info("🔄 Tracker reset inviato → torso tracker → IDLE")
                self.set_state(self.HOMING)
            else:
                self.get_logger().error(f"❌ Switch fallito: {result.message}")
                self.set_state(self.FAULT)

        # ── HOMING ────────────────────────────────────────────────────────
        elif self.state == self.HOMING:
            if not self._homing_command_sent:
                self.ik_done = False
                home_pose = self._make_home_pose()
                self.pub_ik_goal.publish(home_pose)
                self.pub_ik_enable.publish(Bool(data=True))
                self._homing_command_sent = True
                self.get_logger().info(
                    f"🏠 HOMING: goal inviato → poi {self._homing_next_state} | "
                    f"pos=[{self._home_position[0]:.3f}, "
                    f"{self._home_position[1]:.3f}, "
                    f"{self._home_position[2]:.3f}]"
                )
                return

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                self.get_logger().info(
                    f"✅ HOMING completato → {self._homing_next_state}"
                )
                self.set_state(self._homing_next_state)

        # ── EMERGENCY_SWITCHING ───────────────────────────────────────────
        # Usato quando arriva un comando keyboard (emergency o home) mentre
        # il torque_controller potrebbe essere attivo.
        # Azione: switch forzato a JTC, poi HOMING con _homing_next_state.
        elif self.state == self.EMERGENCY_SWITCHING:
            if self._switch_future is None:
                if not self.switch_to_jtc_client.service_is_ready():
                    self.get_logger().warn(
                        "⏳ /safe_switch/to_jtc non disponibile (EMERGENCY_SWITCHING)...",
                        throttle_duration_sec=2.0,
                    )
                    return
                self.get_logger().info(
                    "📞 EMERGENCY_SWITCHING: chiamata /safe_switch/to_jtc"
                )
                self._switch_future = self.switch_to_jtc_client.call_async(
                    Trigger.Request()
                )
                return

            if not self._switch_future.done():
                return

            result = self._switch_future.result()
            if result.success:
                self.get_logger().info(
                    f"✅ Switch to JTC (emergency) → HOMING → {self._homing_next_state}"
                )
                self.set_state(self.HOMING)
            else:
                self.get_logger().error(
                    f"❌ Switch fallito (EMERGENCY_SWITCHING): {result.message} → EMERGENCY"
                )
                self.set_state(self.EMERGENCY)

        # ── EMERGENCY ─────────────────────────────────────────────────────
        # Stato bloccante: tutto disabilitato.
        # Uscita solo via comando "reset" da tastiera.
        elif self.state == self.EMERGENCY:
            pass   # attende _on_keyboard_cmd con cmd="reset"

        # ── FAULT ─────────────────────────────────────────────────────────
        elif self.state == self.FAULT:
            self.pub_ik_enable.publish(Bool(data=False))

        else:
            self.set_state(self.FAULT)


# ======================================================================== #
def main(args=None):
    rclpy.init(args=args)
    node = Z1FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
