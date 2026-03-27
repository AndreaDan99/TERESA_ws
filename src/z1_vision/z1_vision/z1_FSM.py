#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from concurrent.futures import ThreadPoolExecutor

from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker

from tf_transformations import quaternion_matrix, quaternion_from_matrix

from z1_vision.workspace_checker   import WorkspaceChecker
from z1_vision.z1_scan_manager     import ScanManager
from z1_vision.body_search_scanner import BodySearchScanner, ScanAction


class Z1FSM(Node):
    # ── State constants ────────────────────────────────────────────────
    WAITING              = "WAITING"
    CHECKING_WORKSPACE   = "CHECKING_WORKSPACE"
    APPROACHING          = "APPROACHING"
    WAIT_IK_DONE         = "WAIT_IK_DONE"
    WRIST_ALIGN          = "WRIST_ALIGN"
    SWITCHING_TO_TORQUE  = "SWITCHING_TO_TORQUE"
    IMPEDANCE_RUNNING    = "IMPEDANCE_RUNNING"
    SWITCHING_TO_JTC     = "SWITCHING_TO_JTC"
    SCAN_PRELIFT         = "SCAN_PRELIFT"
    HOMING               = "HOMING"
    EMERGENCY_SWITCHING  = "EMERGENCY_SWITCHING"
    EMERGENCY            = "EMERGENCY"
    FAULT                = "FAULT"
    BODY_SCANNING        = "BODY_SCANNING"

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
        self.declare_parameter("approach_mode",        "normal") # "normal" | "vertical"
        self._surface_frame_topic  = self.get_parameter("surface_frame_topic").value
        self._ik_approach_standoff = float(self.get_parameter("ik_approach_standoff").value)
        self._approach_mode        = self.get_parameter("approach_mode").value

        # ── Modalità scansione (delegata a ScanManager) ─────────────────
        # Tutti i parametri scan vengono dichiarati e letti internamente
        # da ScanManager.from_params(): scan_mode, scan_delta_lateral/axial,
        # scan_center_y_offset, scan_clearance_x, wait_ik_timeout_s.
        self._scan_mgr = ScanManager.from_params(self)

        # ── Debug: skip impedance per verificare solo allineamento JTC ──
        # Se True: dopo WAIT_IK_DONE torna in WAITING senza avviare impedance.
        # Utile per verificare visivamente se il JTC allinea l'EE alla normale
        # prima di abilitare il torque controller.
        self.declare_parameter("skip_impedance", False)
        self._skip_impedance = bool(self.get_parameter("skip_impedance").value)

        # ── Timeout WAIT_IK_DONE (dichiarato da ScanManager.from_params) ──
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

        # ── Body scan params ─────────────────────────────────────────────
        self.declare_parameter("body_scan_on_start",           True)
        self.declare_parameter("body_scan_center",             [-0.09, 0.00, 0.44])
        self.declare_parameter("body_scan_ext_y",              0.20)
        self.declare_parameter("body_scan_ext_z",              0.15)
        self.declare_parameter("body_scan_ny",                 2)
        self.declare_parameter("body_scan_nz",                 2)
        self.declare_parameter("body_scan_point_timeout",      4.0)
        self.declare_parameter("body_scan_min_frames",         8)
        self.declare_parameter("body_scan_early_stop",         0.95)
        self.declare_parameter("body_scan_fusion_max_dist",    0.15)  # [m] soglia outlier rejection arco/hips
        self.declare_parameter("body_scan_p3_offset_x",        0.00)  # [m] offset laterale (+X) per fase 3
        self.declare_parameter("body_scan_p3_offset_y",        0.20)  # [m] offset verso fianchi (+Y) per fase 3
        self.declare_parameter("body_scan_p3_offset_z",        0.20)  # [m] offset verso l'alto (+Z) per fase 3
        # Griglia polso fase 1 e 3: sweep angolare nel frame EE
        self.declare_parameter("body_scan_wrist_ny",           3)
        self.declare_parameter("body_scan_wrist_nz",           3)
        self.declare_parameter("body_scan_wrist_angle_y_deg",  12.0)
        self.declare_parameter("body_scan_wrist_angle_z_deg",  12.0)
        self.declare_parameter("body_scan_wrist_timeout",      1.5)
        self.declare_parameter("body_scan_wrist_min_frames",   5)
        # Griglia polso fase 2 (arco): default 1×1 = solo look-at centrale
        # Evita combinazioni arco+wrist che portano fuori workspace/JTC.
        self.declare_parameter("body_scan_arc_wrist_ny",       1)
        self.declare_parameter("body_scan_arc_wrist_nz",       1)

        self._scan_on_start            = bool(self.get_parameter("body_scan_on_start").value)
        self._body_scan_center         = np.array(self.get_parameter("body_scan_center").value, dtype=float)
        self._body_scan_ext_y          = float(self.get_parameter("body_scan_ext_y").value)
        self._body_scan_ext_z          = float(self.get_parameter("body_scan_ext_z").value)
        self._body_scan_ny             = int(self.get_parameter("body_scan_ny").value)
        self._body_scan_nz             = int(self.get_parameter("body_scan_nz").value)
        self._body_scan_timeout        = float(self.get_parameter("body_scan_point_timeout").value)
        self._body_scan_min_fr         = int(self.get_parameter("body_scan_min_frames").value)
        self._body_scan_early          = float(self.get_parameter("body_scan_early_stop").value)
        self._body_scan_fusion_dist    = float(self.get_parameter("body_scan_fusion_max_dist").value)
        self._body_scan_p3_offset_x    = float(self.get_parameter("body_scan_p3_offset_x").value)
        self._body_scan_p3_offset_y    = float(self.get_parameter("body_scan_p3_offset_y").value)
        self._body_scan_p3_offset_z    = float(self.get_parameter("body_scan_p3_offset_z").value)
        self._body_scan_wrist_ny       = int(self.get_parameter("body_scan_wrist_ny").value)
        self._body_scan_wrist_nz       = int(self.get_parameter("body_scan_wrist_nz").value)
        self._body_scan_wrist_ang_y    = float(self.get_parameter("body_scan_wrist_angle_y_deg").value) * np.pi / 180.0
        self._body_scan_wrist_ang_z    = float(self.get_parameter("body_scan_wrist_angle_z_deg").value) * np.pi / 180.0
        self._body_scan_wrist_timeout  = float(self.get_parameter("body_scan_wrist_timeout").value)
        self._body_scan_wrist_min_fr   = int(self.get_parameter("body_scan_wrist_min_frames").value)
        self._body_scan_arc_wrist_ny   = int(self.get_parameter("body_scan_arc_wrist_ny").value)
        self._body_scan_arc_wrist_nz   = int(self.get_parameter("body_scan_arc_wrist_nz").value)

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

        # ── Body scan state ──────────────────────────────────────────────
        # _body_scan_done: True dopo che la scan è stata eseguita in questo ciclo
        #   (reset a False quando si torna in HOMING, così ogni ciclo ri-scansiona)
        # _body_scanner: istanza BodySearchScanner, creata in set_state(BODY_SCANNING)
        # _scan_torso_estimate: ultima posizione 3D valida del torso vista durante
        #   la scan (da /torso_scan_point). Usata per look-at dinamico: quando
        #   il tracker inizia a vedere il torso, i goal IK successivi vengono
        #   riorientati verso la posizione reale invece del look-at fisso pre-calcolato.
        self._body_scan_done: bool                         = False
        self._body_scanner:   BodySearchScanner | None     = None
        self._scan_torso_estimate: np.ndarray | None       = None
        self._scan_phase1_anchor:  np.ndarray | None       = None  # stima torso fine fase 1
        self._scan_phase2_anchor:  np.ndarray | None       = None  # stima torso fusa fine fase 2 (anchor per fase 3)
        self._scan_phase: int                              = 1  # 1=home, 2=arc+wrist, 3=hips
        # _tracker_ready: True dopo aver ricevuto almeno un messaggio su
        # /torso_tracker_state. Garantisce che il nodo tracker sia avviato
        # e il modello YOLO sia caricato prima di iniziare la body scan.
        self._tracker_ready: bool                          = False
        self._tracker_wait_logged: float                   = 0.0   # timestamp ultimo warn

        self.create_subscription(
            PoseStamped, self.torso_locked_topic, self.on_torso_locked, 10
        )
        self.create_subscription(
            PoseStamped, self._surface_frame_topic, self._on_surface_frame, 10
        )
        self.create_subscription(Bool,              self.ik_done_topic,        self.on_ik_done,        10)
        self.create_subscription(Bool,              self.impedance_done_topic, self.on_impedance_done, 10)
        self.create_subscription(String,            self.keyboard_cmd_topic,   self._on_keyboard_cmd,  10)
        self.create_subscription(Float32MultiArray, '/torso_scan_point',       self._on_scan_point,    10)
        self.create_subscription(String,            '/torso_tracker_state',    self._on_tracker_state, 10)

        # ── Publishers ──────────────────────────────────────────────────
        self.pub_ik_enable        = self.create_publisher(Bool,        self.ik_enable_topic,              10)
        self.pub_ik_goal          = self.create_publisher(PoseStamped, self.ik_goal_topic,                 10)
        self.pub_state            = self.create_publisher(String,      self.state_topic,                   10)
        self.pub_impedance_enable = self.create_publisher(Bool,        self.impedance_enable_topic,        10)
        self.pub_out_of_workspace = self.create_publisher(Bool,        self.target_out_of_workspace_topic, 10)
        self.pub_ik_goal_marker      = self.create_publisher(Marker, '/ik_goal_marker',       10)
        self.pub_tracker_reset       = self.create_publisher(Bool,  '/tracker_reset',        10)
        # Body scan: comandi al tracker
        self.pub_tracker_scan_mode   = self.create_publisher(Bool,         '/tracker_scan_mode',  10)
        self.pub_tracker_scan_next   = self.create_publisher(Bool,         '/tracker_scan_next',  10)
        self.pub_torso_scan_seed     = self.create_publisher(PointStamped, '/torso_scan_seed',    10)

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
        self._last_approach_pose: PoseStamped | None = None   # saved per WRIST_ALIGN
        self._wrist_align_sent: bool                 = False
        self._wrist_align_start: float | None        = None
        self._homing_last_send:  float               = 0.0   # timestamp ultimo invio enable+goal in HOMING

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
            self._scan_mgr.reset()   # reset indice scan ad ogni nuovo ciclo

        if s == self.CHECKING_WORKSPACE:
            self._workspace_future   = None
            self._clipped_target     = None
            self._checker_input_pose = None
            self._target_out_of_ws   = False

        if s == self.APPROACHING:
            self._approach_command_sent = False

        if s == self.SCAN_PRELIFT:
            self._scan_mgr.reset_prelift()
            self.ik_done = False

        if s == self.SWITCHING_TO_TORQUE:
            self._switch_future = None

        if s == self.SWITCHING_TO_JTC:
            self._switch_future = None

        if s == self.IMPEDANCE_RUNNING:
            self.impedance_done          = False
            self._impedance_command_sent = False

        if s == self.WAIT_IK_DONE:
            self._wait_ik_start = self.get_clock().now().nanoseconds * 1e-9

        if s == self.WRIST_ALIGN:
            self.ik_done            = False
            self._wrist_align_sent  = False
            self._wrist_align_start = self.get_clock().now().nanoseconds * 1e-9

        if s == self.BODY_SCANNING:
            self.ik_done               = False
            self._scan_torso_estimate  = None
            self._scan_phase1_anchor   = None
            self._scan_phase2_anchor   = None
            self._scan_phase           = 1
            self.get_logger().info('━'*60)
            self.get_logger().info('🔎 BODY SCAN AVVIATA')
            self.get_logger().info('━'*60)
            self.get_logger().info(
                f'▶ FASE 1 — Home wrist sweep '
                f'({self._body_scan_wrist_ny}×{self._body_scan_wrist_nz} pose, '
                f'±{np.degrees(self._body_scan_wrist_ang_y):.0f}°Y '
                f'±{np.degrees(self._body_scan_wrist_ang_z):.0f}°Z)'
            )
            self._body_scanner = BodySearchScanner(
                scan_poses         = self._gen_home_arc_poses(),
                scan_point_timeout = self._body_scan_wrist_timeout,
                scan_min_frames    = self._body_scan_wrist_min_fr,
                early_stop_score   = self._body_scan_early,
                logger             = self.get_logger(),
            )
            self._body_scanner.reset()
            # Attiva scan mode nel tracker
            self.pub_tracker_scan_mode.publish(Bool(data=True))

        if s == self.HOMING:
            self.ik_done              = False
            self._homing_command_sent = False
            # NON resettare _body_scan_done qui: il reset va fatto solo su
            # comando esplicito 'home' / 'reset', non dopo il body scan.

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

    def _make_approach_pose(self, extra_clearance: float = 0.0) -> PoseStamped | None:
        """
        Calcola la posa di approccio in base a self._approach_mode:

        "normal"   — standoff lungo la normale superficiale (approccio frontale):
                      pos  = p_surf + standoff * normal
                      X_ee = -normal  (punta verso il torso)

        "vertical" — braccio sopra al torso, discesa in -Z world (Z è verticale):
                      pos  = [torso.x + dx, torso.y + dy,
                               p_surf.z + standoff + dz + extra_clearance]
                      X_ee = [0, 0, -1]  (punta verso il basso = verso il torso)

        extra_clearance [m]: altezza extra sopra lo standoff (SCAN_PRELIFT: +Z = salita)
                             prima di spostarsi lateralmente al prossimo punto

        In entrambi i casi l'orientamento è la ROTAZIONE MINIMA di R_home
        che porta X_home → x_ee (formula di Rodrigues), senza vincoli su Y/Z.

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
        normal = R[:, 2]   # asse Z surface frame = normale, dal torso verso il robot
        p_surf = np.array([sf.pose.position.x, sf.pose.position.y, sf.pose.position.z])

        if self._approach_mode == 'vertical':
            # ── Modalità verticale: JTC porta l'EE a standoff in Z sopra la superficie ──
            # X:  YOLO torso X + dx - extra_clearance
            #     (extra_clearance>0 solo in SCAN_PRELIFT: arretra in -X prima
            #      di spostarsi lateralmente al prossimo punto)
            # Y:  YOLO torso Y + dy  (offset assiale spalla↔fianco)
            # Z:  p_surf.z + standoff + dz  (altezza sopra superficie + offset laterale)
            # X_ee = [0,0,-1]: asse X EE punta verso il basso (-Z world)
            # L'impedance poi avanza in +X (normal=[-1,0,0]) per toccare il torso.
            torso_ref = self._checker_input_pose if self._checker_input_pose is not None \
                        else self.last_torso_pose
            off = self._scan_mgr.current_offset   # (dx, dy, dz) world frame
            p_approach = np.array([
                torso_ref.pose.position.x + off[0] - extra_clearance,
                torso_ref.pose.position.y + off[1],
                p_surf[2] + self._ik_approach_standoff + off[2]
            ])
            x_ee = np.array([0.0, 0.0, -1.0])
            scan_lbl = f'pt{self._scan_mgr.idx} off=({off[0]:.2f},{off[1]:.2f},{off[2]:.2f})'
            extra_lbl = f' clr_x={extra_clearance:.3f}m' if extra_clearance != 0.0 else ''
            mode_log = (f'vertical ↓Z→+X (standoff={self._ik_approach_standoff:.2f}m{extra_lbl})'
                        f' [{scan_lbl}]')
        else:
            # ── Modalità normale: standoff lungo la normale superficiale ──
            # La normale punta DAL torso VERSO il robot → standoff davanti al torso
            p_approach = p_surf + self._ik_approach_standoff * normal
            # X_ee punta verso il torso (= -normal)
            x_ee = -normal
            mode_log = (f'normal n=[{normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f}]'
                        f' standoff={self._ik_approach_standoff:.3f}m')

        # ── Orientamento: rotazione minima da home per allineare X_home → x_ee ──
        q_approach = self._orientation_for_xee(x_ee)

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
            f'🎯 IK goal [{mode_log}]: '
            f'pos=[{p_approach[0]:.3f},{p_approach[1]:.3f},{p_approach[2]:.3f}]'
        )

        return goal

    def _orientation_for_xee(self, x_ee: np.ndarray) -> np.ndarray:
        """
        Calcola l'orientamento EE con X_ee = x_ee, mantenendo Y_ee il più vicino
        possibile a Y_home (Gram-Schmidt).

        Metodo:
          1. X_ee = x_ee normalizzato
          2. Y_ee = Y_home proiettato ⊥ a X_ee, normalizzato
             (fallback: Z_home se Y_home è quasi parallelo a X_ee)
          3. Z_ee = X_ee × Y_ee

        Vantaggi rispetto a Rodrigues:
          - Nessun flip d'asse quando l'angolo tra x_home e x_ee è grande
          - Y_ee rimane sempre vicino al riferimento home → continuità
        Ritorna quaternione [x, y, z, w].
        """
        R_home = quaternion_matrix(self._home_orientation)[:3, :3]
        x_ee   = x_ee / np.linalg.norm(x_ee)

        # Y_ee: Y_home proiettato ⊥ a X_ee
        y_ref  = R_home[:, 1]
        y_ee   = y_ref - np.dot(y_ref, x_ee) * x_ee
        y_norm = np.linalg.norm(y_ee)
        if y_norm < 1e-3:
            # Y_home quasi parallelo a X_ee → usa Z_home come riferimento
            y_ref  = R_home[:, 2]
            y_ee   = y_ref - np.dot(y_ref, x_ee) * x_ee
            y_norm = np.linalg.norm(y_ee)
        y_ee /= y_norm

        z_ee = np.cross(x_ee, y_ee)

        T = np.eye(4)
        T[:3, 0] = x_ee
        T[:3, 1] = y_ee
        T[:3, 2] = z_ee
        return quaternion_from_matrix(T)

    def _make_wrist_align_pose(self) -> PoseStamped | None:
        """
        Ricomputa l'orientamento EE dalla normale superficiale corrente,
        mantenendo la posizione dell'ultimo goal IK raggiunto.
        Usato per allineare il polso con la normale prima di passare
        all'impedance control.

        "vertical": x_ee = [0,0,-1] (discesa in -Z, stesso di _make_approach_pose)
        "normal":   x_ee = -normal  (X_ee verso il torso)
        """
        if self._last_approach_pose is None:
            self.get_logger().warn('⚠️  WRIST_ALIGN: nessuna approach pose salvata')
            return None

        if self._latest_surface_frame is None:
            self.get_logger().warn(
                '⚠️  WRIST_ALIGN: surface frame non disponibile → uso orientamento approach'
            )
            return self._last_approach_pose

        sf     = self._latest_surface_frame
        q_surf = [sf.pose.orientation.x, sf.pose.orientation.y,
                  sf.pose.orientation.z, sf.pose.orientation.w]
        R      = quaternion_matrix(q_surf)[:3, :3]
        normal = R[:, 2]   # asse Z surface frame = normale, dal torso verso robot

        if self._approach_mode == 'vertical':
            x_ee = np.array([0.0, 0.0, -1.0])
        else:
            x_ee = -normal   # X_ee punta verso il torso

        q_approach = self._orientation_for_xee(x_ee)

        goal = PoseStamped()
        goal.header.frame_id    = 'world'
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position      = self._last_approach_pose.pose.position
        goal.pose.orientation.x = float(q_approach[0])
        goal.pose.orientation.y = float(q_approach[1])
        goal.pose.orientation.z = float(q_approach[2])
        goal.pose.orientation.w = float(q_approach[3])

        self.get_logger().info(
            f'🔄 WRIST_ALIGN: n=[{normal[0]:.2f},{normal[1]:.2f},{normal[2]:.2f}]'
            f' x_ee=[{x_ee[0]:.2f},{x_ee[1]:.2f},{x_ee[2]:.2f}]'
        )
        return goal

    # ──────────────────────────────────────────────────────────────
    def _finish_body_scan(self):
        """Pubblica seed fuso, disattiva scan mode, torna in HOME poi WAITING."""
        # Usa l'anchor della fase più recente disponibile:
        # fase 3 usa anchor fase 2; se fase 3 non è partita usa anchor fase 1.
        anchor = (self._scan_phase2_anchor if self._scan_phase2_anchor is not None
                  else self._scan_phase1_anchor)
        fused = (self._body_scanner.fused_torso_xyz(
                     anchor   = anchor,
                     max_dist = self._body_scan_fusion_dist,
                 )
                 if self._body_scanner is not None else None)
        if fused is not None:
            seed_msg = PointStamped()
            seed_msg.header.stamp    = self.get_clock().now().to_msg()
            seed_msg.header.frame_id = 'world'
            seed_msg.point.x         = float(fused[0])
            seed_msg.point.y         = float(fused[1])
            seed_msg.point.z         = float(fused[2])
            self.pub_torso_scan_seed.publish(seed_msg)
            self.get_logger().info(
                f'📍 Seed torso fuso: [{fused[0]:.3f}, {fused[1]:.3f}, {fused[2]:.3f}]'
            )
        self.pub_ik_enable.publish(Bool(data=False))
        self.pub_tracker_scan_mode.publish(Bool(data=False))
        self._body_scan_done = True
        self.get_logger().info('━'*60)
        self.get_logger().info('🏁 BODY SCAN COMPLETATA → HOMING → WAITING')
        if fused is not None:
            self.get_logger().info(
                f'   Stima finale torso: '
                f'[{fused[0]:.3f}, {fused[1]:.3f}, {fused[2]:.3f}]'
            )
        self.get_logger().info('━'*60)
        self._homing_next_state = self.WAITING
        self.set_state(self.HOMING)

    def _wrist_poses_at(self, pos: np.ndarray, R_base: np.ndarray,
                        ny: int | None = None, nz: int | None = None) -> list:
        """
        Genera una griglia ny × nz di PoseStamped nella posizione `pos`.

        Ogni orientamento è:   R_base @ Ry(alpha) @ Rz(beta)
        dove alpha ∈ [-ang_y, ..., +ang_y] e beta ∈ [-ang_z, ..., +ang_z].

        R_base viene perturbato nel suo proprio frame EE → nessun flip d'asse,
        movimenti piccoli e simmetrici attorno alla direzione base.
        Se ny/nz non specificati, usa i valori di default (fase 1: home sweep).
        """
        n_y   = ny if ny is not None else self._body_scan_wrist_ny
        n_z   = nz if nz is not None else self._body_scan_wrist_nz
        ang_y = self._body_scan_wrist_ang_y
        ang_z = self._body_scan_wrist_ang_z
        now   = self.get_clock().now().to_msg()

        alphas = np.linspace(-ang_y, ang_y, n_y) if n_y > 1 else np.array([0.0])
        betas  = np.linspace(-ang_z, ang_z, n_z) if n_z > 1 else np.array([0.0])

        poses = []
        for alpha in alphas:
            ca, sa = np.cos(alpha), np.sin(alpha)
            Ry = np.array([[ ca, 0.0,  sa],
                           [0.0, 1.0, 0.0],
                           [-sa, 0.0,  ca]])
            for beta in betas:
                cb, sb = np.cos(beta), np.sin(beta)
                Rz = np.array([[cb, -sb, 0.0],
                               [sb,  cb, 0.0],
                               [0.0, 0.0, 1.0]])
                R_new = R_base @ Ry @ Rz
                T = np.eye(4)
                T[:3, :3] = R_new
                q = quaternion_from_matrix(T)

                p = PoseStamped()
                p.header.frame_id    = 'world'
                p.header.stamp       = now
                p.pose.position.x    = float(pos[0])
                p.pose.position.y    = float(pos[1])
                p.pose.position.z    = float(pos[2])
                p.pose.orientation.x = float(q[0])
                p.pose.orientation.y = float(q[1])
                p.pose.orientation.z = float(q[2])
                p.pose.orientation.w = float(q[3])
                poses.append(p)
        return poses

    def _gen_home_arc_poses(self) -> list:
        """
        Fase 1: HOME position con griglia angolare EE pura.

        Base = home_orientation.
        Griglia wrist_ny × wrist_nz: R_home @ Ry(alpha) @ Rz(beta).
        Movimenti piccoli e simmetrici attorno alla direzione home → nessun flip.
        """
        home_pos = np.array(self._home_position, dtype=float)
        R_home   = quaternion_matrix(self._home_orientation)[:3, :3]
        poses    = self._wrist_poses_at(home_pos, R_home)
        ang_y_d  = np.degrees(self._body_scan_wrist_ang_y)
        ang_z_d  = np.degrees(self._body_scan_wrist_ang_z)
        self.get_logger().info(
            f'🗺️  Fase 1: {len(poses)} pose '
            f'(home × {self._body_scan_wrist_ny}Y×{self._body_scan_wrist_nz}Z, '
            f'±{ang_y_d:.1f}°Y ±{ang_z_d:.1f}°Z nel frame EE)'
        )
        return poses

    def _gen_all_wrist_poses(self, torso_estimate: np.ndarray) -> tuple[list, set]:
        """
        Fase 2: posizioni ARCO × griglia angolare EE.
        (Home già visitata in fase 1.)

        Per ogni posizione arco viene inserita prima una posa home intermedia
        (transit, solo movimento, nessuna raccolta dati) in modo da spezzare
        il percorso e facilitare la convergenza del JTC.

        Struttura lista risultante (arc_wrist_ny=arc_wrist_nz=1):
          idx 0: home (transit)
          idx 1: arco pos 1
          idx 2: home (transit)
          idx 3: arco pos 2
          ...

        Ritorna (poses, transit_indices).
        """
        center = self._body_scan_center
        ny     = self._body_scan_ny
        nz     = self._body_scan_nz
        ext_y  = self._body_scan_ext_y
        ext_z  = self._body_scan_ext_z

        ys = (np.linspace(center[1] - ext_y, center[1] + ext_y, ny)
              if ny > 1 else np.array([center[1]]))
        zs = (np.linspace(center[2] - ext_z, center[2] + ext_z, nz)
              if nz > 1 else np.array([center[2]]))

        poses: list          = []
        transit_indices: set = set()

        for z in zs:
            for y in ys:
                # ── home intermedia (transit) ──
                transit_indices.add(len(poses))
                poses.append(self._make_home_pose())

                # ── posa arco ──
                pos = np.array([float(center[0]), float(y), float(z)])
                d   = torso_estimate - pos
                norm = np.linalg.norm(d)
                if norm < 1e-6:
                    R_base = quaternion_matrix(self._home_orientation)[:3, :3]
                else:
                    q_base = self._orientation_for_xee(d / norm)
                    R_base = quaternion_matrix(q_base)[:3, :3]
                poses.extend(self._wrist_poses_at(
                    pos, R_base,
                    ny=self._body_scan_arc_wrist_ny,
                    nz=self._body_scan_arc_wrist_nz,
                ))

        arc_ny  = self._body_scan_arc_wrist_ny
        arc_nz  = self._body_scan_arc_wrist_nz
        n_arc   = ny * nz
        n_wr    = arc_ny * arc_nz
        self.get_logger().info(
            f'🗺️  Fase 2: {len(poses)} pose totali '
            f'({n_arc} pos arco × {n_wr} wrist + {n_arc} home transito, '
            f'look-at verso torso_estimate)'
        )
        return poses, transit_indices


    def _gen_phase3_poses(self, torso_estimate: np.ndarray) -> tuple[list, set]:
        """
        Fase 3: un solo punto offset da home, look-at verso il torso di fase 2.

        Posizione: home + [offset_x, offset_y, offset_z]
          +Y = verso i fianchi (asse corpo)
          +Z = verso l'alto
          +X = laterale

        Orientamento: look-at verso torso_estimate (fase 2 anchor)
        → la camera punta verso il centro del torso da questa angolazione
          laterale/alta, permettendo di vedere spalle e raffinare il centro.

        Preceduto da una home transit per garantire convergenza JTC.
        Ritorna (poses, transit_indices).
        """
        pos = np.array([
            float(self._home_position[0]) + self._body_scan_p3_offset_x,
            float(self._home_position[1]) + self._body_scan_p3_offset_y,
            float(self._home_position[2]) + self._body_scan_p3_offset_z,
        ])
        d    = torso_estimate - pos
        norm = np.linalg.norm(d)
        if norm < 1e-6:
            R_base = quaternion_matrix(self._home_orientation)[:3, :3]
        else:
            q_base = self._orientation_for_xee(d / norm)
            R_base = quaternion_matrix(q_base)[:3, :3]

        poses:           list = []
        transit_indices: set  = set()

        # home transit
        transit_indices.add(0)
        poses.append(self._make_home_pose())

        # posa fase 3
        poses.extend(self._wrist_poses_at(pos, R_base,
                                          ny=self._body_scan_arc_wrist_ny,
                                          nz=self._body_scan_arc_wrist_nz))
        self.get_logger().info(
            f'🗺️  Fase 3: pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] '
            f'(home +X={self._body_scan_p3_offset_x:.2f}m '
            f'+Y={self._body_scan_p3_offset_y:.2f}m '
            f'+Z={self._body_scan_p3_offset_z:.2f}m, '
            f'look-at torso [{torso_estimate[0]:.3f},{torso_estimate[1]:.3f},{torso_estimate[2]:.3f}])'
        )
        return poses, transit_indices

    def _on_tracker_state(self, msg: String):
        """Primo messaggio da /torso_tracker_state → tracker avviato e YOLO caricato."""
        if not self._tracker_ready:
            self._tracker_ready = True
            self.get_logger().info('✅ Torso tracker pronto → body scan abilitata')

    def _on_scan_point(self, msg: Float32MultiArray):
        """
        Callback /torso_scan_point: riceve dati per-frame dal tracker
        durante la body scan e li invia allo scanner.
        data = [score, n_kp, conf, x_world, y_world, z_world]
        """
        if self.state == self.BODY_SCANNING and self._body_scanner is not None:
            self._body_scanner.feed_scan_data(list(msg.data))
        # Aggiorna stima posizione torso per look-at dinamico.
        # Salva l'ultima rilevazione 3D valida (score > 0) indipendentemente
        # dallo stato: se il tracker vede il torso, teniamo la posizione.
        data = list(msg.data)
        if len(data) >= 6 and float(data[0]) > 0.0:
            self._scan_torso_estimate = np.array(data[3:6], dtype=float)

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
            self._body_scan_done = False   # ri-scansiona al prossimo ciclo
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
            self._body_scan_done = False   # ri-scansiona al prossimo ciclo
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
            # Body scan: se abilitato e non ancora eseguito in questo ciclo.
            # Aspetta che il tracker sia pronto (YOLO caricato) prima di iniziare:
            # senza tracker la scan raccoglierebbe solo timeout senza dati.
            if self._scan_on_start and not self._body_scan_done:
                if not self._tracker_ready:
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if now - self._tracker_wait_logged > 5.0:
                        self._tracker_wait_logged = now
                        self.get_logger().warn(
                            '⏳ In attesa che il torso tracker si avvii '
                            '(nessun messaggio su /torso_tracker_state)...'
                        )
                    return
                self.set_state(self.BODY_SCANNING)
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

                if self._scan_mgr.idx == 0:
                    # Punto centrale: calcola normalmente con surface frame live
                    target = self._make_approach_pose()
                    if target is None:
                        return
                    # Salva la posa del centro (pt0) stabile per tutti i punti successivi
                    self._scan_mgr.save_center_approach(target)
                else:
                    # Punti non-centro: calcola da posa centro + offset relativo.
                    # Evita il surface frame live (può derivare durante impedance).
                    c = self._scan_mgr._center_approach_pose
                    if c is None:
                        self.get_logger().warn('⚠️  APPROACHING: center_approach_pose non salvata → WAITING')
                        self.set_state(self.WAITING)
                        return
                    off_n = self._scan_mgr.current_offset
                    off_c = self._scan_mgr.offsets[0]
                    target = PoseStamped()
                    target.header.frame_id    = 'world'
                    target.header.stamp       = self.get_clock().now().to_msg()
                    target.pose.position.x    = c.pose.position.x
                    target.pose.position.y    = c.pose.position.y + (off_n[1] - off_c[1])
                    target.pose.position.z    = c.pose.position.z + (off_n[2] - off_c[2])
                    target.pose.orientation   = c.pose.orientation

                self._last_approach_pose = target   # salvata per WRIST_ALIGN

                p = target.pose.position
                self.get_logger().info(
                    f"🎯 APPROACHING pt{self._scan_mgr.idx}: goal "
                    f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
                )
                self.pub_ik_enable.publish(Bool(data=True))
                self.pub_ik_goal.publish(target)
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
                    self.set_state(self.WRIST_ALIGN)

        # ── WRIST_ALIGN ───────────────────────────────────────────────────
        # Ricomputa l'orientamento EE dalla normale superficiale corrente
        # prima di passare all'impedance control.
        elif self.state == self.WRIST_ALIGN:
            if not self._wrist_align_sent:
                goal = self._make_wrist_align_pose()
                if goal is None:
                    self.get_logger().warn(
                        '⚠️  WRIST_ALIGN: posa non disponibile → SWITCHING_TO_TORQUE diretto'
                    )
                    self.set_state(self.SWITCHING_TO_TORQUE)
                    return
                self.pub_ik_enable.publish(Bool(data=True))
                self.pub_ik_goal.publish(goal)
                self._wrist_align_sent = True
                p = goal.pose.position
                self.get_logger().info(
                    f'🔄 WRIST_ALIGN: goal inviato '
                    f'pos=[{p.x:.3f},{p.y:.3f},{p.z:.3f}]'
                )
                return

            # Timeout
            if self._wrist_align_start is not None:
                elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._wrist_align_start
                if elapsed > self._wait_ik_timeout:
                    self.get_logger().warn(
                        f'⏱️  WRIST_ALIGN timeout ({elapsed:.1f}s) → SWITCHING_TO_TORQUE diretto'
                    )
                    self.pub_ik_enable.publish(Bool(data=False))
                    self.set_state(self.SWITCHING_TO_TORQUE)
                    return

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                self.get_logger().info('✅ WRIST_ALIGN completato → SWITCHING_TO_TORQUE')
                self.set_state(self.SWITCHING_TO_TORQUE)

        # ── BODY_SCANNING ─────────────────────────────────────────────────
        # Scansione a griglia: trova la posa del braccio da cui il tracker
        # vede meglio il torso, poi si sposta lì e sblocca il lock normale.
        elif self.state == self.BODY_SCANNING:
            if self._body_scanner is None:
                self.get_logger().warn('⚠️  BODY_SCANNING: scanner non inizializzato → WAITING')
                self._body_scan_done = True
                self.set_state(self.WAITING)
                return

            now = self.get_clock().now().nanoseconds * 1e-9
            st  = self._body_scanner.tick(ik_done=self.ik_done, now=now)

            if st.action == ScanAction.SEND_IK:
                # Fase 1: usa home_orientation (già nella pose).
                # Fase 2: orientamento look-at + wrist già baked-in in _gen_arc_wrist_poses.
                self.ik_done = False
                self.pub_ik_enable.publish(Bool(data=True))
                self.pub_ik_goal.publish(st.goal)
                p = st.goal.pose.position
                self.get_logger().info(
                    f'🔍 Body scan P{self._scan_phase} IK: '
                    f'[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]'
                )

            elif st.action == ScanAction.RESET_TRACKER:
                # Braccio arrivato alla posa: resetta tracker per raccogliere dati puliti
                self.pub_tracker_scan_next.publish(Bool(data=True))

            elif st.action == ScanAction.EXIT_SCAN_MODE:
                if self._scan_phase == 1:
                    # ── Fase 1 completata → Fase 2 ──────────────────────────
                    self.get_logger().info('✅ FASE 1 completata')
                    if self._scan_torso_estimate is not None:
                        self._scan_phase1_anchor = self._scan_torso_estimate.copy()
                        a1 = self._scan_phase1_anchor
                        poses_p2, transit_p2 = self._gen_all_wrist_poses(self._scan_torso_estimate)
                        self._body_scanner = BodySearchScanner(
                            scan_poses         = poses_p2,
                            scan_point_timeout = self._body_scan_wrist_timeout,
                            scan_min_frames    = self._body_scan_wrist_min_fr,
                            early_stop_score   = self._body_scan_early,
                            logger             = self.get_logger(),
                            transit_indices    = transit_p2,
                        )
                        self._body_scanner.reset()
                        self._scan_phase = 2
                        n_pos   = self._body_scan_ny * self._body_scan_nz
                        ang_y_d = np.degrees(self._body_scan_wrist_ang_y)
                        ang_z_d = np.degrees(self._body_scan_wrist_ang_z)
                        self.get_logger().info('━'*60)
                        self.get_logger().info(
                            f'▶ FASE 2 — Arco wrist sweep '
                            f'({len(poses_p2)} pose, {n_pos} pos × '
                            f'{self._body_scan_arc_wrist_ny}×{self._body_scan_arc_wrist_nz}, '
                            f'±{ang_y_d:.0f}°Y ±{ang_z_d:.0f}°Z)'
                        )
                        self.get_logger().info(
                            f'   anchor fase 1: '
                            f'[{a1[0]:.3f}, {a1[1]:.3f}, {a1[2]:.3f}]'
                        )
                    else:
                        self.get_logger().warn('⚠️  Fase 1 senza detection → skip fase 2+3')
                        self._finish_body_scan()

                elif self._scan_phase == 2:
                    # ── Fase 2 completata → Fase 3 ──────────────────────────
                    self.get_logger().info('✅ FASE 2 completata')
                    fused_p2 = self._body_scanner.fused_torso_xyz(
                        anchor   = self._scan_phase1_anchor,
                        max_dist = self._body_scan_fusion_dist,
                    )
                    self._scan_phase2_anchor = (fused_p2 if fused_p2 is not None
                                                else self._scan_phase1_anchor)
                    a2 = self._scan_phase2_anchor
                    poses_p3, transit_p3 = self._gen_phase3_poses(
                        torso_estimate=a2 if a2 is not None else self._scan_phase1_anchor
                    )
                    self._body_scanner = BodySearchScanner(
                        scan_poses         = poses_p3,
                        scan_point_timeout = self._body_scan_wrist_timeout,
                        scan_min_frames    = self._body_scan_wrist_min_fr,
                        early_stop_score   = self._body_scan_early,
                        logger             = self.get_logger(),
                        transit_indices    = transit_p3,
                    )
                    self._body_scanner.reset()
                    self._scan_phase = 3
                    self.get_logger().info('━'*60)
                    self.get_logger().info(
                        f'▶ FASE 3 — Posizione laterale '
                        f'(home +X={self._body_scan_p3_offset_x:.2f}m '
                        f'+Y={self._body_scan_p3_offset_y:.2f}m '
                        f'+Z={self._body_scan_p3_offset_z:.2f}m, look-at torso)'
                    )

                else:
                    # ── Fase 3 completata → fine scan ───────────────────────
                    self.get_logger().info('✅ FASE 3 completata')
                    self._finish_body_scan()

            elif st.action == ScanAction.FAILED:
                # Nessun punto valido trovato: disattiva scan mode, vai in WAITING
                self.pub_ik_enable.publish(Bool(data=False))
                self.pub_tracker_scan_mode.publish(Bool(data=False))
                self.get_logger().warn(
                    '⚠️  Body scan FAILED (nessun punto valido) → WAITING'
                )
                self._body_scan_done = True   # evita loop infinito
                self.set_state(self.WAITING)
            # ScanAction.WAIT: nessuna azione

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
                next_st = self._scan_mgr.on_jtc_switch_success(self)
                if next_st == "SCAN_PRELIFT":
                    self.set_state(self.SCAN_PRELIFT)
                else:
                    # Scansione completa → HOMING poi WAITING
                    self._post_impedance_hold = True
                    self._homing_next_state   = self.WAITING
                    self.pub_tracker_reset.publish(Bool(data=True))
                    self.get_logger().info("🔄 Tracker reset inviato → torso tracker → IDLE")
                    self.set_state(self.HOMING)
            else:
                self.get_logger().error(f"❌ Switch fallito: {result.message}")
                self.set_state(self.FAULT)

        # ── SCAN_PRELIFT ──────────────────────────────────────────────────
        elif self.state == self.SCAN_PRELIFT:
            if self._scan_mgr.tick_prelift(self):
                self.set_state(self.APPROACHING)

        # ── HOMING ────────────────────────────────────────────────────────
        elif self.state == self.HOMING:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            # Invia (o ri-invia) enable+goal ogni 1.5s finché ik_done non arriva.
            # Gestisce il race condition di startup: se z1_ik_to_jtc parte in
            # ritardo e perde il primo messaggio, lo riceve al retry successivo.
            if not self.ik_done and (
                not self._homing_command_sent
                or now_s - self._homing_last_send > 1.5
            ):
                is_retry = self._homing_command_sent   # True se non è il primo invio
                self.ik_done = False
                home_pose = self._make_home_pose()
                self.pub_ik_goal.publish(home_pose)
                self.pub_ik_enable.publish(Bool(data=True))
                self._homing_command_sent = True
                self._homing_last_send    = now_s
                if is_retry:
                    self.get_logger().warn("🔄 HOMING retry: re-invio enable+goal")
                else:
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
