#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from concurrent.futures import ThreadPoolExecutor

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from z1_vision.workspace_checker import WorkspaceChecker


class Z1FSM(Node):
    WAITING             = "WAITING"
    CHECKING_WORKSPACE  = "CHECKING_WORKSPACE"
    APPROACHING         = "APPROACHING"
    WAIT_IK_DONE        = "WAIT_IK_DONE"
    SWITCHING_TO_TORQUE = "SWITCHING_TO_TORQUE"
    IMPEDANCE_RUNNING   = "IMPEDANCE_RUNNING"
    SWITCHING_TO_JTC    = "SWITCHING_TO_JTC"
    FAULT               = "FAULT"

    def __init__(self):
        super().__init__("z1_fsm")

        # ---- Topic params ----
        self.declare_parameter("torso_locked_topic", "/torso_target_ee_locked")
        self.declare_parameter("ik_enable_topic",    "/ik_enable")
        self.declare_parameter("ik_goal_topic",      "/ik_goal_pose")
        self.declare_parameter("ik_done_topic",      "/ik_done")
        self.declare_parameter("state_topic",        "/z1_fsm/state")
        self.declare_parameter("target_max_age_s",   0.5)

        self.torso_locked_topic = self.get_parameter("torso_locked_topic").value
        self.ik_enable_topic    = self.get_parameter("ik_enable_topic").value
        self.ik_goal_topic      = self.get_parameter("ik_goal_topic").value
        self.ik_done_topic      = self.get_parameter("ik_done_topic").value
        self.state_topic        = self.get_parameter("state_topic").value
        self.target_max_age_s   = float(self.get_parameter("target_max_age_s").value)

        # ---- Impedance interface params ----
        self.declare_parameter("impedance_enable_topic", "/impedance_enable")
        self.declare_parameter("impedance_done_topic",   "/impedance_done")
        self.impedance_enable_topic = self.get_parameter("impedance_enable_topic").value
        self.impedance_done_topic   = self.get_parameter("impedance_done_topic").value

        # ---- Workspace out-of-range topic ----
        self.declare_parameter("target_out_of_workspace_topic", "/target_out_of_workspace")
        self.target_out_of_workspace_topic = self.get_parameter(
            "target_out_of_workspace_topic"
        ).value

        # ---- Workspace checker params ----
        self.declare_parameter("urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/"
            "share/z1_description/urdf/z1.urdf")
        self.declare_parameter("ee_frame",               "link06")
        self.declare_parameter("workspace_safety_margin", 0.30)
        self.declare_parameter("arm_base_pos",            [0.0, 0.0, 0.0])

        urdf_path      = self.get_parameter("urdf_path").value
        ee_frame       = self.get_parameter("ee_frame").value
        safety_margin  = float(self.get_parameter("workspace_safety_margin").value)
        self._arm_base = np.array(
            self.get_parameter("arm_base_pos").value, dtype=float
        )

        # ---- WorkspaceChecker (Pinocchio, bloccante → thread) ----
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
        self._workspace_future     = None        # future del thread corrente
        self._clipped_target: PoseStamped | None = None
        self._checker_input_pose: PoseStamped | None = None
        self._target_out_of_ws     = False       # flag: target era fuori workspace

        # ---- Subscribers ----
        self.last_torso_pose: PoseStamped | None = None
        self.last_torso_time = None
        self.ik_done         = False
        self.impedance_done  = False

        self.create_subscription(PoseStamped, self.torso_locked_topic, self.on_torso_locked, 10)
        self.create_subscription(Bool, self.ik_done_topic,        self.on_ik_done,        10)
        self.create_subscription(Bool, self.impedance_done_topic, self.on_impedance_done, 10)

        # ---- Publishers ----
        self.pub_ik_enable          = self.create_publisher(Bool,        self.ik_enable_topic,              10)
        self.pub_ik_goal            = self.create_publisher(PoseStamped, self.ik_goal_topic,                 10)
        self.pub_state              = self.create_publisher(String,      self.state_topic,                   10)
        self.pub_impedance_enable   = self.create_publisher(Bool,        self.impedance_enable_topic,        10)
        self.pub_out_of_workspace   = self.create_publisher(Bool,        self.target_out_of_workspace_topic, 10)

        # ---- Service clients: switch controller ----
        self.switch_to_torque_client = self.create_client(Trigger, '/safe_switch/to_torque')
        self.switch_to_jtc_client    = self.create_client(Trigger, '/safe_switch/to_jtc')
        self._switch_future = None

        # ---- FSM state ----
        self.state = None
        self._approach_command_sent  = False
        self._impedance_command_sent = False

        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.set_state(self.WAITING)
        self.pub_ik_enable.publish(Bool(data=False))

        self.get_logger().info("🧠 z1_FSM ready")
        self.get_logger().info(f"  torso_locked:      {self.torso_locked_topic}")
        self.get_logger().info(f"  ik_goal:           {self.ik_goal_topic}")
        self.get_logger().info(f"  ik_enable:         {self.ik_enable_topic}")
        self.get_logger().info(f"  ik_done:           {self.ik_done_topic}")
        self.get_logger().info(f"  impedance_enable:      {self.impedance_enable_topic}")
        self.get_logger().info(f"  impedance_done:        {self.impedance_done_topic}")
        self.get_logger().info(f"  out_of_workspace:      {self.target_out_of_workspace_topic}")

    # ================================================================== #
    #  CALLBACKS                                                           #
    # ================================================================== #

    def on_torso_locked(self, msg: PoseStamped):
        self.last_torso_pose = msg
        self.last_torso_time = self.get_clock().now()

    def on_ik_done(self, msg: Bool):
        if msg.data:
            self.ik_done = True

    def on_impedance_done(self, msg: Bool):
        if msg.data:
            self.impedance_done = True

    # ================================================================== #
    #  HELPERS                                                             #
    # ================================================================== #

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

    def torso_target_fresh(self) -> bool:
        if self.last_torso_pose is None or self.last_torso_time is None:
            return False
        age = (self.get_clock().now() - self.last_torso_time).nanoseconds * 1e-9
        return age <= self.target_max_age_s

    def _pose_to_np(self, pose: PoseStamped) -> np.ndarray:
        p = pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def _make_clipped_pose(self, original: PoseStamped, clipped_pos: np.ndarray) -> PoseStamped:
        """Costruisce un PoseStamped con posizione clippata ma orientazione invariata."""
        msg = PoseStamped()
        msg.header              = original.header
        msg.pose.orientation    = original.pose.orientation
        msg.pose.position.x     = float(clipped_pos[0])
        msg.pose.position.y     = float(clipped_pos[1])
        msg.pose.position.z     = float(clipped_pos[2])
        return msg

    # ================================================================== #
    #  FSM TICK (20 Hz)                                                    #
    # ================================================================== #

    def tick(self):

        # ── WAITING ───────────────────────────────────────────────────
        if self.state == self.WAITING:
            if self.torso_target_fresh():
                self.set_state(self.CHECKING_WORKSPACE)

        # ── CHECKING_WORKSPACE ────────────────────────────────────────
        elif self.state == self.CHECKING_WORKSPACE:

            # Avvia il calcolo: serve un target fresco per fare il latch
            if self._workspace_future is None:
                if not self.torso_target_fresh():
                    self.get_logger().warn("⚠️  Nessun target fresco per workspace check → WAITING")
                    self.set_state(self.WAITING)
                    return

                # Se il checker non è disponibile, salta il controllo e committa il target
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

            # Aspetta che il thread finisca (target già latchato: non si torna indietro)
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

        # ── APPROACHING ───────────────────────────────────────────────
        elif self.state == self.APPROACHING:
            # Nessun abort per target non fresco: il robot si impegna
            # sul target latchato in CHECKING_WORKSPACE

            if not self._approach_command_sent:
                self.ik_done = False

                # Usa il target clippato calcolato in CHECKING_WORKSPACE
                target = self._clipped_target if self._clipped_target is not None \
                         else self.last_torso_pose

                self.pub_ik_goal.publish(target)
                self.pub_ik_enable.publish(Bool(data=True))

                self._approach_command_sent = True
                self.set_state(self.WAIT_IK_DONE)

        # ── WAIT_IK_DONE ──────────────────────────────────────────────
        elif self.state == self.WAIT_IK_DONE:
            # Nessun abort per target non fresco: il robot completa il movimento
            # verso il target latchato indipendentemente dalla visibilità della persona

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.SWITCHING_TO_TORQUE)

        # ── SWITCHING_TO_TORQUE ───────────────────────────────────────
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

        # ── IMPEDANCE_RUNNING ─────────────────────────────────────────
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

        # ── SWITCHING_TO_JTC ──────────────────────────────────────────
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
                self.get_logger().info("✅ Switch torque_controller → JTC riuscito → WAITING")
                self.set_state(self.WAITING)
            else:
                self.get_logger().error(f"❌ Switch fallito: {result.message}")
                self.set_state(self.FAULT)

        # ── FAULT ─────────────────────────────────────────────────────
        elif self.state == self.FAULT:
            self.pub_ik_enable.publish(Bool(data=False))

        else:
            self.set_state(self.FAULT)


# ====================================================================== #
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
