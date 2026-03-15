#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


class Z1FSM(Node):
    WAITING             = "WAITING"
    APPROACHING         = "APPROACHING"
    WAIT_IK_DONE        = "WAIT_IK_DONE"
    SWITCHING_TO_TORQUE = "SWITCHING_TO_TORQUE"
    IMPEDANCE_RUNNING   = "IMPEDANCE_RUNNING"
    SWITCHING_TO_JTC    = "SWITCHING_TO_JTC"
    FAULT               = "FAULT"

    def __init__(self):
        super().__init__("z1_fsm")

        # ---- Params (topic names) ----
        self.declare_parameter("torso_locked_topic", "/torso_target_ee_locked")

        self.declare_parameter("ik_enable_topic", "/ik_enable")
        self.declare_parameter("ik_goal_topic",   "/ik_goal_pose")
        self.declare_parameter("ik_done_topic",   "/ik_done")

        self.declare_parameter("state_topic",      "/z1_fsm/state")
        self.declare_parameter("target_max_age_s", 0.5)

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

        # ---- Subscribers ----
        self.last_torso_pose: PoseStamped | None = None
        self.last_torso_time = None
        self.ik_done         = False
        self.impedance_done  = False

        self.create_subscription(PoseStamped, self.torso_locked_topic, self.on_torso_locked, 10)
        self.create_subscription(Bool, self.ik_done_topic,         self.on_ik_done,         10)
        self.create_subscription(Bool, self.impedance_done_topic,  self.on_impedance_done,  10)

        # ---- Publishers ----
        self.pub_ik_enable        = self.create_publisher(Bool,        self.ik_enable_topic,        10)
        self.pub_ik_goal          = self.create_publisher(PoseStamped, self.ik_goal_topic,           10)
        self.pub_state            = self.create_publisher(String,      self.state_topic,             10)
        self.pub_impedance_enable = self.create_publisher(Bool,        self.impedance_enable_topic,  10)

        # ---- Service clients: switch controller ----
        self.switch_to_torque_client = self.create_client(Trigger, '/safe_switch/to_torque')
        self.switch_to_jtc_client    = self.create_client(Trigger, '/safe_switch/to_jtc')
        self._switch_future = None

        # ---- FSM state ----
        self.state = None
        self._approach_command_sent      = False
        self._impedance_command_sent     = False

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.set_state(self.WAITING)

        self.pub_ik_enable.publish(Bool(data=False))

        self.get_logger().info("🧠 z1_FSM ready")
        self.get_logger().info(f"  torso_locked:      {self.torso_locked_topic}")
        self.get_logger().info(f"  ik_goal:           {self.ik_goal_topic}")
        self.get_logger().info(f"  ik_enable:         {self.ik_enable_topic}")
        self.get_logger().info(f"  ik_done:           {self.ik_done_topic}")
        self.get_logger().info(f"  impedance_enable:  {self.impedance_enable_topic}")
        self.get_logger().info(f"  impedance_done:    {self.impedance_done_topic}")

    # -------- Callbacks --------
    def on_torso_locked(self, msg: PoseStamped):
        self.last_torso_pose = msg
        self.last_torso_time = self.get_clock().now()

    def on_ik_done(self, msg: Bool):
        if msg.data:
            self.ik_done = True

    def on_impedance_done(self, msg: Bool):
        if msg.data:
            self.impedance_done = True

    # -------- Helpers --------
    def set_state(self, s: str):
        if s == self.state:
            return

        self.state = s
        self.pub_state.publish(String(data=s))
        self.get_logger().info(f"➡️  FSM state = {s}")

        if s == self.WAITING:
            self.ik_done = False
            self._approach_command_sent = False
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

    # -------- FSM tick --------
    def tick(self):
        # ── WAITING ───────────────────────────────────────────────
        if self.state == self.WAITING:
            if self.torso_target_fresh():
                self.set_state(self.APPROACHING)

        # ── APPROACHING ───────────────────────────────────────────
        elif self.state == self.APPROACHING:
            if not self.torso_target_fresh():
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.WAITING)
                return

            if not self._approach_command_sent:
                self.ik_done = False
                target = self.last_torso_pose

                self.pub_ik_goal.publish(target)
                self.pub_ik_enable.publish(Bool(data=True))

                self._approach_command_sent = True
                self.set_state(self.WAIT_IK_DONE)

        # ── WAIT_IK_DONE ──────────────────────────────────────────
        elif self.state == self.WAIT_IK_DONE:
            if not self.torso_target_fresh():
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.WAITING)
                return

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.SWITCHING_TO_TORQUE)

        # ── SWITCHING_TO_TORQUE ───────────────────────────────────
        elif self.state == self.SWITCHING_TO_TORQUE:
            # Avvia la chiamata al servizio (una sola volta)
            if self._switch_future is None:
                if not self.switch_to_torque_client.service_is_ready():
                    self.get_logger().warn(
                        '⏳ /safe_switch/to_torque non ancora disponibile...',
                        throttle_duration_sec=2.0
                    )
                    return
                self.get_logger().info('📞 Chiamata /safe_switch/to_torque')
                self._switch_future = self.switch_to_torque_client.call_async(Trigger.Request())
                return

            # Aspetta il risultato
            if not self._switch_future.done():
                return

            result = self._switch_future.result()
            if result.success:
                self.get_logger().info('✅ Switch JTC → torque_controller riuscito')
                self.set_state(self.IMPEDANCE_RUNNING)
            else:
                self.get_logger().error(f'❌ Switch fallito: {result.message}')
                self.set_state(self.FAULT)

        # ── IMPEDANCE_RUNNING ─────────────────────────────────────
        elif self.state == self.IMPEDANCE_RUNNING:
            # Invia enable una sola volta
            if not self._impedance_command_sent:
                self.pub_impedance_enable.publish(Bool(data=True))
                self._impedance_command_sent = True
                self.get_logger().info('🦾 Impedance controller avviato')
                return

            # Aspetta che l'impedance controller segnali fine
            if self.impedance_done:
                self.pub_impedance_enable.publish(Bool(data=False))
                self.get_logger().info('✅ Impedance done → SWITCHING_TO_JTC')
                self.set_state(self.SWITCHING_TO_JTC)

        # ── SWITCHING_TO_JTC ──────────────────────────────────────
        elif self.state == self.SWITCHING_TO_JTC:
            if self._switch_future is None:
                if not self.switch_to_jtc_client.service_is_ready():
                    self.get_logger().warn(
                        '⏳ /safe_switch/to_jtc non ancora disponibile...',
                        throttle_duration_sec=2.0
                    )
                    return
                self.get_logger().info('📞 Chiamata /safe_switch/to_jtc')
                self._switch_future = self.switch_to_jtc_client.call_async(Trigger.Request())
                return

            if not self._switch_future.done():
                return

            result = self._switch_future.result()
            if result.success:
                self.get_logger().info('✅ Switch torque_controller → JTC riuscito → WAITING')
                self.set_state(self.WAITING)
            else:
                self.get_logger().error(f'❌ Switch fallito: {result.message}')
                self.set_state(self.FAULT)

        # ── FAULT ─────────────────────────────────────────────────
        elif self.state == self.FAULT:
            self.pub_ik_enable.publish(Bool(data=False))

        else:
            self.set_state(self.FAULT)


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
