#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped


class Z1FSM(Node):
    WAITING = "WAITING"
    APPROACHING = "APPROACHING"
    WAIT_IK_DONE = "WAIT_IK_DONE"
    FAULT = "FAULT"

    def __init__(self):
        super().__init__("z1_fsm")

        # ---- Params (topic names) ----
        # Torso target: pubblicato SOLO quando il tracker è LOCKED
        self.declare_parameter("torso_locked_topic", "/torso_target_ee_locked")

        # IK interface (nuova)
        self.declare_parameter("ik_enable_topic", "/ik_enable")
        self.declare_parameter("ik_goal_topic", "/ik_goal_pose")
        self.declare_parameter("ik_done_topic", "/ik_done")

        self.declare_parameter("state_topic", "/z1_fsm/state")
        self.declare_parameter("target_max_age_s", 0.5)  # target torso “fresco”

        self.torso_locked_topic = self.get_parameter("torso_locked_topic").value
        self.ik_enable_topic = self.get_parameter("ik_enable_topic").value
        self.ik_goal_topic = self.get_parameter("ik_goal_topic").value
        self.ik_done_topic = self.get_parameter("ik_done_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.target_max_age_s = float(self.get_parameter("target_max_age_s").value)

        # ---- Subscribers ----
        self.last_torso_pose: PoseStamped | None = None
        self.last_torso_time = None
        self.ik_done = False

        self.create_subscription(PoseStamped, self.torso_locked_topic, self.on_torso_locked, 10)
        self.create_subscription(Bool, self.ik_done_topic, self.on_ik_done, 10)

        # ---- Publishers ----
        self.pub_ik_enable = self.create_publisher(Bool, self.ik_enable_topic, 10)
        self.pub_ik_goal = self.create_publisher(PoseStamped, self.ik_goal_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)

        # ---- FSM state ----
        self.state = None
        self._approach_command_sent = False

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.set_state(self.WAITING)

        # ensure IK disabled at startup
        self.pub_ik_enable.publish(Bool(data=False))

        self.get_logger().info("🧠 z1_FSM ready")
        self.get_logger().info(f"  torso_locked: {self.torso_locked_topic}")
        self.get_logger().info(f"  ik_goal:      {self.ik_goal_topic}")
        self.get_logger().info(f"  ik_enable:    {self.ik_enable_topic}")
        self.get_logger().info(f"  ik_done:      {self.ik_done_topic}")

    # -------- Callbacks --------
    def on_torso_locked(self, msg: PoseStamped):
        self.last_torso_pose = msg
        self.last_torso_time = self.get_clock().now()

    def on_ik_done(self, msg: Bool):
        if msg.data:
            self.ik_done = True

    # -------- Helpers --------
    def set_state(self, s: str):
        if s == self.state:
            return

        self.state = s
        self.pub_state.publish(String(data=s))
        self.get_logger().info(f"➡️ FSM state = {s}")

        # reset state-entry flags
        if s == self.WAITING:
            self.ik_done = False
            self._approach_command_sent = False
        if s == self.APPROACHING:
            self._approach_command_sent = False

    def torso_target_fresh(self) -> bool:
        if self.last_torso_pose is None or self.last_torso_time is None:
            return False
        age = (self.get_clock().now() - self.last_torso_time).nanoseconds * 1e-9
        return age <= self.target_max_age_s

    # -------- FSM tick --------
    def tick(self):
        if self.state == self.WAITING:
            # Condizione: target locked disponibile (pose fresca)
            if self.torso_target_fresh():
                self.set_state(self.APPROACHING)

        elif self.state == self.APPROACHING:
            # Se perdiamo il target prima di comandare, torna WAITING
            if not self.torso_target_fresh():
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.WAITING)
                return

            # Invia una sola volta goal + enable
            if not self._approach_command_sent:
                self.ik_done = False
                target = self.last_torso_pose  # PoseStamped in world (locked)

                self.pub_ik_goal.publish(target)
                self.pub_ik_enable.publish(Bool(data=True))

                self._approach_command_sent = True
                self.set_state(self.WAIT_IK_DONE)

        elif self.state == self.WAIT_IK_DONE:
            # Safety: se il target smette di arrivare (non fresco), fermati e torna waiting
            if not self.torso_target_fresh():
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.WAITING)
                return

            if self.ik_done:
                self.pub_ik_enable.publish(Bool(data=False))
                self.set_state(self.WAITING)

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