#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.duration import Duration
from tf_transformations import quaternion_from_matrix

from sensor_msgs.msg import JointState


class Z1IKToJTC(Node):
    """
    Servo IK -> JointTrajectoryController (topic command)

    Input:
      - /ik_goal_pose (PoseStamped)   [target pose, published by FSM]
      - /ik_enable    (Bool)          [FSM enable/start approach]


    Output:
      - /ik_done (Bool)               [True when target reached]
      - /joint_trajectory_controller/joint_trajectory (JointTrajectory)
        con joint1..joint6 (q_target[:6])
        - /ik_current_pose (PoseStamped)
        - /ik_reached (Bool)
    """

    def __init__(self):
        super().__init__("z1_ik_to_jtc")

        # ---------------- Params ----------------
        self.declare_parameter(
            "urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf",
        )
        self.declare_parameter("ee_frame", "link06")

        # Servo
        self.declare_parameter("servo_rate", 10.0)     # Hz (lento e stabile)
        self.declare_parameter("traj_dt", 0.40)        # durata del micro-move (s)
        self.declare_parameter("max_q_step", 0.008)     # rad per step (clamp)

        # IK
        self.declare_parameter("ik_max_iter", 80)
        self.declare_parameter("ik_tol", 1e-4)
        self.declare_parameter("ik_damping", 1e-3)

        self.declare_parameter("slowdown_distance", 0.10)  # m
        self.declare_parameter("min_step_scale", 0.15)     # non scendere sotto questo fattore

        # FSM interface
        self.declare_parameter("enable_topic", "/ik_enable")
        self.declare_parameter("done_topic", "/ik_done")
        self.declare_parameter("default_enabled", True)  # per non rompere il comportamento vecchio
        
        # Reached criteria
        self.declare_parameter("reach_pos_tol", 0.05)    # m (5 cm) 
        self.declare_parameter("reach_hold_ticks", 5)   # numero di tick consecutivi per confermare il raggiungimento

        self.declare_parameter("current_pose_topic", "/ik_current_pose")
        self.declare_parameter("reached_topic", "/ik_reached")


        urdf_path = self.get_parameter("urdf_path").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.servo_rate = float(self.get_parameter("servo_rate").value)
        self.traj_dt = float(self.get_parameter("traj_dt").value)
        self.max_q_step = float(self.get_parameter("max_q_step").value)

        self.ik_max_iter = int(self.get_parameter("ik_max_iter").value)
        self.ik_tol = float(self.get_parameter("ik_tol").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)

        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.min_step_scale = float(self.get_parameter("min_step_scale").value)


        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.done_topic = str(self.get_parameter("done_topic").value)
        self.default_enabled = bool(self.get_parameter("default_enabled").value)

        self.reach_pos_tol = float(self.get_parameter("reach_pos_tol").value)
        self.reach_hold_ticks = int(self.get_parameter("reach_hold_ticks").value)

        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.reached_topic = str(self.get_parameter("reached_topic").value)

        # ---------------- Pinocchio ----------------
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(self.ee_frame)

        self.q = pin.neutral(self.model)
        # ---- Sync internal q from Gazebo joint_states (keeps IK consistent with the real robot state)
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        self.q_ready = False

        self.get_logger().info(
            f"✅ URDF caricato: {urdf_path} | nq={self.model.nq} | ee_frame={self.ee_frame} (id={self.ee_id})"
        )

        # ---------------- State ----------------
        self.goal_pose: PoseStamped | None = None

        # FSM interface state
        self.enabled = bool(self.default_enabled)
        self._reach_counter = 0
        self._done_sent = False

        # ---------------- I/O ----------------
        self.create_subscription(PoseStamped, "/ik_goal_pose", self.cb_goal_pose, 10)
        self.create_subscription(Bool, self.enable_topic, self.cb_enable, 10)
        self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 10)
    def cb_joint_states(self, msg: JointState):
        # Keep the IK internal state aligned with what Gazebo/ros2_control is actually executing.
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            for k, jn in enumerate(self.joint_names):
                self.q[k] = float(msg.position[name_to_idx[jn]])
            self.q_ready = True
        except Exception:
            # If names don't match yet, just ignore.
            return

        # Topic command del JointTrajectoryController
        self.pub_cmd = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)
        self.pub_done.publish(Bool(data=False))  # init       

        self.pub_current_pose = self.create_publisher(PoseStamped, self.current_pose_topic, 10)
        self.pub_reached = self.create_publisher(Bool, self.reached_topic, 10)
        self.pub_reached.publish(Bool(data=False))  # init

        # Servo timer
        self.dt = 1.0 / self.servo_rate
        self.timer = self.create_timer(self.dt, self.servo_tick)

        self.get_logger().info(
            f"🦾 Servo IK attivo: rate={self.servo_rate:.1f}Hz, traj_dt={self.traj_dt:.2f}s, joints=joint1..joint6"
        )


    def cb_goal_pose(self, msg: PoseStamped):
        self.goal_pose = msg
        # New target => reset done
        self._reach_counter = 0
        if self._done_sent:
            self._done_sent = False
            self.pub_done.publish(Bool(data=False))
        self.pub_reached.publish(Bool(data=False))

    def cb_enable(self, msg: Bool):
        new_enabled = bool(msg.data)
        if new_enabled != self.enabled:
            self.enabled = new_enabled
            # Reset done bookkeeping on transitions
            self._reach_counter = 0
            self._done_sent = False
            self.pub_done.publish(Bool(data=False))
            self.pub_reached.publish(Bool(data=False))
            if self.enabled:
                self.get_logger().info("🟦 IK ENABLED")
            else:
                self.get_logger().info("🟧 IK DISABLED")

    # ---------------- IK ----------------
    def solve_ik(self, target_SE3: pin.SE3, q_init: np.ndarray) -> np.ndarray | None:
        q = q_init.copy()

        for _ in range(self.ik_max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            err = pin.log(current.inverse() * target_SE3).vector  # 6D

            if np.linalg.norm(err) < self.ik_tol:
                return q

            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL
            )
            JJt = J @ J.T + self.ik_damping * np.eye(6)
            v = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self.model, q, v)

        return None

    # ---------------- Servo loop ----------------
    def servo_tick(self):
        if not self.enabled:
            return
        
        if self.goal_pose is None:
            return

        if self._done_sent:
            return

        if not self.q_ready:
            self.get_logger().warn("Aspetto /joint_states per inizializzare q", throttle_duration_sec=2.0)
            return
        # FK current (serve per prendere la rotazione attuale dell'EE)
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        current = self.data.oMf[self.ee_id]

        # Target POSITION dal goal (ignoro l'orientamento del goal)
        target_pos = np.array([
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y,
            self.goal_pose.pose.position.z
        ], dtype=float)

        # Target SE3: stessa rotazione attuale -> IK "position-only"
        target = pin.SE3(current.rotation, target_pos)

        q_sol = self.solve_ik(target, self.q)
        if q_sol is None:
            self.get_logger().warn("⚠️ IK non converge (tick)", throttle_duration_sec=1.0)
            return

        # (opzionale) aggiorna current dopo solve, ma non è obbligatorio
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        current = self.data.oMf[self.ee_id]

        # Publish current EE pose (world frame)
        Tcw = np.eye(4)
        Tcw[:3, :3] = current.rotation
        Tcw[:3, 3] = current.translation
        qxyzw = quaternion_from_matrix(Tcw)

        cur_msg = PoseStamped()
        cur_msg.header.stamp = self.get_clock().now().to_msg()
        cur_msg.header.frame_id = "link00"
        cur_msg.pose.position.x = float(current.translation[0])
        cur_msg.pose.position.y = float(current.translation[1])
        cur_msg.pose.position.z = float(current.translation[2])
        cur_msg.pose.orientation.x = float(qxyzw[0])
        cur_msg.pose.orientation.y = float(qxyzw[1])
        cur_msg.pose.orientation.z = float(qxyzw[2])
        cur_msg.pose.orientation.w = float(qxyzw[3])
        self.pub_current_pose.publish(cur_msg)

        pos_err = np.linalg.norm(current.translation - target.translation)
        s = np.clip(pos_err / self.slowdown_distance, self.min_step_scale, 1.0)

        # Reached criteria (position only)
        if pos_err <= self.reach_pos_tol:
            self._reach_counter += 1
        else:
            self._reach_counter = 0

        if (self._reach_counter >= self.reach_hold_ticks) and (not self._done_sent):
            self._done_sent = True

            # reached = True (0/1)
            self.pub_reached.publish(Bool(data=True))

            # done = True (evento fine)
            self.pub_done.publish(Bool(data=True))

            # publish current pose at finish (già pubblicata sopra, ma ripubblico per sicurezza)
            self.pub_current_pose.publish(cur_msg)

            self.get_logger().info(f"✅ IK DONE (pos_err={pos_err:.4f} m)")

        # Clamp step (per muoversi lentamente e senza scatti)
        dq = pin.difference(self.model, self.q, q_sol)
        dq = np.clip(dq, -self.max_q_step, self.max_q_step)
        dq = dq * s
        self.q = pin.integrate(self.model, self.q, dq)
        # Clamp to model joint limits (helps prevent divergence / desync)
        self.q = np.minimum(np.maximum(self.q, self.model.lowerPositionLimit), self.model.upperPositionLimit)

        # Pubblica comando al controller SOLO joint1..joint6
        self.publish_jtc(self.q)

    def publish_jtc(self, q_cmd: np.ndarray):
        msg = JointTrajectory()
        msg.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]

        pt = JointTrajectoryPoint()
        pt.positions = q_cmd[:6].tolist() 
        pt.time_from_start = Duration(seconds=self.traj_dt).to_msg()

        msg.points = [pt]
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Z1IKToJTC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()