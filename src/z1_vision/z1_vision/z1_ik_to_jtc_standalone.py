#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import pinocchio as pin

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from rclpy.duration import Duration
from tf_transformations import quaternion_matrix, quaternion_from_matrix


class Z1IKToJTC(Node):
    """\
    Z1 IK -> JointTrajectoryController (topic command) + FSM signals

    INPUT (from FSM):
      - /ik_goal_pose (PoseStamped)
      - /ik_enable    (Bool)

    OUTPUT (to FSM / debug):
      - /ik_done        (Bool)  [one-shot True when reached]
      - /ik_reached     (Bool)  [latched True when reached]
      - /ik_current_pose (PoseStamped)

    COMMAND (to ros2_control):
      - /joint_trajectory_controller/joint_trajectory (JointTrajectory)

    Notes:
      - Keeps the proven damped-least-squares IK core (Pinocchio log + LOCAL Jacobian + integrate).
      - Uses /joint_states to seed q (critical for convergence).
      - Runs a slow servo loop (micro-steps) to avoid big jumps / non-convergence.
    """

    def __init__(self):
        super().__init__("z1_ik_to_jtc")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter(
            "urdf_path",
            "/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf",
        )
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ee_frame", "link06")

        # IK
        self.declare_parameter("ik_max_iter", 100)
        self.declare_parameter("ik_tol", 1e-4)
        self.declare_parameter("ik_damping", 1e-3)

        # Servo
        self.declare_parameter("servo_rate", 10.0)       # Hz
        self.declare_parameter("traj_dt", 0.40)          # s
        self.declare_parameter("max_q_step", 0.008)      # rad per tick (clamp)
        self.declare_parameter("slowdown_distance", 0.10)  # m
        self.declare_parameter("min_step_scale", 0.15)

        # FSM interface
        self.declare_parameter("enable_topic", "/ik_enable")
        self.declare_parameter("done_topic", "/ik_done")
        self.declare_parameter("current_pose_topic", "/ik_current_pose")
        self.declare_parameter("reached_topic", "/ik_reached")
        self.declare_parameter("default_enabled", True)

        # Reached criteria
        self.declare_parameter("reach_pos_tol", 0.05)     # m
        self.declare_parameter("reach_hold_ticks", 5)

        # If True uses goal orientation; if False keeps current EE rotation (position-only IK)
        self.declare_parameter("use_goal_orientation", True)

        urdf_path = self.get_parameter("urdf_path").value
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)

        self.max_iter = int(self.get_parameter("ik_max_iter").value)
        self.tol = float(self.get_parameter("ik_tol").value)
        self.damping = float(self.get_parameter("ik_damping").value)

        self.servo_rate = float(self.get_parameter("servo_rate").value)
        self.traj_dt = float(self.get_parameter("traj_dt").value)
        self.max_q_step = float(self.get_parameter("max_q_step").value)
        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.min_step_scale = float(self.get_parameter("min_step_scale").value)

        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.done_topic = str(self.get_parameter("done_topic").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.reached_topic = str(self.get_parameter("reached_topic").value)
        self.enabled = bool(self.get_parameter("default_enabled").value)

        self.reach_pos_tol = float(self.get_parameter("reach_pos_tol").value)
        self.reach_hold_ticks = int(self.get_parameter("reach_hold_ticks").value)

        self.use_goal_orientation = bool(self.get_parameter("use_goal_orientation").value)

        # ---------------- PINOCCHIO MODEL ----------------
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(self.ee_frame)

        self.q = pin.neutral(self.model)

        # --- Joint mapping (Pinocchio q indices) ---
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_q_index = {}
        for jn in self.joint_names:
            jid = self.model.getJointId(jn)
            if jid == 0:
                raise RuntimeError(f"Joint '{jn}' not found in model")
            self.joint_q_index[jn] = int(self.model.joints[jid].idx_q)

        self.q_ready = False

        self.get_logger().info(
            f"✅ URDF caricato: {urdf_path} | nq={self.model.nq} nv={self.model.nv} | base_frame={self.base_frame} | ee_frame={self.ee_frame} (id={self.ee_id})"
        )
        self.get_logger().info(f"🔧 Joint q indices: {self.joint_q_index}")

        # ---------------- State ----------------
        self.goal_pose: PoseStamped | None = None
        self._reach_counter = 0
        self._done_sent = False

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(PoseStamped, "/ik_goal_pose", self.cb_goal_pose, 10)
        self.create_subscription(Bool, self.enable_topic, self.cb_enable, 10)
        self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 50)

        # ---------------- PUBLISHERS ----------------
        self.pub_cmd = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)
        self.pub_reached = self.create_publisher(Bool, self.reached_topic, 10)
        self.pub_current_pose = self.create_publisher(PoseStamped, self.current_pose_topic, 10)

        # Init outputs
        self.pub_done.publish(Bool(data=False))
        self.pub_reached.publish(Bool(data=False))

        # Servo timer
        self.dt = 1.0 / max(self.servo_rate, 1e-6)
        self.timer = self.create_timer(self.dt, self.servo_tick)

        self.get_logger().info(
            f"🦾 Servo IK attivo: rate={self.servo_rate:.1f}Hz, traj_dt={self.traj_dt:.2f}s, max_q_step={self.max_q_step:.4f} rad"
        )

    # ==========================================================
    # Joint state sync (seed)
    # ==========================================================
    def cb_joint_states(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            for jn in self.joint_names:
                if jn not in name_to_idx:
                    return
                iq = self.joint_q_index[jn]
                self.q[iq] = float(msg.position[name_to_idx[jn]])
            self.q_ready = True
        except Exception:
            return

    # ==========================================================
    # FSM I/O
    # ==========================================================
    def cb_goal_pose(self, msg: PoseStamped):
        self.goal_pose = msg
        self._reach_counter = 0
        if self._done_sent:
            self._done_sent = False
            self.pub_done.publish(Bool(data=False))
        self.pub_reached.publish(Bool(data=False))

    def cb_enable(self, msg: Bool):
        new_enabled = bool(msg.data)
        if new_enabled != self.enabled:
            self.enabled = new_enabled
            self._reach_counter = 0
            self._done_sent = False
            self.pub_done.publish(Bool(data=False))
            self.pub_reached.publish(Bool(data=False))
            self.get_logger().info("🟦 IK ENABLED" if self.enabled else "🟧 IK DISABLED")

    # ==========================================================
    # IK core (proven)
    # ==========================================================
    def _pose_to_se3(self, pose_msg: PoseStamped, current_rotation=None) -> pin.SE3 | None:
        """Convert PoseStamped to Pinocchio SE3.

        If use_goal_orientation=False, uses current_rotation (3x3) for rotation and only goal position.
        """
        p = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ], dtype=float)

        if not self.use_goal_orientation:
            if current_rotation is None:
                return None
            return pin.SE3(current_rotation, p)

        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w
        n = float(np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw))
        if n < 1e-12:
            return None
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

        T = quaternion_matrix([qx, qy, qz, qw])
        T[:3, 3] = p
        return pin.SE3(T[:3, :3], T[:3, 3])

    def solve_ik(self, target_SE3: pin.SE3, q_init: np.ndarray) -> np.ndarray | None:
        q = q_init.copy()

        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[self.ee_id]
            err = pin.log(current.inverse() * target_SE3).vector

            if np.linalg.norm(err) < self.tol:
                return q

            J = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_id,
                pin.ReferenceFrame.LOCAL,
            )

            JJt = J @ J.T + self.damping * np.eye(6)
            v = J.T @ np.linalg.solve(JJt, err)
            q = pin.integrate(self.model, q, v)

        return None

    # ==========================================================
    # Servo loop
    # ==========================================================
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

        # Current EE pose
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        current = self.data.oMf[self.ee_id]

        target = self._pose_to_se3(self.goal_pose, current_rotation=current.rotation)
        if target is None:
            self.get_logger().warn("Goal pose non valida (quat nullo?)", throttle_duration_sec=1.0)
            return

        q_sol = self.solve_ik(target, self.q)
        if q_sol is None:
            self.get_logger().warn("⚠️ IK non converge (tick)", throttle_duration_sec=1.0)
            return

        # Publish current EE pose (world/base frame)
        Tcw = np.eye(4)
        Tcw[:3, :3] = current.rotation
        Tcw[:3, 3] = current.translation
        qxyzw = quaternion_from_matrix(Tcw)

        cur_msg = PoseStamped()
        cur_msg.header.stamp = self.get_clock().now().to_msg()
        cur_msg.header.frame_id = self.base_frame
        cur_msg.pose.position.x = float(current.translation[0])
        cur_msg.pose.position.y = float(current.translation[1])
        cur_msg.pose.position.z = float(current.translation[2])
        cur_msg.pose.orientation.x = float(qxyzw[0])
        cur_msg.pose.orientation.y = float(qxyzw[1])
        cur_msg.pose.orientation.z = float(qxyzw[2])
        cur_msg.pose.orientation.w = float(qxyzw[3])
        self.pub_current_pose.publish(cur_msg)

        # Position error (always)
        pos_err = float(np.linalg.norm(current.translation - target.translation))
        s = float(np.clip(pos_err / max(self.slowdown_distance, 1e-6), self.min_step_scale, 1.0))

        # Reached criteria
        if pos_err <= self.reach_pos_tol:
            self._reach_counter += 1
        else:
            self._reach_counter = 0

        if (self._reach_counter >= self.reach_hold_ticks) and (not self._done_sent):
            self._done_sent = True
            self.pub_reached.publish(Bool(data=True))
            self.pub_done.publish(Bool(data=True))
            self.pub_current_pose.publish(cur_msg)
            self.get_logger().info(f"✅ IK DONE (pos_err={pos_err:.4f} m)")
            return

        # Micro-step towards solution
        dq = pin.difference(self.model, self.q, q_sol)
        dq = np.clip(dq, -self.max_q_step, self.max_q_step) * s
        self.q = pin.integrate(self.model, self.q, dq)

        # Clamp to model limits (safety)
        self.q = np.minimum(np.maximum(self.q, self.model.lowerPositionLimit), self.model.upperPositionLimit)

        # Publish command
        self.publish_jtc(self.q)

    def publish_jtc(self, q_cmd: np.ndarray):
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(q_cmd[self.joint_q_index[jn]]) for jn in self.joint_names]
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