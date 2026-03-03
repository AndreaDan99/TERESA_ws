#!/usr/bin/env python3
"""
Z1 High-Level FSM (pulita + refresh->WAITING)

Stati:
- WAITING
- APPROACHING_JTC
- WAIT_JTC
- SWITCH_TO_TORQUE
- IMPEDANCE_CONTACT
- HOLD_CONTACT
- IMPEDANCE_RETRACT
- SWITCH_TO_JTC
- RETURN_HOME_JTC
- WAIT_RETURN
- FAULT
"""

import rclpy
from rclpy.node import Node
import numpy as np
import subprocess

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from tf_transformations import quaternion_matrix


class Z1FSM(Node):

    def __init__(self):
        super().__init__('z1_FSM')

        # ================= PARAMETERS =================
        self.declare_parameter('approach_offset', 0.20)
        self.declare_parameter('pre_contact_normal_offset', -0.205)
        self.declare_parameter('use_surface_for_approach', False)

        self.declare_parameter('home_position', [0.0755, 0.070, 0.445])
        self.declare_parameter('home_orientation', [-0.0170, 0.2940, 0.0442, 0.9545])

        self.declare_parameter('startup_go_home', True)
        self.declare_parameter('startup_home_delay', 5.0)

        self.declare_parameter('hold_time', 5.0)
        self.declare_parameter('retract_timeout', 6.0)

        self.declare_parameter('jtc_timeout', 25.0)
        self.declare_parameter('switch_timeout', 3.0)

        # Re-publish IK goal while waiting (robust)
        self.declare_parameter('ik_goal_republish_rate', 5.0)  # Hz

        # >>> NEW: refresh / replan threshold (m)
        self.declare_parameter('refresh_replan_dist_thr', 0.15)

        self.declare_parameter('enable_goal_republish', False)

        # ================= READ PARAMETERS =================
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.pre_contact_normal_offset = float(self.get_parameter('pre_contact_normal_offset').value)
        self.use_surface_for_approach = bool(self.get_parameter('use_surface_for_approach').value)

        self.home_position = np.array(self.get_parameter('home_position').value, dtype=np.float64)
        self.home_orientation = self.get_parameter('home_orientation').value

        self.startup_go_home = bool(self.get_parameter('startup_go_home').value)
        self.startup_home_delay = float(self.get_parameter('startup_home_delay').value)

        self.hold_time = float(self.get_parameter('hold_time').value)
        self.retract_timeout = float(self.get_parameter('retract_timeout').value)

        self.jtc_timeout = float(self.get_parameter('jtc_timeout').value)
        self.switch_timeout = float(self.get_parameter('switch_timeout').value)

        self.ik_goal_republish_rate = float(self.get_parameter('ik_goal_republish_rate').value)
        if self.ik_goal_republish_rate <= 0.0:
            self.ik_goal_republish_rate = 5.0

        self.refresh_replan_dist_thr = float(self.get_parameter('refresh_replan_dist_thr').value)

        self.enable_goal_republish = bool(self.get_parameter('enable_goal_republish').value)
        # ================= SUBSCRIBERS =================
        # Target lockato dal tracker (aggiornato quando “refresh/relock”)
        self.sub_target = self.create_subscription(
            PoseStamped, '/torso_target_ee_locked', self.cb_target, 10)

        # Frame superficie (pos+orientazione), se usi approach su normale
        self.sub_surface = self.create_subscription(
            PoseStamped, '/torso_surface_frame', self.cb_surface, 10)

        self.sub_lock_valid = self.create_subscription(
            Bool, '/target_lock_valid', self.cb_lock_valid, 10)

        self.sub_contact = self.create_subscription(
            Bool, '/impedance_contact_detected', self.cb_contact, 10)

        self.sub_ik_done = self.create_subscription(
            Bool, '/ik_jtc/done', self.cb_ik_done, 10)
        self.sub_ik_success = self.create_subscription(
            Bool, '/ik_jtc/success', self.cb_ik_success, 10)

        self.sub_retract_done = self.create_subscription(
            Bool, '/impedance_retract_done', self.cb_retract_done, 10
        )

        # ================= PUBLISHERS =================
        self.pub_ik_goal = self.create_publisher(PoseStamped, '/ik_goal_pose', 10)
        self.pub_state = self.create_publisher(String, '/torso_sm_state', 10)

        # ================= STATE =================
        self.state = 'WAITING'
        self.start_time = self.get_clock().now()
        self.state_start_time = self.get_clock().now()

        self.startup_home_sent = False

        self.target_world = None
        self.surface_frame = None
        self.target_lock_valid = False

        self.contact_detected = False
        self.ik_done = False
        self.ik_success = False

        self.retract_done = False

        self._fault_logged = False

        # Last IK goal for republish
        self.last_ik_goal_pose = None  # type: PoseStamped | None
        self._last_goal_pub_time = self.get_clock().now()

        # >>> NEW: store “source” used to compute the current goal
        # If surface approach: store surface position used
        # Else: store target position used
        self._goal_source_pos = None  # np.array(3,) or None

        self.timer = self.create_timer(1.0 / 30.0, self.step)
        self.get_logger().info('🧠 Z1 FSM READY (refresh->WAITING enabled)')

    # ================= CALLBACKS =================
    def cb_target(self, msg: PoseStamped):
        self.target_world = msg

    def cb_surface(self, msg: PoseStamped):
        self.surface_frame = msg

    def cb_lock_valid(self, msg: Bool):
        self.target_lock_valid = bool(msg.data)

    def cb_contact(self, msg: Bool):
        self.contact_detected = bool(msg.data)

    def cb_ik_done(self, msg: Bool):
        self.ik_done = bool(msg.data)

    def cb_ik_success(self, msg: Bool):
        self.ik_success = bool(msg.data)

    def cb_retract_done(self, msg: Bool):
        self.retract_done = bool(msg.data)

    # ================= UTIL =================
    def publish_state(self):
        self.pub_state.publish(String(data=self.state))

    def publish_ik_goal(self, pose_msg: PoseStamped, source_pos: np.ndarray | None):
        # Store last goal so we can re-publish it while waiting
        self.last_ik_goal_pose = pose_msg
        self._goal_source_pos = None if source_pos is None else source_pos.copy()
        self.pub_ik_goal.publish(pose_msg)
        self._last_goal_pub_time = self.get_clock().now()

    def _maybe_republish_goal(self, now):
        if not self.enable_goal_republish:
            return
        """Re-publish the last IK goal at a fixed rate while waiting for completion."""
        if self.last_ik_goal_pose is None:
            return
        if self.state not in ['WAIT_JTC', 'WAIT_RETURN']:
            return
        if self.ik_done:
            return

        dt = (now - self._last_goal_pub_time).nanoseconds / 1e9
        period = 1.0 / float(self.ik_goal_republish_rate)
        if dt >= period:
            self.pub_ik_goal.publish(self.last_ik_goal_pose)
            self._last_goal_pub_time = now

    def switch_controller(self, direction) -> bool:
        """Blocking controller switch. Returns True if switch succeeded."""
        try:
            cp = subprocess.run(
                [
                    'ros2', 'run', 'z1_vision', 'safe_controller_switch',
                    '--ros-args', '-p', f'switch_direction:={direction}'
                ],
                capture_output=True,
                text=True
            )
            if cp.returncode != 0:
                self.get_logger().error(f"❌ Switch {direction} fallito (code={cp.returncode})")
                if cp.stdout:
                    self.get_logger().error(f"stdout: {cp.stdout.strip()}")
                if cp.stderr:
                    self.get_logger().error(f"stderr: {cp.stderr.strip()}")
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Errore eseguendo switch {direction}: {e}")
            return False

    def enter_state(self, new_state: str):
        self.get_logger().info(f"🔄 {self.state} → {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()

        # Reset flags where it matters
        if new_state in ['WAITING']:
            self.last_ik_goal_pose = None
            self._goal_source_pos = None
            self.ik_done = False
            self.ik_success = False

        if new_state in ['RETURN_HOME_JTC', 'APPROACHING_JTC']:
            self.ik_done = False
            self.ik_success = False

        if new_state == 'FAULT':
            self._fault_logged = False

    def _current_source_pos(self) -> np.ndarray | None:
        """Return current target/surface position used for refresh check."""
        if self.use_surface_for_approach:
            if self.surface_frame is None:
                return None
            p = self.surface_frame.pose.position
            return np.array([p.x, p.y, p.z], dtype=np.float64)
        else:
            if self.target_world is None:
                return None
            p = self.target_world.pose.position
            return np.array([p.x, p.y, p.z], dtype=np.float64)

    def _refresh_should_replan(self) -> bool:
        """True if the target/surface moved enough from the source used to compute the current goal."""
        if self._goal_source_pos is None:
            return False
        cur = self._current_source_pos()
        if cur is None:
            return False
        dist = float(np.linalg.norm(cur - self._goal_source_pos))
        return dist >= self.refresh_replan_dist_thr

    # ================= MAIN FSM =================
    def step(self):
        now = self.get_clock().now()

        # Always republish goal (robustness) when waiting for JTC
        self._maybe_republish_goal(now)

        # ================= WAITING =================
        if self.state == 'WAITING':
            # Startup: manda HOME una sola volta dopo un piccolo delay
            if self.startup_go_home and (not self.startup_home_sent):
                startup_elapsed = (now - self.start_time).nanoseconds / 1e9
                if startup_elapsed >= self.startup_home_delay:
                    self.startup_home_sent = True
                    self.enter_state('RETURN_HOME_JTC')
                    self.publish_state()
                    return

            # proceed only when inputs are available
            if self.use_surface_for_approach:
                if self.target_lock_valid and (self.surface_frame is not None):
                    self.enter_state('APPROACHING_JTC')
            else:
                if self.target_lock_valid and (self.target_world is not None):
                    self.enter_state('APPROACHING_JTC')

        # ================= APPROACHING_JTC =================
        elif self.state == 'APPROACHING_JTC':
            # If lock lost -> go waiting
            if not self.target_lock_valid:
                self.enter_state('WAITING')
                self.publish_state()
                return

            pose = PoseStamped()

            if self.use_surface_for_approach and self.surface_frame is not None:
                # Use surface orientation (normal = Z axis of surface frame)
                pose.header = self.surface_frame.header
                pose.header.stamp = now.to_msg()
                pose.pose = self.surface_frame.pose

                q = pose.pose.orientation
                T = quaternion_matrix([q.x, q.y, q.z, q.w])
                R = T[:3, :3]
                n = R[:, 2]

                p = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ], dtype=np.float64)

                p_goal = p + self.pre_contact_normal_offset * n
                pose.pose.position.x = float(p_goal[0])
                pose.pose.position.y = float(p_goal[1])
                pose.pose.position.z = float(p_goal[2])

                source_pos = p  # store surface position used for refresh
            else:
                # Fallback: old logic (offset on -Z world)
                if self.target_world is None:
                    self.publish_state()
                    return
                pose.header = self.target_world.header
                pose.header.stamp = now.to_msg()
                pose.pose = self.target_world.pose
                pose.pose.position.z -= self.approach_offset

                p = pose.pose.position
                source_pos = np.array([p.x, p.y, p.z], dtype=np.float64)

            # publish goal and go wait
            self._last_goal_pub_time = now
            self.publish_ik_goal(pose, source_pos=source_pos)

        # ================= WAIT_JTC =================
        elif self.state == 'WAIT_JTC':
            # If lock lost -> go waiting
            if not self.target_lock_valid:
                self.get_logger().warn('🔁 Lock perso durante WAIT_JTC → WAITING')
                self.enter_state('WAITING')
                self.publish_state()
                return

            # >>> NEW: if target/surface moved enough -> refresh by restarting from WAITING
            if self._refresh_should_replan():
                self.get_logger().warn(
                    f'🔁 Target moved >= {self.refresh_replan_dist_thr:.2f}m during WAIT_JTC → WAITING (replan)'
                )
                self.enter_state('WAITING')
                self.publish_state()
                return

            if self.ik_done:
                if self.ik_success:
                    self.enter_state('SWITCH_TO_TORQUE')
                else:
                    self.get_logger().error('❌ JTC finished but NOT successful')
                    self.enter_state('FAULT')
                self.publish_state()
                return

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.jtc_timeout:
                self.get_logger().error('❌ JTC timeout')
                self.enter_state('FAULT')

        # ================= SWITCH_TO_TORQUE =================
        elif self.state == 'SWITCH_TO_TORQUE':
            ok = self.switch_controller('to_torque')
            if ok:
                self.enter_state('IMPEDANCE_CONTACT')
            else:
                self.enter_state('FAULT')

        # ================= IMPEDANCE_CONTACT =================
        elif self.state == 'IMPEDANCE_CONTACT':
            if self.contact_detected:
                self.enter_state('HOLD_CONTACT')

        # ================= HOLD_CONTACT =================
        elif self.state == 'HOLD_CONTACT':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.hold_time:
                self.retract_done = False
                self.enter_state('IMPEDANCE_RETRACT')

        # ================= IMPEDANCE_RETRACT =================
        elif self.state == 'IMPEDANCE_RETRACT':
            if self.retract_done:
                self.enter_state('SWITCH_TO_JTC')
                self.publish_state()
                return

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.retract_timeout:
                self.get_logger().error('❌ RETRACT timeout')
                self.enter_state('FAULT')

        # ================= SWITCH_TO_JTC =================
        elif self.state == 'SWITCH_TO_JTC':
            ok = self.switch_controller('to_jtc')
            if ok:
                self.enter_state('RETURN_HOME_JTC')
            else:
                self.enter_state('FAULT')

        # ================= RETURN_HOME_JTC =================
        elif self.state == 'RETURN_HOME_JTC':
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = now.to_msg()

            pose.pose.position.x = float(self.home_position[0])
            pose.pose.position.y = float(self.home_position[1])
            pose.pose.position.z = float(self.home_position[2])

            qx, qy, qz, qw = self.home_orientation
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)

            self._last_goal_pub_time = now
            # source_pos here is home pos (not used for refresh but ok)
            self.publish_ik_goal(pose, source_pos=self.home_position)
            self.enter_state('WAIT_RETURN')

        # ================= WAIT_RETURN =================
        elif self.state == 'WAIT_RETURN':
            if self.ik_done:
                if self.ik_success:
                    self.enter_state('WAITING')
                else:
                    self.get_logger().error('❌ RETURN finished but NOT successful')
                    self.enter_state('FAULT')
                self.publish_state()
                return

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.jtc_timeout:
                self.get_logger().error('❌ RETURN timeout')
                self.enter_state('FAULT')

        # ================= FAULT =================
        elif self.state == 'FAULT':
            if not self._fault_logged:
                self.get_logger().error("🚨 FSM in FAULT state")
                self._fault_logged = True

        self.publish_state()


def main():
    rclpy.init()
    node = Z1FSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()