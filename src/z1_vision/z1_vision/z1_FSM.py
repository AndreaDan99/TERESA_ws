#!/usr/bin/env python3
"""
Z1 High-Level FSM

States:
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
- FAULT_RECOVERY_HOME

This FSM orchestrates:
- IK → JointTrajectoryController
- Safe controller switching
- Impedance activation
- Return home
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
        # Offset iniziale lungo la normale (p_surf + offset * n).
        # Impostalo coerente con impedance_params.yaml (es: -0.205 se poi l'impedenza avanza 0.20 fino a -0.005).
        self.declare_parameter('pre_contact_normal_offset', -0.205)

        # Se True: usa /torso_surface_frame per approach (pos+orientazione).
        # Se False: fallback alla vecchia logica (z-world).
        self.declare_parameter('use_surface_for_approach', True)

        self.declare_parameter('home_position', [0.0411, 0.0103, 0.5133])

        # Home orientation quaternion [x,y,z,w] (frame: world)
        self.declare_parameter('home_orientation', [0.0, 0.0, 0.0, 1.0])

        # Startup: command HOME once after delay
        self.declare_parameter('startup_go_home', True)
        self.declare_parameter('startup_home_delay', 5.0)

        self.declare_parameter('hold_time', 5.0)
        self.declare_parameter('retract_timeout', 6.0)

        self.declare_parameter('jtc_timeout', 25.0)
        self.declare_parameter('switch_timeout', 3.0)

        # Re-publish IK goal while waiting (robust to startup ordering)
        self.declare_parameter('ik_goal_republish_rate', 5.0)  # Hz

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

        # ================= SUBSCRIBERS =================
        self.sub_target = self.create_subscription(
            PoseStamped, '/torso_target_ee_locked', self.cb_target, 10)

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
        self.startup_home_sent = False
        self.target_world = None
        self.surface_frame = None
        self.target_lock_valid = False
        self.contact_detected = False
        self.ik_done = False
        self.ik_success = False
        self.hold_start_time = None

        self.state_start_time = self.get_clock().now()
        self.jtc_goal_sent = False
        self.retract_done = False

        # Last IK goal (for periodic re-publish while waiting)
        self.last_ik_goal_pose = None  # type: PoseStamped | None
        self._last_goal_pub_time = self.get_clock().now()

        self.timer = self.create_timer(1.0/30.0, self.step)

        self.get_logger().info('🧠 NEW Z1 FSM READY')

    # ================= CALLBACKS =================

    def cb_target(self, msg):
        self.target_world = msg

    def cb_surface(self, msg):
        self.surface_frame = msg

    def cb_lock_valid(self, msg: Bool):
        self.target_lock_valid = bool(msg.data)

    def cb_contact(self, msg):
        self.contact_detected = bool(msg.data)

    def cb_ik_done(self, msg):
        self.ik_done = bool(msg.data)
    
    def cb_ik_success(self, msg: Bool):
        self.ik_success = bool(msg.data)

    def cb_retract_done(self, msg: Bool):
        self.retract_done = bool(msg.data)
    # ================= UTIL =================

    def publish_state(self):
        m = String()
        m.data = self.state
        self.pub_state.publish(m)

    def publish_ik_goal(self, pose_msg):
        # Store last goal so we can re-publish it while waiting
        self.last_ik_goal_pose = pose_msg
        self.pub_ik_goal.publish(pose_msg)
        self._last_goal_pub_time = self.get_clock().now()

    def _maybe_republish_goal(self, now):
        """Re-publish the last IK goal at a fixed rate while waiting for completion."""
        if self.last_ik_goal_pose is None:
            return
        # Only re-publish while waiting for JTC result
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

    def enter_state(self, new_state):
        self.get_logger().info(f"🔄 {self.state} → {new_state}")
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        if new_state == 'RETURN_HOME_JTC':
            self.contact_detected = False
            self.ik_done = False
            self.ik_success = False
        if new_state == 'WAITING':
            self.last_ik_goal_pose = None

    # ================= MAIN FSM =================

    def step(self):

        now = self.get_clock().now()
        self._maybe_republish_goal(now)

        # ================= WAITING =================
        if self.state == 'WAITING':
            # Startup: manda HOME una sola volta dopo un piccolo delay
            if self.startup_go_home and (not self.startup_home_sent):
                startup_elapsed = (now - self.start_time).nanoseconds / 1e9
                if startup_elapsed >= self.startup_home_delay:
                    self.startup_home_sent = True
                    self.enter_state('RETURN_HOME_JTC')
                    return

            # Normal operation: proceed only when the required inputs are available
            if self.use_surface_for_approach:
                if self.target_lock_valid and (self.surface_frame is not None):
                    self.enter_state('APPROACHING_JTC')
            else:
                if self.target_lock_valid and (self.target_world is not None):
                    self.enter_state('APPROACHING_JTC')

        # ================= APPROACHING_JTC =================
        elif self.state == 'APPROACHING_JTC':
            pose = PoseStamped()

            if self.use_surface_for_approach and self.surface_frame is not None:
                # Usa la superficie: orientazione allineata alla normale (asse Z del frame superficie)
                pose.header = self.surface_frame.header
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

            else:
                # Fallback: vecchia logica (offset su -Z world)
                if self.target_world is None:
                    return
                pose.header = self.target_world.header
                pose.pose = self.target_world.pose
                pose.pose.position.z -= self.approach_offset

            self.ik_done = False
            self.ik_success = False
            # Allow immediate re-publish while waiting
            self._last_goal_pub_time = self.get_clock().now()
            self.publish_ik_goal(pose)
            self.jtc_goal_sent = True
            self.enter_state('WAIT_JTC')

        # ================= WAIT_JTC =================
        elif self.state == 'WAIT_JTC':
            if self.ik_done:
                if self.ik_success:
                    self.enter_state('SWITCH_TO_TORQUE')
                else:
                    self.get_logger().error('❌ JTC finished but NOT successful')
                    self.enter_state('FAULT')
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

        elif self.state == 'IMPEDANCE_RETRACT':
            # aspetta che l'impedance controller dica “ok, sono tornato indietro”
            if self.retract_done:
                self.enter_state('SWITCH_TO_JTC')
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

            # Fixed home orientation (x,y,z,w)
            qx, qy, qz, qw = self.home_orientation
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)

            self.ik_done = False
            self.ik_success = False
            # Allow immediate re-publish while waiting
            self._last_goal_pub_time = self.get_clock().now()
            self.publish_ik_goal(pose)
            self.enter_state('WAIT_RETURN')

        # ================= WAIT_RETURN =================
        elif self.state == 'WAIT_RETURN':
            if self.ik_done:
                if self.ik_success:
                    self.enter_state('WAITING')
                else:
                    self.get_logger().error('❌ RETURN finished but NOT successful')
                    self.enter_state('FAULT')
                return

            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed > self.jtc_timeout:
                self.get_logger().error('❌ RETURN timeout')
                self.enter_state('FAULT')

        # ================= FAULT =================
        elif self.state == 'FAULT':
            self.get_logger().error("🚨 FSM in FAULT state")

        self.publish_state()



def main():
    rclpy.init()
    node = Z1FSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()