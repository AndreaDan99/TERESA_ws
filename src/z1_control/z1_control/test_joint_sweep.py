#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import time


JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]


# Limiti sicuri - J2/J3 mai a zero per evitare limiti meccanici
SWEEP_LIMITS = {
    'joint1': (-0.26, 0.26),
    'joint2': (0.10, 0.30),        # 0.1-0.3 rad (mai zero!)
    'joint3': (-0.29, -0.10),      # -0.29 a -0.1 rad (mai zero!)
    'joint4': (-0.15, 0.15),
    'joint5': (-0.13, 0.13),
    'joint6': (-0.28, 0.28),
}

MOVE_TIME = 3.0
PAUSE_TIME = 1.0
TOLERANCE = 0.02


class TestJointSweep(Node):

    def __init__(self):
        super().__init__('test_joint_sweep')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()
        self.get_logger().info("Action server ready.")

        self.current_q = None
        self.received_state = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.get_logger().info("Waiting for joint_states...")
        while rclpy.ok() and not self.received_state:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Initial position: {[f'{q:.4f}' for q in self.current_q]}")

    def joint_state_cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = [0.0] * 6
        for i, jn in enumerate(JOINT_NAMES):
            if jn in name_to_idx:
                q[i] = msg.position[name_to_idx[jn]]
        self.current_q = q
        self.received_state = True

    def send_goal_and_wait(self, q_target, move_time):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Calcola distanza dalla posizione corrente
        distance = sum([abs(self.current_q[i] - q_target[i]) for i in range(6)])

        # Se già nella posizione target, skippa
        if distance < 0.001:
            self.get_logger().info("⏭️  Already at target position, skipping...")
            return True

        # Punto finale con velocità/accelerazione ZERO (stop completo)
        p1 = JointTrajectoryPoint()
        p1.positions = list(q_target)
        p1.velocities = [0.0] * 6
        p1.accelerations = [0.0] * 6
        p1.time_from_start.sec = int(move_time)
        p1.time_from_start.nanosec = int((move_time % 1) * 1e9)

        goal.trajectory.points = [p1]

        self.get_logger().info(f"Sending target: {[f'{q:.3f}' for q in q_target]}")

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by controller!")
            return False

        self.get_logger().info("✅ Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"📊 Action result error_code: {result.result.error_code}")

        # Aggiorna posizione attesa
        self.current_q = list(q_target)

        # Aspetta assestamento + verifica tracking
        time.sleep(PAUSE_TIME)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Verifica errore reale vs attesa
        actual_error = [abs(self.current_q[i] - q_target[i]) for i in range(6)]
        max_error = max(actual_error)
        self.get_logger().info(f"📏 Tracking error: {[f'{e:.4f}' for e in actual_error]} (max: {max_error:.4f})")

        if max_error > TOLERANCE:
            self.get_logger().warn(f"⚠️  Tracking error {max_error:.4f} > tolerance {TOLERANCE}")
            return False

        return True

    def run_test(self):
        self.get_logger().info("🚀 Starting joint sweep test...")

        # Definisci SAFE BASE al centro dei range di J2/J3
        safe_base = [
            0.0,      # J1
            0.20,     # J2 centro di (0.10, 0.30)
            -0.20,    # J3 centro di (-0.29, -0.10)
            0.0,      # J4
            0.0,      # J5
            0.0       # J6
        ]

        # Calcola se serve andare alla safe base
        current_distance = sum([abs(self.current_q[i] - safe_base[i]) for i in range(6)])
        
        if current_distance > 0.05:  # se non sei già vicino alla safe base
            self.get_logger().info(f"📍 Moving to safe test position: {[f'{q:.3f}' for q in safe_base]}")
            if not self.send_goal_and_wait(safe_base, MOVE_TIME * 1.5):
                self.get_logger().error("❌ Failed to reach safe position!")
                return False
            time.sleep(1.0)  # pausa extra per assestamento
        else:
            self.get_logger().info("✅ Already at safe test position")

        q_base = safe_base
        self.get_logger().info(f"📍 Base position: {[f'{q:.3f}' for q in q_base]}")

        for i, jn in enumerate(JOINT_NAMES):
            j_min, j_max = SWEEP_LIMITS[jn]
            self.get_logger().info(f"\n🧑‍🔧 Testing {jn}: {j_min:.3f} → {j_max:.3f}")

            # 1. Torna base
            if not self.send_goal_and_wait(q_base, MOVE_TIME):
                return False

            # 2. Vai a minimo
            q_min = list(q_base)
            q_min[i] = j_min
            if not self.send_goal_and_wait(q_min, MOVE_TIME):
                return False

            # 3. Vai a massimo
            q_max = list(q_base)
            q_max[i] = j_max
            if not self.send_goal_and_wait(q_max, MOVE_TIME):
                return False

            # 4. Torna base
            if not self.send_goal_and_wait(q_base, MOVE_TIME):
                return False

        self.get_logger().info("\n🎉 ✅ JOINT SWEEP COMPLETED SUCCESSFULLY")
        self.get_logger().info(f"📋 Summary: All {len(JOINT_NAMES)} joints tested OK")
        return True


def main():
    rclpy.init()
    node = TestJointSweep()
    success = node.run_test()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
