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


# Sequenza di pose per test completo
# J2 e J3 mai a zero per evitare limiti meccanici
POSE_SEQUENCE = [
    {
        'name': 'Safe Start',
        'position': [0.0, 0.20, -0.20, 0.0, 0.0, 0.0],
        'duration': 3.0
    },
    {
        'name': 'Reach Position',
        'position': [0.25, 0.25, -0.25, 0.10, 0.10, 0.0],
        'duration': 4.0
    },
    {
        'name': 'Grasp Ready',
        'position': [0.25, 0.18, -0.18, 0.10, 0.10, 0.10],
        'duration': 3.0
    },
    {
        'name': 'Lift',
        'position': [0.25, 0.28, -0.28, 0.10, 0.10, 0.10],
        'duration': 3.0
    },
    {
        'name': 'Transport',
        'position': [-0.25, 0.25, -0.25, -0.10, -0.10, 0.0],
        'duration': 4.0
    },
    {
        'name': 'Place Position',
        'position': [-0.25, 0.15, -0.15, -0.10, -0.10, 0.0],
        'duration': 3.0
    },
    {
        'name': 'Release',
        'position': [-0.25, 0.20, -0.20, -0.10, -0.10, -0.10],
        'duration': 3.0
    },
    {
        'name': 'Return Safe',
        'position': [0.0, 0.20, -0.20, 0.0, 0.0, 0.0],
        'duration': 4.0
    },
    {
        'name': 'Home Safe',
        'position': [0.0, 0.10, -0.10, 0.0, 0.0, 0.0],
        'duration': 3.0
    }
]

TOLERANCE = 0.02
PAUSE_BETWEEN_POSES = 0.5  # secondi di pausa tra pose


class TestSequence(Node):

    def __init__(self):
        super().__init__('test_sequence')

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

    def execute_pose(self, pose_name, q_target, duration):
        """Esegue una singola posa e verifica tracking"""
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Calcola distanza
        distance = sum([abs(self.current_q[i] - q_target[i]) for i in range(6)])

        if distance < 0.001:
            self.get_logger().info(f"⏭️  Already at '{pose_name}', skipping...")
            return True

        # Punto finale
        p1 = JointTrajectoryPoint()
        p1.positions = list(q_target)
        p1.velocities = [0.0] * 6
        p1.accelerations = [0.0] * 6
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal.trajectory.points = [p1]

        self.get_logger().info(f"📍 Moving to '{pose_name}': {[f'{q:.3f}' for q in q_target]}")

        # Invia goal
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"❌ Goal '{pose_name}' rejected by controller!")
            return False

        # Aspetta risultato
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code != 0:
            self.get_logger().error(f"❌ Pose '{pose_name}' failed with error_code: {result.result.error_code}")
            return False

        # Aggiorna posizione
        self.current_q = list(q_target)

        # Pausa e verifica tracking
        time.sleep(PAUSE_BETWEEN_POSES)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        actual_error = [abs(self.current_q[i] - q_target[i]) for i in range(6)]
        max_error = max(actual_error)

        if max_error > TOLERANCE:
            self.get_logger().warn(f"⚠️  Pose '{pose_name}' tracking error {max_error:.4f} > {TOLERANCE}")
            return False

        self.get_logger().info(f"✅ Pose '{pose_name}' reached (max error: {max_error:.4f})")
        return True

    def run_sequence(self):
        """Esegue l'intera sequenza di pose"""
        
        self.get_logger().info("🚀 Starting pose sequence test...")
        
        # Definisci SAFE BASE
        safe_base = [0.0, 0.20, -0.20, 0.0, 0.0, 0.0]
        
        # Calcola distanza dalla safe base
        current_distance = sum([abs(self.current_q[i] - safe_base[i]) for i in range(6)])
        
        # Se parti da home esatta, fai movimento in 2 step
        home_distance = sum([abs(self.current_q[i]) for i in range(6)])
        
        if home_distance < 0.01:  # sei esattamente a home
            self.get_logger().info("🏠 Starting from home, moving in 2 steps...")
            
            # Step 1: muovi solo J1 per "attivare" il controller
            intermediate = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info(f"📍 Step 1/2: Intermediate position {[f'{q:.3f}' for q in intermediate]}")
            
            if not self.execute_pose('Intermediate', intermediate, 2.0):
                self.get_logger().error("❌ Failed intermediate step!")
                return False
            
            time.sleep(0.5)
            
            # Step 2: vai alla safe base
            self.get_logger().info(f"📍 Step 2/2: Safe base position {[f'{q:.3f}' for q in safe_base]}")
            
            if not self.execute_pose('Safe Base', safe_base, 4.0):
                self.get_logger().error("❌ Failed to reach safe base!")
                return False
            
            time.sleep(1.0)
        
        elif current_distance > 0.05:  # sei lontano da safe base
            self.get_logger().info(f"📍 Moving to safe base position...")
            
            if not self.execute_pose('Safe Base', safe_base, 5.0):
                self.get_logger().error("❌ Failed to reach safe base!")
                return False
            
            time.sleep(1.5)
        
        else:
            self.get_logger().info("✅ Already at safe base position")
        
        self.get_logger().info(f"📋 Total poses in sequence: {len(POSE_SEQUENCE)}")
        
        start_time = time.time()

        for i, pose in enumerate(POSE_SEQUENCE):
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"🎯 Pose {i+1}/{len(POSE_SEQUENCE)}: {pose['name']}")
            self.get_logger().info(f"{'='*60}")

            if not self.execute_pose(pose['name'], pose['position'], pose['duration']):
                self.get_logger().error(f"❌ SEQUENCE FAILED at pose '{pose['name']}'")
                return False

        elapsed_time = time.time() - start_time

        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("🎉 ✅ SEQUENCE COMPLETED SUCCESSFULLY")
        self.get_logger().info(f"📊 Statistics:")
        self.get_logger().info(f"   - Total poses executed: {len(POSE_SEQUENCE)}")
        self.get_logger().info(f"   - Total time: {elapsed_time:.1f} seconds")
        self.get_logger().info(f"   - Average time per pose: {elapsed_time/len(POSE_SEQUENCE):.1f} seconds")
        self.get_logger().info(f"{'='*60}")
        
        return True


def main():
    rclpy.init()
    node = TestSequence()
    success = node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
