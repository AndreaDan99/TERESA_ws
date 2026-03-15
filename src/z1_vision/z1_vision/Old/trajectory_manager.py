#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import tty
import termios


class TrajectoryManager(Node):
    def __init__(self):
        super().__init__('trajectory_manager')
        
        self.pub_traj = self.create_publisher(JointTrajectory, 
                                            '/joint_trajectory_controller/joint_trajectory', 
                                            10)
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Posa di start
        self.start_pos = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]
        # Posa di home/stow
        self.home_pos = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]
        
        self.get_logger().info("""
=== TRAJECTORY MANAGER ===
Premi:
  's' → Start position [%s]
  'h' → Home position [%s]  
  'q' → Quit
""" % (self.start_pos, self.home_pos))

    def send_trajectory(self, positions, duration_sec=10):
        """Pubblica una JointTrajectory verso positions in duration_sec"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        msg.points.append(point)
        
        self.pub_traj.publish(msg)
        self.get_logger().info('Inviata traiettoria verso %s in %d s' % (positions, duration_sec))

    def run(self):
        """Loop principale: legge input da tastiera"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 's':
                        self.send_trajectory(self.start_pos, 10)
                    elif key == 'h':
                        self.send_trajectory(self.home_pos, 5)
                    elif key == 'q':
                        self.get_logger().info('Uscita richiesta. Ricorda di spegnere il bringup!')
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
