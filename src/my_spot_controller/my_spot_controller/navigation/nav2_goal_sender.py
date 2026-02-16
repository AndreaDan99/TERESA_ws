#!/usr/bin/env python3
"""
Nav2 Goal Sender (DRY-RUN MODE)
Simula navigazione senza muovere il robot - solo logging
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import math


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        # --- PARAMETERS ---
        self.declare_parameter('min_goal_interval', 5.0)
        self.declare_parameter('cancel_old_goals', True)
        self.declare_parameter('wait_for_server_timeout', 5.0)
        self.declare_parameter('robot_frame', 'body')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('use_tf_for_pose', False)
        
        # ⭐ NUOVO: modalità dry-run
        self.declare_parameter('dry_run_mode', True)  # ← Attiva simulazione
        
        self.min_interval = self.get_parameter('min_goal_interval').value
        self.cancel_old = self.get_parameter('cancel_old_goals').value
        self.server_timeout = self.get_parameter('wait_for_server_timeout').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.use_tf = self.get_parameter('use_tf_for_pose').value
        self.dry_run = self.get_parameter('dry_run_mode').value  # ← Flag
        
        # --- ACTION CLIENT (solo se NON dry-run) ---
        if not self.dry_run:
            self.nav_client = ActionClient(
                self, 
                FollowPath, 
                '/follow_path'
            )
        else:
            self.nav_client = None
            self.get_logger().warn('🔶 DRY-RUN MODE ENABLED - Robot will NOT move!')
        
        # --- SUBSCRIBERS ---
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            '/human_goal_pose',
            self.goal_cb, 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )
        
        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- STATE ---
        self.current_goal_handle = None
        self.last_goal_time = None
        self.is_navigating = False
        self.current_robot_pose = None
        
        # ⭐ Statistiche dry-run
        self.total_goals_received = 0
        self.goals_accepted = 0
        self.goals_rejected = 0
        
        self.get_logger().info(
            f'✅ Nav2 Goal Sender ready '
            f'(dry_run={self.dry_run}, min_interval={self.min_interval}s)'
        )
    
    def odom_cb(self, msg: Odometry):
        """Aggiorna posizione robot"""
        if not self.use_tf:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.current_robot_pose = pose
    
    def get_robot_pose(self):
        """Ottieni pose corrente robot"""
        if self.use_tf:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.robot_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                
                pose = PoseStamped()
                pose.header.frame_id = self.odom_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                
                return pose
            
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
                return None
        else:
            return self.current_robot_pose
    
    def quaternion_to_yaw(self, q):
        """Converti quaternion a yaw (angolo su Z)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def goal_cb(self, msg: PoseStamped):
        """
        Callback goal: valida e logga tutto (dry-run) o invia (normale)
        """
        self.total_goals_received += 1
        
        # ========== LOG HEADER ==========
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'📍 NEW GOAL RECEIVED (#{self.total_goals_received})')
        self.get_logger().info('=' * 80)
        
        # --- CHECK 1: Rate limiting ---
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9
            
            if elapsed < self.min_interval:
                self.get_logger().warn(
                    f'❌ REJECTED: Too soon (elapsed={elapsed:.1f}s < min={self.min_interval}s)'
                )
                self.goals_rejected += 1
                self.print_stats()
                return
        
        # --- CHECK 2: Robot pose ---
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('❌ REJECTED: Robot pose not available')
            self.goals_rejected += 1
            self.print_stats()
            return
        
        # --- CHECK 3: Controller disponibile (solo se NON dry-run) ---
        if not self.dry_run:
            if not self.nav_client.wait_for_server(timeout_sec=self.server_timeout):
                self.get_logger().warn('❌ REJECTED: Controller /follow_path not available')
                self.goals_rejected += 1
                self.print_stats()
                return
        
        # ========== GOAL ACCEPTED ==========
        self.goals_accepted += 1
        self.get_logger().info('✅ GOAL ACCEPTED - Processing...')
        
        # --- Calcola distanza e heading ---
        dx = msg.pose.position.x - robot_pose.pose.position.x
        dy = msg.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        goal_heading = math.atan2(dy, dx)
        robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
        heading_error = math.degrees(goal_heading - robot_yaw)
        
        # Normalizza angolo [-180, 180]
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360
        
        # --- LOG DETTAGLIATO ---
        self.get_logger().info(f'🤖 ROBOT STATE:')
        self.get_logger().info(f'   Position: ({robot_pose.pose.position.x:.3f}, {robot_pose.pose.position.y:.3f}, {robot_pose.pose.position.z:.3f})')
        self.get_logger().info(f'   Yaw: {math.degrees(robot_yaw):.1f}°')
        self.get_logger().info(f'   Frame: {robot_pose.header.frame_id}')
        
        self.get_logger().info(f'🎯 GOAL STATE:')
        self.get_logger().info(f'   Position: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})')
        goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        self.get_logger().info(f'   Yaw: {math.degrees(goal_yaw):.1f}°')
        self.get_logger().info(f'   Frame: {msg.header.frame_id}')
        
        self.get_logger().info(f'📏 NAVIGATION METRICS:')
        self.get_logger().info(f'   Distance to goal: {distance:.3f} m')
        self.get_logger().info(f'   Heading to goal: {math.degrees(goal_heading):.1f}°')
        self.get_logger().info(f'   Heading error: {heading_error:.1f}°')
        
        # Stima tempo (velocità media 0.5 m/s)
        estimated_time = distance / 0.5
        self.get_logger().info(f'   Estimated time: {estimated_time:.1f}s (@ 0.5 m/s)')
        
        # ========== DRY-RUN vs REAL ==========
        if self.dry_run:
            self.get_logger().info('🔶 DRY-RUN MODE: Skipping actual navigation')
            self.get_logger().info(f'   → Robot WOULD navigate {distance:.2f}m with {heading_error:.1f}° turn')
            
            # Simula risultato
            if distance < 0.1:
                self.get_logger().info('   → Simulated result: ALREADY AT GOAL')
            elif distance > 10.0:
                self.get_logger().warn('   → Simulated result: GOAL TOO FAR (>10m)')
            elif abs(heading_error) > 170:
                self.get_logger().warn('   → Simulated result: REQUIRES 180° TURN')
            else:
                self.get_logger().info('   → Simulated result: NAVIGATION FEASIBLE ✅')
        
        else:
            # MODALITÀ REALE: crea path e invia
            self.get_logger().info('🚀 REAL MODE: Sending goal to controller...')
            
            # Cancella goal precedente
            if self.cancel_old and self.current_goal_handle is not None:
                self.get_logger().info('   Cancelling previous goal...')
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_response_cb)
            
            # Crea path
            path = Path()
            path.header.frame_id = msg.header.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses.append(robot_pose)
            path.poses.append(msg)
            
            # Crea goal
            nav_goal = FollowPath.Goal()
            nav_goal.path = path
            nav_goal.controller_id = 'FollowPath'
            nav_goal.goal_checker_id = 'general_goal_checker'
            
            # Invia
            send_goal_future = self.nav_client.send_goal_async(
                nav_goal,
                feedback_callback=self.feedback_cb
            )
            send_goal_future.add_done_callback(self.goal_response_cb)
        
        # Aggiorna timestamp
        self.last_goal_time = self.get_clock().now()
        
        # Statistiche
        self.print_stats()
        self.get_logger().info('=' * 80)
    
    def print_stats(self):
        """Stampa statistiche cumulative"""
        self.get_logger().info(f'📊 STATISTICS:')
        self.get_logger().info(f'   Total goals: {self.total_goals_received}')
        self.get_logger().info(f'   Accepted: {self.goals_accepted} ({100*self.goals_accepted/max(1,self.total_goals_received):.1f}%)')
        self.get_logger().info(f'   Rejected: {self.goals_rejected} ({100*self.goals_rejected/max(1,self.total_goals_received):.1f}%)')
    
    def goal_response_cb(self, future):
        """Callback controller (solo modalità reale)"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal REJECTED by controller')
            self.is_navigating = False
            return
        
        self.get_logger().info('✅ Goal ACCEPTED by controller - Navigation started')
        self.current_goal_handle = goal_handle
        self.is_navigating = True
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)
    
    def result_cb(self, future):
        """Callback risultato navigazione"""
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎉 Navigation SUCCEEDED!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('⚠️ Navigation CANCELED')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ Navigation ABORTED')
        else:
            self.get_logger().warn(f'Navigation ended: status={status}')
        
        self.current_goal_handle = None
        self.is_navigating = False
    
    def cancel_response_cb(self, future):
        """Callback cancellazione"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Previous goal canceled')
    
    def feedback_cb(self, feedback_msg):
        """Feedback navigazione (solo modalità reale)"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'📍 Navigation: distance={feedback.distance_to_goal:.2f}m, speed={feedback.speed:.2f}m/s',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
