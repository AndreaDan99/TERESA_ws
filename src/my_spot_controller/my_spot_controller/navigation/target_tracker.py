#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
import math

class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        
        # --- TF2 SETUP ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- SUBSCRIBERS ---
        self.current_pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', 
            self.current_pose_cb, 10)
        
        # Subscriber alla bounding box corpo intero (rossa)
        self.bbox_body_sub = self.create_subscription(
            Marker, '/human_pose/bounding_box',
            self.bbox_body_cb, 10)
        
        # Subscriber alla bounding box torso (verde)
        self.bbox_torso_sub = self.create_subscription(
            Marker, '/human_pose/torso_bounding_box',
            self.bbox_torso_cb, 10)
        
        # Subscriber alla postura
        self.posture_sub = self.create_subscription(
            String, '/human_pose/posture',
            self.posture_cb, 10)
        # In target_tracker aggiungere subscriber
        self.obstacle_sub = self.create_subscription(
            String, '/spot/obstacle_warning',
            self.obstacle_cb, 10)
        
        self.motion_sub = self.create_subscription(
            String, '/spot/motion_direction', 
            self.motion_cb, 10)

        
        # --- PUBLISHERS ---
        self.pose_error_pub = self.create_publisher(PoseStamped, '/pose_error', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # --- STATE ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0  # orientamento robot
        
        # Bounding boxes (in frame odom, dopo trasformazione)
        self.body_box_odom = None    # Box corpo intero (rossa) in odom
        self.torso_box_odom = None   # Box torso (verde) in odom
        self.posture = "UNKNOWN"
        
   
        self.last_bbox_time = None
        
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.goal_active = False
        self.goal_start_time = None
        
        # Parametri
        self.declare_parameter('timeout', 15.0)
        self.declare_parameter('distance_threshold', 0.15)  # 15cm dal bordo box
        self.declare_parameter('angle_threshold', 0.1)  # ~5.7 gradi
        self.declare_parameter('auto_start', True)
        self.declare_parameter('lateral_offset_lying', 0.7)
        self.declare_parameter('target_frame', 'odom')  # Frame di riferimento target
        
    
        self.declare_parameter('bbox_timeout', 2.0)  # Tollera perdita detection per 2s
        self.declare_parameter('emergency_distance', 0.5)  # Distanza minima sicurezza (50cm)
        
        self.timeout = self.get_parameter('timeout').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.auto_start = self.get_parameter('auto_start').value
        self.lateral_offset = self.get_parameter('lateral_offset_lying').value
        self.target_frame = self.get_parameter('target_frame').value
        

        self.bbox_timeout = self.get_parameter('bbox_timeout').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        
        # Timer per calcolo errore continuo
        self.create_timer(0.1, self.compute_error)
        
        self.get_logger().info(
            f'Target Tracker initialized (bbox_timeout={self.bbox_timeout}s, '
            f'emergency_dist={self.emergency_distance}m, target_frame={self.target_frame})'
        )
    def motion_cb(self, msg: String):
        if self.body_box_odom is None:  # Orbbec ha perso tracking
        # Persona potrebbe essere nella direzione del movimento
            self.get_logger().info(f'Hint: person might be at {msg.data}')
            

    def obstacle_cb(self, msg: String):
        if 'LEFT' in msg.data or 'RIGHT' in msg.data:
        # Rallenta o ferma temporaneamente
            self.obstacle_detected = True

    def current_pose_cb(self, msg):
        """Aggiorna posizione corrente da odom"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Estrai yaw da quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def transform_marker_to_odom(self, marker: Marker):
        """
        Trasforma marker da camera_color_optical_frame a odom usando TF2
        """
        try:
            # Lookup trasformazione camera -> odom
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # target frame (odom)
                marker.header.frame_id,  # source frame (camera_color_optical_frame)
                marker.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Trasforma posizione marker
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            
            # Crea marker trasformato
            marker_odom = Marker()
            marker_odom.header.frame_id = self.target_frame
            marker_odom.header.stamp = marker.header.stamp
            marker_odom.type = marker.type
            marker_odom.action = marker.action
            marker_odom.ns = marker.ns
            marker_odom.id = marker.id
            
            # Trasforma posizione
            marker_odom.pose.position.x = marker.pose.position.x + tx
            marker_odom.pose.position.y = marker.pose.position.y + ty
            marker_odom.pose.position.z = marker.pose.position.z + tz
            
            # Orientamento rimane uguale (assumiamo box allineate)
            marker_odom.pose.orientation = marker.pose.orientation
            
            # Copia scale e color
            marker_odom.scale = marker.scale
            marker_odom.color = marker.color
            
            return marker_odom
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {marker.header.frame_id} to {self.target_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return None
    
    def bbox_body_cb(self, msg: Marker):
        """Riceve bounding box corpo intero (rossa) e trasforma in odom"""
        if msg.action == Marker.DELETE:
            self.body_box_odom = None
            # ⭐ NON fermiamo subito, aspettiamo timeout
            return
        
        # Trasforma in odom
        self.body_box_odom = self.transform_marker_to_odom(msg)
        
        if self.body_box_odom is not None:
            # ⭐ Aggiorna timestamp ultima bbox valida
            self.last_bbox_time = self.get_clock().now()
            
            # Calcola target solo se valida
            self.update_target_from_body_box()
    
    def bbox_torso_cb(self, msg: Marker):
        """Riceve bounding box torso (verde) e trasforma in odom"""
        if msg.action == Marker.DELETE:
            self.torso_box_odom = None
            return
        
        self.torso_box_odom = self.transform_marker_to_odom(msg)
    
    def posture_cb(self, msg: String):
        """Riceve stato postura"""
        self.posture = msg.data
    
    def update_target_from_body_box(self):
        """
        Calcola punto target dal bordo della bounding box corpo
        Considera postura per approccio laterale se LYING
        """
        if self.body_box_odom is None:
            return
        
        # Centro della box (posizione persona)
        box_center_x = self.body_box_odom.pose.position.x
        box_center_y = self.body_box_odom.pose.position.y
        box_center_z = self.body_box_odom.pose.position.z
        
        # Dimensioni box (include safety margin)
        box_size_x = self.body_box_odom.scale.x
        box_size_y = self.body_box_odom.scale.y
        box_size_z = self.body_box_odom.scale.z
        
        # --- Calcola punto target sul bordo della box ---
        
        # Caso speciale: persona sdraiata -> approccio laterale
        if self.posture == "LYING":
            # Target a lato della persona (offset laterale)
            dx = box_center_x - self.current_x
            dy = box_center_y - self.current_y
            
            # Vettore perpendicolare (ruotato 90°)
            perp_x = -dy
            perp_y = dx
            norm = math.sqrt(perp_x**2 + perp_y**2)
            
            if norm > 1e-6:
                perp_x /= norm
                perp_y /= norm
                
                # Target = centro box + offset laterale + margine sicurezza
                target_x = box_center_x + perp_x * (box_size_y/2 + self.lateral_offset)
                target_y = box_center_y + perp_y * (box_size_y/2 + self.lateral_offset)
                
                # Orientamento verso la persona
                target_yaw = math.atan2(box_center_y - target_y, box_center_x - target_x)
            else:
                return  # Robot già al centro, skip
        
        else:
            # Caso normale (STANDING/SITTING): approccio frontale
            dx = box_center_x - self.current_x
            dy = box_center_y - self.current_y
            dz = box_center_z - self.current_z
            
            dist_to_center = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if dist_to_center < 1e-6:
                return  # Robot già al centro
            
            # Direzione normalizzata robot → box
            dir_x = dx / dist_to_center
            dir_y = dy / dist_to_center
            dir_z = dz / dist_to_center
            
            # Calcola intersezione con faccia box più vicina
            t_x = (box_size_x / 2.0) / abs(dir_x) if abs(dir_x) > 1e-6 else float('inf')
            t_y = (box_size_y / 2.0) / abs(dir_y) if abs(dir_y) > 1e-6 else float('inf')
            t_z = (box_size_z / 2.0) / abs(dir_z) if abs(dir_z) > 1e-6 else float('inf')
            
            # Prendi il minimo (prima faccia intersecata)
            t = min(t_x, t_y, t_z)
            
            # Punto target = centro box - t * direzione
            target_x = box_center_x - t * dir_x
            target_y = box_center_y - t * dir_y
            
            # Orientamento verso la persona
            target_yaw = math.atan2(dy, dx)
        
        # Aggiorna goal (solo se cambiato significativamente o nuovo goal)
        if self.target_x is None or \
           abs(target_x - self.target_x) > 0.05 or \
           abs(target_y - self.target_y) > 0.05:
            
            self.target_x = target_x
            self.target_y = target_y
            self.target_yaw = target_yaw
            
            # Auto-attiva goal se configurato E non già attivo
            if self.auto_start and not self.goal_active:
                self.goal_active = True
                self.goal_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'New goal: edge of box at ({target_x:.2f}, {target_y:.2f}), '
                    f'posture={self.posture}'
                )
    
    def publish_zero_error(self):
        """Pubblica errore zero per fermare Spot"""
        error_msg = PoseStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'odom'
        error_msg.pose.position.x = 0.0
        error_msg.pose.position.y = 0.0
        error_msg.pose.position.z = 0.0
        self.pose_error_pub.publish(error_msg)
    
    def compute_error(self):
        """Calcola errore e pubblica su /pose_error per controller"""
        
        if not self.goal_active or self.target_x is None:
            return
        
        if self.body_box_odom is None:
            if self.last_bbox_time is not None:
                elapsed_since_last = (self.get_clock().now() - self.last_bbox_time).nanoseconds * 1e-9
                
                if elapsed_since_last > self.bbox_timeout:
                    # Timeout superato: STOP
                    self.goal_active = False
                    self.publish_zero_error()
                    self.get_logger().warn(
                        f'STOP: Lost detection for {elapsed_since_last:.1f}s (timeout={self.bbox_timeout}s)'
                    )
                    
                    # Pubblica goal_reached (fallito)
                    msg = Bool()
                    msg.data = False
                    self.goal_reached_pub.publish(msg)
                    return
                else:
                    # Dentro timeout: continua con ultimo target noto
                    self.get_logger().info(
                        f'Detection lost for {elapsed_since_last:.1f}s, continuing with last known target',
                        throttle_duration_sec=1.0
                    )
        
        # Calcola errore
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        distance = math.sqrt(error_x**2 + error_y**2)
    
        if distance < self.emergency_distance and self.body_box_odom is None:
            self.goal_active = False
            self.publish_zero_error()
            self.get_logger().error(
                f'🚨 EMERGENCY STOP: distance={distance:.2f}m < {self.emergency_distance}m AND lost detection'
            )
            
            msg = Bool()
            msg.data = False
            self.goal_reached_pub.publish(msg)
            return
        
        # Errore angolare
        error_yaw = self.target_yaw - self.current_yaw
        # Normalizza tra -pi e pi
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        
        # Check timeout goal
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
        
        # Goal raggiunto?
        if distance < self.distance_threshold or elapsed > self.timeout:
            self.goal_active = False
            reason = "reached target" if distance < self.distance_threshold else f"timeout {self.timeout}s"
            self.get_logger().info(f'Goal reached: {reason}, dist={distance:.3f}m, time={elapsed:.1f}s')
            
            # Pubblica goal_reached
            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)
            
            # Pubblica errore zero per fermare robot
            self.publish_zero_error()
            return
        
        # Pubblica errore per controller
        error_msg = PoseStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'odom'
        error_msg.pose.position.x = error_x
        error_msg.pose.position.y = error_y
        error_msg.pose.position.z = distance  # Usa z per distance totale
        error_msg.pose.orientation.z = error_yaw  # Errore angolare in z
        
        self.pose_error_pub.publish(error_msg)
        
        # Log throttled
        if int(elapsed * 10) % 10 == 0:  # Ogni secondo
            bbox_status = "OK" if self.body_box_odom is not None else "LOST"
            self.get_logger().info(
                f'Error: dist={distance:.2f}m, yaw={math.degrees(error_yaw):.1f}°, '
                f't={elapsed:.1f}s, bbox={bbox_status}'
            )


def main():
    rclpy.init()
    node = TargetTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
