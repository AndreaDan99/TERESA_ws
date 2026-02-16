#!/usr/bin/env python3
"""
Human-Aware Target Generator
Genera goal pose "socialmente consapevoli" vicino alla persona rilevata
per navigazione autonoma con Nav2
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import numpy as np
import math

class HumanTargetGenerator(Node):
    def __init__(self):
        super().__init__('human_aware_target_generator')
        
        # --- SUBSCRIBERS ---
        # Bounding box corpo intero
        self.bbox_body_sub = self.create_subscription(
            Marker, '/human_pose/bounding_box',
            self.bbox_body_cb, 10)
        
        # Postura persona
        self.posture_sub = self.create_subscription(
            String, '/human_pose/posture',
            self.posture_cb, 10)
        
        # --- PUBLISHERS ---
        # Goal pose per Nav2
        self.goal_pub = self.create_publisher(
            PoseStamped, '/human_goal_pose', 10)
        
        # Marker visualizzazione goal in RViz
        self.goal_marker_pub = self.create_publisher(
            Marker, '/human_goal_marker', 10)
        
        # --- STATE ---
        self.body_box = None
        self.posture = "UNKNOWN"
        self.last_goal_time = None
        
        # --- PARAMETERS ---
        # Social distances (ISO 13482 / proxemics)
        self.declare_parameter('intimate_distance', 0.45)    # 0-45cm: famiglia
        self.declare_parameter('personal_distance', 1.2)     # 45-120cm: amici
        self.declare_parameter('social_distance', 3.6)       # 120-360cm: conoscenti
        
        # Approccio per postura
        self.declare_parameter('standing_approach_dist', 1.5)  # frontale
        self.declare_parameter('sitting_approach_dist', 1.0)   # più vicino, frontale
        self.declare_parameter('lying_lateral_offset', 0.8)    # laterale, distanza sociale
        
        # Safety & update rate
        self.declare_parameter('min_update_interval', 2.0)  # Non aggiornare goal troppo spesso
        self.declare_parameter('goal_frame', 'odom')        # Frame goal (odom o map)
        
        self.intimate_dist = self.get_parameter('intimate_distance').value
        self.personal_dist = self.get_parameter('personal_distance').value
        self.social_dist = self.get_parameter('social_distance').value
        
        self.standing_dist = self.get_parameter('standing_approach_dist').value
        self.sitting_dist = self.get_parameter('sitting_approach_dist').value
        self.lying_offset = self.get_parameter('lying_lateral_offset').value
        
        self.min_interval = self.get_parameter('min_update_interval').value
        self.goal_frame = self.get_parameter('goal_frame').value
        
        # Timer per generazione goal periodica
        self.create_timer(1.0, self.generate_goal)
        
        self.get_logger().info(
            f'✅ Human-Aware Target Generator ready '
            f'(standing={self.standing_dist}m, sitting={self.sitting_dist}m, '
            f'lying_lateral={self.lying_offset}m)'
        )
    
    def bbox_body_cb(self, msg: Marker):
        """Riceve bounding box corpo"""
        if msg.action == Marker.DELETE:
            self.body_box = None
            return
        self.body_box = msg
    
    def posture_cb(self, msg: String):
        """Riceve postura"""
        self.posture = msg.data
    
    def generate_goal(self):
        """
        Genera goal pose intelligente basato su:
        - Posizione persona (bounding box)
        - Postura (standing/sitting/lying)
        - Proxemics (distanze sociali)
        """
        if self.body_box is None:
            return
        
        # Rate limiting: non aggiornare troppo spesso
        if self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9
            if elapsed < self.min_interval:
                return
        
        # Centro bounding box (posizione persona in frame camera)
        person_x = self.body_box.pose.position.x
        person_y = self.body_box.pose.position.y
        person_z = self.body_box.pose.position.z
        
        box_size_x = self.body_box.scale.x
        box_size_y = self.body_box.scale.y
        box_size_z = self.body_box.scale.z
        
        # --- CALCOLA GOAL POSE BASATO SU POSTURA ---
        
        if self.posture == "LYING":
            # Persona sdraiata: approccio LATERALE a distanza sociale
            goal_pose = self._compute_lateral_approach(
                person_x, person_y, person_z,
                box_size_x, box_size_y,
                offset=self.lying_offset
            )
            approach_type = "LATERAL (lying)"
        
        elif self.posture == "SITTING":
            # Persona seduta: approccio FRONTALE a distanza personale (più vicino)
            goal_pose = self._compute_frontal_approach(
                person_x, person_y, person_z,
                box_size_x, box_size_y,
                distance=self.sitting_dist
            )
            approach_type = "FRONTAL (sitting)"
        
        elif self.posture == "STANDING":
            # Persona in piedi: approccio FRONTALE a distanza sociale
            goal_pose = self._compute_frontal_approach(
                person_x, person_y, person_z,
                box_size_x, box_size_y,
                distance=self.standing_dist
            )
            approach_type = "FRONTAL (standing)"
        
        else:
            # Postura sconosciuta: approccio conservativo a distanza sociale
            goal_pose = self._compute_frontal_approach(
                person_x, person_y, person_z,
                box_size_x, box_size_y,
                distance=self.social_dist
            )
            approach_type = "CONSERVATIVE (unknown)"
        
        if goal_pose is None:
            return
        
        # Pubblica goal
        self.goal_pub.publish(goal_pose)
        self.last_goal_time = self.get_clock().now()
        
        # Visualizza marker RViz
        self._publish_goal_marker(goal_pose)
        
        self.get_logger().info(
            f'📍 New goal: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}) '
            f'[{approach_type}]',
            throttle_duration_sec=3.0
        )
    
    def _compute_frontal_approach(self, px, py, pz, sx, sy, distance):
        """
        Calcola goal per approccio frontale alla persona
        Goal = persona - distance lungo direzione robot→persona
        """
        # Vettore da origine camera (Spot) a persona
        dx = px
        dy = py
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 1e-3:
            return None
        
        # Direzione normalizzata
        dir_x = dx / dist
        dir_y = dy / dist
        
        # Goal = persona - distance * direzione
        # (ci fermiamo a 'distance' metri dalla persona)
        goal_x = px - distance * dir_x
        goal_y = py - distance * dir_y
        
        # Orientamento: guarda verso la persona
        goal_yaw = math.atan2(py - goal_y, px - goal_x)
        
        return self._create_pose_stamped(goal_x, goal_y, goal_yaw)
    
    def _compute_lateral_approach(self, px, py, pz, sx, sy, offset):
        """
        Calcola goal per approccio laterale (persona sdraiata)
        Goal = a lato della persona, a distanza sociale
        """
        # Vettore robot→persona
        dx = px
        dy = py
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 1e-3:
            return None
        
        # Vettore perpendicolare (ruotato 90°)
        perp_x = -dy / dist
        perp_y = dx / dist
        
        # Goal = centro persona + offset laterale + metà larghezza box
        goal_x = px + perp_x * (sy/2 + offset)
        goal_y = py + perp_y * (sy/2 + offset)
        
        # Orientamento: guarda verso la persona
        goal_yaw = math.atan2(py - goal_y, px - goal_x)
        
        return self._create_pose_stamped(goal_x, goal_y, goal_yaw)
    
    def _create_pose_stamped(self, x, y, yaw):
        """Crea PoseStamped con orientamento quaternion"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.goal_frame
        
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # Converti yaw → quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        
        return pose
    
    def _publish_goal_marker(self, goal_pose):
        """Pubblica marker arrow per visualizzare goal in RViz"""
        marker = Marker()
        marker.header = goal_pose.header
        marker.ns = "human_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose = goal_pose.pose
        
        marker.scale.x = 0.5  # Lunghezza freccia
        marker.scale.y = 0.1  # Larghezza freccia
        marker.scale.z = 0.1
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.goal_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HumanTargetGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
