#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float32
import numpy as np
import math

# ============================================================
# SKELETON EDGES (COCO 17 keypoints)
# ============================================================
SKELETON_EDGES = [
    (0, 1), (0, 2),           # Nose → Eyes
    (1, 3), (2, 4),           # Eyes → Ears
    (0, 5), (0, 6),           # Nose → Shoulders
    (5, 6),                   # Shoulders
    (5, 7), (7, 9),           # Left arm
    (6, 8), (8, 10),          # Right arm
    (5, 11), (6, 12),         # Torso
    (11, 12),                 # Hips
    (11, 13), (13, 15),       # Left leg
    (12, 14), (14, 16),       # Right leg
]


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def compute_angle(p1, p2, p3):
    """
    Calcola angolo al punto p2 formato da p1-p2-p3.
    
    Args:
        p1, p2, p3: numpy arrays (3D points)
    
    Returns:
        float: angolo in gradi (0-180), o None se invalido
    """
    if p1 is None or p2 is None or p3 is None:
        return None
    
    v1 = p1 - p2
    v2 = p3 - p2
    
    len_v1 = np.linalg.norm(v1)
    len_v2 = np.linalg.norm(v2)
    
    if len_v1 < 1e-6 or len_v2 < 1e-6:
        return None
    
    v1_norm = v1 / len_v1
    v2_norm = v2 / len_v2
    
    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    
    return np.rad2deg(angle_rad)


def percentile_height(points, up_axis, percentile):
    """
    Calcola altezza al percentile specificato lungo up_axis.
    
    Args:
        points: numpy array (N, 3)
        up_axis: numpy array (3,) - vettore verticale [0, 0, 1]
        percentile: int (0-100)
    
    Returns:
        float: altezza in metri, o None se insufficienti punti
    """
    if len(points) < 2:
        return None
    
    heights = np.dot(points, up_axis)
    return float(np.percentile(heights, percentile))



class PostureClassifier(Node):
    def __init__(self):
        super().__init__('posture_classifier')
        
        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('frame_id', 'my_spot/odom')
        self.declare_parameter('up_axis', [0.0, 0.0, 1.0])
        
        # ✅ AGGIUNGI QUESTI SE MANCANO
        self.declare_parameter('hip_to_base_stand', 0.35)
        self.declare_parameter('hip_to_base_sit', 0.20)
        self.declare_parameter('lying_height_max', 0.45)
        self.declare_parameter('lying_angle_min', 65.0)
        
        # Leggi parametri
        frame_id = self.get_parameter('frame_id').value
        up_axis = self.get_parameter('up_axis').value
        
        # ✅ LEGGI I PARAMETRI
        self.hip_to_base_stand = float(self.get_parameter('hip_to_base_stand').value)
        self.hip_to_base_sit = float(self.get_parameter('hip_to_base_sit').value)
        self.lying_height_max = float(self.get_parameter('lying_height_max').value)
        self.lying_angle_min = float(self.get_parameter('lying_angle_min').value)
        
        # Setup frame e up vector
        self.up = np.array(up_axis, dtype=np.float64)
        
        # Subscribers
        self.sub_points = self.create_subscription(
            PoseArray, '/human_pose/points_3d',
            self.cb_points, 10
        )
        
        # Publishers
        self.pub_posture = self.create_publisher(String, '/human_pose/posture', 10)
        self.pub_conf = self.create_publisher(Float32, '/human_pose/posture_confidence', 10)
        self.pub_height = self.create_publisher(Float32, '/human_pose/body_height_m', 10)
        self.pub_angle = self.create_publisher(Float32, '/human_pose/torso_angle_deg', 10)
        self.pub_torso_marker = self.create_publisher(Marker, '/human_pose/torso_marker', 10)
        
        # Skeleton edges per visualizzazione
        self.edges = SKELETON_EDGES
        
        self.get_logger().info(
            f'✅ Posture Classifier initialized\n'
            f'   Frame: {frame_id}\n'
            f'   Up axis: {up_axis}\n'
            f'   Hip-to-base thresholds: stand={self.hip_to_base_stand}m, sit={self.hip_to_base_sit}m\n'
            f'   Lying thresholds: height<{self.lying_height_max}m, angle>{self.lying_angle_min}°'
        )

    # ============================================================
    # Callback - OTTIMIZZATO
    # ============================================================
    
    def cb_points(self, msg: PoseArray):
        """
        Riceve skeleton 3D e classifica postura.
        msg contiene SEMPRE 17 Pose (alcune potrebbero essere NaN).
        """
        #Estrai sempre 17 punti
        pts_list = []
        for pose in msg.poses:
            p = pose.position
            
            # Check NaN
            if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
                pts_list.append(None)
            else:
                pts_list.append(np.array([p.x, p.y, p.z], dtype=np.float64))
        
        #Verifica lunghezza
        if len(pts_list) != 17:
            self.get_logger().warn(
                f'Received {len(pts_list)} points instead of 17!',
                throttle_duration_sec=2.0
            )
            return
        
        # Stima postura
        posture, conf, height, angle, origin, vec = self.estimate_posture(pts_list)
        
        # Pubblica risultati
        self.pub_posture.publish(String(data=posture))
        self.pub_conf.publish(Float32(data=conf))
        self.pub_height.publish(Float32(data=height))
        self.pub_angle.publish(Float32(data=angle))
        
        # Pubblica marker torso
        if origin is not None and vec is not None:
            self.publish_torso_marker(origin, vec, msg.header.stamp)

    
    # ============================================================
    # Core logic 
    # ============================================================
    
    def estimate_posture(self, pts_list):
        """Stima postura da lista keypoints 3D."""
        
        # Validazione input
        if not pts_list or len(pts_list) < 17:
            self.get_logger().warn(
                f'Invalid input: need 17 points, got {len(pts_list) if pts_list else 0}',
                throttle_duration_sec=2.0
            )
            return "UNKNOWN", 0.0, float('nan'), float('nan'), None, None
        
        pts = pts_list
        
        # Estrai keypoints validi
        valid_points = []
        for p in pts:
            if p is not None:
                valid_points.append(p)
        
        # Check minimo keypoints
        if len(valid_points) < 4:
            self.get_logger().warn(
                f'Too few valid points: {len(valid_points)}/17',
                throttle_duration_sec=2.0
            )
            return "UNKNOWN", 0.0, float('nan'), float('nan'), None, None
        
        valid_array = np.array(valid_points)
        
        # ============================================================
        # METRICA 1: Body Height
        # ============================================================
        h10 = percentile_height(valid_array, self.up, 10)
        h90 = percentile_height(valid_array, self.up, 90)
        height = h90 - h10 if (h10 is not None and h90 is not None) else 0.0
        
        # Indici COCO
        L_HIP, R_HIP = 11, 12
        L_ANKLE, R_ANKLE = 15, 16
        L_SHOULDER, R_SHOULDER = 5, 6
        L_KNEE, R_KNEE = 13, 14
        
        # ============================================================
        # METRICA 2: Hip-to-Base Distance
        # ============================================================
        hip_to_base = None
        hip_mid = None  # ✅ INIZIALIZZA QUI per evitare UnboundLocalError
        
        if pts[L_HIP] is not None and pts[R_HIP] is not None:
            hip_mid = 0.5 * (pts[L_HIP] + pts[R_HIP])
            
            # Base = media caviglie
            base_pts = []
            if pts[L_ANKLE] is not None:
                base_pts.append(pts[L_ANKLE])
            if pts[R_ANKLE] is not None:
                base_pts.append(pts[R_ANKLE])
            
            if len(base_pts) > 0:
                base = np.mean(base_pts, axis=0)
                hip_to_base = abs(np.dot(hip_mid - base, self.up))
        
        # ============================================================
        # METRICA 3: Torso Angle
        # ============================================================
        torso_angle = None
        sh_mid = None
        torso_vec = None
        
        if pts[L_SHOULDER] is not None and pts[R_SHOULDER] is not None:
            sh_mid = 0.5 * (pts[L_SHOULDER] + pts[R_SHOULDER])
            
            # ✅ CHECK: hip_mid deve esistere prima di calcolare torso
            if hip_mid is not None:
                torso_vec = sh_mid - hip_mid
                torso_len = np.linalg.norm(torso_vec)
                
                if torso_len > 0.05:
                    torso_vec_norm = torso_vec / torso_len
                    cos_angle = np.clip(np.dot(torso_vec_norm, self.up), -1.0, 1.0)
                    torso_angle = np.arccos(cos_angle)
        
        # ============================================================
        # CLASSIFICAZIONE
        # ============================================================
        
        # CASO 1: LYING (priorità alta)
        if torso_angle is not None and height < self.lying_height_max:
            if torso_angle > np.deg2rad(self.lying_angle_min):
                return "LYING", 0.75, height, np.rad2deg(torso_angle), hip_mid, torso_vec
        
        # CASO 2: STANDING
        if hip_to_base is not None:
            if hip_to_base > self.hip_to_base_stand:
                conf = 0.6
                # Bonus confidence se ginocchia estese
                if pts[L_KNEE] is not None and pts[L_HIP] is not None and pts[L_ANKLE] is not None:
                    knee_angle = compute_angle(pts[L_HIP], pts[L_KNEE], pts[L_ANKLE])
                    if knee_angle is not None and knee_angle > 160:
                        conf += 0.1
                
                return "STANDING", conf, height, np.rad2deg(torso_angle) if torso_angle else float('nan'), hip_mid, torso_vec
        
        # CASO 3: SITTING
        if hip_to_base is not None:
            if hip_to_base < self.hip_to_base_sit:
                conf = 0.6
                # Bonus confidence se ginocchia piegate
                if pts[L_KNEE] is not None and pts[L_HIP] is not None and pts[L_ANKLE] is not None:
                    knee_angle = compute_angle(pts[L_HIP], pts[L_KNEE], pts[L_ANKLE])
                    if knee_angle is not None and knee_angle < 140:
                        conf += 0.1
                
                return "SITTING", conf, height, np.rad2deg(torso_angle) if torso_angle else float('nan'), hip_mid, torso_vec
        
        # CASO 4: UNKNOWN
        return "UNKNOWN", 0.0, height, np.rad2deg(torso_angle) if torso_angle else float('nan'), hip_mid, torso_vec

    
    def _knee_angle(self, h, k, a):
        """Calcola angolo ginocchio."""
        h = np.array(h)
        k = np.array(k)
        a = np.array(a)
        
        v1 = h - k
        v2 = a - k
        
        if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
            return None
        
        return angle_deg(v1, v2)
    
    # ============================================================
    # Visualization
    # ============================================================
    
    def publish_torso_marker(self, origin, vec, stamp):
        """Pubblica marker torso."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = "torso"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.scale.y = 0.06
        m.scale.z = 0.06
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0
        
        p0 = origin
        p1 = origin + normalize(vec) * 0.45
        m.points.append(Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])))
        m.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
        
        self.pub_marker.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PostureClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
