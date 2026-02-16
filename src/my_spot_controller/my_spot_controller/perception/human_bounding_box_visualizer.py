#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

# Indici COCO torso
L_SHOULDER = 5
R_SHOULDER = 6
L_HIP      = 11
R_HIP      = 12


class HumanBoundingBoxVisualizer(Node):
    """
    Visualizza due bounding box 3D:
    - una che circonda tutta la persona (con margine di sicurezza)
    - una che circonda solo il busto (spalle + anche) - PRECISA
    
    Input: /human_pose/points_3d (PoseArray con 17 keypoints COCO)
    Output:
      /human_pose/bounding_box      (Marker CUBE, persona intera)
      /human_pose/torso_bounding_box (Marker CUBE, solo busto)
    """

    def __init__(self):
        super().__init__("human_bounding_box_visualizer")

        # Parametri
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("safety_margin_body", 0.5)   # margine attorno alla persona
        self.declare_parameter("torso_width_scale", 1.3)    # scala larghezza spalle
        self.declare_parameter("torso_depth_scale", 1.2)    # scala profondità torso

        self.frame_id = self.get_parameter("frame_id").value
        self.margin_body = float(self.get_parameter("safety_margin_body").value)
        self.torso_width_scale = float(self.get_parameter("torso_width_scale").value)
        self.torso_depth_scale = float(self.get_parameter("torso_depth_scale").value)

        # Subscriber - keypoints 3D
        self.sub = self.create_subscription(
            PoseArray,
            "/human_pose/points_3d",
            self.cb_points,
            10
        )

        # Publisher - box marker corpo intero
        self.pub_marker_body = self.create_publisher(
            Marker,
            "/human_pose/bounding_box",
            10
        )

        # Publisher - box marker busto
        self.pub_marker_torso = self.create_publisher(
            Marker,
            "/human_pose/torso_bounding_box",
            10
        )

        self.get_logger().info(
            f"✅ Human Bounding Box Visualizer READY "
            f"(margin_body={self.margin_body}m, torso_scale=w:{self.torso_width_scale} d:{self.torso_depth_scale})"
        )

    def cb_points(self, msg: PoseArray):
        """
        Riceve keypoints, calcola:
        - bounding box 3D attorno alla persona intera
        - bounding box 3D attorno al busto (spalle+anche) PRECISA
        """
        # ------------------------------
        # 1) Corpo intero: tutti i punti validi
        # ------------------------------
        pts_all = []
        for p in msg.poses:
            if not math.isnan(p.position.x):
                pts_all.append(
                    np.array([p.position.x, p.position.y, p.position.z],
                             dtype=np.float64)
                )

        if len(pts_all) < 4:
            # Troppo pochi punti: nascondi entrambe le box
            self.publish_box(
                center=None, size=None, stamp=msg.header.stamp,
                ns="human_bounding", marker_id=0,
                color=(1.0, 0.0, 0.0, 0.0),  # trasparente → delete
                publisher=self.pub_marker_body
            )
            self.publish_box(
                center=None, size=None, stamp=msg.header.stamp,
                ns="torso_bounding", marker_id=1,
                color=(0.0, 1.0, 0.0, 0.0),
                publisher=self.pub_marker_torso
            )
            return

        pts_all_arr = np.array(pts_all)  # shape (N, 3)

        min_x = np.min(pts_all_arr[:, 0])
        max_x = np.max(pts_all_arr[:, 0])
        min_y = np.min(pts_all_arr[:, 1])
        max_y = np.max(pts_all_arr[:, 1])
        min_z = np.min(pts_all_arr[:, 2])
        max_z = np.max(pts_all_arr[:, 2])

        center_body = np.array([
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
            (min_z + max_z) / 2.0
        ])

        size_body = np.array([
            (max_x - min_x) + 2 * self.margin_body,
            (max_y - min_y) + 2 * self.margin_body,
            (max_z - min_z) + 2 * self.margin_body
        ])

        # ------------------------------
        # 2) Busto PRECISO: calcolo geometrico
        # ------------------------------
        center_torso = None
        size_torso = None

        # Estrai i 4 punti del torso
        torso_indices = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
        torso_points = {}
        
        for idx in torso_indices:
            if idx < len(msg.poses):
                p = msg.poses[idx]
                if not math.isnan(p.position.x):
                    torso_points[idx] = np.array([p.position.x, p.position.y, p.position.z])

        # Serve almeno 3 punti su 4 per calcolare il torso
        if len(torso_points) >= 3:
            # Calcola centro spalle e centro anche
            shoulder_points = []
            hip_points = []
            
            if L_SHOULDER in torso_points:
                shoulder_points.append(torso_points[L_SHOULDER])
            if R_SHOULDER in torso_points:
                shoulder_points.append(torso_points[R_SHOULDER])
            if L_HIP in torso_points:
                hip_points.append(torso_points[L_HIP])
            if R_HIP in torso_points:
                hip_points.append(torso_points[R_HIP])
            
            if len(shoulder_points) > 0 and len(hip_points) > 0:
                shoulder_center = np.mean(shoulder_points, axis=0)
                hip_center = np.mean(hip_points, axis=0)
                
                # Centro torso = punto medio tra spalle e anche
                center_torso = 0.5 * (shoulder_center + hip_center)
                
                # Calcola dimensioni basate su geometria del corpo
                # Larghezza (X): distanza tra spalle (se disponibili entrambe)
                if L_SHOULDER in torso_points and R_SHOULDER in torso_points:
                    shoulder_width = np.linalg.norm(
                        torso_points[R_SHOULDER] - torso_points[L_SHOULDER]
                    )
                else:
                    # Stima dalla distanza anche
                    if L_HIP in torso_points and R_HIP in torso_points:
                        shoulder_width = np.linalg.norm(
                            torso_points[R_HIP] - torso_points[L_HIP]
                        ) * 1.1  # spalle leggermente più larghe
                    else:
                        shoulder_width = 0.4  # default 40cm
                
                # Altezza (Y): distanza spalle-anche
                torso_height = np.linalg.norm(shoulder_center - hip_center)
                
                # Profondità (Z): stima da larghezza spalle
                torso_depth = shoulder_width * 0.5  # torso ~metà della larghezza
                
                # Applica scale factors per margine di sicurezza
                size_torso = np.array([
                    shoulder_width * self.torso_width_scale,
                    torso_height * 1.2,  # +20% in altezza
                    torso_depth * self.torso_depth_scale
                ])

        # ------------------------------
        # 3) Pubblica entrambe le box
        # ------------------------------
        # Corpo intero: rosso trasparente
        self.publish_box(
            center=center_body, size=size_body, stamp=msg.header.stamp,
            ns="human_bounding", marker_id=0,
            color=(1.0, 0.0, 0.0, 0.25),  # RGBA
            publisher=self.pub_marker_body
        )

        # Busto: verde trasparente (se disponibile)
        if center_torso is not None:
            self.publish_box(
                center=center_torso, size=size_torso, stamp=msg.header.stamp,
                ns="torso_bounding", marker_id=1,
                color=(0.0, 1.0, 0.0, 0.3),
                publisher=self.pub_marker_torso
            )
        else:
            # se non riesce a stimare il busto, rimuove marker busto
            self.publish_box(
                center=None, size=None, stamp=msg.header.stamp,
                ns="torso_bounding", marker_id=1,
                color=(0.0, 1.0, 0.0, 0.0),
                publisher=self.pub_marker_torso
            )

    def publish_box(self, center, size, stamp, ns, marker_id, color, publisher):
        """
        Pubblica marker CUBE in RViz.
        Se center=None, manda DELETE.
        """
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = Marker.CUBE

        if center is None or size is None:
            m.action = Marker.DELETE
        else:
            m.action = Marker.ADD

            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])

            m.pose.orientation.w = 1.0  # allineato al frame camera

            m.scale.x = float(size[0])
            m.scale.y = float(size[1])
            m.scale.z = float(size[2])

            m.color.r, m.color.g, m.color.b, m.color.a = color

        publisher.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = HumanBoundingBoxVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
