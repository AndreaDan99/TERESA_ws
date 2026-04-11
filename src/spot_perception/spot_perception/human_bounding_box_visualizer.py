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
      /human_pose/bounding_box       (Marker CUBE, persona intera)
      /human_pose/torso_bounding_box (Marker CUBE, solo busto)
    """

    def __init__(self):
        super().__init__("human_bounding_box_visualizer")

        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("safety_margin_body", 0.5)
        self.declare_parameter("torso_width_scale", 1.3)
        self.declare_parameter("torso_depth_scale", 1.2)

        self.frame_id = self.get_parameter("frame_id").value
        self.margin_body = float(self.get_parameter("safety_margin_body").value)
        self.torso_width_scale = float(self.get_parameter("torso_width_scale").value)
        self.torso_depth_scale = float(self.get_parameter("torso_depth_scale").value)

        self.sub = self.create_subscription(
            PoseArray,
            "/human_pose/points_3d",
            self.cb_points,
            10
        )

        self.pub_marker_body = self.create_publisher(
            Marker, "/human_pose/bounding_box", 10
        )

        self.pub_marker_torso = self.create_publisher(
            Marker, "/human_pose/torso_bounding_box", 10
        )

        self.get_logger().info(
            f"✅ Human Bounding Box Visualizer READY "
            f"(margin_body={self.margin_body}m, torso_scale=w:{self.torso_width_scale} d:{self.torso_depth_scale})"
        )

    def cb_points(self, msg: PoseArray):
        # 1) Corpo intero
        pts_all = []
        for p in msg.poses:
            if not math.isnan(p.position.x):
                pts_all.append(
                    np.array([p.position.x, p.position.y, p.position.z], dtype=np.float64)
                )

        if len(pts_all) < 4:
            self.publish_box(None, None, msg.header.stamp, "human_bounding", 0,
                             (1.0, 0.0, 0.0, 0.0), self.pub_marker_body)
            self.publish_box(None, None, msg.header.stamp, "torso_bounding", 1,
                             (0.0, 1.0, 0.0, 0.0), self.pub_marker_torso)
            return

        pts_all_arr = np.array(pts_all)

        min_x, max_x = np.min(pts_all_arr[:, 0]), np.max(pts_all_arr[:, 0])
        min_y, max_y = np.min(pts_all_arr[:, 1]), np.max(pts_all_arr[:, 1])
        min_z, max_z = np.min(pts_all_arr[:, 2]), np.max(pts_all_arr[:, 2])

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

        # 2) Busto PRECISO
        center_torso = None
        size_torso = None

        torso_indices = [L_SHOULDER, R_SHOULDER, L_HIP, R_HIP]
        torso_points = {}

        for idx in torso_indices:
            if idx < len(msg.poses):
                p = msg.poses[idx]
                if not math.isnan(p.position.x):
                    torso_points[idx] = np.array([p.position.x, p.position.y, p.position.z])

        if len(torso_points) >= 3:
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
                center_torso = 0.5 * (shoulder_center + hip_center)

                if L_SHOULDER in torso_points and R_SHOULDER in torso_points:
                    shoulder_width = np.linalg.norm(
                        torso_points[R_SHOULDER] - torso_points[L_SHOULDER]
                    )
                else:
                    if L_HIP in torso_points and R_HIP in torso_points:
                        shoulder_width = np.linalg.norm(
                            torso_points[R_HIP] - torso_points[L_HIP]
                        ) * 1.1
                    else:
                        shoulder_width = 0.4

                torso_height = np.linalg.norm(shoulder_center - hip_center)
                torso_depth = shoulder_width * 0.5

                size_torso = np.array([
                    shoulder_width * self.torso_width_scale,
                    torso_height * 1.2,
                    torso_depth * self.torso_depth_scale
                ])

        # 3) Pubblica
        self.publish_box(center_body, size_body, msg.header.stamp,
                         "human_bounding", 0, (1.0, 0.0, 0.0, 0.25), self.pub_marker_body)

        if center_torso is not None:
            self.publish_box(center_torso, size_torso, msg.header.stamp,
                             "torso_bounding", 1, (0.0, 1.0, 0.0, 0.3), self.pub_marker_torso)
        else:
            self.publish_box(None, None, msg.header.stamp,
                             "torso_bounding", 1, (0.0, 1.0, 0.0, 0.0), self.pub_marker_torso)

    def publish_box(self, center, size, stamp, ns, marker_id, color, publisher):
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
            m.pose.orientation.w = 1.0
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
