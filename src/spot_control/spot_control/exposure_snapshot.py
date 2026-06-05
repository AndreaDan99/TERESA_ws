#!/usr/bin/env python3
"""
exposure_snapshot.py — RealSense snapshot node for TERESA exposure review.

Captures a single JPEG frame from the Z1-mounted RealSense camera when
the user clicks a grid point in EXPOSURE_REVIEW mode (via /exposure/goto_point).
The snapshot is taken 1 second after /ik_done confirms the arm has arrived
at the revisiting pose, published on /exposure/snapshot for the web UI,
and saved to disk.
"""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import MarkerArray


class ExposureSnapshot(Node):

    def __init__(self):
        super().__init__('exposure_snapshot')

        self._output_dir = str(
            self.declare_parameter('snapshot_output_dir', '/tmp')
            .get_parameter_value().string_value
        )
        self._delay = float(
            self.declare_parameter('snapshot_delay_s', 1.0)
            .get_parameter_value().double_value
        )

        self._last_frame: Image | None = None
        self._bridge = CvBridge()
        self._snapshot_pending = False
        self._capture_deadline: float | None = None
        self._goto_idx: int = -1
        self._grid_markers: dict[int, str] = {}

        self._sub_camera = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_camera, 10)
        self._sub_goto = self.create_subscription(
            Int32, '/exposure/goto_point', self._cb_goto, 10)
        self._sub_ik = self.create_subscription(
            Bool, '/ik_done', self._cb_ik, 10)
        self._sub_grid = self.create_subscription(
            MarkerArray, '/exposure/grid_markers', self._cb_grid, 10)

        self._pub_snapshot = self.create_publisher(
            Image, '/exposure/snapshot', 10)

        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('Exposure snapshot node ready')

    def _cb_camera(self, msg: Image):
        self._last_frame = msg

    def _cb_goto(self, msg: Int32):
        idx = msg.data
        if idx < 0:
            return
        self._goto_idx = idx
        self._snapshot_pending = True
        self._capture_deadline = None
        self.get_logger().info(f'Snapshot requested for point {idx}')

    def _cb_ik(self, msg: Bool):
        if msg.data and self._snapshot_pending and self._capture_deadline is None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            self._capture_deadline = now_s + self._delay
            self.get_logger().info(
                f'IK done for point {self._goto_idx}, '
                f'snapshot in {self._delay:.1f}s')

    def _cb_grid(self, msg: MarkerArray):
        for m in msg.markers:
            ns = m.ns or ''
            if ns.startswith('exposure_grid_'):
                region = ns.replace('exposure_grid_', '')
                self._grid_markers[m.id] = region

    def _tick(self):
        if not self._snapshot_pending or self._capture_deadline is None:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s < self._capture_deadline:
            return

        self._capture_deadline = None
        self._snapshot_pending = False

        if self._last_frame is None:
            self.get_logger().warn('Snapshot skipped: no camera frame available')
            return

        region = self._grid_markers.get(self._goto_idx, 'unknown')
        label = f'{region}_{self._goto_idx}'
        frame = Image()
        frame.header.stamp = self._last_frame.header.stamp
        frame.header.frame_id = f'exposure_snapshot_{label}'
        frame.height = self._last_frame.height
        frame.width = self._last_frame.width
        frame.encoding = self._last_frame.encoding
        frame.is_bigendian = self._last_frame.is_bigendian
        frame.step = self._last_frame.step
        frame.data = self._last_frame.data

        self._pub_snapshot.publish(frame)
        self.get_logger().info(f'Snapshot published: {label}')

        try:
            cv_img = self._bridge.imgmsg_to_cv2(
                self._last_frame, desired_encoding='bgr8')
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            out_path = (Path(self._output_dir) /
                        f'exposure_snapshot_{label}_{ts}.jpg')
            cv2.imwrite(str(out_path), cv_img)
            self.get_logger().info(f'Snapshot saved: {out_path}')
        except Exception as e:
            self.get_logger().warn(f'Snapshot save failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ExposureSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
