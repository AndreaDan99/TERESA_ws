#!/usr/bin/env python3
"""
capture_exposure.py — TERESA exposure photo capture (ROS service trigger).

Trigger via ROS2 service from another terminal:
  ros2 service call /capture/trigger std_srvs/srv/Trigger   → close-up
  ros2 service call /capture/trigger std_srvs/srv/Trigger "{data: 'w'}"  → wide
  ros2 service call /capture/trigger std_srvs/srv/Trigger "{data: 'q'}"  → quit

Usage (inside teresa_gpu):
  source /opt/ros/humble/install/setup.bash
  python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


class ExposureCapture(Node):
    def __init__(self, out_dir):
        super().__init__('exposure_capture')
        self.out_dir = Path(out_dir)
        self.close_up_dir = self.out_dir / 'close_up'
        self.close_up_dir.mkdir(parents=True, exist_ok=True)

        self._rs_color = None
        self._rs_depth = None
        self._rs_info = None
        self._counter = 1
        self._frame_count = 0

        # Subscribers
        self._sub_rs_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_rs_color,
            qos_profile_sensor_data)
        self._sub_rs_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._cb_rs_depth, qos_profile_sensor_data)
        self._sub_rs_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._cb_rs_info, qos_profile_sensor_data)

        # Camera info auto-save
        self.create_timer(2.0, self._try_save_camera_info)
        # Status
        self.create_timer(4.0, self._tick_status)

        # ROS2 services for capture trigger (Trigger has no request fields in Humble)
        self._srv_cu = self.create_service(Trigger, '/capture/close_up', self._on_close_up)
        self._srv_wide = self.create_service(Trigger, '/capture/wide', self._on_wide)
        self._srv_quit = self.create_service(Trigger, '/capture/quit', self._on_quit)

        self.get_logger().info(
            '📸 TERESA Exposure Capture\n'
            f'   Out: {self.out_dir}\n'
            '   Commands (from another terminal):\n'
            '     ros2 service call /capture/close_up std_srvs/srv/Trigger  → close-up\n'
            '     ros2 service call /capture/wide     std_srvs/srv/Trigger  → wide\n'
            '     ros2 service call /capture/quit     std_srvs/srv/Trigger  → quit'
        )

    # ── Callbacks ──────────────────────────────────────────────

    def _cb_rs_color(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8).copy().reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                self._rs_color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            else:
                self._rs_color = data
            self._frame_count += 1
        except Exception as e:
            if not getattr(self, '_cb_err_logged', False):
                self.get_logger().error(f'decode error (color): {e}')
                self._cb_err_logged = True

    def _cb_rs_depth(self, msg):
        try:
            self._rs_depth = np.frombuffer(msg.data, dtype=np.uint16).copy().reshape(msg.height, msg.width)
        except Exception:
            pass

    def _cb_rs_info(self, msg):
        self._rs_info = msg

    def _try_save_camera_info(self):
        ci_path = self.out_dir / 'camera_info.json'
        if ci_path.exists() or self._rs_info is None:
            return
        import json
        K = self._rs_info.k
        info = {
            'fx': float(K[0]), 'fy': float(K[4]),
            'cx': float(K[2]), 'cy': float(K[5]),
            'width': self._rs_info.width, 'height': self._rs_info.height,
        }
        with open(ci_path, 'w') as f:
            json.dump(info, f, indent=2)
        self.get_logger().info('✓ camera_info.json saved')

    # ── Trigger services ───────────────────────────────────────

    def _on_close_up(self, request, response):
        self._save_close_up()
        response.success = True
        response.message = f'Close-up #{self._counter - 1} saved'
        return response

    def _on_wide(self, request, response):
        self._save_wide()
        response.success = True
        response.message = 'Wide shot saved'
        return response

    def _on_quit(self, request, response):
        self.get_logger().info(f'Quit. {self._counter - 1} close-ups saved.')
        response.success = True
        response.message = 'Shutting down'
        self.create_timer(0.2, lambda: rclpy.shutdown())
        return response

    # ── Save ───────────────────────────────────────────────────

    def _save_wide(self):
        if self._rs_color is None:
            self.get_logger().warn('No RealSense frame yet')
            return
        path = self.out_dir / 'wide_color.png'
        cv2.imwrite(str(path), self._rs_color)
        self.get_logger().info(f'WIDE → {path}')
        dp = self.out_dir / 'wide_depth.png'
        if self._rs_depth is not None:
            cv2.imwrite(str(dp), self._rs_depth)
        else:
            cv2.imwrite(str(dp), np.zeros(self._rs_color.shape[:2], dtype=np.uint16))

    def _save_close_up(self):
        if self._rs_color is None:
            self.get_logger().warn('No RealSense frame yet')
            return
        cp = self.close_up_dir / f'{self._counter:02d}_color.png'
        dp = self.close_up_dir / f'{self._counter:02d}_depth.png'
        cv2.imwrite(str(cp), self._rs_color)
        if self._rs_depth is not None:
            cv2.imwrite(str(dp), self._rs_depth)
        else:
            cv2.imwrite(str(dp), np.zeros(self._rs_color.shape[:2], dtype=np.uint16))
        self.get_logger().info(f'CLOSE-UP #{self._counter} → {cp}')
        self._counter += 1

    # ── Status ─────────────────────────────────────────────────

    def _tick_status(self):
        rs_ok = '✓' if self._rs_color is not None else '✗'
        self.get_logger().info(
            f'Status | RS: {rs_ok}  frames: {self._frame_count}  |  close-ups: {self._counter - 1}'
        )


def main():
    ap = argparse.ArgumentParser(description='TERESA exposure photo capture')
    ap.add_argument('--out_dir', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = ExposureCapture(args.out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
