#!/usr/bin/env python3
"""
capture_exposure.py — Interactive photo capture for TERESA exposure experiment.

Shows a live preview from the RealSense (hand-held).
  [W]     → save wide shot from Orbbec (color)
  [SPACE] → save close-up from RealSense (color + depth)
  [Q]     → quit

Photos are saved to --out_dir in the structure expected by run_exposure_offline.py:
  wide_color.png / wide_depth.png
  close_up/01_color.png / 01_depth.png / ...

Usage (inside teresa_gpu container):
  python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ExposureCapture(Node):
    def __init__(self, out_dir):
        super().__init__('exposure_capture')
        self.bridge = CvBridge()
        self.out_dir = Path(out_dir)
        self.close_up_dir = self.out_dir / 'close_up'
        self.close_up_dir.mkdir(parents=True, exist_ok=True)

        # ── State ──────────────────────────────────────────────
        self._rs_color = None       # latest RealSense color (BGR)
        self._rs_depth = None       # latest RealSense depth (mm, np.ndarray)
        self._rs_info = None        # CameraInfo
        self._orbbec_color = None   # latest Orbbec color (BGR)
        self._counter = 1           # close-up photo number
        self._last_save_time = 0.0

        # ── Subscribers ────────────────────────────────────────
        self._sub_rs_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self._cb_rs_color, 10)
        self._sub_rs_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._cb_rs_depth, 10)
        self._sub_rs_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._cb_rs_info, 10)
        self._sub_orb_color = self.create_subscription(
            Image, '/orbbec/color/image_raw',
            self._cb_orb_color, 10)

        # ── Camera info auto-save ──────────────────────────────
        self.create_timer(2.0, self._try_save_camera_info)

        # ── Preview at 10 Hz ───────────────────────────────────
        self.create_timer(0.1, self._preview)

        self.get_logger().info(
            f'📸 TERESA Exposure Capture\n'
            f'   Out: {self.out_dir}\n'
            f'   [W] Wide shot (Orbbec)\n'
            f'   [SPACE] Close-up (RealSense)\n'
            f'   [Q] Quit'
        )

    def _cb_rs_color(self, msg):
        try:
            self._rs_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _cb_rs_depth(self, msg):
        try:
            self._rs_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception:
            pass

    def _cb_rs_info(self, msg):
        self._rs_info = msg

    def _cb_orb_color(self, msg):
        try:
            self._orbbec_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _try_save_camera_info(self):
        """Save camera_info.json once we have the intrinsics."""
        ci_path = self.out_dir / 'camera_info.json'
        if ci_path.exists() or self._rs_info is None:
            return
        K = self._rs_info.k
        import json
        info = {
            'fx': float(K[0]), 'fy': float(K[4]),
            'cx': float(K[2]), 'cy': float(K[5]),
            'width': self._rs_info.width, 'height': self._rs_info.height,
        }
        with open(ci_path, 'w') as f:
            json.dump(info, f, indent=2)
        self.get_logger().info(f'✓ Camera info saved → {ci_path}')

    # ── Preview ────────────────────────────────────────────────

    def _preview(self):
        if self._rs_color is None:
            return

        frame = self._rs_color.copy()
        H, W = frame.shape[:2]

        # Overlay info
        cv2.putText(frame, f"Close-up: {self._counter}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "[W]ide  [SPACE]close-up  [Q]uit", (10, H - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # Status indicator
        now = time.time()
        if now - self._last_save_time < 0.4:
            cv2.circle(frame, (W - 20, 20), 8, (0, 255, 0), -1)  # green flash

        cv2.imshow('TERESA Exposure Capture (RealSense preview)', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('w'):
            self._save_wide()
        elif key == 32:   # SPACE
            self._save_close_up()
        elif key == ord('q'):
            self.get_logger().info('Quitting…')
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)

    # ── Save ───────────────────────────────────────────────────

    def _save_wide(self):
        if self._orbbec_color is None:
            self.get_logger().warn('⚠ No Orbbec frame yet — wait for the stream')
            return

        color_path = self.out_dir / 'wide_color.png'
        cv2.imwrite(str(color_path), self._orbbec_color)
        self._last_save_time = time.time()
        self.get_logger().info(f'✓ Wide (Orbbec) → {color_path}')

        # Orbbec doesn't provide aligned depth easily;
        # we save a placeholder so the script doesn't break.
        depth_path = self.out_dir / 'wide_depth.png'
        if not depth_path.exists():
            dummy = np.zeros((480, 640), dtype=np.uint16)
            cv2.imwrite(str(depth_path), dummy)
            self.get_logger().info(f'  Depth placeholder → {depth_path}')

    def _save_close_up(self):
        if self._rs_color is None:
            self.get_logger().warn('⚠ No RealSense frame yet — wait for the stream')
            return

        color_path = self.close_up_dir / f'{self._counter:02d}_color.png'
        depth_path = self.close_up_dir / f'{self._counter:02d}_depth.png'

        cv2.imwrite(str(color_path), self._rs_color)

        if self._rs_depth is not None:
            cv2.imwrite(str(depth_path), self._rs_depth)
        else:
            self.get_logger().warn('  No depth frame — saving placeholder')
            dummy = np.zeros(self._rs_color.shape[:2], dtype=np.uint16)
            cv2.imwrite(str(depth_path), dummy)

        self._last_save_time = time.time()
        self.get_logger().info(
            f'✓ Close-up #{self._counter} → {color_path}')
        self._counter += 1


# ═══════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description='TERESA exposure photo capture')
    ap.add_argument('--out_dir', required=True,
                    help='Experiment directory (e.g. /work/exposure/exp_01)')
    args = ap.parse_args()

    rclpy.init()
    node = ExposureCapture(args.out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
