#!/usr/bin/env python3
"""
capture_exposure.py — TERESA exposure photo capture.

Trigger: [w]+ENTER = wide, [ENTER] = close-up, [q]+ENTER = quit.
Save happens INSIDE the frame callback → always the latest frame.

Usage (inside teresa_gpu):
  source /opt/ros/humble/install/setup.bash
  python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
"""

import argparse
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data


class ExposureCapture(Node):
    def __init__(self, out_dir):
        super().__init__('exposure_capture')
        self.out_dir = Path(out_dir)
        self.cu_dir = self.out_dir / 'close_up'
        self.cu_dir.mkdir(parents=True, exist_ok=True)

        self._color = None
        self._depth = None
        self._info = None
        self._counter = 0
        self._frame_n = 0

        # Commands from stdin thread
        self._cmd = None
        self._running = True

        self._sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._on_color, qos_profile_sensor_data)
        self._sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._on_depth, qos_profile_sensor_data)
        self._sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._on_info, qos_profile_sensor_data)

        # Status timer
        self.create_timer(4.0, self._tick_status)

        print(f'📸 TERESA Exposure Capture\n'
              f'   Out: {self.out_dir}\n'
              f'   [w+ENTER]=wide  [ENTER]=close-up  [q+ENTER]=quit\n')

    # ── Frame callback (main thread) ─────────────────────────────

    def _on_color(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).copy().reshape(msg.height, msg.width, 3)
        self._color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else data
        self._frame_n += 1

        # Process pending command — save happens HERE with freshest frame
        cmd = self._cmd
        if cmd is not None:
            self._cmd = None
            if cmd == 'w':
                self._save_wide()
            elif cmd == 'c':
                self._save_close_up()
            elif cmd == 'q':
                print(f'Done. {self._counter} close-ups saved.')
                self._running = False
                self.create_timer(0.1, lambda: rclpy.shutdown())

    def _on_depth(self, msg):
        self._depth = np.frombuffer(msg.data, dtype=np.uint16).copy().reshape(msg.height, msg.width)

    def _on_info(self, msg):
        if self._info is None:
            self._info = msg
            self._save_camera_info()

    def _save_camera_info(self):
        import json
        K = self._info.k
        info = {'fx': float(K[0]), 'fy': float(K[4]), 'cx': float(K[2]), 'cy': float(K[5]),
                'width': self._info.width, 'height': self._info.height}
        with open(self.out_dir / 'camera_info.json', 'w') as f:
            json.dump(info, f, indent=2)
        print('✓ camera_info.json saved')

    # ── Save (called from callback on main thread) ────────────────

    def _save_wide(self):
        cv2.imwrite(str(self.out_dir / 'wide_color.png'), self._color)
        d = self._depth
        cv2.imwrite(str(self.out_dir / 'wide_depth.png'),
                   d if d is not None else np.zeros(self._color.shape[:2], dtype=np.uint16))
        print(f'✓ WIDE  (frame #{self._frame_n})')

    def _save_close_up(self):
        self._counter += 1
        cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_color.png'), self._color)
        d = self._depth
        cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_depth.png'),
                   d if d is not None else np.zeros(self._color.shape[:2], dtype=np.uint16))
        print(f'✓ CLOSE-UP #{self._counter}  (frame #{self._frame_n})')

    # ── Status ───────────────────────────────────────────────────

    def _tick_status(self):
        print(f'  [frames: {self._frame_n}  close-ups: {self._counter}]')


# ═══════════════════════════════════════════════════════════════════

def stdin_thread(node):
    """Read stdin in background thread, set command flag for main thread."""
    while node._running:
        try:
            line = sys.stdin.readline()
            if not line:
                break  # EOF
            line = line.strip().lower()
            if line == 'w':
                node._cmd = 'w'
            elif line == 'q':
                node._cmd = 'q'
                break
            elif line == '':
                node._cmd = 'c'
        except (EOFError, KeyboardInterrupt):
            break


def main():
    ap = argparse.ArgumentParser(description='TERESA exposure photo capture')
    ap.add_argument('--out_dir', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = ExposureCapture(args.out_dir)

    # Start stdin reader in background thread
    t = threading.Thread(target=stdin_thread, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
