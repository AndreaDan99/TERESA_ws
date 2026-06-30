#!/usr/bin/env python3
"""
capture_exposure.py — TERESA exposure photo capture.

Minimal: spin_once loop + non-blocking stdin.
  [w+ENTER] = wide shot
  [ENTER]   = close-up
  [q+ENTER] = quit

Usage (inside teresa_gpu):
  source /opt/ros/humble/install/setup.bash
  python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
"""

import argparse
import select
import sys
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

        self._sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._on_color, qos_profile_sensor_data)
        self._sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._on_depth, qos_profile_sensor_data)
        self._sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._on_info, qos_profile_sensor_data)

        print(f'📸 TERESA Exposure Capture\n'
              f'   Out: {self.out_dir}\n'
              f'   [w+ENTER]=wide  [ENTER]=close-up  [q+ENTER]=quit\n')

    def _on_color(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).copy().reshape(msg.height, msg.width, 3)
        self._color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else data
        self._frame_n += 1

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

    def _save_wide(self):
        if self._color is None:
            print('⚠ No frame yet'); return
        cv2.imwrite(str(self.out_dir / 'wide_color.png'), self._color)
        d = self._depth
        cv2.imwrite(str(self.out_dir / 'wide_depth.png'),
                   d if d is not None else np.zeros(self._color.shape[:2], dtype=np.uint16))
        print(f'✓ WIDE  (frame #{self._frame_n})')

    def _save_close_up(self):
        if self._color is None:
            print('⚠ No frame yet'); return
        self._counter += 1
        cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_color.png'), self._color)
        d = self._depth
        cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_depth.png'),
                   d if d is not None else np.zeros(self._color.shape[:2], dtype=np.uint16))
        print(f'✓ CLOSE-UP #{self._counter}  (frame #{self._frame_n})')

    def run(self):
        status_t = self.get_clock().now()
        tick = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            tick += 1

            # Check stdin only every 20 spins (1 Hz) to not starve frame delivery
            if tick % 20 == 0 and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line:
                    line = line.strip().lower()
                    # Flush to get latest frame
                    for _ in range(6):
                        rclpy.spin_once(self, timeout_sec=0.02)
                    if line == 'w':
                        self._save_wide()
                    elif line == 'q':
                        print(f'Done. {self._counter} close-ups saved.')
                        return
                    elif line == '':
                        self._save_close_up()

            now = self.get_clock().now()
            if (now - status_t).nanoseconds * 1e-9 > 4.0:
                print(f'  [frames: {self._frame_n}  close-ups: {self._counter}]')
                status_t = now


def main():
    ap = argparse.ArgumentParser(description='TERESA exposure photo capture')
    ap.add_argument('--out_dir', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = ExposureCapture(args.out_dir)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
