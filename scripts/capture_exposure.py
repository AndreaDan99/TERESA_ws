#!/usr/bin/env python3
"""
capture_exposure.py — Interactive photo capture for TERESA exposure experiment.

Two modes (auto-detected):
  GUI mode   (DISPLAY set, e.g. Jetson desktop or ssh -X):
             Live OpenCV preview, press [W] for wide, [SPACE] for close-up.

  TTY mode   (no DISPLAY, e.g. plain SSH):
             Terminal-driven. Press ENTER to capture close-up, 'w'+ENTER for wide.
             Use rqt_image_view in another terminal for live preview:
               docker exec -it teresa_core bash
               rqt_image_view /camera/camera/color/image_raw

Photos saved to --out_dir in the structure expected by run_exposure_offline.py:
  wide_color.png / wide_depth.png
  close_up/01_color.png / 01_depth.png / ...

Usage (inside teresa_gpu or teresa_core container):
  python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
"""

import argparse
import os
import select
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data

HAS_DISPLAY = os.environ.get('DISPLAY', '') != ''


class ExposureCapture(Node):
    def __init__(self, out_dir):
        super().__init__('exposure_capture')
        self.out_dir = Path(out_dir)
        self.close_up_dir = self.out_dir / 'close_up'
        self.close_up_dir.mkdir(parents=True, exist_ok=True)

        self._rs_color = None
        self._rs_depth = None
        self._rs_info = None
        self._orbbec_color = None
        self._counter = 1
        self._last_save_time = 0.0
        self._running = True

        # ── Subscribers ────────────────────────────────────────
        self._sub_rs_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._cb_rs_color,
            qos_profile_sensor_data)
        self._sub_rs_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._cb_rs_depth, qos_profile_sensor_data)
        self._sub_rs_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._cb_rs_info, qos_profile_sensor_data)
        self._sub_orb_color = self.create_subscription(
            Image, '/orbbec/color/image_raw', self._cb_orb_color,
            qos_profile_sensor_data)

        # Camera info auto-save after 2 s
        self.create_timer(2.0, self._try_save_camera_info)

        mode = 'GUI (OpenCV preview)' if HAS_DISPLAY else 'TTY (terminal keys)'
        self.get_logger().info(
            f'📸 TERESA Exposure Capture  |  Mode: {mode}\n'
            f'   Out: {self.out_dir}\n'
            f'   Wide: Orbbec  |  Close-up: RealSense'
        )

        if HAS_DISPLAY:
            self.create_timer(0.1, self._tick_gui)
        else:
            self.get_logger().info(
                '   [ENTER] close-up    w+[ENTER] wide    q+[ENTER] quit\n'
                '   Preview: rqt_image_view /camera/camera/color/image_raw'
            )
            self._stdin_thread = threading.Thread(target=self._stdin_loop,
                                                  daemon=True)
            self._stdin_thread.start()
            self._pending_wide = False
            self._pending_close_up = False
            self.create_timer(0.1, self._tick_process_save)  # 10 Hz save check
            self.create_timer(4.0, self._tick_tty_status)

    # ── Callbacks ──────────────────────────────────────────────

    def _cb_rs_color(self, msg):
        try:
            # Direct numpy decode — bypasses cv_bridge for reliability
            data = np.frombuffer(msg.data, dtype=np.uint8).copy().reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                self._rs_color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                self._rs_color = data
            else:
                self._rs_color = data
            self._frame_count = getattr(self, '_frame_count', 0) + 1
        except Exception as e:
            if not getattr(self, '_cb_rs_err_logged', False):
                self.get_logger().error(f'decode error (color): {e}')
                self._cb_rs_err_logged = True

    def _cb_rs_depth(self, msg):
        try:
            self._rs_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        except Exception as e:
            if not getattr(self, '_cb_depth_err_logged', False):
                self.get_logger().error(f'decode error (depth): {e}')
                self._cb_depth_err_logged = True

    def _cb_rs_info(self, msg):
        self._rs_info = msg

    def _cb_orb_color(self, msg):
        try:
            self._orbbec_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _try_save_camera_info(self):
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
        self.get_logger().info(f'✓ camera_info.json saved')

    # ── GUI mode (OpenCV preview) ──────────────────────────────

    def _tick_gui(self):
        if self._rs_color is None:
            return
        frame = self._rs_color.copy()
        H, W = frame.shape[:2]

        cv2.putText(frame, f"Close-up: {self._counter}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, "[W]ide  [SPACE]close-up  [Q]uit",
                    (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (200, 200, 200), 1)

        now = time.time()
        if now - self._last_save_time < 0.4:
            cv2.circle(frame, (W - 20, 20), 8, (0, 255, 0), -1)

        cv2.imshow('TERESA Capture — RealSense preview', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('w'):
            self._save_wide()
        elif key == 32:
            self._save_close_up()
        elif key == ord('q'):
            self._shutdown()

    # ── TTY mode (terminal, no display) ────────────────────────

    def _stdin_loop(self):
        """Read stdin line-by-line in a background thread."""
        while self._running:
            if select.select([sys.stdin], [], [], 0.3)[0]:
                line = sys.stdin.readline()
                # EOF (stdin closed) → empty string before stripping
                if not line:
                    time.sleep(0.3)
                    continue
                line = line.strip().lower()
                if line == 'w':
                    self._pending_wide = True
                    time.sleep(0.3)
                elif line == 'q':
                    self._shutdown()
                    return
                elif line == '':
                    self._pending_close_up = True
                    time.sleep(0.3)

    def _tick_process_save(self):
        """Process pending saves on main thread where frames are fresh."""
        if self._pending_wide:
            self._pending_wide = False
            self._save_wide()
        if self._pending_close_up:
            self._pending_close_up = False
            self._save_close_up()

    def _tick_tty_status(self):
        """Periodic status print."""
        rs_ok = '✓' if self._rs_color is not None else '✗'
        depth_ok = '✓' if self._rs_depth is not None else '✗'
        n_frames = getattr(self, '_frame_count', 0)
        self.get_logger().info(
            f'Status  |  RS color: {rs_ok}  depth: {depth_ok}  '
            f'frames: {n_frames}  |  # close-ups: {self._counter - 1}'
        )

    # ── Save ───────────────────────────────────────────────────

    def _save_wide(self):
        """Save wide shot from RealSense (hold it at Orbbec height, ~0.5 m)."""
        if self._rs_color is None:
            self.get_logger().warn('⚠ No RealSense frame yet')
            return
        color_path = self.out_dir / 'wide_color.png'
        cv2.imwrite(str(color_path), self._rs_color)
        self._last_save_time = time.time()
        self.get_logger().info(f'✓ WIDE (RealSense) → {color_path}')

        depth_path = self.out_dir / 'wide_depth.png'
        if self._rs_depth is not None:
            cv2.imwrite(str(depth_path), self._rs_depth)
            self.get_logger().info(f'  Depth → {depth_path}')
        else:
            h, w = self._rs_color.shape[:2]
            dummy = np.zeros((h, w), dtype=np.uint16)
            cv2.imwrite(str(depth_path), dummy)
            self.get_logger().warn('  No depth — placeholder saved')

    def _save_close_up(self):
        if self._rs_color is None:
            self.get_logger().warn('⚠ No RealSense frame yet')
            return
        color_path = self.close_up_dir / f'{self._counter:02d}_color.png'
        depth_path = self.close_up_dir / f'{self._counter:02d}_depth.png'

        cv2.imwrite(str(color_path), self._rs_color)
        if self._rs_depth is not None:
            cv2.imwrite(str(depth_path), self._rs_depth)
        else:
            h, w = self._rs_color.shape[:2]
            dummy = np.zeros((h, w), dtype=np.uint16)
            cv2.imwrite(str(depth_path), dummy)
            self.get_logger().warn('  No depth — placeholder saved')

        self._last_save_time = time.time()
        self.get_logger().info(
            f'✓ CLOSE-UP #{self._counter} → {color_path}')
        self._counter += 1

    def _shutdown(self):
        self.get_logger().info(f'Done. {self._counter - 1} close-ups saved.')
        self._running = False
        cv2.destroyAllWindows()
        rclpy.shutdown()
        sys.exit(0)


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
