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
import hashlib
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _make_qos():
    """RELIABLE + KEEP_LAST(1) — garantisce che ogni frame pubblicato
    venga recapitato, senza accumulare code di frame vecchi."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    )


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
        self._last_frame_stamp_s = 0.0
        self._last_frame_arrival = 0.0  # wall clock
        self._last_saved_hash = None    # MD5 del frame precedente
        self._stall_warned = False

        # Commands from stdin thread
        self._cmd = None
        self._running = True

        qos = _make_qos()
        self._sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._on_color, qos)
        self._sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self._on_depth, qos)
        self._sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._on_info, qos)

        # Status timer (ogni 4 s)
        self.create_timer(4.0, self._tick_status)

        print(f'📸 TERESA Exposure Capture  [QoS: RELIABLE, depth=1]\n'
              f'   Out: {self.out_dir}\n'
              f'   [w+ENTER]=wide  [ENTER]=close-up  [q+ENTER]=quit\n')

    # ── Frame callback (main thread) ─────────────────────────────

    def _on_color(self, msg):
        # ── diagnostica: timestamp del frame ───────────────
        now_wall = time.time()
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delta_s = stamp_s - self._last_frame_stamp_s if self._last_frame_stamp_s > 0 else -1
        delta_wall = now_wall - self._last_frame_arrival if self._last_frame_arrival > 0 else -1
        self._last_frame_stamp_s = stamp_s
        self._last_frame_arrival = now_wall

        # ── decodifica ─────────────────────────────────────
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8).copy()
            expected = msg.height * msg.width * 3
            if data.size != expected:
                self.get_logger().warn(
                    f'Frame #{self._frame_n + 1}: size mismatch '
                    f'(got {data.size}, expected {expected} for '
                    f'{msg.width}x{msg.height}x3 enc={msg.encoding})',
                    throttle_duration_sec=5.0)
            data = data.reshape(msg.height, msg.width, 3)
        except Exception as e:
            self.get_logger().error(f'Reshape failed: {e}', throttle_duration_sec=5.0)
            return

        if msg.encoding == 'rgb8':
            self._color = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            self._color = data
        else:
            self.get_logger().warn(
                f'Unexpected encoding "{msg.encoding}" — using as-is',
                throttle_duration_sec=5.0)
            self._color = data

        self._frame_n += 1

        # ── diagnostica: warn se frame identico al precedente ─
        cur_hash = hashlib.md5(self._color.tobytes()).hexdigest()
        if self._last_saved_hash is not None and cur_hash == self._last_saved_hash and not self._stall_warned:
            self.get_logger().warn(
                f'⚠  Frame #{self._frame_n} è IDENTICO al precedente salvato '
                f'— la camera potrebbe essere bloccata!',
                throttle_duration_sec=5.0)
            self._stall_warned = True

        # ── mostra info ogni 30 frame ──────────────────────
        if self._frame_n % 30 == 0:
            self.get_logger().info(
                f'Frame #{self._frame_n} | stamp={stamp_s:.3f} '
                f'Δstamp={delta_s:.3f}s Δwall={delta_wall:.3f}s '
                f'enc={msg.encoding} {msg.width}x{msg.height} '
                f'size={len(msg.data)}B')

        # ── Process pending command — save happens HERE ─────
        cmd = self._cmd
        if cmd is not None:
            self._cmd = None
            if cmd == 'w':
                self._save_wide(cur_hash)
            elif cmd == 'c':
                self._save_close_up(cur_hash)
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

    def _save_wide(self, frame_hash):
        ok = cv2.imwrite(str(self.out_dir / 'wide_color.png'), self._color)
        if not ok:
            self.get_logger().error('❌ imwrite FAILED for wide_color.png')
        d = self._depth
        if d is not None:
            cv2.imwrite(str(self.out_dir / 'wide_depth.png'), d)
        else:
            cv2.imwrite(str(self.out_dir / 'wide_depth.png'),
                       np.zeros(self._color.shape[:2], dtype=np.uint16))
        dup = ' (DUPLICATE!)' if frame_hash == self._last_saved_hash else ''
        self._last_saved_hash = frame_hash
        self._stall_warned = False
        print(f'✓ WIDE  (frame #{self._frame_n}, hash={frame_hash[:8]}){dup}')

    def _save_close_up(self, frame_hash):
        self._counter += 1
        ok = cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_color.png'), self._color)
        if not ok:
            self.get_logger().error(f'❌ imwrite FAILED for {self._counter:02d}_color.png')
        d = self._depth
        if d is not None:
            cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_depth.png'), d)
        else:
            cv2.imwrite(str(self.cu_dir / f'{self._counter:02d}_depth.png'),
                       np.zeros(self._color.shape[:2], dtype=np.uint16))
        dup = ' (DUPLICATE!)' if frame_hash == self._last_saved_hash else ''
        self._last_saved_hash = frame_hash
        self._stall_warned = False
        print(f'✓ CLOSE-UP #{self._counter}  (frame #{self._frame_n}, hash={frame_hash[:8]}){dup}')

    # ── Status ───────────────────────────────────────────────────

    def _tick_status(self):
        elapsed = time.time() - self._last_frame_arrival if self._last_frame_arrival > 0 else -1
        stall = ''
        if elapsed > 2.0:
            stall = f'  ⚠ NO FRAMES for {elapsed:.1f}s!'
        print(f'  [frames: {self._frame_n}  close-ups: {self._counter}  '
              f'Δlast={elapsed:.1f}s]{stall}')


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
                print(f'  → wide command queued (frame #{node._frame_n})')
            elif line == 'q':
                node._cmd = 'q'
                print(f'  → quit command queued')
                break
            elif line == '':
                node._cmd = 'c'
                print(f'  → close-up command queued (frame #{node._frame_n})')
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
