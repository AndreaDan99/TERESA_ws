#!/usr/bin/env python3
"""
TF Monitor — verifica 4 condizioni prima di dare il via libera al WBC.

Condizioni:
  1. /joint_states ricevuto            → Z1 driver attivo
  2. /orbbec/color/image_raw ricevuto  → Orbbec camera viva
  3. /camera/color/image_raw ricevuto  → RealSense camera viva
  4. 7 catene TF disponibili           → TF tree completo

Solo quando TUTTE e 4 sono vere → pubblica /wbc/tf_ready = True.

Uso:
  ros2 run spot_control tf_monitor
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration
import rclpy.time

# TF chains to check (source, target, description)
TF_CHAINS = [
    ('my_spot/odom',          'my_spot/body',              'SpotCore DDS'),
    ('my_spot/body',          'world',                     'Z1 mount on Spot'),
    ('world',                 'link00',                    'Z1 arm root (robot_state_publisher)'),
    ('my_spot/body',          'orbbec_link',               'Orbbec mount'),
    ('orbbec_link',           'orbbec_color_optical_frame', 'Orbbec optical'),
    ('link00',                'link06',                    'Z1 arm chain'),
    ('link06',                'camera_link',               'Realsense mount'),
    ('camera_link',           'camera_color_optical_frame',  'Realsense optical'),
]


class TFMonitorNode(Node):

    def __init__(self):
        super().__init__('tf_monitor')

        self.declare_parameter('check_rate', 1.0)
        self.declare_parameter('tf_timeout', 1.0)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('orbbec_topic', '/orbbec/color/image_raw')
        self.declare_parameter('realsense_topic', '/camera/color/image_raw')

        self._check_rate  = float(self.get_parameter('check_rate').value)
        self._tf_timeout  = float(self.get_parameter('tf_timeout').value)
        self._js_topic    = self.get_parameter('joint_states_topic').value
        self._orbbec_topic = self.get_parameter('orbbec_topic').value
        self._rs_topic    = self.get_parameter('realsense_topic').value

        # ── TF ────────────────────────────────────────────────────────
        self._tf = Buffer()
        TransformListener(self._tf, self)

        # ── Topic subscriptions ───────────────────────────────────────
        self._js_ok = False
        self._orbbec_ok = False
        self._rs_ok = False

        self.create_subscription(JointState, self._js_topic, self._cb_js, 10)
        self.create_subscription(Image, self._orbbec_topic, self._cb_orbbec, 10)
        self.create_subscription(Image, self._rs_topic, self._cb_realsense, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._pub_ready = self.create_publisher(Bool, '/wbc/tf_ready', 10)
        self._pub_status = self.create_publisher(String, '/wbc/tf_status', 10)

        # ── State ─────────────────────────────────────────────────────
        self._done = False  # prevent re-publish
        self._tf_failures: dict = {}  # track which TF chains fail

        self.create_timer(1.0 / self._check_rate, self._tick)

        self.get_logger().info(
            'TF Monitor avviato.\n'
            '  4 condizioni: Z1 driver | Orbbec | RealSense | 8 catene TF\n'
            '  Diagnostica manuale: bash src/spot_control/scripts/tf_diag.sh')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cb_js(self, _msg: JointState) -> None:
        if not self._js_ok:
            self._js_ok = True
            self.get_logger().info('Z1 driver: joint_states OK')

    def _cb_orbbec(self, _msg: Image) -> None:
        if not self._orbbec_ok:
            self._orbbec_ok = True
            self.get_logger().info('Orbbec: /orbbec/color/image_raw OK')

    def _cb_realsense(self, _msg: Image) -> None:
        if not self._rs_ok:
            self._rs_ok = True
            self.get_logger().info('RealSense: /camera/color/image_raw OK')

    # ── Tick ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._done:
            return

        # ── TF check ──────────────────────────────────────────────────
        tf_ok_count = 0
        tf_total = len(TF_CHAINS)
        for src, tgt, desc in TF_CHAINS:
            key = f'{src}→{tgt}'
            if key in self._tf_failures:
                # already known bad, retry
                pass
            try:
                self._tf.lookup_transform(
                    src, tgt, rclpy.time.Time(),
                    timeout=Duration(seconds=self._tf_timeout))
                self._tf_failures.pop(key, None)
                tf_ok_count += 1
            except TransformException:
                self._tf_failures[key] = desc

        # Re-count after retry on previous failures
        if self._tf_failures:
            tf_ok_count = tf_total - len(self._tf_failures)

        # ── Status ────────────────────────────────────────────────────
        c1 = 'OK' if self._js_ok else 'WAIT'
        c2 = 'OK' if self._orbbec_ok else 'WAIT'
        c3 = 'OK' if self._rs_ok else 'WAIT'
        c4 = f'{tf_ok_count}/{tf_total}'

        status_line = (
            f'[Z1:{c1}] [Orbbec:{c2}] [RealSense:{c3}] '
            f'[TF:{c4}]')
        self._pub_status.publish(String(data=status_line))

        # ── Missing details ───────────────────────────────────────────
        if self._tf_failures:
            missing = ', '.join(f'{k}({v})' for k, v in self._tf_failures.items())
            self.get_logger().info(
                f'{status_line}  —  TF mancanti: {missing}',
                throttle_duration_sec=3.0)
        else:
            self.get_logger().info(status_line, throttle_duration_sec=3.0)

        # ── All conditions met? ───────────────────────────────────────
        if self._js_ok and self._orbbec_ok and self._rs_ok and tf_ok_count == tf_total:
            self._done = True
            self.get_logger().info(
                '========================================\n'
                ' TUTTO PRONTO\n'
                ' Z1 driver OK | Orbbec OK | RealSense OK | TF 7/7 OK\n'
                ' /wbc/tf_ready = True\n'
                ' Ora puoi lanciare i terminali applicativi.\n'
                '========================================')
            self._pub_ready.publish(Bool(data=True))
            self._pub_status.publish(String(data='READY'))


def main(args=None):
    rclpy.init(args=args)
    node = TFMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
