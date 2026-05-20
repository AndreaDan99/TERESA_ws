#!/usr/bin/env python3
"""
TF Monitor — controlla che SpotCore pubblichi i TF necessari.

Aspetta in loop finché i frame odom→body non diventano disponibili,
poi pubblica /wbc/tf_ready = True e notifica l'utente.

Uso:
  ros2 run spot_control tf_monitor
  ros2 run spot_control tf_monitor --ros-args -p body_frame:=my_spot/body -p odom_frame:=my_spot/odom
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import rclpy.time

from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException


DIAG_MSG = (
    'Diagnostica: ros2 topic list | grep tf\n'
    '  Verifica: 1) spot_ros2 attivo su SpotCore?\n'
    '            2) ROS_DOMAIN_ID uguale?\n'
    '            3) spot_name="my_spot"?'
)


class TFMonitorNode(Node):

    def __init__(self):
        super().__init__('tf_monitor')

        self.declare_parameter('odom_frame', 'my_spot/odom')
        self.declare_parameter('body_frame', 'my_spot/body')
        self.declare_parameter('check_rate',  1.0)  # Hz
        self.declare_parameter('timeout',     1.0)  # seconds per lookup

        self._odom_frame  = self.get_parameter('odom_frame').value
        self._body_frame  = self.get_parameter('body_frame').value
        self._check_rate  = float(self.get_parameter('check_rate').value)
        self._timeout     = float(self.get_parameter('timeout').value)

        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._ready = False
        self._warn_count = 0

        self._pub_ready  = self.create_publisher(Bool,   '/wbc/tf_ready',  10)
        self._pub_status = self.create_publisher(String, '/wbc/tf_status', 10)

        self.create_timer(1.0 / self._check_rate, self._check)

        self.get_logger().info(
            f'TF Monitor avviato.\n'
            f'  Attendo {self._odom_frame} → {self._body_frame} ...\n'
            f'  {DIAG_MSG}')

    def _check(self) -> None:
        if self._ready:
            return

        try:
            self._tf.lookup_transform(
                self._odom_frame, self._body_frame,
                rclpy.time.Time(), timeout=Duration(seconds=self._timeout))
        except TransformException:
            self._warn_count += 1
            if self._warn_count == 1 or self._warn_count % 5 == 0:
                self.get_logger().warn(
                    f'TF {self._odom_frame} → {self._body_frame} '
                    f'non ancora disponibile (tentativo {self._warn_count}).\n'
                    f'  {DIAG_MSG}')
            self._pub_status.publish(String(data='WAITING_TF'))
            return

        self._ready = True
        self.get_logger().info(
            f'========================================\n'
            f' TF DISPONIBILE: {self._odom_frame} → {self._body_frame} OK\n'
            f' SpotCore connesso via DDS.\n'
            f' Ora premi "s" sul keyboard controller per avviare.\n'
            f'========================================')
        self._pub_ready.publish(Bool(data=True))
        self._pub_status.publish(String(data='TF_READY'))


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
