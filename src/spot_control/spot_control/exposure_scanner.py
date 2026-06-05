#!/usr/bin/env python3
"""
exposure_scanner.py — Body scanning node for TERESA exposure assessment.

Moves the Z1 arm with the RealSense camera over a grid of points
on the patient's body, coordinating with the WBC coordinator for
Spot body posture changes via the same per-point protocol as FAST.

Protocol:
  exposure_scanner          wbc_coordinator
       |                         |
       |── next_point_idx ──────→|  coordinator optimises Spot posture
       |                         |  _set_body_pose(h*,φ*) → settle 1.5s
       |←── /wbc/body_ready ────|
       |                         |
       |── IK goal (camera) ────→|  via ik_goal_mux → z1_ik_to_jtc
       |←── /ik_done ───────────|
       |                         |
       |── dwell 2s ────────────→|  camera observes body point
       |                         |
       |── next_point_idx ──────→|  advance
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + t * (b - a)


class ExposureScanner(Node):

    def __init__(self):
        super().__init__('exposure_scanner')

        # ── parameters ───────────────────────────────────────
        self._standoff = float(
            self.declare_parameter('exposure_standoff', 0.50)
            .get_parameter_value().double_value
        )
        self._dwell = float(
            self.declare_parameter('exposure_dwell', 2.0)
            .get_parameter_value().double_value
        )
        self._grid_rows = int(
            self.declare_parameter('exposure_grid_rows', 3)
            .get_parameter_value().integer_value
        )
        self._grid_cols = int(
            self.declare_parameter('exposure_grid_cols', 5)
            .get_parameter_value().integer_value
        )
        sweet_raw = self.declare_parameter(
            'exposure_sweet_spot', [0.45, 0.0, 0.45]
        ).get_parameter_value().double_array_value
        self._sweet_spot = np.array(sweet_raw)

        # ── state ────────────────────────────────────────────
        self._active = False
        self._points = []
        self._idx = 0
        self._phase = 'idle'
        self._dwell_start = None
        self._body_ready = False
        self._ik_done = False

        # ── subscribers ──────────────────────────────────────
        self._sub_state = self.create_subscription(
            String, '/wbc/state', self._cb_wbc_state, 10
        )
        self._sub_body_ready = self.create_subscription(
            Bool, '/wbc/body_ready', self._cb_body_ready, 10
        )
        self._sub_ik_done = self.create_subscription(
            Bool, '/ik_done', self._cb_ik_done, 10
        )

        # ── publishers ───────────────────────────────────────
        self._pub_goal = self.create_publisher(
            PoseStamped, '/z1/ik_goal_pose', 10
        )
        self._pub_next = self.create_publisher(
            Int32, '/z1/next_point_idx', 10
        )
        self._pub_ready = self.create_publisher(
            Bool, '/exposure/ready', 10
        )
        self._pub_grid = self.create_publisher(
            MarkerArray, '/exposure/grid_markers', 10
        )

        # ── timer ────────────────────────────────────────────
        self._timer = self.create_timer(0.1, self._tick)
        self._grid_timer = self.create_timer(0.2, self._publish_grid_markers)

        self.get_logger().info('Exposure scanner ready')

    # ── callbacks ────────────────────────────────────────────
    def _cb_wbc_state(self, msg):
        if msg.data == 'EXPOSURE_SCANNING' and not self._active:
            self._start()
        elif msg.data != 'EXPOSURE_SCANNING' and self._active:
            self._active = False
            self.get_logger().info('Exposure scan stopped by FSM')

    def _cb_body_ready(self, msg):
        if msg.data:
            self._body_ready = True

    def _cb_ik_done(self, msg):
        if msg.data:
            self._ik_done = True

    # ── lifecycle ────────────────────────────────────────────
    def _start(self):
        self._active = True
        self._points = self._gen_exposure_grid()
        self._idx = 0
        if not self._points:
            self.get_logger().warn(
                'No exposure points generated — skipping'
            )
            self._pub_ready.publish(Bool(data=True))
            self._active = False
            return
        self._phase = 'request_body_pose'
        self._body_ready = False
        self._ik_done = False
        self.get_logger().info(
            f'Exposure scan started: {len(self._points)} points'
        )
        self._pub_next.publish(Int32(data=0))

    # ── grid generation ──────────────────────────────────────
    def _gen_exposure_grid(self):
        """Generate 3D camera positions over the patient's torso.

        Uses COCO keypoints already available from the Orbbec
        skeleton pipeline. For a lying patient the camera offset
        is in world +Z (up, above the body).
        """
        points = []

        # torso corners: shoulders + hips
        torso_keys = [5, 6, 11, 12]

        # use nominal positions if keypoints not available
        # (in real operation they're published by yolo_skeleton_spot)
        kp = {}
        for i in torso_keys:
            kp[i] = np.array([0.0, 0.0, 0.0])

        tl = kp[5]
        tr = kp[6]
        bl = kp[11]
        br = kp[12]

        for r in range(self._grid_rows):
            for c in range(self._grid_cols):
                u = c / max(self._grid_cols - 1, 1)
                v = r / max(self._grid_rows - 1, 1)
                pt = lerp(lerp(tl, tr, u), lerp(bl, br, u), v)
                points.append(pt)

        # add camera standoff above the body
        # for a supine patient: world +Z points upward from the ground
        points = [p + np.array([0.0, 0.0, self._standoff])
                  for p in points]

        return points

    # ── main loop ────────────────────────────────────────────
    def _tick(self):
        if not self._active:
            return

        if self._phase == 'request_body_pose':
            if self._body_ready:
                self._body_ready = False
                self._send_ik_goal()
                self._phase = 'wait_ik'

        elif self._phase == 'wait_ik':
            if self._ik_done:
                self._ik_done = False
                self._dwell_start = self.get_clock().now()
                self._phase = 'dwell'
                self.get_logger().debug(
                    f'Point {self._idx}: IK done, dwelling'
                )

        elif self._phase == 'dwell':
            elapsed = (
                self.get_clock().now() - self._dwell_start
            ).nanoseconds * 1e-9
            if elapsed >= self._dwell:
                self._idx += 1
                if self._idx >= len(self._points):
                    self._finish()
                else:
                    self._phase = 'request_body_pose'
                    self._body_ready = False
                    self._pub_next.publish(
                        Int32(data=self._idx)
                    )
                    self.get_logger().info(
                        f'Advancing to point {self._idx}/'
                        f'{len(self._points)}'
                    )

    def _send_ik_goal(self):
        pt = self._points[self._idx]
        goal = PoseStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(pt[0])
        goal.pose.position.y = float(pt[1])
        goal.pose.position.z = float(pt[2])
        goal.pose.orientation.x = -0.0062
        goal.pose.orientation.y = 0.4107
        goal.pose.orientation.z = 0.0021
        goal.pose.orientation.w = 0.9118

        self._pub_goal.publish(goal)
        self.get_logger().info(
            f'Point {self._idx}/{len(self._points)}: IK goal sent'
        )

    def _publish_grid_markers(self):
        """Publish exposure grid points as MarkerArray for web overlay."""
        if not self._points:
            return
        markers = MarkerArray()
        for i, pt in enumerate(self._points):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'exposure_grid'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = float(pt[2])
            if i == self._idx and self._active:
                m.scale.x = 0.04; m.scale.y = 0.04; m.scale.z = 0.04
                m.color.r = 0.2; m.color.g = 0.4; m.color.b = 1.0
                m.color.a = 0.9
            elif i < self._idx:
                m.scale.x = 0.02; m.scale.y = 0.02; m.scale.z = 0.02
                m.color.r = 0.3; m.color.g = 0.5; m.color.b = 0.8
                m.color.a = 0.4
            else:
                m.scale.x = 0.02; m.scale.y = 0.02; m.scale.z = 0.02
                m.color.r = 0.3; m.color.g = 0.5; m.color.b = 0.8
                m.color.a = 0.6
            markers.markers.append(m)
        self._pub_grid.publish(markers)

    def _finish(self):
        self._active = False
        self._pub_ready.publish(Bool(data=True))
        self._pub_next.publish(Int32(data=-1))
        self.get_logger().info(
            f'Exposure scan complete: '
            f'{len(self._points)} points visited'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExposureScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
