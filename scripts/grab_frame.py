#!/usr/bin/env python3
"""Grab one color + one depth frame from a ROS2 camera and save to disk.
Read-only: subscribes, saves a JPEG (color) + PNG colormap & .npy (depth), exits.
Usage: grab_frame.py --color /orbbec/color/image_raw --depth /orbbec/depth/image_raw --out /work/cam_check --prefix orbbec
"""
import sys, os, time, argparse
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--color', default='/orbbec/color/image_raw')
    ap.add_argument('--depth', default='/orbbec/depth/image_raw')
    ap.add_argument('--out', default='/work/cam_check')
    ap.add_argument('--prefix', default='orbbec')
    ap.add_argument('--timeout', type=float, default=25.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = Node('grab_frame')
    bridge = CvBridge()
    got = {'color': None, 'depth': None}
    cnt = {'color': 0, 'depth': 0}

    def mk(key):
        def cb(msg):
            cnt[key] += 1
            if got[key] is None:
                got[key] = msg
        return cb

    node.create_subscription(Image, a.color, mk('color'), qos_profile_sensor_data)
    node.create_subscription(Image, a.depth, mk('depth'), qos_profile_sensor_data)

    t0 = time.time()
    while rclpy.ok() and (got['color'] is None or got['depth'] is None) and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    res = {}
    if got['color'] is not None:
        try:
            img = bridge.imgmsg_to_cv2(got['color'], desired_encoding='bgr8')
            p = f"{a.out}/{a.prefix}_color.jpg"
            cv2.imwrite(p, img)
            res['color'] = {'shape': list(img.shape), 'dtype': str(img.dtype),
                            'encoding': got['color'].encoding, 'path': p, 'msgs': cnt['color']}
        except Exception as e:
            res['color_error'] = str(e)
    if got['depth'] is not None:
        try:
            d = np.asarray(bridge.imgmsg_to_cv2(got['depth'], desired_encoding='passthrough'))
            np.save(f"{a.out}/{a.prefix}_depth.npy", d)
            valid = d[d > 0]
            mx = float(d.max()) if d.size else 1.0
            vis = cv2.applyColorMap(cv2.convertScaleAbs(d, alpha=255.0 / (mx or 1.0)), cv2.COLORMAP_JET)
            cv2.imwrite(f"{a.out}/{a.prefix}_depth.png", vis)
            res['depth'] = {'shape': list(d.shape), 'dtype': str(d.dtype),
                            'encoding': got['depth'].encoding,
                            'valid_frac': round(float(valid.size) / float(d.size), 3) if d.size else 0,
                            'min_mm': float(valid.min()) if valid.size else 0,
                            'max_mm': float(valid.max()) if valid.size else 0, 'msgs': cnt['depth']}
        except Exception as e:
            res['depth_error'] = str(e)

    import json
    print("RESULT " + json.dumps(res))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ('color' in res and 'depth' in res) else 2)


if __name__ == '__main__':
    main()
