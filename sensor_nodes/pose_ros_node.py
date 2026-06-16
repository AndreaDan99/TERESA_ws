#!/usr/bin/env python3
"""GPU 2D pose + posture ROS2 node — subscribes a color Image topic, runs YOLO11-pose on the
Jetson GPU, classifies posture (lying/sitting/standing) with the wound_det geometric classifier,
publishes 2D keypoints (PoseArray) + posture (String) + an overlay Image, and records mp4.

Reuses bench_pose.py's classify(); COCO-17 skeleton.

Run (inside teresa_gpu):
  python3 pose_ros_node.py --topic /orbbec/color/image_raw --model /work/models/yolo11x-pose.pt \
    --save-dir /work/live_pose --video /work/live_pose/pose.mp4 --rate 12
"""
import os, json, time, argparse
import numpy as np, cv2
# CRITICAL: import ultralytics BEFORE torch / rclpy msgs. If torch (or some ROS libs) are
# imported first, YOLO GPU inference silently returns ZERO detections on the Jetson stack
# (an OpenMP / backend-init clash). Verified: ultralytics-first -> detections; torch-first -> 0.
from ultralytics import YOLO
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge

COCO17_NAMES = ["nose", "l_eye", "r_eye", "l_ear", "r_ear", "l_shoulder", "r_shoulder",
                "l_elbow", "r_elbow", "l_wrist", "r_wrist", "l_hip", "r_hip",
                "l_knee", "r_knee", "l_ankle", "r_ankle"]
COCO17_EDGES = [(0, 1), (0, 2), (1, 3), (2, 4), (0, 5), (0, 6), (5, 7), (7, 9), (6, 8), (8, 10),
                (5, 6), (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]
CANON = ["nose", "l_shoulder", "r_shoulder", "l_hip", "r_hip", "l_knee", "r_knee", "l_ankle", "r_ankle"]
IDX = {n: i for i, n in enumerate(COCO17_NAMES)}


def _ang(a, b, c):
    if a is None or b is None or c is None:
        return None
    v1, v2 = np.asarray(a) - np.asarray(b), np.asarray(c) - np.asarray(b)
    n = np.linalg.norm(v1) * np.linalg.norm(v2)
    return None if n < 1e-6 else float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / n, -1, 1))))


def _mid(p, q):
    return None if p is None or q is None else (np.asarray(p) + np.asarray(q)) / 2.0


def classify(kp, gravity_down=(0.0, 1.0)):
    def get(name):
        i = IDX.get(name)
        if i is None:
            return None
        v = kp[i]
        return v if np.all(np.isfinite(v)) else None
    sh, hp = _mid(get("l_shoulder"), get("r_shoulder")), _mid(get("l_hip"), get("r_hip"))
    if sh is None or hp is None:
        return "unknown", 0.0
    hip_angs, knee_angs = [], []
    for s in ["l", "r"]:
        ha = _ang(get(f"{s}_shoulder"), get(f"{s}_hip"), get(f"{s}_knee"))
        ka = _ang(get(f"{s}_hip"), get(f"{s}_knee"), get(f"{s}_ankle"))
        if ha is not None:
            hip_angs.append(ha)
        if ka is not None:
            knee_angs.append(ka)
    hip_flex = float(np.mean(hip_angs)) if hip_angs else None
    knee_flex = float(np.mean(knee_angs)) if knee_angs else None
    torso = sh - hp
    up = -np.asarray(gravity_down, float); up = up / (np.linalg.norm(up) + 1e-9)
    tn = np.linalg.norm(torso)
    body_tilt = float(np.degrees(np.arccos(np.clip(np.dot(torso, up) / (tn + 1e-9), -1, 1)))) if tn > 1e-6 else None
    valid = np.array([p for p in [get(n) for n in CANON] if p is not None])
    aspect = None
    if len(valid) >= 3:
        w = valid[:, 0].max() - valid[:, 0].min(); h = valid[:, 1].max() - valid[:, 1].min()
        aspect = float(w / (h + 1e-6))
    flexed = ((hip_flex is not None and hip_flex < 120) or (knee_flex is not None and knee_flex < 110))
    if flexed:
        posture = "lying" if (body_tilt is not None and body_tilt > 60) else "sitting"
    else:
        horiz = (body_tilt is not None and body_tilt > 45) or (aspect is not None and aspect > 1.2)
        posture = "lying" if horiz else "standing"
    conf = 0.5
    if body_tilt is not None:
        conf = float(np.clip(abs(body_tilt - 45) / 45, 0.2, 1.0))
    if aspect is not None and posture == "lying":
        conf = max(conf, float(np.clip(aspect - 1.0, 0.2, 1.0)))
    return posture, round(conf, 3)


class PoseNode(Node):
    def __init__(self, a):
        super().__init__('pose_ros_node')
        self.a = a
        self.bridge = CvBridge()
        self.latest = None
        self.n = 0
        self.n_det = 0
        self.t_infer = 0.0
        self.posture_counts = {}
        self.vw = None
        self.grav = tuple(float(x) for x in a.gravity.split(","))
        if a.save_dir:
            os.makedirs(a.save_dir, exist_ok=True)
        self.get_logger().info(f"loading YOLO {a.model} …")
        self.model = YOLO(a.model)
        self.get_logger().info(f"pose ready on {torch.cuda.get_device_name(0)}; subscribing {a.topic}")
        self.create_subscription(Image, a.topic, self._cb, qos_profile_sensor_data)
        self.pub_kp = self.create_publisher(PoseArray, '/perception/pose/points_2d', 10)
        self.pub_posture = self.create_publisher(String, '/perception/pose/posture', 10)
        self.pub_overlay = self.create_publisher(Image, '/perception/pose/overlay', 1)
        self.create_timer(1.0 / max(1.0, a.rate), self._process)

    def _cb(self, m): self.latest = m

    def dump_metrics(self):
        if not self.a.save_dir or not self.n:
            return
        dom = max(self.posture_counts, key=self.posture_counts.get) if self.posture_counts else "unknown"
        m = dict(node="pose", model=os.path.basename(self.a.model),
                 device=torch.cuda.get_device_name(0), torch=torch.__version__,
                 n_frames=self.n, fps=round(self.n / self.t_infer, 2) if self.t_infer else 0,
                 det_rate=round(self.n_det / self.n, 3),
                 dominant_posture=dom, posture_counts=self.posture_counts)
        with open(os.path.join(self.a.save_dir, "metrics.json"), "w") as f:
            json.dump(m, f, indent=2)
        self.get_logger().info("METRICS " + json.dumps(m))

    def _process(self):
        if self.latest is None:
            return
        msg = self.latest
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W = bgr.shape[:2]
        t0 = time.time()
        r = self.model.predict(bgr, device=0, verbose=False, conf=0.25)[0]
        dt = time.time() - t0
        self.t_infer += dt; self.n += 1
        kp = np.full((17, 2), np.nan, np.float32)
        box = None
        if r.boxes is not None and len(r.boxes) > 0:
            areas = (r.boxes.xywh[:, 2] * r.boxes.xywh[:, 3]).cpu().numpy()
            i = int(np.argmax(areas))
            box = r.boxes.xyxy[i].cpu().numpy()
            if r.keypoints is not None:
                xy = r.keypoints.xy[i].cpu().numpy(); kc = r.keypoints.conf[i].cpu().numpy()
                for j in range(17):
                    if kc[j] > 0.02 and float(xy[j][0]) + float(xy[j][1]) > 0:
                        kp[j] = xy[j]
        posture, pconf = classify(kp, self.grav)
        # fallback: a detected person box that is much wider than tall == lying
        if posture == "unknown" and box is not None:
            asp = (box[2] - box[0]) / (box[3] - box[1] + 1e-6)
            if asp > 1.3:
                posture, pconf = "lying", round(float(min(1.0, asp - 1.0)), 3)
            elif asp < 0.7:
                posture, pconf = "standing", round(float(min(1.0, 1.0 - asp)), 3)
        if box is not None:
            self.n_det += 1
        if posture != "unknown":
            self.posture_counts[posture] = self.posture_counts.get(posture, 0) + 1
        # publish
        pa = PoseArray(); pa.header = msg.header
        for (x, y) in kp:
            p = Pose(); p.position.x = float(x); p.position.y = float(y); p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_kp.publish(pa)
        self.pub_posture.publish(String(data=f"{posture} {pconf}"))
        # overlay
        if box is not None:
            cv2.rectangle(bgr, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 180, 255), 2)
        for x, y in edges_pts(kp, COCO17_EDGES):
            cv2.line(bgr, x, y, (255, 128, 0), 2)
        for jx, jy in kp:
            if np.isfinite([jx, jy]).all():
                cv2.circle(bgr, (int(jx), int(jy)), 4, (0, 220, 0), -1)
        fps = self.n / self.t_infer if self.t_infer else 0.0
        cv2.putText(bgr, f"YOLO-pose live | {fps:.1f} fps", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        col = (0, 255, 0) if posture == "lying" else (0, 200, 255) if posture != "unknown" else (120, 120, 120)
        cv2.putText(bgr, f"POSTURE: {posture.upper()} ({pconf})", (10, H - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        if self.a.video:
            if self.vw is None:
                self.vw = cv2.VideoWriter(self.a.video, cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.a.out_fps, (W, H))
            self.vw.write(bgr)
        if self.a.save_dir and self.n % self.a.save_every == 0:
            cv2.imwrite(os.path.join(self.a.save_dir, f"pose_{self.n:05d}.jpg"), bgr)
        if self.n % 10 == 0:
            self.get_logger().info(f"frame {self.n} | {fps:.1f} fps | posture {posture} ({pconf})")
        if self.a.max_frames and self.n >= self.a.max_frames:
            if self.vw is not None:
                self.vw.release()
            raise SystemExit(0)


def edges_pts(kp, edges):
    out = []
    for x, y in edges:
        pa, pb = kp[x], kp[y]
        if np.isfinite(pa).all() and np.isfinite(pb).all():
            out.append(((int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1]))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/orbbec/color/image_raw')
    ap.add_argument('--model', default='/work/models/yolo11x-pose.pt')
    ap.add_argument('--rate', type=float, default=12.0)
    ap.add_argument('--gravity', default='0.0,1.0')
    ap.add_argument('--save-dir', default='')
    ap.add_argument('--save-every', type=int, default=4)
    ap.add_argument('--video', default='')
    ap.add_argument('--out-fps', type=float, default=10.0)
    ap.add_argument('--max-frames', type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = PoseNode(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.dump_metrics()
        if node.vw is not None:
            node.vw.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
