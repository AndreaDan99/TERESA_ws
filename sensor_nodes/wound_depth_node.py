#!/usr/bin/env python3
"""Depth-fused zero-shot wound detection ROS2 node — subscribes color + registered/aligned
depth + camera_info, runs Grounding-DINO (or OWLv2) on the GPU, back-projects each detection
box-centre through the camera intrinsics to a **3D point in the camera optical frame**, and
publishes the 3D wound points (PoseArray) + an overlay (with metric XYZ burned in) + records mp4.

This is the 3D target the Z1 arm would eventually servo to.

Run (inside teresa_gpu):
  # Orbbec (Spot cam):
  python3 wound_depth_node.py --color /orbbec/color/image_raw --depth /orbbec/depth/image_raw \
    --info /orbbec/color/camera_info --method gdino --save-dir /work/live_wound3d --video /work/live_wound3d/wound3d.mp4
  # RealSense (arm cam):
  python3 wound_depth_node.py --color /camera/camera/color/image_raw \
    --depth /camera/camera/aligned_depth_to_color/image_raw --info /camera/camera/color/camera_info ...
"""
import os, json, time, argparse
import numpy as np, cv2
from PIL import Image as PILImage
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge

WOUND_VOCAB = ["wound", "open wound", "laceration", "cut", "bleeding wound",
               "blood", "injury", "bruise", "surgical incision", "skin lesion"]
GDINO_BOX_THRESHOLD, GDINO_TEXT_THRESHOLD = 0.18, 0.12
OWL_THRESHOLD, OWL_NMS_IOU = 0.08, 0.30
CFG = {"owlv2": "google/owlv2-base-patch16-ensemble",
       "gdino": "IDEA-Research/grounding-dino-base"}


def robust_depth_mm(depth, u, v, r=6):
    """median of valid (>0) depth in an r-patch around (u,v); None if no return."""
    H, W = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    x0, x1 = max(0, u - r), min(W, u + r + 1)
    y0, y1 = max(0, v - r), min(H, v + r + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size >= 4 else None


class WoundDepthNode(Node):
    def __init__(self, a):
        super().__init__('wound_depth_node')
        self.a = a
        self.bridge = CvBridge()
        self.latest = None
        self.depth = None
        self.K = None
        self.n = 0
        self.n_with_det = 0
        self.n_dets_total = 0
        self.n_with3d = 0
        self.confs = []
        self.t_infer = 0.0
        self.vw = None
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if a.save_dir:
            os.makedirs(a.save_dir, exist_ok=True)
        mid = CFG[a.method]
        self.get_logger().info(f"loading {a.method} {mid} on {self.device} …")
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        self.proc = AutoProcessor.from_pretrained(mid)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(mid).to(self.device).eval()
        self.gdino_text = ". ".join(v.lower() for v in WOUND_VOCAB) + "."
        self.create_subscription(Image, a.color, self._cb_color, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth, self._cb_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, a.info, self._cb_info, qos_profile_sensor_data)
        self.pub_pts = self.create_publisher(PoseArray, '/perception/wound/points_3d', 10)
        self.pub_dets = self.create_publisher(String, '/perception/wound/dets', 10)
        self.pub_overlay = self.create_publisher(Image, '/perception/wound/overlay', 1)
        self.get_logger().info(f"wound+depth ready; color={a.color} depth={a.depth}")
        self.create_timer(a.period, self._process)

    def dump_metrics(self):
        if not self.a.save_dir or not self.n:
            return
        m = dict(node="wound_depth", method=self.a.method, device=self.device, torch=torch.__version__,
                 n_frames=self.n, fps=round(self.n / self.t_infer, 2) if self.t_infer else 0,
                 detection_rate=round(self.n_with_det / self.n, 3),
                 mean_dets_per_frame=round(self.n_dets_total / self.n, 2),
                 total_3d_points=self.n_with3d,
                 mean_conf=round(float(np.mean(self.confs)), 3) if self.confs else None,
                 peak_vram_mb=round(torch.cuda.max_memory_allocated() / 1e6, 1) if self.device == "cuda" else 0)
        with open(os.path.join(self.a.save_dir, "metrics.json"), "w") as f:
            json.dump(m, f, indent=2)
        self.get_logger().info("METRICS " + json.dumps(m))

    def _cb_color(self, m): self.latest = m
    def _cb_depth(self, m):
        self.depth = np.asarray(self.bridge.imgmsg_to_cv2(m, desired_encoding='passthrough'))
    def _cb_info(self, m): self.K = np.array(m.k).reshape(3, 3)

    def _infer(self, pil):
        Wd, Ht = pil.size
        dets = []
        if self.a.method == "gdino":
            inp = self.proc(images=pil, text=self.gdino_text, return_tensors="pt").to(self.device)
            with torch.no_grad():
                out = self.model(**inp)
            try:
                res = self.proc.post_process_grounded_object_detection(
                    out, inp.input_ids, box_threshold=GDINO_BOX_THRESHOLD,
                    text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
            except TypeError:
                res = self.proc.post_process_grounded_object_detection(
                    out, inp.input_ids, threshold=GDINO_BOX_THRESHOLD,
                    text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
            labs = res.get("text_labels", res.get("labels"))
            for b, s, l in zip(res["boxes"].cpu().numpy(), res["scores"].cpu().numpy(), labs):
                dets.append({"box": [float(v) for v in b], "score": float(s),
                             "label": l if isinstance(l, str) else "wound"})
        else:
            inp = self.proc(text=[WOUND_VOCAB], images=pil, return_tensors="pt").to(self.device)
            with torch.no_grad():
                out = self.model(**inp)
            res = self.proc.post_process_object_detection(
                out, threshold=OWL_THRESHOLD,
                target_sizes=torch.tensor([(Ht, Wd)]).to(self.device))[0]
            from torchvision.ops import nms
            bx, sc, lb = res["boxes"], res["scores"], res["labels"]
            keep = nms(bx, sc, OWL_NMS_IOU).cpu().numpy().tolist() if len(bx) else []
            bxn, scn, lbn = bx.cpu().numpy(), sc.cpu().numpy(), lb.cpu().numpy()
            for k in keep:
                li = int(lbn[k])
                dets.append({"box": [float(v) for v in bxn[k]], "score": float(scn[k]),
                             "label": WOUND_VOCAB[li] if 0 <= li < len(WOUND_VOCAB) else "wound"})
        return dets

    def _process(self):
        if self.latest is None or self.K is None:
            return
        msg = self.latest
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        H, W = rgb.shape[:2]
        depth = self.depth
        # registered/aligned depth should match color res; guard-resize if not
        if depth is not None and depth.shape[:2] != (H, W):
            depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        if self.device == "cuda":
            torch.cuda.synchronize()
        t0 = time.time()
        dets = self._infer(PILImage.fromarray(rgb))
        if self.device == "cuda":
            torch.cuda.synchronize()
        dt = time.time() - t0
        self.n += 1
        self.t_infer += dt
        self.n_dets_total += len(dets)
        self.n_with_det += 1 if dets else 0
        self.confs += [d["score"] for d in dets]

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        pa = PoseArray(); pa.header = msg.header
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            zmm = robust_depth_mm(depth, u, v) if depth is not None else None
            p3 = None
            if zmm is not None:
                Z = zmm / 1000.0
                X = (u - cx) * Z / fx; Y = (v - cy) * Z / fy
                p3 = (X, Y, Z)
                d["xyz_m"] = [round(X, 3), round(Y, 3), round(Z, 3)]
                pose = Pose(); pose.position.x = float(X); pose.position.y = float(Y)
                pose.position.z = float(Z); pose.orientation.w = 1.0; pa.poses.append(pose)
            cv2.rectangle(bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(bgr, (int(u), int(v)), 5, (0, 0, 255), -1)
            lab = f'{d["label"]} {d["score"]:.2f}'
            cv2.putText(bgr, lab, (int(x1), max(12, int(y1) - 22)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)
            if p3 is not None:
                cv2.putText(bgr, f'XYZ {p3[0]:+.2f},{p3[1]:+.2f},{p3[2]:.2f}m', (int(x1), max(24, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1, cv2.LINE_AA)
        self.pub_pts.publish(pa)
        self.pub_dets.publish(String(data=json.dumps(dets)))
        n3d = len(pa.poses)
        self.n_with3d += n3d
        cv2.putText(bgr, f"WOUND+DEPTH {self.a.method} | {1.0/dt:.2f} fps | {len(dets)} det | {n3d} w/3D",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        if self.a.video:
            if self.vw is None:
                self.vw = cv2.VideoWriter(self.a.video, cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.a.out_fps, (W, H))
            self.vw.write(bgr)
        if self.a.save_dir:
            cv2.imwrite(os.path.join(self.a.save_dir, f"wound3d_{self.n:05d}.jpg"), bgr)
        self.get_logger().info(f"frame {self.n} | {1.0/dt:.2f} fps | {len(dets)} det | {n3d} with 3D "
                               f"| {[d.get('xyz_m') for d in dets]}")
        if self.a.max_frames and self.n >= self.a.max_frames:
            if self.vw is not None:
                self.vw.release()
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--color', default='/orbbec/color/image_raw')
    ap.add_argument('--depth', default='/orbbec/depth/image_raw')
    ap.add_argument('--info', default='/orbbec/color/camera_info')
    ap.add_argument('--method', default='gdino', choices=list(CFG))
    ap.add_argument('--period', type=float, default=1.5)
    ap.add_argument('--save-dir', default='')
    ap.add_argument('--video', default='')
    ap.add_argument('--out-fps', type=float, default=2.0)
    ap.add_argument('--max-frames', type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = WoundDepthNode(a)
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
