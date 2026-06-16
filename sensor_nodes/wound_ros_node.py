#!/usr/bin/env python3
"""Zero-shot wound detection as a live ROS2 node — subscribes a color Image topic,
runs Grounding-DINO (or OWLv2) on the Jetson GPU, publishes an overlay Image + a JSON
String of detections, and saves periodic overlays. Reuses bench_wound.py's exact
transformers-4.44.2 API. GDINO is ~1 fps on the Orin, so it runs on a slow timer.

Run (inside teresa_gpu, same DDS domain as the camera container):
  python3 wound_ros_node.py --topic /orbbec/color/image_raw --method gdino \
    --save-dir /work/live_wound_ros --period 2.0
"""
import os, json, time, argparse
import numpy as np, cv2
from PIL import Image as PILImage
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

WOUND_VOCAB = ["wound", "open wound", "laceration", "cut", "bleeding wound",
               "blood", "injury", "bruise", "surgical incision", "skin lesion"]
GDINO_BOX_THRESHOLD, GDINO_TEXT_THRESHOLD = 0.18, 0.12
OWL_THRESHOLD, OWL_NMS_IOU = 0.08, 0.30
CFG = {"owlv2": "google/owlv2-base-patch16-ensemble",
       "gdino": "IDEA-Research/grounding-dino-base"}


class WoundNode(Node):
    def __init__(self, a):
        super().__init__('wound_ros_node')
        self.a = a
        self.bridge = CvBridge()
        self.latest = None
        self.n = 0
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if a.save_dir:
            os.makedirs(a.save_dir, exist_ok=True)
        mid = CFG[a.method]
        self.get_logger().info(f"loading {a.method} {mid} on {self.device} …")
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        self.proc = AutoProcessor.from_pretrained(mid)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(mid).to(self.device).eval()
        self.gdino_text = ". ".join(v.lower() for v in WOUND_VOCAB) + "."
        self.owl_queries = [WOUND_VOCAB]
        self.get_logger().info(f"wound model ready; subscribing {a.topic}")
        self.create_subscription(Image, a.topic, self._cb, qos_profile_sensor_data)
        self.pub_overlay = self.create_publisher(Image, '/perception/wound/overlay', 1)
        self.pub_dets = self.create_publisher(String, '/perception/wound/dets', 10)
        self.create_timer(a.period, self._process)

    def _cb(self, msg):
        self.latest = msg

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
        else:  # owlv2
            inp = self.proc(text=self.owl_queries, images=pil, return_tensors="pt").to(self.device)
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
        if self.latest is None:
            return
        msg = self.latest
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        pil = PILImage.fromarray(rgb)
        if self.device == "cuda":
            torch.cuda.synchronize()
        t0 = time.time()
        dets = self._infer(pil)
        if self.device == "cuda":
            torch.cuda.synchronize()
        dt = time.time() - t0
        self.n += 1
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for d in dets:
            x1, y1, x2, y2 = [int(v) for v in d["box"]]
            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(bgr, f'{d["label"]} {d["score"]:.2f}', (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(bgr, f"WOUND {self.a.method} | {1.0/dt:.2f} fps | {len(dets)} det", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        self.pub_dets.publish(String(data=json.dumps(dets)))
        if self.a.save_dir:
            cv2.imwrite(os.path.join(self.a.save_dir, f"wound_{self.n:05d}.jpg"), bgr)
        self.get_logger().info(f"frame {self.n} | {1.0/dt:.2f} fps | {len(dets)} det "
                               f"| {[round(d['score'],2) for d in dets]}")
        if self.a.max_frames and self.n >= self.a.max_frames:
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/orbbec/color/image_raw')
    ap.add_argument('--method', default='gdino', choices=list(CFG))
    ap.add_argument('--period', type=float, default=2.0, help='seconds between inferences')
    ap.add_argument('--save-dir', default='')
    ap.add_argument('--max-frames', type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    node = WoundNode(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
