#!/usr/bin/env python3
"""
InjuryDetectorGDINO — ROS2 node for TERESA Exposure Body Scanning phase.

Subscribes to color / depth / camera_info / body_bbox / skeleton_3d topics,
runs GroundingDINO zero-shot inference on the cropped body region,
back-projects detection bbox centres to 3D (camera frame), transforms to
world frame via TF, and applies a distance filter against SMPL skeleton
joints.  Publishes filtered detections as JSON on /exposure/detection_raw.

Derived from the proven sensor_nodes/wound_depth_node.py pattern.
"""
import json
import time
import threading
import numpy as np
import cv2
from PIL import Image as PILImage
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

# ═══════════════════════════════════════════════════════════════════
#  Vocab — zero-shot text prompt for GroundingDINO
# ═══════════════════════════════════════════════════════════════════
EXPOSURE_VOCAB = [
    "open wound", "laceration", "cut", "bleeding wound",
    "puncture wound", "abrasion", "avulsion",
    "burn", "skin burn", "second degree burn", "third degree burn",
    "blister", "charred skin",
    "scar", "surgical scar", "keloid scar", "healed wound",
    "bruise", "hematoma", "contusion", "ecchymosis",
    "skin lesion", "rash", "ulcer", "pressure sore",
    "bandage", "dressing", "medical tape", "gauze",
]

# ═══════════════════════════════════════════════════════════════════
#  Utilities  (replicated from wound_depth_node.py lines 38-46)
# ═══════════════════════════════════════════════════════════════════


def robust_depth_mm(depth, u, v, r=6):
    """Median of valid ( >0 ) depth pixels in an r‑patch around (u,v).

    Returns None when fewer than 4 valid pixels exist.
    """
    H, W = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    x0, x1 = max(0, u - r), min(W, u + r + 1)
    y0, y1 = max(0, v - r), min(H, v + r + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size >= 4 else None


class InferenceTimeout(Exception):
    """Raised when GroundingDINO inference exceeds the configured timeout."""


# ═══════════════════════════════════════════════════════════════════
#  Node
# ═══════════════════════════════════════════════════════════════════


class InjuryDetectorGDINO(Node):
    """ROS2 node — zero-shot wound/scar/burn detection via GroundingDINO."""

    def __init__(self):
        super().__init__('injury_detector_gdino')

        # ── ROS2 Parameters ──────────────────────────────────────
        self.declare_parameter('model_id', 'IDEA-Research/grounding-dino-base')
        self.declare_parameter('box_threshold', 0.18)
        self.declare_parameter('text_threshold', 0.12)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('inference_timeout', 5.0)
        self.declare_parameter('cache_dir', '')
        self.declare_parameter('distance_filter_threshold_m', 0.15)
        self.declare_parameter('bbox_margin', 0.15)
        self.declare_parameter('process_period', 1.5)
        self.declare_parameter('color_topic', '/exposure/frame_to_process')
        self.declare_parameter('camera_info_topic', '/exposure/camera_info')
        self.declare_parameter('depth_topic', '/exposure/depth_frame')
        self.declare_parameter('body_bbox_topic', '/exposure/body_bbox')
        self.declare_parameter('skeleton_3d_topic', '/exposure/skeleton_3d')
        self.declare_parameter('detection_topic', '/exposure/detection_raw')

        # Resolve
        self._model_id = str(self.get_parameter('model_id').value)
        self._box_threshold = float(self.get_parameter('box_threshold').value)
        self._text_threshold = float(self.get_parameter('text_threshold').value)
        self._device = str(self.get_parameter('device').value)
        self._inference_timeout = float(self.get_parameter('inference_timeout').value)
        self._cache_dir = str(self.get_parameter('cache_dir').value).strip() or None
        self._distance_filter_thr = float(self.get_parameter('distance_filter_threshold_m').value)
        self._bbox_margin = float(self.get_parameter('bbox_margin').value)
        self._process_period = float(self.get_parameter('process_period').value)

        self._color_topic = str(self.get_parameter('color_topic').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._depth_topic = str(self.get_parameter('depth_topic').value)
        self._body_bbox_topic = str(self.get_parameter('body_bbox_topic').value)
        self._skeleton_3d_topic = str(self.get_parameter('skeleton_3d_topic').value)
        self._detection_topic = str(self.get_parameter('detection_topic').value)

        # ── State ───────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_color = None       # sensor_msgs.msg.Image
        self._depth = None              # np.ndarray (mm, passthrough)
        self._latest_body_bbox = None   # [x1, y1, x2, y2] in full-frame pixels
        self._latest_skeleton_3d = []   # list of np.array([x,y,z]) in camera frame
        self._K = None                  # 3×3 camera intrinsics
        self._camera_frame = None       # optical frame_id from CameraInfo
        self._crop_offset = (0, 0)      # (ox, oy) pixel offset added by crop step

        # ══ TF ══════════════════════════════════════════════════
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ══ Load GroundingDINO (HuggingFace transformers) ═══════
        self.get_logger().info(
            f"Loading GroundingDINO  {self._model_id}  on  {self._device} …"
        )
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

        load_kwargs = {}
        if self._cache_dir:
            load_kwargs['cache_dir'] = self._cache_dir
            self.get_logger().info(f"HF cache dir: {self._cache_dir}")

        self._processor = AutoProcessor.from_pretrained(self._model_id, **load_kwargs)
        self._model = (
            AutoModelForZeroShotObjectDetection.from_pretrained(
                self._model_id, **load_kwargs
            )
            .to(self._device)
            .eval()
        )

        # Build text prompt — one lowercase sentence per class
        self._gdino_text = ". ".join(v.lower() for v in EXPOSURE_VOCAB) + "."
        self.get_logger().info(
            f"GroundingDINO loaded.  Vocab: {len(EXPOSURE_VOCAB)} injury classes."
        )

        # ══ Subscribers ═════════════════════════════════════════
        self.create_subscription(
            Image, self._color_topic, self._cb_color, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._cb_info, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, self._depth_topic, self._cb_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32MultiArray, self._body_bbox_topic, self._cb_body_bbox, 10,
        )
        self.create_subscription(
            PoseArray, self._skeleton_3d_topic, self._cb_skeleton_3d, 10,
        )

        # ══ Publishers ══════════════════════════════════════════
        self._pub_detections = self.create_publisher(
            String, self._detection_topic, 10,
        )

        # ══ Processing timer ════════════════════════════════════
        self.create_timer(self._process_period, self._process)

        self.get_logger().info(
            f"InjuryDetectorGDINO ready  |  subs: [{self._color_topic}, "
            f"{self._depth_topic}, {self._body_bbox_topic}, "
            f"{self._skeleton_3d_topic}]  →  pub: {self._detection_topic}"
        )

    # ── Callbacks ────────────────────────────────────────────────

    def _cb_color(self, msg):
        self._latest_color = msg

    def _cb_depth(self, msg):
        self._depth = np.asarray(
            self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        )

    def _cb_info(self, msg):
        self._K = np.array(msg.k).reshape(3, 3)
        self._camera_frame = msg.header.frame_id

    def _cb_body_bbox(self, msg):
        if len(msg.data) == 4:
            self._latest_body_bbox = list(msg.data)

    def _cb_skeleton_3d(self, msg):
        joints = []
        for pose in msg.poses:
            joints.append(
                np.array([pose.position.x, pose.position.y, pose.position.z])
            )
        self._latest_skeleton_3d = joints

    # ── Crop logic ───────────────────────────────────────────────

    def _apply_crop(self, image, body_bbox):
        """Crop image to body_bbox + margin; store offset for back-projection.

        Returns the cropped image (or the full image when bbox is missing).
        """
        if body_bbox and len(body_bbox) == 4:
            x1, y1, x2, y2 = [int(v) for v in body_bbox]
            w, h = x2 - x1, y2 - y1
            ox = max(0, int(x1 - w * self._bbox_margin))
            oy = max(0, int(y1 - h * self._bbox_margin))
            ex = min(image.shape[1], int(x2 + w * self._bbox_margin))
            ey = min(image.shape[0], int(y2 + h * self._bbox_margin))
            self._crop_offset = (ox, oy)
            return image[oy:ey, ox:ex]
        self._crop_offset = (0, 0)
        return image

    # ── GroundingDINO inference  (99 % from wound_depth_node.py) ─

    def _infer_raw(self, pil_image):
        """Core GroundingDINO forward pass → list of detection dicts."""
        Wd, Ht = pil_image.size
        dets = []

        inp = self._processor(
            images=pil_image, text=self._gdino_text, return_tensors="pt",
        ).to(self._device)

        with torch.no_grad():
            out = self._model(**inp)

        try:
            res = self._processor.post_process_grounded_object_detection(
                out, inp.input_ids,
                box_threshold=self._box_threshold,
                text_threshold=self._text_threshold,
                target_sizes=[(Ht, Wd)],
            )[0]
        except TypeError:
            # HuggingFace API drift — older versions use `threshold`
            res = self._processor.post_process_grounded_object_detection(
                out, inp.input_ids,
                threshold=self._box_threshold,
                text_threshold=self._text_threshold,
                target_sizes=[(Ht, Wd)],
            )[0]

        labs = res.get("text_labels", res.get("labels"))
        for b, s, l in zip(
            res["boxes"].cpu().numpy(),
            res["scores"].cpu().numpy(),
            labs,
        ):
            dets.append({
                "box": [float(v) for v in b],
                "score": float(s),
                "label": l if isinstance(l, str) else "injury",
            })
        return dets

    def _infer_with_timeout(self, pil_image):
        """Run _infer_raw in a daemon thread with a join timeout.

        Raises InferenceTimeout when the forward pass exceeds
        ``inference_timeout`` seconds.  Note: the GPU kernel is NOT
        aborted — the timeout only prevents the node from blocking.
        """
        result = []
        exception = None
        completed = False
        lock = threading.Lock()

        def _target():
            nonlocal exception, completed
            try:
                dets = self._infer_raw(pil_image)
                with lock:
                    result.extend(dets)
                    completed = True
            except Exception as e:
                with lock:
                    exception = e
                    completed = True

        thread = threading.Thread(target=_target, daemon=True)
        thread.start()
        thread.join(timeout=self._inference_timeout)

        with lock:
            if not completed:
                self.get_logger().error(
                    f"GroundingDINO inference timed out after "
                    f"{self._inference_timeout:.1f}s"
                )
                raise InferenceTimeout(
                    f"Inference exceeded {self._inference_timeout}s timeout"
                )
            if exception is not None:
                raise exception

        return result

    # ── Distance filter ──────────────────────────────────────────

    def _distance_filter(self, detections_3d, skeleton_3d_world):
        """Drop detections whose world position is > distance_filter_thr from
        the nearest SMPL joint.  Populates ``distance_to_body_m`` on kept items.
        """
        if not skeleton_3d_world:
            for d in detections_3d:
                d['distance_to_body_m'] = None
            return detections_3d

        kept = []
        for det in detections_3d:
            pos_w = det['position_world']
            if pos_w is None:
                continue  # drop detections without valid 3D back-projection

            pw = np.array(pos_w)
            min_dist = min(
                np.linalg.norm(pw - joint)
                for joint in skeleton_3d_world
                if not np.isnan(joint).any()
            )
            if min_dist < self._distance_filter_thr:
                det['distance_to_body_m'] = float(min_dist)
                kept.append(det)

        return kept

    # ── TF helper ────────────────────────────────────────────────

    def _camera_to_world(self, point_camera, stamp):
        """Transform a 3-D point from the camera optical frame to world.

        ``point_camera`` : sequence (x, y, z) in metres.
        Returns a numpy array [x, y, z] in the world frame, or None on failure.
        """
        if self._camera_frame is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', self._camera_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.15),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup  {self._camera_frame}→world  failed: {e}",
                throttle_duration_sec=5.0,
            )
            return None

        ps = PoseStamped()
        ps.header.frame_id = self._camera_frame
        ps.header.stamp = stamp
        ps.pose.position.x = float(point_camera[0])
        ps.pose.position.y = float(point_camera[1])
        ps.pose.position.z = float(point_camera[2])
        ps.pose.orientation.w = 1.0

        ps_world = tf2_geometry_msgs.do_transform_pose(ps, tf)
        return np.array([
            ps_world.pose.position.x,
            ps_world.pose.position.y,
            ps_world.pose.position.z,
        ])

    # ── Main processing  ─────────────────────────────────────────

    def _process(self):
        """Periodic processing — triggered by the node timer at
        ``process_period`` Hz.

        Grabs the latest cached frames, crops to the body bounding box, runs
        GroundingDINO inference with a timeout, back-projects box centres to
        3-D, transforms to world, filters by skeleton distance, and publishes
        the result as JSON.
        """
        if self._latest_color is None or self._K is None:
            return

        msg = self._latest_color
        stamp = msg.header.stamp

        # Decode
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        H, W = rgb.shape[:2]

        # Body crop
        cropped = self._apply_crop(rgb, self._latest_body_bbox)
        ox, oy = self._crop_offset

        # ── Inference ────────────────────────────────────────
        if self._device == 'cuda':
            torch.cuda.synchronize()
        t0 = time.time()
        try:
            dets = self._infer_with_timeout(PILImage.fromarray(cropped))
        except InferenceTimeout:
            return  # error already logged
        except Exception as e:
            self.get_logger().error(f"Inference exception: {e}")
            return
        if self._device == 'cuda':
            torch.cuda.synchronize()
        dt = time.time() - t0

        depth = self._depth

        # Intrinsics
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]

        # ── Back-project → 3-D (camera), then → world ────────
        detections_out = []
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            # bbox centre in *full-frame* coordinates
            u = (x1 + x2) / 2.0 + ox
            v = (y1 + y2) / 2.0 + oy

            zmm = robust_depth_mm(depth, u, v) if depth is not None else None

            pos_cam, pos_world = None, None
            if zmm is not None:
                Z = zmm / 1000.0
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                pos_cam = [float(X), float(Y), float(Z)]
                pos_world = self._camera_to_world(pos_cam, stamp)

            det_dict = {
                "class": d["label"],
                "confidence": round(d["score"], 4),
                "bbox": [float(x1 + ox), float(y1 + oy),
                         float(x2 + ox), float(y2 + oy)],
                "position_camera": pos_cam,
                "position_world": pos_world.tolist() if pos_world is not None else None,
            }
            detections_out.append(det_dict)

        # ── Skeleton → world for distance filter ──────────────
        skeleton_world = []
        if self._latest_skeleton_3d:
            for joint_cam in self._latest_skeleton_3d:
                jw = self._camera_to_world(joint_cam, stamp)
                if jw is not None:
                    skeleton_world.append(jw)

        # ── Distance filter ───────────────────────────────────
        detections_out = self._distance_filter(detections_out, skeleton_world)

        # ── Publish ───────────────────────────────────────────
        output = json.dumps({"detections": detections_out})
        self._pub_detections.publish(String(data=output))

        n_det = len(detections_out)
        self.get_logger().info(
            f"Exposure GDINO  |  {1.0 / dt:.1f} fps  |  "
            f"{len(dets)} raw  →  {n_det} after distance filter  "
            f"(thr={self._distance_filter_thr:.2f}m)"
        )


# ═══════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    node = InjuryDetectorGDINO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
