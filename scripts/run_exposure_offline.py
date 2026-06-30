#!/usr/bin/env python3
"""
run_exposure_offline.py — TERESA exposure experiment (offline).

Replicates the onboard pipeline with the RealSense D415:
  - NLF (24 SMPL joints) on the wide Orbbec-height photo
  - GroundingDINO on wide + close-up RGB-D photos
  - Depth back-projection → 3D positions in camera frame
  - Distance filter (wide shot only, using NLF keypoints)
  - Identical GDINO model, vocab & thresholds as the ROS2 nodes.

Usage:
  python scripts/run_exposure_offline.py --exp_dir experiments/exp_01

Directory structure per experiment:
  experiments/exp_01/
    wide_color.png          # RGB from Orbbec height (~0.5 m)
    wide_depth.png          # 16-bit PNG, values = millimetres
    camera_info.json        # (optional) {"fx":612, "fy":612, "cx":323, "cy":239, ...}
    close_up/
      01_color.png          # top-down RGB covering one body region
      01_depth.png          # aligned depth in mm
      02_color.png
      02_depth.png
      ...                   # 4-5 pairs typically

Output (written into exp_dir):
  nlf_keypoints.json        # 24 SMPL joints from NLF
  wide_overlay.jpg          # wide shot with GDINO boxes
  close_up/overlays/
    01_overlay.jpg          # each close-up with boxes + detection IDs
    ...
  predictions.json          # all detections with 3D positions, ready for review
"""

import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision  # noqa: F401 — needed by NLF TorchScript model
from PIL import Image

# ═══════════════════════════════════════════════════════════════════════
#  Jetson TorchScript workaround (from bench_nlf.py)
#  Without this, the 2nd NLF forward pass deadlocks on Orin.
# ═══════════════════════════════════════════════════════════════════════
torch._C._jit_set_profiling_executor(False)
torch._C._jit_set_profiling_mode(False)
for _fn, _a in [("_jit_set_texpr_fuser_enabled", False),
                ("_jit_override_can_fuse_on_gpu", False),
                ("_jit_override_can_fuse_on_cpu", False),
                ("_jit_set_nvfuser_enabled", False)]:
    try:
        getattr(torch._C, _fn)(_a)
    except Exception:
        pass

# ═══════════════════════════════════════════════════════════════════════
#  Constants — identical to the ROS2 nodes
# ═══════════════════════════════════════════════════════════════════════

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

GDINO_BOX_THRESHOLD = 0.18
GDINO_TEXT_THRESHOLD = 0.12
GDINO_MODEL_ID = "IDEA-Research/grounding-dino-base"

# Intrinsics fallback — RealSense D415 640×480 (fx,fy vary slightly per unit)
DEFAULT_K = {"fx": 612.0, "fy": 612.0, "cx": 323.0, "cy": 239.0,
             "width": 640, "height": 480}

# SMPL-24 joint indices (from sml_pose_indices.py)
PELVIS, HIP_LEFT, HIP_RIGHT = 0, 1, 2
SPINE1, KNEE_LEFT, KNEE_RIGHT = 3, 4, 5
SPINE2, ANKLE_LEFT, ANKLE_RIGHT = 6, 7, 8
SPINE3, FOOT_LEFT, FOOT_RIGHT = 9, 10, 11
NECK, COLLAR_LEFT, COLLAR_RIGHT = 12, 13, 14
HEAD = 15
SHOULDER_LEFT, SHOULDER_RIGHT = 16, 17
ELBOW_LEFT, ELBOW_RIGHT = 18, 19
WRIST_LEFT, WRIST_RIGHT = 20, 21
HAND_LEFT, HAND_RIGHT = 22, 23
NUM_JOINTS = 24

SMPL_JOINT_NAMES = [
    "PELVIS", "HIP_LEFT", "HIP_RIGHT", "SPINE1", "KNEE_LEFT", "KNEE_RIGHT",
    "SPINE2", "ANKLE_LEFT", "ANKLE_RIGHT", "SPINE3", "FOOT_LEFT", "FOOT_RIGHT",
    "NECK", "COLLAR_LEFT", "COLLAR_RIGHT", "HEAD", "SHOULDER_LEFT",
    "SHOULDER_RIGHT", "ELBOW_LEFT", "ELBOW_RIGHT", "WRIST_LEFT", "WRIST_RIGHT",
    "HAND_LEFT", "HAND_RIGHT",
]

# ═══════════════════════════════════════════════════════════════════════
#  NLF inference  (identical to nlf_skeleton.py._run_nlf_inference)
# ═══════════════════════════════════════════════════════════════════════


def load_nlf(model_path, device="cuda"):
    if not os.path.exists(model_path):
        print(f"[NLF] Model not found: {model_path}")
        return None
    print(f"[NLF] Loading {model_path} on {device} …")
    use_cuda = device == "cuda" and torch.cuda.is_available()
    map_loc = "cuda" if use_cuda else "cpu"
    try:
        model = torch.jit.load(model_path, map_location=map_loc)
    except RuntimeError:
        # Fallback: load on CPU then move
        model = torch.jit.load(model_path, map_location="cpu")
        if use_cuda:
            model = model.cuda()
    model.eval()
    print("[NLF] Model loaded.")
    return model


def run_nlf(model, img_rgb, conf_thr=0.3, device="cuda"):
    """Returns list[dict] per person: joints3d, joints3d_mm, conf, bbox_score,
    box_id, vertices3d (opt), bbox_xyxy (opt)."""
    if model is None:
        return []

    image_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).unsqueeze(0)
    if device == "cuda" and torch.cuda.is_available():
        image_tensor = image_tensor.cuda()

    with torch.inference_mode():
        pred = model.detect_smpl_batched(
            image_tensor, default_fov_degrees=55.0, num_aug=1,
            detector_threshold=conf_thr, internal_batch_size=64,
            suppress_implausible_poses=True)

    if (not pred.get("joints3d") or len(pred["joints3d"]) == 0
            or len(pred["joints3d"][0]) == 0):
        return []

    joints_mm = pred["joints3d"][0].cpu().numpy()
    joints_m = joints_mm / 1000.0
    n_people = joints_m.shape[0]

    if pred.get("joint_uncertainties") and len(pred["joint_uncertainties"]) > 0:
        uncert_mm = pred["joint_uncertainties"][0].cpu().numpy()
        conf = np.exp(-uncert_mm / 50.0)
    else:
        conf = np.ones((n_people, NUM_JOINTS), dtype=np.float64)

    box_scores = np.ones(n_people, dtype=np.float64)
    box_ids = list(range(n_people))
    boxes_xyxy = None
    if pred.get("boxes") and len(pred["boxes"]) > 0:
        boxes = pred["boxes"][0].cpu().numpy()
        if boxes.shape[0] == n_people:
            box_scores = boxes[:, 4]
            # NLF returns [cx, cy, w, h] — convert to [x1, y1, x2, y2]
            cx, cy, bw, bh = (boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3])
            boxes_xyxy = np.stack([
                cx - bw / 2, cy - bh / 2,
                cx + bw / 2, cy + bh / 2], axis=1)
            if boxes.shape[1] > 5:
                box_ids = [int(b) for b in boxes[:, 5]]

    vertices_list = [None] * n_people
    if pred.get("vertices3d") and len(pred["vertices3d"]) > 0:
        verts_mm = pred["vertices3d"][0].cpu().numpy()
        if verts_mm.shape[0] == n_people:
            for p in range(n_people):
                vertices_list[p] = verts_mm[p] / 1000.0

    detections = []
    for p in range(n_people):
        det = {"joints3d": joints_m[p], "joints3d_mm": joints_mm[p],
               "conf": conf[p].astype(np.float64),
               "bbox_score": float(box_scores[p]), "box_id": box_ids[p]}
        if vertices_list[p] is not None:
            det["vertices3d"] = vertices_list[p]
        if boxes_xyxy is not None:
            det["bbox_xyxy"] = boxes_xyxy[p].tolist()
        detections.append(det)
    return detections


# ═══════════════════════════════════════════════════════════════════════
#  GroundingDINO  (identical to injury_detector_gdino.py._infer_raw)
# ═══════════════════════════════════════════════════════════════════════


def load_gdino(device="cuda", cache_dir=None):
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
    print(f"[GDINO] Loading {GDINO_MODEL_ID} on {device} …")
    load_kwargs = {}
    if cache_dir:
        load_kwargs["cache_dir"] = cache_dir
    processor = AutoProcessor.from_pretrained(GDINO_MODEL_ID, **load_kwargs)
    model = (AutoModelForZeroShotObjectDetection
             .from_pretrained(GDINO_MODEL_ID, **load_kwargs)
             .to(device).eval())
    text_prompt = ". ".join(v.lower() for v in EXPOSURE_VOCAB) + "."
    print(f"[GDINO] Loaded.  Vocab: {len(EXPOSURE_VOCAB)} classes.")
    return processor, model, text_prompt


def run_gdino(processor, model, text_prompt, pil_image, device="cuda"):
    """Returns list[dict]: {box: [x1,y1,x2,y2], score, label}."""
    Wd, Ht = pil_image.size
    dets = []
    inp = processor(images=pil_image, text=text_prompt,
                    return_tensors="pt").to(device)
    with torch.no_grad():
        out = model(**inp)
    try:
        res = processor.post_process_grounded_object_detection(
            out, inp.input_ids, box_threshold=GDINO_BOX_THRESHOLD,
            text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
    except TypeError:
        res = processor.post_process_grounded_object_detection(
            out, inp.input_ids, threshold=GDINO_BOX_THRESHOLD,
            text_threshold=GDINO_TEXT_THRESHOLD, target_sizes=[(Ht, Wd)])[0]
    labs = res.get("text_labels", res.get("labels"))
    for b, s, l in zip(res["boxes"].cpu().numpy(),
                       res["scores"].cpu().numpy(), labs):
        dets.append({"box": [float(v) for v in b], "score": float(s),
                     "label": l if isinstance(l, str) else "injury"})
    return dets


# ═══════════════════════════════════════════════════════════════════════
#  Depth back-projection  (identical to injury_detector_gdino.py)
# ═══════════════════════════════════════════════════════════════════════


def robust_depth_mm(depth, u, v, r=6):
    """Median of valid (>0) depth pixels in an r-patch around (u,v).
    Returns None when < 4 valid pixels exist."""
    H, W = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    x0, x1 = max(0, u - r), min(W, u + r + 1)
    y0, y1 = max(0, v - r), min(H, v + r + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size >= 4 else None


def back_project(u, v, depth, K):
    """2D pixel (u,v) + depth (mm) + intrinsics → 3D point (m) in camera frame."""
    zmm = robust_depth_mm(depth, u, v)
    if zmm is None:
        return None
    Z = zmm / 1000.0
    X = (u - K["cx"]) * Z / K["fx"]
    Y = (v - K["cy"]) * Z / K["fy"]
    return np.array([X, Y, Z], dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════════
#  Distance filter  (identical to injury_detector_gdino.py)
# ═══════════════════════════════════════════════════════════════════════


def distance_filter(detections, skeleton_3d, thr_m=0.15):
    """Drop detections whose 3D position is > thr_m from the nearest SMPL joint.
    Adds 'distance_to_body_m' on kept items."""
    if skeleton_3d is None or (hasattr(skeleton_3d, '__len__') and len(skeleton_3d) == 0):
        for d in detections:
            d["distance_to_body_m"] = None
        return detections

    kept = []
    for det in detections:
        pos = det.get("position_3d")
        if pos is None:
            continue
        pw = np.array(pos)
        min_dist = min(np.linalg.norm(pw - joint)
                       for joint in skeleton_3d
                       if not np.isnan(joint).any())
        if min_dist < thr_m:
            det["distance_to_body_m"] = float(min_dist)
            kept.append(det)
    return kept


# ═══════════════════════════════════════════════════════════════════════
#  Body crop  (wide shot only, using NLF bbox)
# ═══════════════════════════════════════════════════════════════════════


def crop_to_body(image, bbox_xyxy, margin_pct=0.15):
    """Returns (cropped_image, (ox, oy)).
    bbox_xyxy can be [x1,y1,x2,y2] (xyxy) or [cx,cy,w,h] (YOLO) — auto-detects."""
    if not bbox_xyxy or len(bbox_xyxy) != 4:
        return image, (0, 0)
    a, b, c, d = [float(v) for v in bbox_xyxy]
    # Detect format: if a > c it's likely [cx,cy,w,h]
    if a > c:
        cx, cy, bw, bh = a, b, c, d
        x1, y1 = cx - bw / 2, cy - bh / 2
        x2, y2 = cx + bw / 2, cy + bh / 2
    else:
        x1, y1, x2, y2 = a, b, c, d
    w_box, h_box = x2 - x1, y2 - y1
    if w_box <= 0 or h_box <= 0:
        return image, (0, 0)
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    ox = max(0, int(x1 - w_box * margin_pct))
    oy = max(0, int(y1 - h_box * margin_pct))
    ex = min(image.shape[1], int(x2 + w_box * margin_pct))
    ey = min(image.shape[0], int(y2 + h_box * margin_pct))
    if ex <= ox or ey <= oy:
        return image, (0, 0)
    return image[oy:ey, ox:ex], (ox, oy)


# ═══════════════════════════════════════════════════════════════════════
#  Drawing
# ═══════════════════════════════════════════════════════════════════════


def draw_detections(bgr, dets, color=(0, 255, 0)):
    """Draw bounding boxes with detection IDs and labels. Returns overlay."""
    out = bgr.copy()
    for i, d in enumerate(dets):
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        lab = f'D{i} {d["label"]} {d["score"]:.2f}'
        (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(out, (x1, y1 - th - 4), (x1 + tw + 2, y1), color, -1)
        cv2.putText(out, lab, (x1 + 1, y1 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    return out


# ═══════════════════════════════════════════════════════════════════════
#  Camera info
# ═══════════════════════════════════════════════════════════════════════


def load_camera_info(exp_dir):
    """Load K from camera_info.json, or use D415 defaults."""
    ci_path = Path(exp_dir) / "camera_info.json"
    if ci_path.exists():
        with open(ci_path) as f:
            d = json.load(f)
        K = {"fx": d["fx"], "fy": d["fy"], "cx": d["cx"], "cy": d["cy"]}
        print(f"[CAM] Loaded intrinsics from camera_info.json  "
              f"fx={K['fx']:.1f} fy={K['fy']:.1f}")
        return K
    print(f"[CAM] camera_info.json not found — using D415 defaults  "
          f"fx={DEFAULT_K['fx']:.1f} fy={DEFAULT_K['fy']:.1f}")
    return dict(DEFAULT_K)


def load_depth(path):
    """Read a 16-bit PNG depth image. Returns np.ndarray (mm) or None."""
    if not path.exists():
        return None
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        # Try PNG with PIL for non-standard encodings
        pil_img = Image.open(path)
        depth = np.array(pil_img, dtype=np.uint16)
    return depth


# ═══════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════


def main():
    ap = argparse.ArgumentParser(
        description="TERESA offline exposure experiment — NLF + GDINO + depth")
    ap.add_argument("--exp_dir", required=True)
    ap.add_argument("--nlf_model", default="nlf_s_multi.torchscript")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--cache_dir", default=None, help="HF cache directory")
    ap.add_argument("--skip_nlf", action="store_true")
    args = ap.parse_args()

    exp_dir = Path(args.exp_dir)
    if not exp_dir.is_dir():
        sys.exit(f"ERROR: {exp_dir} not found.")

    wide_rgb_path = exp_dir / "wide_color.png"
    wide_depth_path = exp_dir / "wide_depth.png"
    close_up_dir = exp_dir / "close_up"

    if not wide_rgb_path.exists():
        sys.exit(f"ERROR: {wide_rgb_path} not found.")
    if not close_up_dir.is_dir():
        sys.exit(f"ERROR: {close_up_dir} not found.")

    device = args.device
    if device == "cuda" and not torch.cuda.is_available():
        print("[WARN] CUDA not available — falling back to CPU")
        device = "cpu"

    K = load_camera_info(exp_dir)

    print(f"═{'═' * 58}")
    print(f"  TERESA Offline Exposure Experiment")
    print(f"  Device: {device}  |  Experiment: {exp_dir}")
    print(f"═{'═' * 58}")

    # ═══════════════════════════════════════════════════════════════
    #  Step 1 — NLF on wide shot
    # ═══════════════════════════════════════════════════════════════
    nlf_detections = []
    body_bbox = None
    nlf_skeleton_cam = []   # SMPL joints in camera frame (m) for distance filter

    if not args.skip_nlf:
        nlf_model = load_nlf(args.nlf_model, device)
        if nlf_model is not None:
            print("\n[NLF] Wide shot …")
            wide_rgb = np.array(Image.open(wide_rgb_path).convert("RGB"))
            t0 = time.time()
            nlf_detections = run_nlf(nlf_model, wide_rgb, device=device)
            dt = time.time() - t0
            print(f"[NLF] {len(nlf_detections)} person(s) in {dt:.1f}s")

            if nlf_detections:
                best = nlf_detections[0]
                body_bbox = best.get("bbox_xyxy")
                nlf_skeleton_cam = best["joints3d"]  # (24,3) m, camera frame
                print(f"[NLF] Body bbox: {body_bbox}  "
                      f"bbox_score={best['bbox_score']:.3f}")

                kp_json = {
                    "joints3d_mm": best["joints3d_mm"].tolist(),
                    "joint_names": SMPL_JOINT_NAMES,
                    "bbox_score": best["bbox_score"],
                    "bbox_xyxy": body_bbox,
                }
                kp_path = exp_dir / "nlf_keypoints.json"
                with open(kp_path, "w") as f:
                    json.dump(kp_json, f, indent=2)
                print(f"[NLF] Keypoints → {kp_path}")
        else:
            print("[NLF] Skipped — model not found.")
    else:
        print("[NLF] Skipped (--skip_nlf).")

    # ═══════════════════════════════════════════════════════════════
    #  Step 2 — Load GroundingDINO
    # ═══════════════════════════════════════════════════════════════
    print("\n[GDINO] Loading …")
    gdino_proc, gdino_model, gdino_text = load_gdino(device, args.cache_dir)

    # Warmup on wide shot (cropped if bbox available)
    warmup_pil = Image.open(wide_rgb_path).convert("RGB")
    if body_bbox:
        warmup_rgb = np.array(warmup_pil)
        warmup_cropped, _ = crop_to_body(warmup_rgb, body_bbox)
        warmup_pil = Image.fromarray(warmup_cropped)
    print("[GDINO] Warmup …")
    for _ in range(2):
        run_gdino(gdino_proc, gdino_model, gdino_text, warmup_pil, device)
    if device == "cuda":
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
    print("[GDINO] Ready.")

    # ═══════════════════════════════════════════════════════════════
    #  Step 3 — GDINO on wide shot  (with depth + distance filter)
    # ═══════════════════════════════════════════════════════════════
    print("\n[GDINO] Wide shot …")
    wide_depth = load_depth(wide_depth_path)
    wide_pil = Image.open(wide_rgb_path).convert("RGB")
    wide_bgr = cv2.imread(str(wide_rgb_path))

    if body_bbox:
        wide_cropped, wide_crop_offset = crop_to_body(wide_bgr, body_bbox)
        wide_pil_cropped = Image.fromarray(
            cv2.cvtColor(wide_cropped, cv2.COLOR_BGR2RGB))
    else:
        wide_pil_cropped = wide_pil
        wide_crop_offset = (0, 0)

    t0 = time.time()
    wide_dets = run_gdino(gdino_proc, gdino_model, gdino_text,
                          wide_pil_cropped, device)
    if device == "cuda":
        torch.cuda.synchronize()
    dt = time.time() - t0

    ox, oy = wide_crop_offset
    for d in wide_dets:
        d["box"] = [d["box"][0] + ox, d["box"][1] + oy,
                    d["box"][2] + ox, d["box"][3] + oy]
        d["source"] = "wide"

    # Back-project wide detections to 3D
    for d in wide_dets:
        x1, y1, x2, y2 = d["box"]
        uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        pos3d = back_project(uc, vc, wide_depth, K) if wide_depth is not None else None
        d["position_3d"] = pos3d.tolist() if pos3d is not None else None

    # Distance filter (wide shot only — we have NLF keypoints here)
    wide_dets_before = len(wide_dets)
    wide_dets = distance_filter(wide_dets, nlf_skeleton_cam, thr_m=0.15)
    print(f"[GDINO] Wide: {wide_dets_before} raw → {len(wide_dets)} "
          f"after distance filter in {dt:.1f}s")

    wide_overlay = draw_detections(wide_bgr, wide_dets)
    cv2.imwrite(str(exp_dir / "wide_overlay.jpg"), wide_overlay)
    print(f"[GDINO] wide_overlay.jpg saved")

    # ═══════════════════════════════════════════════════════════════
    #  Step 4 — GDINO on close-up photos  (with depth, no distance filter)
    # ═══════════════════════════════════════════════════════════════
    close_up_paths = sorted(
        [p for p in close_up_dir.glob("*_color.png")
         if "overlay" not in p.name])

    if not close_up_paths:
        close_up_paths = sorted(
            [p for p in close_up_dir.glob("*.jpg") if "overlay" not in p.name] +
            [p for p in close_up_dir.glob("*.png") if "overlay" not in p.name])

    if not close_up_paths:
        sys.exit(f"ERROR: no *_color.png or .jpg found in {close_up_dir}")

    print(f"\n[GDINO] {len(close_up_paths)} close-up photos …")
    overlays_dir = close_up_dir / "overlays"
    overlays_dir.mkdir(exist_ok=True)

    all_close_dets = []
    total_infer_time = 0.0

    for idx, rgb_path in enumerate(close_up_paths):
        # Match depth file: 01_color.png → 01_depth.png
        stem = rgb_path.stem
        if stem.endswith("_color"):
            depth_stem = stem.replace("_color", "_depth")
        else:
            depth_stem = stem + "_depth"
        depth_path = rgb_path.with_name(depth_stem + rgb_path.suffix)

        pil_img = Image.open(rgb_path).convert("RGB")
        bgr_img = cv2.imread(str(rgb_path))
        if bgr_img is None:
            print(f"  [{idx+1}/{len(close_up_paths)}] {rgb_path.name} — SKIP")
            continue
        depth = load_depth(depth_path)

        t0 = time.time()
        dets = run_gdino(gdino_proc, gdino_model, gdino_text, pil_img, device)
        if device == "cuda":
            torch.cuda.synchronize()
        dt = time.time() - t0
        total_infer_time += dt

        # Back-project + assign IDs
        for didx, d in enumerate(dets):
            d["source"] = rgb_path.name
            x1, y1, x2, y2 = d["box"]
            uc, vc = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            pos3d = back_project(uc, vc, depth, K) if depth is not None else None
            d["position_3d"] = pos3d.tolist() if pos3d is not None else None

        all_close_dets.extend(dets)

        ov = draw_detections(bgr_img, dets)
        ov_path = overlays_dir / f"{rgb_path.stem}_overlay.jpg"
        cv2.imwrite(str(ov_path), ov)

        print(f"  [{idx+1}/{len(close_up_paths)}] {rgb_path.name}: "
              f"{len(dets)} dets, {dt:.1f}s")

    n_with = sum(1 for p in close_up_paths if p.exists())
    print(f"\n[GDINO] Close-up summary:")
    print(f"  Photos: {len(close_up_paths)}  |  "
          f"Total detections: {len(all_close_dets)}  |  "
          f"Mean time: {total_infer_time / max(len(close_up_paths), 1) * 1000:.0f} ms")

    # ═══════════════════════════════════════════════════════════════
    #  Step 5 — Combine (no NMS — user groups duplicates manually)
    # ═══════════════════════════════════════════════════════════════
    all_dets = wide_dets + all_close_dets

    # Re-index all detections with a global ID for easy reference
    for i, d in enumerate(all_dets):
        d["id"] = i

    # ═══════════════════════════════════════════════════════════════
    #  Step 6 — Save predictions
    # ═══════════════════════════════════════════════════════════════
    predictions = {
        "experiment": str(exp_dir),
        "n_photos": len(close_up_paths) + 1,   # wide + close-ups
        "n_close_up_photos": len(close_up_paths),
        "n_detections": len(all_dets),
        "n_wide_detections": len(wide_dets),
        "n_wide_raw_before_filter": wide_dets_before,
        "n_close_up_detections": len(all_close_dets),
        "mean_inference_time_ms": round(
            total_infer_time / max(len(close_up_paths), 1) * 1000, 0),
        "gdino_model": GDINO_MODEL_ID,
        "box_threshold": GDINO_BOX_THRESHOLD,
        "text_threshold": GDINO_TEXT_THRESHOLD,
        "vocab_size": len(EXPOSURE_VOCAB),
        "nlf_bbox_score": nlf_detections[0]["bbox_score"] if nlf_detections else None,
        "camera_intrinsics": K,
        # Each detection: {id, box, score, label, source, position_3d, ...}
        # source="wide" → from wide shot (coarse, distance-filtered)
        # source="01_color.png" → from close-up (per-region refinement)
        # After review, user adds: "verified": "tp"/"fp", "wound_id": "w1", ...
        "detections": all_dets,
    }

    pred_path = exp_dir / "predictions.json"
    with open(pred_path, "w") as f:
        json.dump(predictions, f, indent=2)

    print(f"\n{'═' * 60}")
    print(f"  ✓ {len(all_dets)} detections saved → {pred_path}")
    print(f"\n  Two-stage detection summary:")
    print(f"    Wide shot:  {wide_dets_before} raw → {len(wide_dets)} after distance filter")
    print(f"    Close-ups:  {len(all_close_dets)} across {len(close_up_paths)} photos")
    print(f"    Combined:   {len(all_dets)} total detections for manual review")
    print(f"\n  Next — manual review:")
    print(f"  1. Open close_up/overlays/ and wide_overlay.jpg")
    print(f"  2. For each detection, add to predictions.json:")
    print(f'     "verified": "tp" or "fp"')
    print(f'     "wound_id": "w1"  (same id = same wound seen in multiple photos)')
    print(f"  3. Add a \"ground_truth\" list with all real wounds you placed")
    print(f"  4. Add a \"missed\" list with wound IDs GDINO did NOT find")
    print(f"  5. Run: python scripts/compute_exposure_metrics.py "
          f"--predictions {pred_path}")
    print(f"{'═' * 60}\n")


if __name__ == "__main__":
    main()
