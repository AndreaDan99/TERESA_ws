"""Pose detection + posture benchmark/overlay on the Jetson Orin.
Ports wound_det's pose suite: backend = yolo (COCO-17) | mediapipe (BlazePose-33).
Also runs the posture classifier (lying/sitting/standing) and burns the label on the overlay.

Usage:
  python3 bench_pose.py --backend yolo     --model /work/models/yolo11x-pose.pt --frames DIR --out OUT
  python3 bench_pose.py --backend mediapipe --model /work/models/pose_landmarker_heavy.task --frames DIR --out OUT
Options: --limit N --fps F --no-video --gravity gx,gy
"""
import os, sys, time, json, glob, argparse
import numpy as np, cv2

COCO17_NAMES = ["nose","l_eye","r_eye","l_ear","r_ear","l_shoulder","r_shoulder",
                "l_elbow","r_elbow","l_wrist","r_wrist","l_hip","r_hip",
                "l_knee","r_knee","l_ankle","r_ankle"]
COCO17_EDGES = [(0,1),(0,2),(1,3),(2,4),(0,5),(0,6),(5,7),(7,9),(6,8),(8,10),
                (5,6),(5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16)]
BLAZE_NAMES = ["nose","l_eye_in","l_eye","l_eye_out","r_eye_in","r_eye","r_eye_out","l_ear",
               "r_ear","mouth_l","mouth_r","l_shoulder","r_shoulder","l_elbow","r_elbow","l_wrist",
               "r_wrist","l_pinky","r_pinky","l_index","r_index","l_thumb","r_thumb","l_hip","r_hip",
               "l_knee","r_knee","l_ankle","r_ankle","l_heel","r_heel","l_foot_index","r_foot_index"]
BLAZE_EDGES = [(11,12),(11,13),(13,15),(12,14),(14,16),(11,23),(12,24),(23,24),
               (15,17),(15,19),(15,21),(16,18),(16,20),(16,22),(23,25),(24,26),
               (25,27),(26,28),(27,29),(28,30),(27,31),(28,32),(0,11),(0,12)]
CANON = ["nose","l_shoulder","r_shoulder","l_hip","r_hip","l_knee","r_knee","l_ankle","r_ankle"]


# ---------- posture classifier (ported from wound_det/methods/posture/run_posture.py) ----------
def _ang(a, b, c):
    if a is None or b is None or c is None: return None
    v1, v2 = np.asarray(a)-np.asarray(b), np.asarray(c)-np.asarray(b)
    n = np.linalg.norm(v1)*np.linalg.norm(v2)
    if n < 1e-6: return None
    return float(np.degrees(np.arccos(np.clip(np.dot(v1, v2)/n, -1, 1))))

def _mid(p, q):
    if p is None or q is None: return None
    return (np.asarray(p)+np.asarray(q))/2.0

def classify(kp, idx, gravity_down=(0.0, 1.0)):
    def get(name):
        i = idx.get(name)
        if i is None: return None
        v = kp[i]
        return v if np.all(np.isfinite(v)) else None
    sh, hp = _mid(get("l_shoulder"), get("r_shoulder")), _mid(get("l_hip"), get("r_hip"))
    if sh is None or hp is None: return "unknown", 0.0, {}
    hip_angs, knee_angs = [], []
    for s in ["l", "r"]:
        ha = _ang(get(f"{s}_shoulder"), get(f"{s}_hip"), get(f"{s}_knee"))
        ka = _ang(get(f"{s}_hip"), get(f"{s}_knee"), get(f"{s}_ankle"))
        if ha is not None: hip_angs.append(ha)
        if ka is not None: knee_angs.append(ka)
    hip_flex = float(np.mean(hip_angs)) if hip_angs else None
    knee_flex = float(np.mean(knee_angs)) if knee_angs else None
    torso = sh - hp
    up = -np.asarray(gravity_down, float); up = up/(np.linalg.norm(up)+1e-9)
    tn = np.linalg.norm(torso)
    body_tilt = float(np.degrees(np.arccos(np.clip(np.dot(torso, up)/(tn+1e-9), -1, 1)))) if tn > 1e-6 else None
    valid = np.array([p for p in [get(n) for n in CANON] if p is not None])
    aspect = None
    if len(valid) >= 3:
        w = valid[:,0].max()-valid[:,0].min(); h = valid[:,1].max()-valid[:,1].min()
        aspect = float(w/(h+1e-6))
    feats = dict(hip_flex=hip_flex, knee_flex=knee_flex, body_tilt=body_tilt, aspect=aspect)
    flexed = ((hip_flex is not None and hip_flex < 120) or (knee_flex is not None and knee_flex < 110))
    if flexed:
        posture = "lying" if (body_tilt is not None and body_tilt > 60) else "sitting"
    else:
        horiz = (body_tilt is not None and body_tilt > 45) or (aspect is not None and aspect > 1.2)
        posture = "lying" if horiz else "standing"
    conf = 0.5
    if body_tilt is not None: conf = float(np.clip(abs(body_tilt-45)/45, 0.2, 1.0))
    if aspect is not None and posture == "lying": conf = max(conf, float(np.clip(aspect-1.0, 0.2, 1.0)))
    return posture, round(conf, 3), feats


def compute_det_rate(kpts, conf, thr=0.3):
    T = kpts.shape[0]
    vf = np.array([np.isfinite(kpts[t]).all() and (conf[t] > thr).mean() > 0.5 for t in range(T)])
    return float(vf.mean())


def ap():
    p = argparse.ArgumentParser()
    p.add_argument("--backend", required=True, choices=["yolo", "mediapipe"])
    p.add_argument("--model", required=True)
    p.add_argument("--frames", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--fps", type=float, default=4.0)
    p.add_argument("--no-video", action="store_true")
    p.add_argument("--gravity", default="0.0,1.0")
    return p.parse_args()


def main():
    a = ap()
    os.makedirs(os.path.join(a.out, "overlays"), exist_ok=True)
    grav = tuple(float(x) for x in a.gravity.split(","))
    frames = sorted(glob.glob(os.path.join(a.frames, "*.jpg")))
    if a.limit: frames = frames[:a.limit]
    T = len(frames); assert T, f"no frames in {a.frames}"
    names = COCO17_NAMES if a.backend == "yolo" else BLAZE_NAMES
    edges = COCO17_EDGES if a.backend == "yolo" else BLAZE_EDGES
    J = len(names); idx = {n: i for i, n in enumerate(names)}
    kpts = np.full((T, J, 2), np.nan, np.float32); conf = np.zeros((T, J), np.float32)

    import torch
    dev_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"
    if a.backend == "yolo":
        from ultralytics import YOLO
        model = YOLO(a.model)
        def infer(fp, t):
            r = model.predict(fp, device=0, verbose=False, conf=0.25)[0]
            if r.keypoints is not None and r.boxes is not None and len(r.boxes) > 0:
                areas = (r.boxes.xywh[:,2]*r.boxes.xywh[:,3]).cpu().numpy()
                i = int(np.argmax(areas))
                xy = r.keypoints.xy[i].cpu().numpy(); kc = r.keypoints.conf[i].cpu().numpy()
                for j in range(J):
                    if kc[j] > 0.05: kpts[t,j] = xy[j]; conf[t,j] = kc[j]
    else:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        opts = mp_vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=a.model),
            running_mode=mp_vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=0.5, min_tracking_confidence=0.5)
        lm = mp_vision.PoseLandmarker.create_from_options(opts)
        _mp_ts = [0]  # strictly-monotonic ms timestamp (VIDEO mode requirement)
        def infer(fp, t):
            img = cv2.imread(fp); H, W = img.shape[:2]
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            _mp_ts[0] += 33
            res = lm.detect_for_video(mp_img, _mp_ts[0])
            if res.pose_landmarks:
                for j, p in enumerate(res.pose_landmarks[0]):
                    kpts[t,j] = [p.x*W, p.y*H]; conf[t,j] = getattr(p, "visibility", 1.0)

    img0 = cv2.imread(frames[0]); H, W = img0.shape[:2]
    vw = None if a.no_video else cv2.VideoWriter(os.path.join(a.out, "overlay.mp4"),
                                                 cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    # warmup
    infer(frames[0], 0); kpts[0] = np.nan; conf[0] = 0
    postures = []
    t0 = time.time()
    for t, fp in enumerate(frames):
        infer(fp, t)
        post, pconf, _ = classify(kpts[t], idx, grav)
        postures.append(post)
        if vw is not None:
            img = cv2.imread(fp)
            for jx, jy in kpts[t]:
                if np.isfinite([jx, jy]).all(): cv2.circle(img, (int(jx), int(jy)), 4, (0, 220, 0), -1)
            for x, y in edges:
                pa, pb = kpts[t, x], kpts[t, y]
                if np.isfinite(pa).all() and np.isfinite(pb).all():
                    cv2.line(img, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])), (255, 128, 0), 2)
            cv2.putText(img, f"{a.backend} | {os.path.basename(fp)}", (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            col = (0, 255, 0) if post == "lying" else (0, 200, 255) if post != "unknown" else (120, 120, 120)
            cv2.putText(img, f"POSTURE: {post.upper()} ({pconf})", (10, H-18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
            vw.write(img)
            if t % 5 == 0: cv2.imwrite(os.path.join(a.out, "overlays", os.path.basename(fp)), img)
    if vw is not None: vw.release()
    dt = time.time()-t0

    det_rate = compute_det_rate(kpts, conf)
    valid_post = [p for p in postures if p != "unknown"]
    dist = {p: round(valid_post.count(p)/len(valid_post), 3) for p in set(valid_post)} if valid_post else {}
    dominant = max(dist, key=dist.get) if dist else "unknown"
    metrics = dict(backend=a.backend, model=os.path.basename(a.model), device=dev_name,
                   torch=torch.__version__, n_frames=T, n_joints=J,
                   detection_rate=round(det_rate, 4), fps=round(T/dt, 2),
                   ms_per_frame=round(dt/T*1000, 1),
                   posture_classified_rate=round(len(valid_post)/T, 4),
                   dominant_posture=dominant, posture_distribution=dist, gravity_down=list(grav))
    np.savez_compressed(os.path.join(a.out, "track.npz"), kpts=kpts, conf=conf,
                        meta=json.dumps(dict(joint_names=names, backend=a.backend, fps=a.fps, res=[W, H])))
    with open(os.path.join(a.out, "posture.json"), "w") as f:
        json.dump([{"frame": os.path.basename(frames[t]), "posture": postures[t]} for t in range(T)], f)
    with open(os.path.join(a.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print("METRICS", json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
