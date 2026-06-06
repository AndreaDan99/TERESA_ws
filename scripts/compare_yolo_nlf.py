#!/usr/bin/env python3
"""
Confronto YOLO vs NLF — velocità e output visivo.
Uso:
  python3 scripts/compare_yolo_nlf.py [immagine.jpg]
  python3 scripts/compare_yolo_nlf.py frame.jpg
"""

import time, sys, os
import cv2
import numpy as np

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(WORKSPACE)

# ── 1. Carica immagine ─────────────────────────────────────────
if len(sys.argv) > 1:
    path = sys.argv[1]
    print(f"📷 Carico: {path}")
    frame = cv2.imread(path)
    if frame is None:
        print(f"❌ Impossibile leggere {path}")
        sys.exit(1)
else:
    print("📷 Catturo frame dalla webcam...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"   Trovata camera su /dev/video{i}")
                break
        else:
            print("❌ Nessuna camera trovata. Specifica un file: python3 scripts/compare_yolo_nlf.py immagine.jpg")
            sys.exit(1)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("❌ Impossibile leggere il frame.")
        sys.exit(1)

frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
h, w = frame_rgb.shape[:2]
print(f"   Frame: {w}×{h}")

# ── 2. YOLO inference ──────────────────────────────────────────
print("\n🟢 YOLO11n-pose...")
try:
    from ultralytics import YOLO
    t0 = time.monotonic()
    yolo = YOLO("yolo11n-pose.pt")
    results = yolo(frame_rgb, conf=0.25, verbose=False)
    dt_yolo = time.monotonic() - t0

    n_people = len(results[0].keypoints) if results[0].keypoints is not None else 0
    print(f"   Tempo: {dt_yolo:.2f}s | Persone: {n_people} | ~{1/dt_yolo:.1f} FPS")

    # Salva immagine YOLO
    yolo_img = results[0].plot()
    cv2.imwrite("/tmp/yolo_output.jpg", yolo_img)
    print(f"   Immagine: /tmp/yolo_output.jpg")
except ImportError:
    print("   ⚠️ ultralytics non installato. pip install ultralytics")

# ── 3. NLF inference ───────────────────────────────────────────
print("\n🔵 NLF (Neural Localizer Fields)...")
try:
    import torch
    import torchvision

    t0 = time.monotonic()
    model = torch.jit.load("nlf_s_multi.torchscript", map_location="cpu").eval()
    dt_load = time.monotonic() - t0
    print(f"   Caricamento: {dt_load:.1f}s")

    tensor = torch.from_numpy(frame_rgb).permute(2, 0, 1).unsqueeze(0)

    t0 = time.monotonic()
    with torch.inference_mode():
        pred = model.detect_smpl_batched(
            tensor, default_fov_degrees=55.0, num_aug=1,
            detector_threshold=0.3, internal_batch_size=64,
            suppress_implausible_poses=True,
        )
    dt_nlf = time.monotonic() - t0

    n_people = len(pred["joints3d"][0]) if pred.get("joints3d") else 0
    print(f"   Tempo: {dt_nlf:.1f}s | Persone: {n_people} | ~{1/dt_nlf:.2f} FPS")

    # Disegna keypoint NLF sull'immagine
    nlf_img = frame.copy()
    if n_people > 0:
        joints = pred["joints3d"][0][0].cpu().numpy()  # (24, 3) mm
        joints2d = pred["joints2d"][0][0].cpu().numpy() if "joints2d" in pred else None

        if joints2d is not None:
            for j in range(len(joints2d)):
                x, y = int(joints2d[j][0]), int(joints2d[j][1])
                if 0 <= x < w and 0 <= y < h:
                    cv2.circle(nlf_img, (x, y), 4, (255, 100, 0), -1)
                    cv2.putText(nlf_img, str(j), (x+5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 100, 0), 1)

    cv2.imwrite("/tmp/nlf_output.jpg", nlf_img)
    print(f"   Immagine: /tmp/nlf_output.jpg")
except Exception as e:
    print(f"   ❌ {e}")

# ── 4. Confronto ────────────────────────────────────────────────
print("\n" + "="*50)
print("📊 CONFRONTO")
print("="*50)
try:
    print(f"   YOLO: {dt_yolo:.2f}s ({1/dt_yolo:.1f} FPS)")
    print(f"   NLF:  {dt_nlf:.1f}s ({1/dt_nlf:.2f} FPS)")
    print(f"   NLF è {dt_nlf/dt_yolo:.0f}× più lento di YOLO")
except:
    pass
print(f"\n   Immagini salvate in /tmp/")
print(f"   YOLO: /tmp/yolo_output.jpg")
print(f"   NLF:  /tmp/nlf_output.jpg")
