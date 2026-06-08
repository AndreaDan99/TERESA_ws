# Wound/Injury Detection & Pose — Comparison & Jetson Orin Deployment Recommendations

Benchmarks measured on an **RTX 5070 Ti (Blackwell sm_120, 16 GB)** over the 80-frame (2 fps) test set
of the mannequin clip. The deployment target is an **NVIDIA Jetson Orin** where **≤1 FPS is acceptable**,
so *every* method below already clears the throughput bar on the desktop GPU — the real questions are
**detection quality on moulage wounds**, **VRAM/footprint**, and **whether it ports cleanly to Orin/TensorRT**.

> No wound training data exists, so this is a **zero-shot / open-vocabulary** study. "Quality" is qualitative
> (does it fire on the real wounds: forearm laceration, abdominal incision, thigh laceration?) plus the
> measured detection-rate; a small hand-annotated precision/recall check is in `data/eval/` (approximate).

## 1. Headline recommendation

**Best wound pipeline for the robot (accuracy-first, ≤1 FPS):**
```
RGB frame
  └─ Grounding DINO  (or OWLv2)   text="wound. blood. laceration. cut. injury."   → wound boxes
       └─ SAM 2.1  (or MedSAM)    box-prompt                                       → wound mask
            └─ fuse mask centroid with ROS RGB-D depth + camera_info               → 3D wound point → arm target
```
- **Grounding DINO** = highest recall (also catches the abdominal incision + stoma); **OWLv2** = cleanest/precise
  with far less noise and 2× the speed — use OWLv2 when false positives are costly, GDINO when misses are costly.
  *Measured on the annotated set (§2.5): GDINO & Florence-2 hit **11/11** wounds; OWLv2 only **6/11** (misses the
  thigh + most of the incision). For a triage robot where missing a wound is the worse error, lead with GDINO.*
- **SAM 2.1 / MedSAM** turn the box into a tight mask for area/centroid → the arm needs the mask, not just a box.
- **CLIPSeg** is the best *single-model* text→mask alternative if you want to skip the two-stage setup.

**Fast/edge-first alternative:** **YOLO-World / YOLOE** run at 100–150 FPS and export cleanly to TensorRT, but
their open-vocab recall on "wound" is low (~0.25) — only viable **after fine-tuning** on a small wound set
(use the zero-shot methods above to auto-label).

**Classical Lab-redness CV** (308 FPS, 0 GPU) is a free always-on pre-filter/fallback that catches obvious
bleeding, but is fooled by the orange shorts edge.

## 2. Measured benchmark (desktop RTX 5070 Ti)

| # | method | type | FPS | VRAM MB | det.rate | finds the wound? | edge/TensorRT |
|---|--------|------|----:|--------:|---------:|------------------|---------------|
| 11 | **OWLv2** | open-vocab box | 12.9 | 809 | 0.93 | ✅ clean (forearm + incision) | partial (HF; ONNX possible) |
| 10 | **Grounding DINO** | open-vocab box | 7.2 | 2267 | 0.98 | ✅ best recall, noisier | partial (heavy; ONNX export non-trivial) |
| 20 | **Grounded-SAM2** | wound mask | 7.2 | 1609 | 0.98 | ✅ best masks | partial (SAM2 → TRT possible) |
| 21 | **MedSAM** | wound mask | 12.1 | 1993 | 0.98 | ✅ medical-domain masks | partial (ViT-B → TRT) |
| 22 | **CLIPSeg** | text→mask | 10.9 | 854 | 0.86 | ✅ clean semantic mask | partial |
| 15 | Florence-2 | phrase grounding | 9.4 | 1803 | 1.0 | ◻ broad boxes | hard (VLM, generative) |
| 00 | Classical CV | color box+mask | 308 | 0 (CPU) | 0.95 | ◧ red wounds only, FP on shorts | yes (CPU/NEON) |
| 13 | YOLO-World v2 | open-vocab box | 99 | 1013 | 0.25 | ◧ low recall | **yes** (TRT/ONNX) |
| 14 | YOLOE-11L-seg | open-vocab seg | 153 | 286 | 0.24 | ◧ low recall | **yes** (TRT/ONNX) |
| 23 | FastSAM+redness | mask | 35 | 625 | 0.13 | ◧ high-precision/low-recall | yes |
| 25 | SAM-auto→CLIP | mask | 0.65 | 4489 | 0.28 | ◧ slow, moderate | no (heavy) |
| 24 | MobileSAM (everything) | mask | 0.9 | 3875 | 0.20 | ◧ slow as-run | partial (box-prompt is fast) |
| 12 | OWL-ViT (base) | open-vocab box | 79 | 636 | 0.01 | ✗ basically misses | yes (but useless here) |
| 26 | SAM 3 concept | concept mask | — | — | — | ⏭ gated (HF token) | n/a |
| 30 | Deepskin (supervised) | wound seg | *cpu* | — | *see explore.md* | domain-gap test | CPU-only (TF) |

(Full machine-readable table: `results/_summary/metrics_table.csv`. Stretch methods 31/32/40/41 appended after their runs.)

## 2.5 Approximate wound-recall eval (small hand-annotated set)
6 frames, 11 clearly-visible wound instances (`data/eval/annotations.json`). **Lenient** match (IoU>0.1 OR
center-inside) because GT boxes are approximate and methods emit loose boxes. Indicative recall ranking, **not** AP.

| method | recall | dets/eval-frame | per-type (forearm·incision·thigh·upperarm) |
|---|---:|---:|---|
| **Grounding DINO** | **100%** (11/11) | 6.3 | 4/4 · 4/4 · 2/2 · 1/1 — catches everything, noisiest |
| **Florence-2** | **100%** (11/11) | 2.3 | 4/4 · 4/4 · 2/2 · 1/1 — every wound, ⅓ the boxes (coarse/large) |
| Grounded-SAM2 / MedSAM / wound-3D | **100%** | 6.3 | inherit GDINO boxes |
| **OWLv2** | 55% (6/11) | 2.2 | 4/4 · 1/4 · 0/2 · 1/1 — precise, misses thigh + most incision |
| YOLO-World | 36% | 0.3 | 0/4 · 1/4 · 2/2 · 1/1 |
| CLIPSeg | 27% | 1.0 | 2/4 · 0/4 · 1/2 · 0/1 |
| Deepskin (supervised) | 27% | 2.5 | 0/4 · 2/4 · 1/2 · 0/1 — misses lacerations (domain gap) |
| Classical CV | 18% | 6.2 | 2/4 · 0/4 · 0/2 · 0/1 — forearm reds only |
| OWL-ViT / YOLOE / FastSAM / MobileSAM / SAM-auto+CLIP | 0% | ~0 | don't localize the annotated wounds |

**Read:** to *not miss wounds*, **Grounding DINO** (recall) → SAM2/MedSAM is the pick; **Florence-2** matches its
recall with far fewer boxes but coarser localization; **OWLv2** is the high-precision option if missing the
harder thigh/incision wounds is acceptable. (Script: `scripts/eval_recall.py`.)

## 3. Findings by family
- **Open-vocab detectors (zero-shot):** the workhorses here. GDINO/OWLv2 genuinely localize the moulage
  lacerations and the sutured incision with no training. GDINO trades precision for recall; OWLv2 is the
  cleaner, lighter, faster pick. The YOLO-family open-vocab models (World/E) are *much* faster and
  TensorRT-friendly but under-fire on "wound" (text embeddings biased to common objects).
- **Promptable segmentation:** SAM 2.1 and MedSAM both produce excellent wound masks *given a box* — they
  don't find wounds themselves, so they must follow a detector. MedSAM (medical fine-tune) and SAM 2.1
  perform comparably on these high-contrast moulage cuts; SAM 2.1 also brings video tracking for free.
- **Text→mask single-stage:** CLIPSeg is the standout — one model, clean wound heatmaps, modest VRAM.
- **Class-agnostic + color gate:** FastSAM/MobileSAM + a Lab-redness filter give high precision but low
  recall and (for MobileSAM "segment-everything") poor speed — prefer box-prompted SAM for the robot.
- **Supervised wound models (Deepskin/FUSegNet):** under-fire as expected — Deepskin (real-wound U-Net) misses
  the moulage lacerations entirely (fires on skin/shorts tone), confirming the domain gap. Useful only as
  crop-level refiners on real tissue.
- **Burn classifier on wounds (Skin-Burn YOLOv7):** silent on wide shots, but in close-up it *localizes* the
  moulage laceration tightly (82 FPS) while labeling it a high-confidence **"3rd-degree burn" (0.90)** — i.e. a
  burn detector can double as a close-up wound *localizer*, but its class labels don't transfer to lacerations.
- **Classical CV:** not obsolete — free, deterministic, runs on the CPU, and a good sanity pre-filter.

## 4. Jetson Orin deployment recommendations

**Throughput context.** We need ≤1 FPS. Rough FP16-dense capability vs the RTX 5070 Ti (~44 FP16 TFLOPS):
- **Orin AGX 64 GB** (~5.3 FP16 TFLOPS dense, 275 INT8 TOPS sparse): ≈ 8–12× slower than the 5070 Ti dense,
  but huge 64 GB unified memory. → runs the **full GDINO→SAM2 pipeline** at well above 1 FPS after TensorRT FP16.
- **Orin NX 16 GB** (~2.5 FP16 TFLOPS): ≈ 15–20× slower → the heavy GDINO/SAM stack lands around 0.5–1.5 FPS
  with TensorRT FP16 — **fine for our ≤1 FPS bar**; YOLO-World/E run real-time.
- **Orin Nano 8 GB** (~1.3 FP16 TFLOPS, no DLA on some SKUs): tight. Favor **OWLv2/CLIPSeg or a fine-tuned
  YOLOE** + box-prompted **MobileSAM/EfficientSAM**; the full GDINO→SAM-large stack will be slow but may still
  clear 1 frame/2–3 s.

**Concrete recommendation by SKU:**

| Orin SKU | wound detection | segmentation | pose | notes |
|----------|-----------------|--------------|------|-------|
| **AGX 64 GB** | Grounding DINO **or** OWLv2 (TRT FP16) | SAM 2.1 / MedSAM (TRT FP16) | ViTPose or MediaPipe | best accuracy; comfortably ≤1 FPS; 64 GB fits everything |
| **NX 16 GB** | OWLv2 (TRT FP16) | MedSAM (ViT-B, TRT) or MobileSAM box-prompt | MediaPipe / YOLO-pose | balanced; quantize SAM to FP16 |
| **Nano 8 GB** | fine-tuned **YOLOE/YOLO-World** (TRT INT8) or OWLv2 | EfficientSAM / MobileSAM (box-prompt) | MediaPipe (CPU/GPU) or YOLO-pose-n | lightest; lean on INT8 + small SAM |

**Export / optimization path (what to actually do on the Orin):**
1. **Flash JetPack 6.x** (CUDA 12.x, TensorRT 10.x) — matches our cu128 model lineage.
2. **YOLO-World / YOLOE / YOLO-pose:** `yolo export format=engine half=True` (Ultralytics → TensorRT directly);
   add `int8=True data=<calib>` on Nano. These are the cleanest exports.
3. **OWLv2 / CLIPSeg / Grounding DINO / SAM:** export the vision/text encoders to **ONNX**
   (`torch.onnx.export` / `optimum`), then `trtexec --onnx=... --fp16 --saveEngine=...`. SAM's image encoder
   (ViT) is the cost center — convert it to a TensorRT FP16 engine; the prompt decoder is cheap.
4. **MedSAM / SAM 2.1:** export the ViT image encoder to TRT FP16; reuse the lightweight mask decoder in PyTorch.
5. Use **`torch2trt`** or **`nvidia-modelopt`** for INT8 PTQ where accuracy allows (calibrate on a few dozen frames).
6. Run perception **once per second** triggered by the planner; cache the last mask + 3D point; let depth/ROS
   provide the metric scale (see §5).
7. Optional: **DeepStream / Triton** if you later batch multiple cameras.

**Memory budget:** on a 16 GB NX, the heaviest single model here (SAM-large image encoder, ~2–4 GB FP16) fits
with the detector loaded; avoid running SAM "segment-everything" (3.9 GB + slow) — always box-prompt.

## 5. Using the depth camera (3D wound localization for the arm)
The robot already has RGB-D from ROS. The wound **mask** (from SAM/MedSAM) projects to a 3D point with the
real `camera_info` intrinsics: take the mask centroid `(u,v)`, read the registered depth `Z`, then
`X=(u-cx)Z/fx, Y=(v-cy)Z/fy`. This is exact with the real depth — the monocular `41_wound_3d` demo here only
*approximates* scale (Depth-Anything V2 + assumed intrinsics) to illustrate the pipeline; **use the ROS depth
in deployment**. The mask also gives wound **area**/orientation for grasp/approach planning, and the bed-corner
**ArUco** markers give a stable world frame + a bed-plane to suppress mattress false positives and to resolve
posture (lying) unambiguously.

## 6. Pose & posture
Off-the-shelf pose models track the supine mannequin on full-body frames. For Orin: **YOLO11-pose** (Ultralytics
→ TensorRT, fast) or **MediaPipe BlazePose** (best coverage, light, has a built-in GPU delegate). The
**posture classifier** (articulation + body-orientation geometry) robustly returns **lying**; in deployment,
rectify the 3D pose with Spot's IMU gravity / the ArUco bed-plane so lying-vs-standing is exact rather than
appearance-based. ViTPose gives the best 2D accuracy but is top-down (needs a person detector). NLF adds a 3D
SMPL mesh (useful for reach planning) at ~2.5 FPS.

## 7. Limitations & next steps
- **No moulage-trained model exists** → zero-shot is load-bearing; the biggest accuracy win would be to
  **auto-label** frames with GDINO/OWLv2+SAM and **fine-tune a small YOLOE/YOLO-seg** (real-time + accurate on Orin).
- Open-vocab detectors emit duplicate/loose boxes → add class-agnostic **NMS** + the **ArUco body-ROI** + a
  **depth-plane filter** to cut mattress/shorts false positives.
- Quantitative precision/recall here is from a **small hand-annotated set** (`data/eval/`) — expand it for
  statistically firm numbers.
- Validate the **TensorRT FP16/INT8** engines on the actual Orin (accuracy drop + real FPS) before field use.
