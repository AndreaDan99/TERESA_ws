# Wound / Injury Detection — Exploration Log

Living log of every algorithm explored for **wound/injury detection** and **pose/posture**
on the Spot + arm + Jetson Orin medical-triage scenario. Companion docs:
[`troubleshooting.md`](troubleshooting.md) · [`docs/research.md`](docs/research.md) (literature survey) ·
[`docs/comparison.md`](docs/comparison.md) (benchmarks + Jetson recommendations) · `report.html` (gallery).

## Scenario
A Boston Dynamics **Spot** robot with a mounted arm and an **NVIDIA Jetson Orin** must
autonomously detect wounds/injuries on a human-like medical mannequin so the arm can assist.
Input is RGB from a depth (RGB-D) camera; depth comes separately from ROS. Inference may run
slowly (≤1 FPS). **No wound training data** → zero-shot / open-vocabulary / pretrained methods
are the priority, with the **widest practical coverage** of open-source SOTA.

### Test clip & injuries
`MicrosoftTeams-video.mp4` — 960×540, 30 fps, 40.2 s. Mannequin ("Bob") supine on a purple
mattress, orange shorts, camera panning from above. Injuries present (qualitative ground truth):

| id | description | clearest frame |
|----|-------------|----------------|
| forearm_laceration | left forearm, red moulage laceration (most distinct) | `frame_00026` |
| thigh_laceration | upper-thigh moulage laceration | `frame_00046` |
| abdominal_incision | long sutured surgical incision across abdomen | `frame_00026` |
| stoma | colostomy/stoma site (medical device, *not* trauma) | `frame_00026` |

### Dataset
- `data/frames/` — 80 frames @2 fps (`frame_%05d.jpg`, 960×540) — method test set.
- `data/frames_5fps/` — 201 frames @5 fps — for smooth grid videos.
- Hero frame for cross-method montage: `frame_00026.jpg`.

## Hardware / environments
- **GPU:** RTX 5070 Ti (Blackwell **sm_120**, 16 GB) → PyTorch **cu128** (torch 2.11.0+cu128, cap (12,0) ✅).
- Envs: **`wound`** (all wound detect/seg except Florence), **`obj_florence`** (Florence-2, transformers 4.49),
  **`pose_basic`** (all pose), **`obj_depth`** (UniDepth), **`wound_tf`** (Deepskin/TF).
- Reused harness + weights from sibling `~/PROJECTS/Object_detection` and `~/PROJECTS/Body_prediction`.

---

## Wound / injury detection methods — RESULTS
Status: ✅ works · ⚠️ runs but weak/limited · ❌ failed/blocked · 🟡 in progress
FPS & VRAM measured on the RTX 5070 Ti over 80 frames (≤1 FPS is the deployment bar, so all are fast enough).

| # | method | env | type | status | FPS | VRAM MB | det.rate | wound quality (qualitative) |
|---|--------|-----|------|--------|-----|---------|----------|------------------------------|
| 00 | Classical CV (Lab redness) | wound | box+mask | ✅ | 308 | 0 (CPU) | 0.95 | finds forearm laceration + stoma; FP on shorts edge/skin creases; cheap baseline |
| 10 | Grounding DINO | wound | box | ✅ | 7.2 | 2267 | 0.975 | **finds forearm wound + abdominal incision + stoma**; noisy (spurious "skin lesion"/"incision" on large skin/bed) |
| 11 | **OWLv2** | wound | box | ✅ | 12.9 | 809 | 0.925 | **cleanest** — precise boxes on forearm laceration + abdominal incision, little noise |
| 12 | OWL-ViT (base) | wound | box | ⚠️ | 78.7 | 636 | 0.013 | almost never fires on wounds — base model too weak for this concept |
| 13 | YOLO-World v2 | wound | box | ⚠️ | 98.6 | 1013 | 0.25 | occasional hits; very fast (edge-friendly) but low recall (common-object bias) |
| 14 | YOLOE-11L-seg | wound | seg | ⚠️ | 153 | 286 | 0.237 | fastest + masks, but low recall on wounds |
| 15 | Florence-2 | obj_florence | grounding | ✅ | 9.4 | 1803 | 1.0 | phrase-grounding fires every frame; broad boxes, less wound-specific |
| 20 | **Grounded-SAM2** (GDINO→SAM2.1) | wound | mask | ✅ | 7.2 | 1609 | 0.975 | **best masks** — clean wound masks from GDINO boxes (ideal for arm targeting) |
| 21 | **MedSAM** (GDINO→MedSAM) | wound | mask | ✅ | 12.1 | 1993 | 0.975 | medical-domain masks from GDINO boxes; tight on lacerations |
| 22 | **CLIPSeg** | wound | semantic mask | ✅ | 10.9 | 854 | 0.863 | clean text-prompted wound heatmap/mask (mean conf 0.41) |
| 23 | FastSAM + redness | wound | mask | ⚠️ | 35 | 625 | 0.125 | high precision / low recall (only fires on strongly red regions) |
| 24 | MobileSAM + redness | wound | mask | ⚠️ | 0.9 | 3875 | 0.20 | **slow** (segment-everything mode); heavy VRAM; not edge-ideal as-run |
| 25 | SAM-auto → CLIP | wound | mask | ⚠️ | 0.65 | 4489 | 0.275 | works after CLIP-API fix; **slow** (SAM-everything + CLIP); moderate |
| 26 | SAM 3 concept | wound | mask | ❌ | | | | **gated** — needs HF_TOKEN + license acceptance (skipped gracefully) |
| 30 | Deepskin (real wound seg) | wound_tf | seg | ⚠️ | 0.6 (CPU) | — | 0.85* | **misses the moulage lacerations** — fires on skin/shorts tone, not the real cuts → confirms domain gap (*mostly FPs) |
| 40 | Depth Anything V2 | wound | metric depth | ✅ | — | — | — | per-frame metric depth map (for the 3D lift) |
| 41 | Wound box + depth → 3D | wound | 3D point | ✅ | — | — | — | annotates each GDINO wound box with Z (~1.25 m forearm); demo of mask→3D for arm targeting |
| 31 | Le0Dev YOLO-SAM-wound repo | wound | box+mask | ❌ | | | | repo cloned, but its trained wound-YOLO weights are **not published** (`best.pt` unavailable) → skipped |
| 32 | Michael-OvO Skin-Burn repo | wound | cls/box | ✅ | 82 | 1531 | 0.125 | YOLOv7 burn model; **tightly boxes the thigh laceration but calls it "3rd degree burn" @0.90** — wound↔burn confusion; silent on wide shots |

### Takeaways (wound detection)
- **Zero-shot open-vocabulary works** on the moulage lacerations despite no training data.
  **OWLv2** gives the cleanest precise boxes; **Grounding DINO** has the highest recall (catches the
  incision + stoma too) at the cost of noise; the two pair naturally with **SAM2/MedSAM** for masks.
  *Measured recall on the small annotated set (`scripts/eval_recall.py`): GDINO & Florence-2 = 11/11,
  OWLv2 = 6/11 (misses thigh + most of the incision). For triage (don't miss wounds) → lead with GDINO.*
- **YOLO-World / YOLOE / OWL-ViT** are fast/edge-friendly but have low recall on "wound" (their
  text-embeddings are tuned for common objects) — would benefit from fine-tuning or visual prompts.
- **CLIPSeg** is the best single-stage text→mask option; **Grounded-SAM2 / MedSAM** the best two-stage masks.
- **Classical redness CV** is essentially free and catches the obvious red wounds — a useful pre-filter /
  fallback — but is fooled by the orange shorts edge.
- **Best pipeline for the robot:** GDINO (or OWLv2) box → SAM2/MedSAM mask → fuse with ROS depth for the
  3D wound point. See [`docs/comparison.md`](docs/comparison.md).

## Pose / posture methods — RESULTS
| method | env | output | status | FPS | det.rate | notes |
|--------|-----|--------|--------|-----|----------|-------|
| YOLO11x-pose | pose_basic | COCO-17 2D | ✅ | 41 | 0.29 | clean full-body lying skeleton on overview frames |
| ViTPose++ | pose_basic | COCO-17 2D | ✅ | 25 | 0.425 | best 2D detection rate; top-down (needs person box) |
| MediaPipe BlazePose | pose_basic | 33 kpts 2D+3D | ✅ | 19 | 0.85 | highest detection; 3D world landmarks |
| NLF (SMPL) | pose_basic | 24 joints 3D+mesh | ✅ | 2.6 | 0.625 | 3D SMPL mesh (camera mm); slow but richest |
| RTMPose | pose_basic | COCO-17 2D | ❌ | | | weight download from openmmlab **timed out** (network blocked) |
| **Posture classifier** | pose_basic | lying/sitting/standing | ✅ | — | — | **all backends agree: LYING** (NLF 100%, YOLO 91%, ViTPose 87%, MediaPipe 69%) |

### Takeaways (pose/posture)
- Off-the-shelf pose models track the **lying mannequin** well on full-body frames; detection drops on
  zoomed body-part close-ups (no full body). **MediaPipe** has the best coverage, **NLF** the cleanest 3D.
- The **posture classifier** (articulation + body-orientation geometry) robustly reports **LYING**; minor
  "sitting" confusion comes from close-up frames where a bent arm skews the articulation angle.
- Posture (lying vs standing) is ultimately a gravity-relative question → in deployment, rectify the 3D
  pose with Spot's IMU gravity / the ArUco bed-plane for an unambiguous answer.

---

## Findings log (running)
- 2026-06-08: Completed the full study — **17 wound methods executed** + SAM3 (gated/skipped) + YOLO-SAM-wound
  (weights unavailable), **4 pose methods + posture**, depth-3D lift, aggregation/grids/HTML, and an approximate
  recall eval on a hand-annotated set.
- **Best wound pipeline:** Grounding DINO (recall 11/11) → SAM 2.1 / MedSAM (masks) → + ROS depth → 3D point.
  Florence-2 matches GDINO's recall with ⅓ the boxes; OWLv2 is precise but 6/11 (misses thigh + most incision).
- **Domain gap confirmed twice:** Deepskin (real-wound U-Net) misses the moulage lacerations; the Skin-Burn
  YOLOv7 *localizes* a laceration tightly but mislabels it a high-confidence "3rd-degree burn".
- **Posture = lying**, agreed across all 4 pose backends.
- **Skipped/blocked:** SAM3 (gated, HF token), RTMPose (openmmlab download timeout), Le0Dev YOLO-SAM-wound
  (private weights — a faithful auto-loading repro is at `scripts/run_yolo_sam_wound.py`).
