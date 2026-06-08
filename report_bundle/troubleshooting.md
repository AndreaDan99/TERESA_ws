# Troubleshooting Log — wound_det

Bugs, gotchas, and resolutions. Seeded with lessons inherited from the sibling projects
(`~/PROJECTS/Object_detection`, `~/PROJECTS/Body_prediction`) and updated live as issues arise.
Newest entries at the bottom of each section.

## Platform / GPU (RTX 5070 Ti, Blackwell sm_120)
- **Torch must be cu128.** `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128`
  installs `torch 2.11.0+cu128`. Verify with `torch.cuda.get_device_capability() == (12, 0)`.
  Wrong wheel ⇒ `CUDA error: no kernel image available for execution on the device`. ✅ confirmed working.
- **Install torch FIRST**, before `ultralytics`/anything that might drag a cu126 wheel.
- **Do NOT install `xformers` or `flash-attn`** — their wheels lag sm_120 and force-downgrade torch.
  PyTorch SDPA already dispatches fused attention kernels on sm_120, so they're unnecessary.
- **GPU is a 16 GB singleton** → run inference jobs **sequentially**; check `nvidia-smi`. SAM/SAM2 peak ~3–4 GB.

## onnxruntime CUDA ExecutionProvider (affects RTMPose / ONNX benchmarks)
- `onnxruntime-gpu 1.26` in the `wound` and `pose_basic` envs reports providers
  `['AzureExecutionProvider', 'CPUExecutionProvider']` — **CUDAExecutionProvider missing**.
  Cause: ORT can't find CUDA/cuDNN libs (torch ships them as pip `nvidia-*-cu12` wheels in a
  non-standard path). **Workaround:** RTMPose runs fine on the **CPU EP** at our ≤1 FPS target.
  To try GPU later: `export LD_LIBRARY_PATH=$(python -c "import os,nvidia;print(os.path.dirname(nvidia.__file__))")/...`
  pointing at the torch-bundled cudnn/cublas dirs, or `pip install onnxruntime-gpu` matching system CUDA.

## Environment / version pins
- **Florence-2 breaks on transformers 5.x** (`AttributeError: forced_bos_token_id` / `_supports_sdpa`).
  Use the isolated `obj_florence` env (`transformers==4.49.0`) and patch missing config attrs +
  strip `flash_attn` from `get_imports` (see `ref_run_florence.py`).
- Everything else uses `transformers 5.10.2` (env `wound` / `obj_hf`).

## Open-vocabulary detectors
- **Grounding DINO:** use the **HF port** `IDEA-Research/grounding-dino-base` (the official repo compiles
  `MultiScaleDeformableAttention` CUDA op → fails without nvcc on Blackwell). Text must be **lowercase,
  `". "`-joined, ending with `.`** e.g. `"wound. blood. laceration."`.
- **OWLv2:** emits many overlapping duplicate boxes → apply class-agnostic NMS (`torchvision.ops.nms`,
  iou=0.3). Default threshold 0.5 too high for unusual targets; use ~0.1–0.22.

## Promptable segmentation
- **SAM 2.1 via transformers** (`Sam2Model`/`Sam2Processor`) needs **no native build** — works on sm_120.
  Box-prompt returns `pred_masks [B, n_boxes, 3, H, W]`; pick `argmax(iou_scores)` of the 3 multimask outputs.
- **SAM2 video multi-object:** add ALL objects in ONE batched `add_inputs_to_inference_session` call
  (`obj_ids=list(range(N)), input_boxes=[all_boxes]`); separate calls raise
  `maskmem_features cannot be empty...`. Offload to CPU (`inference_state_device="cpu"`) for long clips.
- **SAM 3** (`facebook/sam3`) is **gated** → needs `HF_TOKEN` + license acceptance. Concept API:
  `proc.add_text_prompt(session, "wound")`; high VRAM, call `torch.cuda.empty_cache()` periodically.
- **CLIPSeg** outputs 352×352 logits → resize to frame, `sigmoid()`, threshold ~0.35. Semantic (not instance).

## Pose
- **ViTPose** is top-down → needs person boxes first (run YOLO11x detector). `dataset_index=0` → COCO-17.
  Normalize heatmap peaks via `proc.post_process_pose_estimation(out, boxes=[box_xywh])`.
- **MediaPipe 0.10.35+** removed `mp.solutions.pose`; use the **Tasks API** with
  `pose_landmarker_heavy.task`. `pose_world_landmarks` are in **meters**.
- **NLF:** `import torchvision` BEFORE `torch.jit.load`. `detect_smpl_batched` may omit the `boxes` key —
  don't depend on it; pick subject by largest 2D joint span. `joints3d` in **mm**.
- **Metrics bug:** `np.linalg.norm(a, -1)` treats `-1` as the norm *order*, not axis →
  always pass `axis=-1`.

## Domain-gap caveats for THIS scene (from research survey)
- Supervised wound/burn/skin-lesion models were trained on **real human tissue**, photographed in
  close-up → expect under-firing / mislabeling on **moulage on plastic**. Use them as crop-level refiners.
- **Colored marker dots** on Bob may be detected as wounds (false positives) → filter by size/shape.
- The **orange shorts** are the dominant red/orange distractor for color-based methods → discriminate
  blood-red from orange in Lab space (`a*` high but `b*` not high).
- Use **depth to suppress the flat mattress plane** and **ArUco corners to crop to a body/bed ROI**.

## Live issues (this project)
- **SAM-auto+CLIP (`run_sam_auto_clip.py`) — `'BaseModelOutputWithPooling' object has no attribute 'norm'`.**
  In transformers 5.x, `CLIPModel.get_text_features` / `get_image_features` returned a `ModelOutput`,
  not a tensor, so `.norm()` failed. **Fix:** bypass with the explicit projected path —
  `clip.text_projection(clip.text_model(**t).pooler_output)` and
  `clip.visual_projection(clip.vision_model(**i).pooler_output)` (both land in the 512-d CLIP space).
- **RTMPose — weight download timed out.** `rtmlib` fetches ONNX weights from `download.openmmlab.com`,
  which is unreachable here (`URLError: [Errno 110] Connection timed out`). RTMPose left unavailable;
  use YOLO-pose / ViTPose / MediaPipe / NLF (all working). To enable RTMPose: pre-stage its ONNX files offline.
- **OWL-ViT base / YOLO-World / YOLOE: low recall on "wound".** Not a bug — their text embeddings are tuned
  for common objects, so "wound/laceration" rarely fires (OWL-ViT det-rate 0.013). Prefer Grounding DINO / OWLv2;
  these fast open-vocab models would need fine-tuning or *visual* prompts to detect wounds well.
- **`pip install deepskin` → "No matching distribution".** Deepskin is GitHub-only:
  `pip install git+https://github.com/Nico-Curti/Deepskin.git` (TensorFlow; runs CPU here since TF lacks sm_120).
- **SAM 3 gated.** `facebook/sam3` needs `HF_TOKEN` + license acceptance → script writes `status:"skipped"` and exits 0.
- **MobileSAM segment-everything is slow/heavy** (0.9 FPS, ~3.9 GB) via ultralytics. For edge, use *box-prompted*
  SAM (MedSAM/SAM2) or FastSAM rather than full automatic-mask generation.
- **`ffmpeg` not on PATH inside the conda envs.** `conda run -n wound` shadows the base PATH, so scripts calling
  `"ffmpeg"` crash with `FileNotFoundError`. Fix: resolve via `shutil.which("ffmpeg")` then fall back to
  `/home/jamie/miniconda3/bin/ffmpeg`. (Applied in `scripts/make_videos.py`, `results/make_pose_grid.py`.)
- **Deepskin colormap.** `wound_segmentation` returns an HxWx3 uint8 semantic colormap (wound/body/background),
  not a probability map — read the wound class from the red-dominant channel. On our moulage it mostly fires on
  skin/shorts tone, **not** the real lacerations (domain gap), so its detection-rate is largely false positives.
