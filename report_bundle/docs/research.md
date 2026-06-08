# Wound/Injury Detection & Pose — Literature & Methods Survey

Executive summary: This survey supports a Boston Dynamics Spot + manipulator-arm robot running an NVIDIA Jetson Orin with an RGB-D camera, tasked with autonomously detecting moulage laceration wounds (simulated bleeding cuts on forearm/upper-arm/thigh) and classifying posture (lying/sitting/standing) on a supine CPR mannequin ("Bob") on a hospital mattress. Because there is **no wound-specific training data** and inference at ≤1 FPS is acceptable, the dominant strategy is a **zero-shot, two-stage open-vocabulary pipeline** — open-vocab detector (text → box) → promptable segmenter (box → mask) → RGB-D depth fusion for 3D wound localization/measurement — with clinical wound segmenters (FUSegNet, Deepskin) used only as crop-level refiners and a geometric posture classifier driven by an off-the-shelf pose model. We are honest throughout about the **moulage/plastic-skin domain gap**: every supervised wound, burn, and skin-lesion model below was trained on real human tissue and will likely under-fire or mislabel Bob's simulated cuts, so promptable/open-vocab methods plus depth and ArUco-region constraints are the load-bearing components.

Locally, several candidate weights are already on disk under `/home/jamie/PROJECTS/andrea/wound_det/models/` (symlinked): `yolo11x.pt`, `yolo11x-seg.pt`, `yolo11x-pose.pt`, `yolo12x.pt`, `yoloe-11l-seg.pt`, `yolov8x-worldv2.pt`, `rtdetr-x.pt`, `FastSAM-x.pt`, `mobile_sam.pt`, plus pose assets `pose_landmarker_heavy.task` (MediaPipe BlazePose) and `nlf_l_multi_0.3.2.torchscript` (NLF 3D pose). Probe frames confirming the scene (high-contrast dark-red gash on tan plastic forearm, orange shorts, ArUco at bed corners) are in `/home/jamie/PROJECTS/andrea/wound_det/_probe/` (`crop_arm_13s.png`, `crop_thigh_23s.png`, `contact_sheet.png`).

---

## 1. Wound detection & segmentation SOTA (supervised, clinical-domain)

There is **no public wound model trained on moulage lacerations on a mannequin**. The best supervised wound segmenters are trained almost exclusively on *clinical close-ups of diabetic-foot ulcers (DFU) / chronic wounds / pressure ulcers* photographed at ~20–40 cm — a centered, cropped ulcer, not a laceration seen from a Spot arm camera ~1 m away. So they are best used as a **refine/verify stage on cropped limbs**, not as scene-scale detectors. Treat all reported Dice/mIoU below as the authors' numbers on their *own* test distributions; none are measured on moulage and they will not transfer directly. A 2025 meta-analysis ([Visual Computer, doi 10.1007/s00371-025-04133-y](https://link.springer.com/article/10.1007/s00371-025-04133-y)) warns these scores correlate with *small test sets* and don't generalize.

Strongest public, code+weights options for a refiner stage:

- **FUSegNet / x-FUSegNet** ([github.com/mrinal054/FUSegNet](https://github.com/mrinal054/FUSegNet), MIT, PyTorch via `segmentation_models.pytorch`). EfficientNet-b7 encoder + parallel scSE (P-scSE) decoder. **Dice 92.70%** on the AZH Chronic Wound set; the x-FUSegNet variant scored **89.23%, #1 on the MICCAI 2021 FUSeg leaderboard**. **Pretrained weights on Google Drive**, ready-to-run `fusegnet_test.py` / `xfusegnet_test.py`. The highest-value drop-in refiner found. Heavy (b7) but fine at 1 FPS on Orin.
- **HarDNet-DFUS** ([github.com/kytimmylai/DFUC2022](https://github.com/kytimmylai/DFUC2022)) — 1st place DFUC2022, **mean Dice 0.7287** on that harder challenge set; efficient HarDNet backbone, good Orin fit. Weights referenced in repo.
- **Deepskin** ([github.com/Nico-Curti/Deepskin](https://github.com/Nico-Curti/Deepskin), MIT, `pip install deepskin`) — EfficientNet-b3 U-Net (TF/Keras), trained semi-supervised on **smartphone** wound images (closer to our RGB camera than dermoscopy), plus a PWAT severity score. Crucially does **3-class segmentation: wound ROI / patient-body ROI / background** — the body class helps separate Bob's skin/limbs from the mattress. API `from deepskin import wound_segmentation`. Lowest-friction baseline (one pip install). Pin the TF version (>2.16 can give odd masks).
- **WoundAmbit benchmark (ECML-PKDD 2025)** ([arXiv:2504.06185](https://arxiv.org/abs/2504.06185), code [github.com/VanessaBorst/woundambit](https://github.com/VanessaBorst/woundambit), models on Zenodo `10.5281/zenodo.15123640`). The most useful *comparison study* for us: benchmarks 12 models (U-Net, FUSegNet, HarDNet-DFUS, FCBFormer, HiFormer, MISSFormer, SegFormer, SegNeXt, VWFormer, InternImage, **TransNeXt**) on chronic-wound data with 5-fold CV *and an out-of-distribution test set*. **TransNeXt generalized best (mIoU ~79.4–79.8%, mDSC ~88.5–88.7%)**, and all 12 ran **≥1 img/s on CPU** — so any clears our 1-FPS Orin bar. The paper to copy methodology and model-selection from.
- **uwm-bigdata/wound-segmentation** ([github.com/uwm-bigdata/wound-segmentation](https://github.com/uwm-bigdata/wound-segmentation)) — Wang et al. (*Scientific Reports* 2020) reference repo (TF 2.6). Ships the AZH dataset *and* training code for U-Net / MobileNetV2 / SegNet / VGG16 / Mask-RCNN with some trained `.hdf5` weights; canonical AZH source. License/weights not clearly stated.

Reality check specific to Bob: expect a real drop on moulage texture, full-body scale, and mattress/orange-shorts distractors, and the **colored marker dots** may be confused for wounds. Mitigations: use **depth to suppress the flat mattress plane**, use **ArUco corners to crop to the bed/body ROI**, and treat the marker dots as a known false-positive class filtered by size/shape.

---

## 2. Zero-shot / open-vocabulary detection & promptable segmentation (our primary path)

With zero training data, the winning pattern is a **two-stage open-vocab pipeline**: an open-vocabulary *detector* (text → boxes) feeds a *promptable segmenter* (box → mask), optionally re-scored by a CLIP head. Probe frames confirm the moulage lacerations are **saturated dark-red against uniform tan/pink plastic** — high contrast, which favors detection — but the "skin" is featureless molded plastic, and medical foundation models were trained on radiology, not RGB dermatology, so their priors don't transfer.

### Tier 1 — text-prompt → mask in one model (try first)
- **SAM 3 (Segment Anything with Concepts)** ([github.com/facebookresearch/sam3](https://github.com/facebookresearch/sam3), [arXiv:2511.16719](https://arxiv.org/abs/2511.16719)) — Nov 2025; the single most relevant new model. Does **Promptable Concept Segmentation**: give it the noun phrase `"wound"` / `"bloody cut"` / `"laceration"` and it detects+segments+tracks *every* matching instance in image or video, with a "presence head" decoupling recognition from localization. ~848M params, weights gated on HF (`facebook/sam3` / `facebook/sam3.1`, custom SAM License, commercial+research), needs Python 3.12+/PyTorch 2.7+/CUDA 12.6+. Wired into [Ultralytics](https://docs.ultralytics.com/models/sam-3/) and Roboflow. At ≤1 FPS this is the headline candidate — it can replace the whole two-stage stack. Failure mode: "wound" is a soft concept; the presence head may fire on red marker dots or the orange-shorts edge → threshold and post-filter by ArUco body region.
- **YOLOE (THU-MIG)** ([github.com/THU-MIG/yoloe](https://github.com/THU-MIG/yoloe) / [Ultralytics](https://docs.ultralytics.com/models/yoloe/)) — ICCV 2025, "Real-Time Seeing Anything." Open-vocab **detection AND instance segmentation** with text/visual/prompt-free prompts, +3.5 AP over YOLO-Worldv2 on LVIS at 1.4× faster. S/M variants target edge (TensorRT/ONNX/TFLite) → the *fast* fallback if SAM 3 is too heavy. (`yoloe-11l-seg.pt` already on disk locally.)

### Tier 2 — open-vocab detector → SAM segmenter (the robust classic)
- **Grounding DINO** ([github.com/IDEA-Research/GroundingDINO](https://github.com/IDEA-Research/GroundingDINO), ECCV 2024) — the workhorse text→box detector. SwinT checkpoint downloadable, HF-integrated ([`IDEA-Research/grounding-dino-tiny`](https://huggingface.co/IDEA-Research/grounding-dino-tiny), [transformers docs](https://huggingface.co/docs/transformers/en/model_doc/grounding-dino)). NVIDIA ships an **official Jetson deployment** ([JPS GDINO service](https://docs.nvidia.com/jetson/jps/inference-services/gdino.html)) at ~11.8 FPS on AGX Orin FP16 (some users see [2–3 FPS](https://github.com/NVIDIA-AI-IOT/jetson-platform-services/issues/3), still fine). Prompt dot-separated lowercase: `"wound . laceration . cut . blood ."`. Its open-set medical viability is supported by [GroundingDINO for Open-Set Lesion Detection (OpenReview 2024)](https://openreview.net/forum?id=Rvdet5Tm9n). Grounding DINO 1.5/1.6 Pro and [Edge](https://arxiv.org/abs/2405.10300) are **API-gated** ([API repo](https://github.com/IDEA-Research/Grounding-DINO-1.5-API)) — cloud sanity-check only.
- **Grounded-SAM / Grounded-SAM-2** ([Grounded-Segment-Anything](https://github.com/IDEA-Research/Grounded-Segment-Anything), Apache-2.0; [Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2)) — the assembled pipeline (Grounding DINO/Florence-2 → SAM/SAM 2) that turns `"laceration . cut . wound . bleeding . blood"` into tracked masks across video, matching our "RGB video, depth from ROS" setup. Ships lightweight **Grounded-MobileSAM / Grounded-FastSAM / Grounded-Efficient-SAM** edge variants. Most plug-and-play way to get text → mask + track without glue code.
- **SAM 2.1** ([github.com/facebookresearch/sam2](https://github.com/facebookresearch/sam2)) as the segmenter behind any detector. SAM v2 is *promptable only* (no text) — always pair with an open-vocab detector and drive it with a **box, not auto mode**: the [2D-medical SAM eval (arXiv:2305.00109)](https://arxiv.org/abs/2305.00109) and [SAM.MD (arXiv:2304.05396)](https://arxiv.org/abs/2304.05396) show box/multi-point prompts dominate single-point and that SAM zero-shot is strongest on **RGB-like dermoscopic/endoscopic** images (encouraging for our RGB lacerations); the fully-automatic "everything" mode is unreliable on low-contrast/texture-starved targets.

### Tier 3 — other open-vocab detectors (breadth / cross-checks)
- **YOLO-World (AILab-CVC)** ([github.com/AILab-CVC/YOLO-World](https://github.com/AILab-CVC/YOLO-World), CVPR 2024) via Ultralytics: `YOLOWorld("yolov8x-world.pt").set_classes(["bleeding wound","laceration","cut on skin"])`. Edge-friendly, V2.1 weights (Feb 2025). Use *descriptive* prompts ("bleeding wound" beats "wound"). (`yolov8x-worldv2.pt` already on disk.)
- **OWLv2 / OWL-ViT** ([`google/owlv2-large-patch14-ensemble`](https://huggingface.co/google/owlv2-large-patch14-ensemble), [`google/owlv2-base-patch16`](https://huggingface.co/google/owlv2-base-patch16), [docs](https://huggingface.co/docs/transformers/model_doc/owlv2)) — CLIP-backbone zero-shot detector supporting **image-exemplar** querying (hand it a cropped example wound — very useful with no training data). Slower/lower-recall than GDINO on small objects but a good independent vote. Deploy the Orin-optimized **NanoOWL** (below) rather than full HF OWLv2.
- **DINO-X** ([github.com/IDEA-Research/DINO-X-API](https://github.com/IDEA-Research/DINO-X-API), arXiv 2411.14347) — top open-world accuracy incl. *prompt-free* "detect anything," but **API-only, no downloadable weights** → cloud cross-check only.

### Tier 4 — efficient SAM variants (segmentation stage on Orin)
All take box/point prompts (no text), slot behind any detector: **FastSAM** ([repo](https://github.com/CASIA-LMC-Lab/FastSAM), YOLOv8-seg based, ~50× faster than SAM, [TensorRT-exportable](https://docs.ultralytics.com/models/fast-sam/); `FastSAM-x.pt` on disk), **MobileSAM** ([repo](https://github.com/ChaoningZhang/MobileSAM), ~7× smaller/~5× faster than FastSAM; `mobile_sam.pt` on disk), **EfficientSAM** ([repo](https://github.com/yformer/EfficientSAM), ~+4 AP over MobileSAM), **EdgeSAM** ([repo](https://github.com/chongzhou96/EdgeSAM), fastest on-device, ~37× faster than SAM). Since we run ≤1 FPS, **accuracy beats speed** — prefer full SAM 2.1 (hiera-large) or EfficientViT-SAM/EfficientSAM over FastSAM, whose masks are coarser on thin laceration shapes.

### Tier 5 — medical / CLIP-driven (use cautiously; domain mismatch)
- **MedCLIP-SAMv2** ([github.com/HealthX-Lab/MedCLIP-SAMv2](https://github.com/HealthX-Lab/MedCLIP-SAMv2), [arXiv:2409.19483](https://arxiv.org/abs/2409.19483)) — text-driven medical segmentation (BiomedCLIP + M2IB → SAM), zero-shot. Conceptually ideal but validated on ultrasound/MRI/X-ray/CT, not RGB skin → likely poor transfer; quick test, low expectation.
- **MedSAM** ([github.com/bowang-lab/MedSAM](https://github.com/bowang-lab/MedSAM), Nature Comms 2024; `medsam_vit_b.pth` 375 MB CC-BY-4.0 on [Zenodo](https://zenodo.org/records/10689643) / `10.5281/zenodo.10689643`) and **MedSAM2** ([github.com/bowang-lab/MedSAM2](https://github.com/bowang-lab/MedSAM2), [arXiv:2504.03600](https://arxiv.org/abs/2504.03600), Apache-2.0, weights `wanglab/MedSAM2`) plus **LiteMedSAM** — box-promptable, ~10× faster lite/distilled variants for low-resource inference. But repos/docs make **no claim of RGB/dermatology support** (CT/MRI/ultrasound/video only), so on RGB plastic skin vanilla SAM/SAM2 likely beat them — A/B test, don't assume.
- **BiomedCLIP** ([HF](https://huggingface.co/microsoft/BiomedCLIP-PubMedBERT_256-vit_base_patch16_224)) / plain **CLIP/OpenCLIP** — use as a *region re-classifier*: crop each candidate box and score against `["a bleeding laceration wound","intact skin","orange fabric","plastic"]` to suppress false positives (marker dots, shorts). The cheapest accuracy win on top of any detector.
- **ClipSAM** ([arXiv:2401.12665](https://arxiv.org/abs/2401.12665)) — CLIP-localization + SAM-refine framed as *zero-shot anomaly segmentation*; a wound on uniform skin is literally an anomaly, so this framing is a creative angle worth a test.

### Auto-labeling to bootstrap a fast custom model
**Autodistill** ([github.com/autodistill/autodistill](https://github.com/autodistill/autodistill)) + [`autodistill-grounded-sam`](https://github.com/autodistill/autodistill-grounded-sam) (and Roboflow's [Grounded-SAM-2 auto-label notebook](https://github.com/roboflow/notebooks/blob/main/notebooks/grounded-sam-2-auto-label.ipynb)) let you point an ontology like `{"deep red bleeding cut on skin":"wound"}` at our video, auto-generate masks/boxes with the foundation models above, then distill into a tiny YOLOv8/YOLO11/YOLO26-seg that runs fast on Orin — the path from "zero-shot demo" to "deployable Spot model."

### Expected failure modes on moulage/plastic skin (design around these)
1. **Color over-trigger** — every text model is tempted by orange shorts and red marker dots; mitigate with CLIP-rescoring and ArUco-bounded body masking; prompt negatively where supported.
2. **Texture-starved skin** — SAM under-segments or grabs the whole limb on featureless plastic; **always box-prompt SAM, never auto**.
3. **Concept ambiguity** — concrete phrases ("deep red bleeding gash", "open laceration") and SAM 3 / OWLv2 *image-exemplar* prompting beat the bare word "wound".
4. **Medical-model domain gap** — MedSAM/MedSAM2/MedCLIP expect grayscale radiology; expect underperformance vs generic SAM.
5. **Glare/specular highlights** on glossy moulage can read as a separate bright object and split masks.

---

## 3. Datasets & pretrained weights (downloadable, ready-to-test)

Almost every public wound dataset/checkpoint is from the **chronic-wound / DFU domain**, so there is a real **domain gap** to our supine full-body laceration scene. Treat these as (a) baselines to fine-tune/benchmark, (b) sources of *red-wound color priors*, and (c) data to combine with the zero-shot methods above. The two most practically runnable pretrained wound models are **FUSegNet** and **Deepskin** (both MIT, both ship weights — see §1). All Dice/accuracy figures are authors' self-reported numbers on their own DFU/chronic-wound sets and will not transfer to moulage.

### Pretrained weights you can run today
FUSegNet/xFUSegNet, Deepskin, uwm-bigdata/wound-segmentation — detailed in §1. Plus **Le0Dev/wound_segmentator** ([github.com/Le0Dev/wound_segmentator](https://github.com/Le0Dev/wound_segmentator)) — Attention-U-Net, pretrained weights on Google Drive (mean IoU 0.71 over 552 test images), torch 1.13/cu11.6, 256px, lightweight enough for Jetson.

### Core segmentation datasets (fine-tuning / benchmarking)
- **AZH Wound Care Center** — 1,010 train images + pixel masks (889 patients), inside the uwm-bigdata repo (`azh_wound_care_center_dataset_patches.zip`). The de-facto small wound-seg standard.
- **MICCAI 2021 FUSeg Challenge** ([dataset dir](https://github.com/uwm-bigdata/wound-segmentation/tree/master/data/Foot%20Ulcer%20Segmentation%20Challenge), [challenge site](https://fusc.grand-challenge.org/), inference code [github.com/masih4/Foot_Ulcer_Segmentation](https://github.com/masih4/Foot_Ulcer_Segmentation)) — 1,210 foot-ulcer images, pixel masks. DFU-domain.
- **DFUC 2020 / 2021 / 2022** ([dfu-challenge.github.io](https://dfu-challenge.github.io/)) — largest DFU sets: 2020 = 4,000 images with **bounding-box detection** labels; 2021 = 15,683 (classification); 2022 = ~4,000 with **pixel masks**. **Gated** (request via challenge site, non-commercial).
- **Medetec Wound Database** ([medetec.co.uk](https://www.medetec.co.uk/files/medetec-image-databases.html)) — ~594 images, ~15 categories, **copyright-free**, *no masks*. Most diverse *appearance* set (includes traumatic/surgical/laceration-like wounds closer to moulage) — good for color-prior tuning.
- **Leoscode "Wound segmentation [2760 samples]"** ([Kaggle](https://www.kaggle.com/datasets/leoscode/wound-segmentation-images)) — aggregated/re-annotated Medetec+FUSeg+WSNET with binary masks; easiest single download for a generic wound-vs-background model. Pairs with [Le0Dev/wound_segmentator](https://github.com/Le0Dev/wound_segmentator).
- **WoundSeg / WSNet** ([github.com/subbareddy248/WSNET](https://github.com/subbareddy248/WSNET), [WACV'23 PDF](https://openaccess.thecvf.com/content/WACV2023/papers/Oota_WSNet_Towards_an_Effective_Method_for_Wound_Image_Segmentation_WACV_2023_paper.pdf)) — diverse 8-type wound-seg set, reported 84.7% Dice.
- **Syn3DWound** ([lebrat.github.io/Syn3DWound](https://lebrat.github.io/Syn3DWound/), ISBI 2024) — *open-source synthetic* 3D wounds rendered on avatars with 2D+3D masks and wound-bed geometry. Ideal for sanity-checking our segmentation/metric pipeline against moulage-like (non-real) wounds before touching real data.

### Detection / classification (HuggingFace & Roboflow, directly pullable)
- **Roboflow Universe wound/injury** — [search](https://universe.roboflow.com/search?q=class%3Awound); concrete: [yolo-7tpjl/wound v9](https://universe.roboflow.com/yolo-7tpjl/wound/dataset/9) and [Injury-Detection (yolov8-gqqv4)](https://universe.roboflow.com/yolov8-gqqv4/injury-detection-fuv6x). Closest "wound/injury **in a scene**, bounding-box" data; one-line YOLO/COCO export; crowd-sourced quality — verify.
- **Hemg/Wound-Image-classification** ([HF](https://huggingface.co/Hemg/Wound-Image-classification)) — ViT fine-tuned for wound *classification*; useful as a yes/no "is there a wound here" head on cropped regions.
- **SurgWound** ([HF dataset](https://huggingface.co/datasets/xuxuxuxuxu/SurgWound)) — 697 surgical-wound images with attribute/diagnostic labels; sutured/incision appearance arguably closer to a moulage laceration than an ulcer.
- **Kaggle classification sets** — [ibrahimfateen/wound-classification](https://www.kaggle.com/datasets/ibrahimfateen/wound-classification), [yasinpratomo/wound-dataset](https://www.kaggle.com/datasets/yasinpratomo/wound-dataset) (cut/bruise/burn/abrasion/ulcer); verify counts/license on Kaggle.

---

## 4. The two reference repos + similar

### Le0Dev/YOLO-SAM-wound-detect-and-segment — the most directly relevant repo
A **two-stage detect-then-segment** pipeline, essentially the blueprint for Bob (verified from [`YOSAW_inference.ipynb`](https://github.com/Le0Dev/YOLO-SAM-wound-detect-and-segment/blob/main/YOSAW_inference.ipynb)):
- **Stage 1 (detect):** custom-fine-tuned Ultralytics YOLOv8 from `./trained_yolo/best.pt` → wound boxes.
- **Stage 2 (segment):** **SAM ViT-Base via HuggingFace `transformers`** (not Meta's checkpoint): `SamConfig.from_pretrained("facebook/sam-vit-base")` → `SamModel`, then a fine-tuned `./SAM_model_checkpoint_test_5epochs.pth` (only ~5 epochs). The YOLO box is the **box prompt**: `processor(test_image, input_boxes=[[box]], ...)`.
- 100% Jupyter, **no `requirements.txt`, no LICENSE, no published metrics** (treat license as all-rights-reserved by default, weights as research-quality). Training source is almost certainly the author's [Kaggle 2,760-sample set](https://www.kaggle.com/datasets/leoscode/wound-segmentation-images) (real chronic wounds, **not** moulage → domain gap on Bob).

**Takeaway:** the architecture (box prompt → SAM mask) is exactly what we want and runs at 1 FPS, but the bundled `best.pt` will under-detect moulage. The right move is to **keep the YOLO→SAM pattern but swap in an open-vocab detector** (GroundingDINO / YOLO-World, prompt "bleeding cut, laceration, wound") and a **stronger promptable segmenter** (SAM 2 / MobileSAM), using FUSegNet only for refinement/comparison. A cleaner medical implementation of the same idea is **Danialmoa/YoloSAM** ([github.com/Danialmoa/YoloSAM](https://github.com/Danialmoa/YoloSAM)) — YOLO region-prompting + SAM, explicitly "tailored for lesion or scar localization."

### Michael-OvO/Skin-Burn-Detection-Classification — burn-severity detector
A **YOLOv7-based** burn detector classifying by degree (`1st_degree`, `2nd_degree`, …). Precision 88%, **mAP@0.5 = 72%** (author's private set). Python 3.7.13, PyTorch 1.12, Flask demo. **MIT license**, runnable via `python detect.py`; released training notebooks use a *public* burn set so **downloadable weights ≠ the 88% model**. Relevance: burns ≠ bleeding lacerations, so it won't detect Bob's cuts directly — but it is the closest **MIT-licensed, fully-runnable** YOLO injury detector with a serving wrapper. **Reuse the structure (and at most the boxes), not the class labels.**

### Comparable wound/injury/bleeding repos worth pulling
- **mrinal054/FUSegNet** — strongest segmentation quality (details in §1).
- **pavan98765/Auto-WCEBleedGen** ([github.com/pavan98765/Auto-WCEBleedGen](https://github.com/pavan98765/Auto-WCEBleedGen)) — **YOLOv8-X bleeding detector** (capsule endoscopy): 96.10% classification acc, mAP@0.5 76.8%, IoU 80.75% on 6,345 images; `best.pt` included. GI-mucosa domain won't transfer, but it's the cleanest **bleeding-specific** YOLOv8 example and validates "blood as a detectable class."
- **meheditusar/PISeg** ([github.com/meheditusar/PISeg](https://github.com/meheditusar/PISeg)) — pressure-injury **staging** (YOLOv8 + ensembles); sparse docs/no metrics, useful mainly as a Roboflow-dataset pointer (`advances-in-wound-care-pi-dataset`).
- **akabircs/WoundTissue** ([github.com/akabircs/WoundTissue](https://github.com/akabircs/WoundTissue)) — wound tissue-type reference (granulation/slough/eschar), less relevant to clean moulage cuts.

---

## 5. Burn & skin-lesion adjacent — transferability to moulage trauma

Key question: do adjacent medical models transfer zero-shot to red moulage lacerations? Short answer: **the segmentation foundations (MedSAM, FUSegNet, AZH) are the most promising and downloadable; burn/ISIC *classifiers* are weak fits** because they learn real human skin texture and dermoscopy and will mostly mis-fire on plastic mannequin skin. None has seen moulage — treat every claim as a hypothesis to validate.

- **Wound/DFU segmentation (BEST fit)** — FUSegNet and uwm-bigdata/AZH (§1) learn exactly the moulage signal: a saturated red region with irregular boundary against skin. The redness of moulage blood should make them *easier*, not harder, than the desaturated chronic wounds they trained on. Output is a binary mask, directly what we need to crop/measure. Tissue sub-segmentation ([uwm-bigdata/DFUTissueSegNet](https://github.com/uwm-bigdata/DFUTissueSegNet)) is less relevant (moulage lacks faithful tissue types).
- **Promptable foundations (BEST general fallback)** — box-prompt **MedSAM / LiteMedSAM** (§2): you give it a box (from an open-vocab detector or a red-color heuristic) and it returns a tight mask, sidestepping the recognition problem. MedSAM2 adds 3D/video propagation. Likely the single most robust zero-shot wound-seg path; still A/B against generic SAM 2 on RGB.
- **Burn-depth classifiers (WEAK)** — Michael-OvO YOLOv7 (§4); also [CarstenIsert/DeepBurn](https://github.com/CarstenIsert/DeepBurn) (older, less maintained). Use only the *box*, ignore the burn-degree class. Ultrasound/terahertz burn work is wrong modality.
- **ISIC skin-lesion models (POOR)** — e.g. [Improving-skin-lesion-segmentation-with-self-training](https://github.com/Oichii/Improving-skin-lesion-segmentation-with-self-training) (SOTA on ISIC-2018/PH2): contact dermoscopy close-ups of pigmented moles — wrong scale/optics/color. Not recommended beyond using ISIC to fine-tune a backbone if we ever get labels.
- **Bleeding/hemorrhage detection (NICHE but on-point)** — since the moulage is *simulated bleeding*, **BlooDet** ("Synergistic Bleeding Region and Point Detection in Laparoscopic Surgical Videos," [arXiv:2503.22174](https://arxiv.org/abs/2503.22174), 2025) releases an open dataset + dual mask/point framework whose red-region branch may transfer. Brain/GI hemorrhage (CT/endoscopy) does **not** transfer. A pragmatic floor: **HSV red-color thresholding + connected components** — stage blood is far more saturated/uniform than real tissue, so this classical baseline is a strong sanity check against any learned model.
- **Bruise/abrasion (MARGINAL)** — forensic models mostly rely on alternate-light-source/UV imaging our RGB-D rig lacks, and weights are largely unreleased. Background only.

Bottom-line for Bob: (1) primary box-prompt **MedSAM/LiteMedSAM**; (2) primary **FUSegNet / AZH** on cropped regions; (3) cheap **HSV red-blood threshold** baseline; (4) optional **burn YOLOv7** as a box proposer (discard degree labels); (5) skip ISIC dermoscopy, brain/GI hemorrhage, ALS bruise models for zero-shot.

---

## 6. Pose & posture (lying vs sitting vs standing)

Bob lies **supine on a mattress, uncovered, in orange shorts**, with limbs in non-canonical, partly self-occluded orientations. Most 2D pose models are trained on **COCO Keypoints**, biased toward upright/standing people, so lying bodies are out-of-distribution: confidences drop, L/R limbs swap, and the top-down *person detector* may miss a horizontal box. The in-bed literature ([Yazıcı et al., "In-Bed Pose Estimation: A Review," 2024, arXiv:2402.00700](https://arxiv.org/abs/2402.00700)) is explicit and encouraging: **uncovered in-bed poses are the easy case; the hard part is blankets** — which we don't have. So a strong RGB model + our depth should suffice; the main risks are the detector and L/R confusion, not the keypoint regressor. At ≤1 FPS we can favor accuracy-heavy models and pick by qualitative keypoint quality on lying poses, not COCO mAP.

### Pose models by family
- **ViTPose / ViTPose++** — plain ViT backbone + light decoder; ViTPose++ adds a 6-dataset MoE head that generalizes better to atypical configurations. Native in [transformers (`VitPoseForPoseEstimation`)](https://huggingface.co/docs/transformers/en/model_doc/vitpose), checkpoints [`usyd-community/vitpose-plus-base`](https://huggingface.co/usyd-community/vitpose-plus-base) / [`-large`](https://huggingface.co/usyd-community/vitpose-plus-large); turnkey runner [easy_ViTPose](https://github.com/JunkyByte/easy_ViTPose). Top "accuracy" pick; top-down → watch the detector on a horizontal subject. Measured **ViTPose-S 6.54 ms (TensorRT FP32) on Orin AGX**, RT-DETRv2-S+ViTPose-S pipeline 19.5 ms ([arXiv:2502.15737](https://arxiv.org/pdf/2502.15737)).
- **Sapiens (Meta, ECCV'24 Oral)** — foundation model pretrained on **300M human images**; best zero-shot bet for weird orientations. Does 2D pose, **body-part segmentation**, depth, normals at 1K res, 0.3B–2B params. Part masks help localize "forearm/thigh"; normals/depth corroborate ROS depth. Models on [facebook/sapiens (HF)](https://huggingface.co/facebook/sapiens) / [GitHub](https://github.com/facebookresearch/sapiens), e.g. [sapiens-pose-1b](https://huggingface.co/facebook/sapiens-pose-1b). Caveat: 1B/2B at 1K is heavy — on Orin the **0.3B/0.6B** variants are the realistic targets.
- **RTMW / RTMPose / RTMO / RTMW3D (OpenMMLab MMPose)** — best accuracy-vs-speed and easiest to deploy. [RTMW](https://arxiv.org/abs/2407.08634) first open model past **70 mAP on COCO-WholeBody** (RTMW-l 70.2, 133 keypoints incl. hands/feet near a wound). [RTMO](https://arxiv.org/abs/2312.07526) is **one-stage** (no separate detector → removes the horizontal-bbox failure mode), RTMO-l 74.8 AP. **RTMW3D** adds monocular 3D whole-body (crude torso-axis). [`rtmlib`](https://github.com/Tau-J/rtmlib) (`pip install rtmlib`) runs these as **ONNX with onnxruntime/OpenVINO/TensorRT, no mmcv/mmpose** — easiest path to a TensorRT engine. RTMPose exports cleanly via [MMDeploy](https://github.com/open-mmlab/mmpose/blob/main/projects/rtmpose/README.md) (Apache-2.0, lists Jetson). RTMPose-m 75.8% COCO AP, 430+ FPS on a 1660 Ti (Orin column unverified, "real-time").
- **YOLO11-pose (Ultralytics)** ([docs](https://docs.ultralytics.com/tasks/pose/)) — pragmatic real-time baseline, one-stage (detect+17 keypoints), trivial install, clean TensorRT export. Lower on atypical lying poses than ViTPose/Sapiens but gives a working end-to-end loop immediately and a keypoint source for the posture rule. (`yolo11x-pose.pt` already on disk.)
- **MediaPipe BlazePose & MoveNet** — fast/lightweight but the detector+ROI tracker **assumes a roughly upright torso** and degrades on supine subjects. Cheap baselines/sanity checks only, not the primary detector. (`pose_landmarker_heavy.task` BlazePose on disk.) OpenPose is superseded for our purposes. (Local `nlf_l_multi_0.3.2.torchscript` NLF 3D pose is another candidate to spot-check.)

### Posture classification — a geometric heuristic, not a learned model
A simple, explainable rule on 2D keypoints is robust (the fall-detection literature confirms this). Core signal: **orientation of the torso axis** (shoulder-midpoint → hip-midpoint, optionally extended through knees/ankles), measured against gravity (from Spot/IMU or the **bed plane from ArUco+depth**, not image-vertical):
- **Standing**: torso near-vertical + large head→ankle vertical span. **Sitting**: torso near-vertical but hips/knees folded (small hip→ankle extent, large knee-bend angle). **Lying/supine**: torso axis near-horizontal — Bob's dominant cue.
- Corroborate with **bounding-box aspect ratio** (wide > tall ⇒ lying) and **hip-center height in the bed/world frame**. Because we have **depth + ArUco bed corners**, project keypoints to the **bed plane**: "all major joints within a few cm of the mattress plane" is an extremely strong, camera-independent lying detector that sidesteps COCO upright bias.
- Learned alternative: fall-detection work ([EAAI 2024 survey](https://dl.acm.org/doi/10.1016/j.engappai.2024.109809); [MDPI Sensors 2022](https://www.mdpi.com/1424-8220/22/12/4544); [PIFR, PLOS One 2025](https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0325253); [YOLOv11+SVM standing/lying](https://arxiv.org/html/2503.01436)) shows ~9 joint-angle features + a tiny SVM/MLP cleanly separates classes (~88–94%). For 3 static classes a tiny SVM/MLP suffices; no temporal model needed for a static mannequin.

### In-bed / supine-specific resources
The benchmark dataset is **SLP** ([ostadabbas/SLP-Dataset-and-Code](https://github.com/ostadabbas/SLP-Dataset-and-Code)) — 109 subjects, ~14.7k samples, **RGB+LWIR+depth+pressure**, supine/left/right categories — for sanity-testing models on real supine RGB and fine-tuning if needed. There is also a **Mannequin In-Bed Dataset** (realistic mannequins, RGB+LWIR, 14-joint — literally uses mannequins like Bob) and **Patient MoCap** (RGB+depth, 180k frames). [ostadabbas/Seeing-Under-the-Cover](https://github.com/ostadabbas/Seeing-Under-the-Cover) is only needed if we ever cover the mannequin. Honest caveat: all published mAP/AP are upright COCO benchmarks and do **not** predict supine accuracy — validate empirically on our clip.

---

## 7. Edge / Jetson Orin deployment — what's realistic at ≤1 FPS

Because our latency budget is generous, almost every "heavy" open-vocab model becomes viable; the real constraints are **VRAM**, **TensorRT/ONNX export friction for transformer ops**, and **whether a TensorRT-optimized variant already exists**. Numbers below are from authoritative sources.

### Tier 1 — fast and trivially deployable (well under budget)
- **Ultralytics YOLOv8 / YOLO11 / YOLO26 detection+segmentation** — the workhorse for marker-dot detection, ArUco-region cropping, and (after light fine-tuning or as a region proposer) wound localization. One-line TensorRT export; [Jetson benchmarks](https://docs.ultralytics.com/guides/nvidia-jetson/). **YOLO26n** on Orin Nano Super (JetPack 6.1): TensorRT FP16 4.57 ms / INT8 3.80 ms; Orin NX 16GB FP16 4.13 / INT8 3.49 ms. [Seeed](https://www.seeedstudio.com/blog/2023/03/30/yolov8-performance-benchmarks-on-nvidia-jetson-devices/): YOLOv8n ~52 FPS FP16 / ~65 INT8 on Orin NX; AGX Orin runs YOLOv8x INT8 ~75 FPS — so we can run the big `-x` models for accuracy with no latency concern. Caveat: INT8/half export occasionally [fails on Orin Nano](https://github.com/ultralytics/ultralytics/issues/17841) without matching TensorRT/JetPack + a calibration set.
- **RTMPose (MMDeploy → TensorRT)** — best posture feed; lists Jetson as a target, RTMPose-m 430+ FPS on a 1660 Ti (Orin "real-time," exact FPS unverified).

### Tier 2 — heavy but viable thanks to NVIDIA's TensorRT-optimized variants (the sweet spot for zero-shot wounds)
- **NanoOWL** ([NVIDIA-AI-IOT/nanoowl](https://github.com/NVIDIA-AI-IOT/nanoowl)) — OWL-ViT re-engineered with TensorRT for Jetson; directly relevant given no training data. On AGX Orin: **ViT-B/32 ~95 FPS** (28 mAP), **ViT-B/16 ~25 FPS** (31.7 mAP). A [robotics edge survey (PMC12583037)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12583037/) measured NanoOWL patch32 FP16 at 9.81 ms and a full **NanoOWL+EfficientViT-SAM-L0 pipeline at 47.5 FPS / 84.6% mIoU** — a complete open-vocab detect-then-segment pipeline far inside budget. Arguably our strongest zero-shot edge candidate.
- **NanoSAM** ([NVIDIA-AI-IOT/nanosam](https://github.com/NVIDIA-AI-IOT/nanosam)) — ResNet18 encoder distilled from MobileSAM, TensorRT-ready. AGX Orin: encoder 27 ms, **full pipeline 8.1 ms** (~5× faster than MobileSAM), mIoU 0.706. Pair NanoOWL boxes → NanoSAM masks → depth lookup for 3D wound localization.
- **EfficientViT-SAM** ([mit-han-lab/efficientvit](https://github.com/mit-han-lab/efficientvit/blob/master/applications/efficientvit_sam/README.md)) — higher accuracy at real-time speeds. Orin FP16 end-to-end: L0 8.2 ms, L1 10.2, L2 12.9, XL0 22.5, XL1 37.2 ms. The upgrade if NanoSAM masks are too coarse on small wounds.
- **Grounding DINO** — [JPS GDINO container](https://docs.nvidia.com/jetson/jps/inference-services/gdino.html) ~11.8 FPS on AGX Orin at 1080p; [GDINO 1.5 Edge](https://arxiv.org/pdf/2405.10300) >10 FPS on Orin NX at 640². More expressive phrase grounding than OWL-ViT; use the official container ([Roboflow deploy guide](https://roboflow.com/how-to-deploy/deploy-grounding-dino-to-nvidia-jetson)).
- **ViTPose** — ViTPose-S 6.54 ms TensorRT FP32 on Orin AGX (above). Note: ViT/transformer layers **cannot use Orin's DLA** (JetPack 6.2) — keep on GPU.

### Tier 3 — deployable only because we tolerate ≤1 FPS (heavy / fragile export)
- **OWLv2 (full HF)** ~300 ms/inference, needs a powerful GPU ([52 FPS on V100](https://arxiv.org/html/2306.09683v3)) → use **NanoOWL** instead.
- **Vanilla SAM / SAM2 (ViT-H/-L)** — seconds-per-image on Orin even with TensorRT; SAM-b ~41,700 ms on CPU ([SAM2 docs](https://docs.ultralytics.com/models/sam-2/)). Offline/occasional only; prefer NanoSAM/FastSAM/EfficientViT-SAM. SAM2-TensorRT frameworks exist ([TIER IV](https://medium.com/tier-iv-tech-blog/high-performance-sam2-inference-framework-with-tensorrt-9b01dbab4bf7)) but are integration-heavy.
- **FastSAM-s** (23.9 MB, YOLOv8-seg under the hood) — easy TensorRT export, good whole-scene fallback but class-agnostic (needs CLIP/text post-filter).
- **YOLO-World / YOLOE** — attractive zero-shot detectors (YOLO-World-S 26.07 ms on AGX Orin per the [edge survey](https://pmc.ncbi.nlm.nih.gov/articles/PMC12583037/)), but [YOLOE TensorRT export on Jetson can be slow / partially unsupported](https://community.ultralytics.com/t/yoloe-inference-very-slow-on-jetson-with-tensorrt/1443/23) (text-encoder path) — validate export before committing.

### Practical Orin notes
- **Export**: Ultralytics native `export(format="engine")` for YOLO/FastSAM/YOLOE; NVIDIA `torch2trt` for OWL-ViT/SAM-distilled; MMDeploy ONNX→TensorRT for RTMPose/ViTPose. Avoid hand-rolling `torch.onnx.export` for transformer attention.
- **Precision**: FP16 is the safe default (no calibration data needed); INT8 gives ~1.3–1.5× but needs a calibration set we lack — stay FP16 given the loose budget.
- **DLA**: useful for YOLO conv backbones; transformers (OWL-ViT, GDINO, ViTPose, SAM encoders) **cannot** use DLA — keep on GPU.
- **Containers**: use Jetson AI Lab / `dustynv` / NGC (jps-gdino, DeepStream) to sidestep JetPack/TensorRT/PyTorch version hell.
- **Recommended Orin pipeline**: YOLO11 (markers/coarse body) → NanoOWL open-vocab wound prompts → NanoSAM (or EfficientViT-SAM-L0) masks → depth lookup at mask centroid → RTMPose/ViTPose for posture. Every stage is independently measured well under 1 s on AGX Orin.

---

## 8. Robotics & RGB-D wound perception

This is the closest published analog to our exact setup (mobile robot + arm + RGB-D camera localizing and measuring wounds). A small but fast-growing literature on **robot-driven 3D wound assessment** exists, but almost none of the *end-to-end robotic wound* papers release code — they are blueprints, assembled from generic CV components plus the wound datasets/weights above.

### Closest published system: robotic triage with foundation models (Dec 2025)
[*A Multi-Robot Platform for Robotic Triage Combining Onboard Sensing and Foundation Models*](https://arxiv.org/abs/2512.08754) (arXiv:2512.08754) is almost a direct blueprint: prompt **Grounding DINO** to find the human → **SAM2** to mask clutter → **BlazePose** skeleton → re-prompt Grounding DINO on the masked victim with `"wound"`, `"blood"`, `"amputation"`. Adds a **DINOv3** severe-hemorrhage classifier and a compact VLM (**NVILA-Lite-2B**) for per-region yes/no trauma questions. Honest result: it is **hard** — per-region validation Severe Hemorrhage 50.5%, Head 83.5%, Torso 74.6%, Upper-extremity 68.7%, Lower-extremity 64.2%. They run onboard an RTX 4000 SFF Ada (20 GB, heavier than Orin) but the model choices (SAM2, BlazePose, a 2B VLM) are Orin-feasible at 1 FPS. Takeaway: open-vocab + body-part grounding works but is noisy → combine detector + VLM verification, and exploit our **ArUco corners + depth** to constrain the search.

### Robot-driven RGB-D wound measurement
- **2D/3D Wound Segmentation on a Robot-Driven Reconstruction System** ([Sensors 2023, PMC10058897](https://pmc.ncbi.nlm.nih.gov/articles/PMC10058897/)) — RealSense RGB-D + Photoneo scanner on a 7-DoF Kinova Gen3 arm (architecturally ~Spot+arm+RealSense). Pipeline: **MobileNetV2** 2D classifier + **GrabCut** refinement → 3D active-contour on the mesh → perimeter/area/volume (errors: perimeter 0.89–4.96%, area 2.86–8.02%, volume 4.59–8.94%) on a **Seymour II wound-care training model** — validating training-model/moulage wounds as a legitimate evaluation target. No code, but a clean re-implementable recipe. Follow-on: [Autonomous Robot-Driven Chronic Wound 3D Reconstruction (Robotics 2025, MDPI 14/3/30)](https://www.mdpi.com/2218-6581/14/3/30).
- **CSIRO Data61 wound suite** — [Wound3DAssist (arXiv:2508.17635)](https://arxiv.org/abs/2508.17635) (monocular video → 3D reconstruction + seg + tissue classification, mm-level), [Non-Invasive 3D Wound Measurement with RGB-D (arXiv:2601.19014)](https://arxiv.org/abs/2601.19014) (RGB-D odometry + B-spline surface, sub-mm on silicone phantoms, real-time — closest to our RealSense use), [WoundNeRF / Multi-View Consistent Wound Segmentation (arXiv:2601.16487)](https://arxiv.org/abs/2601.16487) (NeRF-SDF view-consistent masks, code promised). Plus [3D point-cloud rat-wound segmentation+measurement (Biomed. Signal Process. Control 2025)](https://www.sciencedirect.com/science/article/abs/pii/S1746809425001934) (PointNet++ + convex-hull volume) if we go point-cloud-native. These validate that **sub-mm RGB-D wound metrics are achievable**.

### Triage robots, Spot, and integration
- **DARPA Triage Challenge** ([site](https://triagechallenge.darpa.mil/); Year 2 concluded Oct 2025, winners DART/MSAI per [DARPA news](https://www.darpa.mil/news/2025/dart-msai-triumph-darpa-triage-challenge)) — UGV/UAV teams autonomously detect casualties + external hemorrhage/wounds. Its [Data Competition](https://triagechallenge.darpa.mil/about/data-competition) dataset would be the most on-point casualty/injury benchmark, but **a public download link is unverified** — watch, don't rely on it yet.
- **MIT/Brigham "Dr. Spot"** ([MIT News](https://news.mit.edu/2020/spot-robot-vital-signs-0831); [PMC9096356](https://pmc.ncbi.nlm.nih.gov/articles/PMC9096356/)) — vitals not wounds, but a reusable Spot perception stack (InsightFace ROI selection, `solvePnP` head-pose, IR respiration/temp, rPPG heart rate). No code.
- **Spot integration**: [bdaiinstitute/spot_ros2](https://github.com/bdaiinstitute/spot_ros2) (Spot SDK → ROS 2, arm control + wrist 4K + stereo depth, MoveIt end-effector control — how we'd feed a wound mask+depth ROI into an arm-targeting goal) and [sandialabs/spot_bt_ros](https://github.com/sandialabs/spot_bt_ros) (behavior-tree autonomy).

### Downloadable building block: detect→segment→depth template
[**FusionVision** (Sensors 2024, PMC11086350)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086350/), code [github.com/safouaneelg/FusionVision](https://github.com/safouaneelg/FusionVision) — YOLO detection → **FastSAM** segmentation → RealSense depth back-projection → per-object point cloud (demoed on a D435i). A near-drop-in template: swap YOLO for an **open-vocab** detector ([GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) / [YOLO-World](https://github.com/AILab-CVC/YOLO-World)) and FastSAM/SAM for the mask, then fetch depth at the mask for 3D localization and arm targeting.

### Moulage detection specifically — a genuine gap
There is essentially **no published CV work on detecting *moulage* (simulated wounds) as a task** — searches return only moulage *creation* guides ([StatPearls](https://www.ncbi.nlm.nih.gov/books/NBK549886/), Laerdal, Nasco). This gap is itself a finding and a point in favor of zero-shot/open-vocab methods. The closest training resource is the synthetic **Syn3DWound** ([lebrat.github.io/Syn3DWound](https://lebrat.github.io/Syn3DWound/)) (§3).

---

## Candidate methods to test

| Method | Family | Zero-shot? | Edge-friendly | Source |
|---|---|---|---|---|
| SAM 3 (concepts) | Text→mask foundation | Yes | Heavy but ≤1 FPS OK | [facebookresearch/sam3](https://github.com/facebookresearch/sam3) |
| Grounding DINO (+ Grounded-SAM/SAM2) | Open-vocab detector → SAM | Yes | Yes (NVIDIA JPS TRT, ~11.8 FPS) | [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO) / [Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2) |
| NanoOWL (OWL-ViT) | Open-vocab detector (TRT) | Yes | Yes (Orin-optimized, ~25–95 FPS) | [NVIDIA-AI-IOT/nanoowl](https://github.com/NVIDIA-AI-IOT/nanoowl) |
| OWLv2 / OWL-ViT (+ image exemplar) | Open-vocab detector | Yes | Via NanoOWL | [owlv2 docs](https://huggingface.co/docs/transformers/model_doc/owlv2) |
| YOLO-World | Open-vocab detector | Yes | Yes (TRT export; ~26 ms) | [AILab-CVC/YOLO-World](https://github.com/AILab-CVC/YOLO-World) |
| YOLOE | Open-vocab detect+seg | Yes | Yes (export caveats) | [THU-MIG/yoloe](https://github.com/THU-MIG/yoloe) |
| SAM 2.1 (box-prompted) | Promptable segmenter | Yes (with box) | Heavy; ≤1 FPS OK | [facebookresearch/sam2](https://github.com/facebookresearch/sam2) |
| NanoSAM | Promptable segmenter (TRT) | Yes (with box) | Yes (~8 ms full pipe) | [NVIDIA-AI-IOT/nanosam](https://github.com/NVIDIA-AI-IOT/nanosam) |
| EfficientViT-SAM | Promptable segmenter | Yes (with box) | Yes (8–37 ms on Orin) | [mit-han-lab/efficientvit](https://github.com/mit-han-lab/efficientvit) |
| MobileSAM / FastSAM / EfficientSAM / EdgeSAM | Efficient SAM variants | Yes (with box) | Yes (TRT export) | [MobileSAM](https://github.com/ChaoningZhang/MobileSAM) / [FastSAM](https://github.com/CASIA-LMC-Lab/FastSAM) / [EfficientSAM](https://github.com/yformer/EfficientSAM) / [EdgeSAM](https://github.com/chongzhou96/EdgeSAM) |
| MedSAM / LiteMedSAM / MedSAM2 | Medical promptable seg | Yes (with box) | Lite/Med2 yes | [bowang-lab/MedSAM](https://github.com/bowang-lab/MedSAM) / [MedSAM2](https://github.com/bowang-lab/MedSAM2) |
| MedCLIP-SAMv2 | Text→mask medical | Yes | Moderate | [HealthX-Lab/MedCLIP-SAMv2](https://github.com/HealthX-Lab/MedCLIP-SAMv2) |
| ClipSAM | Zero-shot anomaly seg | Yes | Moderate | [arXiv:2401.12665](https://arxiv.org/abs/2401.12665) |
| CLIP / OpenCLIP / BiomedCLIP rescorer | Region re-classifier | Yes | Yes | [BiomedCLIP](https://huggingface.co/microsoft/BiomedCLIP-PubMedBERT_256-vit_base_patch16_224) |
| FUSegNet / x-FUSegNet | Supervised wound seg | No | Yes (≤1 FPS) | [mrinal054/FUSegNet](https://github.com/mrinal054/FUSegNet) |
| Deepskin | Supervised wound+body seg | No | Yes | [Nico-Curti/Deepskin](https://github.com/Nico-Curti/Deepskin) |
| HarDNet-DFUS | Supervised wound seg | No | Yes | [kytimmylai/DFUC2022](https://github.com/kytimmylai/DFUC2022) |
| TransNeXt (WoundAmbit) | Supervised wound seg (best OOD) | No | Yes (≥1 img/s CPU) | [VanessaBorst/woundambit](https://github.com/VanessaBorst/woundambit) |
| Le0Dev/wound_segmentator (Attn-U-Net) | Supervised wound seg | No | Yes | [Le0Dev/wound_segmentator](https://github.com/Le0Dev/wound_segmentator) |
| Le0Dev YOLO-SAM | Detect→SAM pipeline | No (swap detector) | Yes | [YOLO-SAM-wound-detect-and-segment](https://github.com/Le0Dev/YOLO-SAM-wound-detect-and-segment) |
| Danialmoa/YoloSAM | Detect→SAM (lesion/scar) | No | Yes | [Danialmoa/YoloSAM](https://github.com/Danialmoa/YoloSAM) |
| Auto-WCEBleedGen (YOLOv8-X) | Bleeding detector (GI) | No | Yes | [pavan98765/Auto-WCEBleedGen](https://github.com/pavan98765/Auto-WCEBleedGen) |
| BlooDet | Bleeding region+point (surgical) | No | Moderate | [arXiv:2503.22174](https://arxiv.org/abs/2503.22174) |
| Skin-Burn-Detection (YOLOv7) | Burn detector (box proposer) | No | Yes | [Michael-OvO/Skin-Burn-Detection-Classification](https://github.com/Michael-OvO/Skin-Burn-Detection-Classification) |
| HSV red-blood threshold + CC | Classical color baseline | Yes | Yes (trivial) | (classical / OpenCV) |
| Roboflow wound/injury YOLO | Supervised detector | No | Yes | [Injury-Detection](https://universe.roboflow.com/yolov8-gqqv4/injury-detection-fuv6x) |
| Autodistill (Grounded-SAM) | Auto-label → distill | Yes (label) | Yes (distilled YOLO) | [autodistill](https://github.com/autodistill/autodistill) |
| FusionVision | Detect→FastSAM→RGB-D 3D | Yes (swap detector) | Yes | [safouaneelg/FusionVision](https://github.com/safouaneelg/FusionVision) |
| ViTPose / ViTPose++ | Top-down pose | Yes (pretrained) | Yes (6.5 ms Orin) | [vitpose docs](https://huggingface.co/docs/transformers/en/model_doc/vitpose) |
| Sapiens (0.3B/0.6B) | Pose + part-seg foundation | Yes (pretrained) | Partial (use small) | [facebookresearch/sapiens](https://github.com/facebookresearch/sapiens) |
| RTMW / RTMO / RTMPose (rtmlib) | Pose (1-stage/whole-body) | Yes (pretrained) | Yes (TRT via rtmlib/MMDeploy) | [Tau-J/rtmlib](https://github.com/Tau-J/rtmlib) / [MMPose](https://github.com/open-mmlab/mmpose) |
| YOLO11-pose | Pose (real-time baseline) | Yes (pretrained) | Yes (TRT) | [ultralytics pose](https://docs.ultralytics.com/tasks/pose/) |
| BlazePose / MoveNet | Lightweight pose | Yes (pretrained) | Yes (degrades supine) | [BlazePose](https://research.google/blog/on-device-real-time-body-pose-tracking-with-mediapipe-blazepose/) |
| Torso-axis + bed-plane rule (+ SVM) | Posture classifier | Yes (geometric) | Yes (trivial) | (geometric / fall-detection lit) |

## Key datasets & weights

| Name | Type | Link |
|---|---|---|
| FUSegNet / x-FUSegNet weights | Wound-seg pretrained (MIT, Google Drive) | [github.com/mrinal054/FUSegNet](https://github.com/mrinal054/FUSegNet) |
| Deepskin weights | Wound+body seg pretrained (pip) | [github.com/Nico-Curti/Deepskin](https://github.com/Nico-Curti/Deepskin) |
| HarDNet-DFUS weights | Wound-seg pretrained | [github.com/kytimmylai/DFUC2022](https://github.com/kytimmylai/DFUC2022) |
| WoundAmbit models (TransNeXt etc.) | Wound-seg benchmark weights (Zenodo) | [github.com/VanessaBorst/woundambit](https://github.com/VanessaBorst/woundambit) |
| Le0Dev/wound_segmentator weights | Attn-U-Net wound-seg (Google Drive) | [github.com/Le0Dev/wound_segmentator](https://github.com/Le0Dev/wound_segmentator) |
| MedSAM / LiteMedSAM | Promptable medical seg (CC-BY-4.0, Zenodo) | [zenodo.org/records/10689643](https://zenodo.org/records/10689643) |
| MedSAM2 | Promptable medical seg (Apache-2.0, HF `wanglab/MedSAM2`) | [github.com/bowang-lab/MedSAM2](https://github.com/bowang-lab/MedSAM2) |
| SAM 3 weights | Concept-promptable foundation (gated HF) | [github.com/facebookresearch/sam3](https://github.com/facebookresearch/sam3) |
| AZH Wound Care Center | Chronic-wound seg dataset (in repo) | [github.com/uwm-bigdata/wound-segmentation](https://github.com/uwm-bigdata/wound-segmentation) |
| MICCAI 2021 FUSeg | Foot-ulcer seg dataset | [fusc.grand-challenge.org](https://fusc.grand-challenge.org/) |
| DFUC 2020/2021/2022 | DFU detection/cls/seg datasets (gated) | [dfu-challenge.github.io](https://dfu-challenge.github.io/) |
| Medetec Wound Database | Diverse wound images (copyright-free, no masks) | [medetec.co.uk](https://www.medetec.co.uk/files/medetec-image-databases.html) |
| Leoscode wound-segmentation (2760) | Aggregated wound-seg + masks (Kaggle) | [kaggle.com/.../wound-segmentation-images](https://www.kaggle.com/datasets/leoscode/wound-segmentation-images) |
| WSNet / WoundSeg | Diverse 8-type wound-seg dataset | [github.com/subbareddy248/WSNET](https://github.com/subbareddy248/WSNET) |
| Syn3DWound | Synthetic 3D wound dataset (2D+3D masks) | [lebrat.github.io/Syn3DWound](https://lebrat.github.io/Syn3DWound/) |
| Roboflow wound/injury sets | Detection datasets + hosted models | [universe.roboflow.com (wound)](https://universe.roboflow.com/search?q=class%3Awound) |
| Hemg/Wound-Image-classification | Wound classifier (ViT, HF) | [huggingface.co/Hemg/Wound-Image-classification](https://huggingface.co/Hemg/Wound-Image-classification) |
| SurgWound | Surgical-wound images + labels (HF) | [huggingface.co/datasets/xuxuxuxuxu/SurgWound](https://huggingface.co/datasets/xuxuxuxuxu/SurgWound) |
| Kaggle wound classification | Wound-type cls datasets | [kaggle.com/.../wound-classification](https://www.kaggle.com/datasets/ibrahimfateen/wound-classification) |
| BlooDet dataset | Surgical bleeding region/point | [arXiv:2503.22174](https://arxiv.org/abs/2503.22174) |
| Skin-Burn weights (YOLOv7) | Burn detector pretrained (MIT) | [github.com/Michael-OvO/Skin-Burn-Detection-Classification](https://github.com/Michael-OvO/Skin-Burn-Detection-Classification) |
| Auto-WCEBleedGen weights | Bleeding detector (YOLOv8-X) | [github.com/pavan98765/Auto-WCEBleedGen](https://github.com/pavan98765/Auto-WCEBleedGen) |
| ViTPose++ checkpoints | Pose weights (HF) | [huggingface.co/usyd-community/vitpose-plus-base](https://huggingface.co/usyd-community/vitpose-plus-base) |
| Sapiens pose/seg checkpoints | Pose+part-seg foundation (HF) | [huggingface.co/facebook/sapiens](https://huggingface.co/facebook/sapiens) |
| RTMW / RTMO / RTMPose | Pose weights (MMPose / rtmlib) | [github.com/open-mmlab/mmpose](https://github.com/open-mmlab/mmpose) |
| SLP Dataset | In-bed RGB+LWIR+depth+pressure pose | [github.com/ostadabbas/SLP-Dataset-and-Code](https://github.com/ostadabbas/SLP-Dataset-and-Code) |

---

## Recommended test shortlist

Ranked by promise on our actual data. **[EDGE]** = directly deployable on Jetson Orin; **[OFFLINE/HEAVY]** = run it but expect to lean on the optimized variant for the robot.

1. **Grounded-SAM-2 (Grounding DINO → SAM 2.1, box-prompted)** [EDGE via JPS GDINO + efficient SAM] — the robust zero-shot workhorse; prompt `"laceration . cut . wound . bleeding . blood ."`, get tracked masks on video. The default pipeline.
2. **NanoOWL + NanoSAM** [EDGE] — fully Orin-optimized open-vocab detect→segment at far above 1 FPS; the production edge pipeline once prompts are tuned.
3. **SAM 3 (concept prompts)** [OFFLINE/HEAVY, ≤1 FPS OK] — single-model text→mask+track; potentially replaces the whole stack; verify it doesn't over-fire on shorts/marker dots.
4. **HSV red-blood threshold + connected components** [EDGE] — trivial classical floor; moulage blood is highly saturated/uniform, so this may be a surprisingly strong baseline and a great gate/sanity check.
5. **YOLO-World / YOLOE** [EDGE] (`yolov8x-worldv2.pt`, `yoloe-11l-seg.pt` already local) — fast open-vocab cross-check with descriptive prompts; YOLOE also gives masks.
6. **FUSegNet / x-FUSegNet** [EDGE] — best supervised wound segmenter as a *refiner* on cropped candidate limbs; the redness of moulage may help here.
7. **Deepskin** [EDGE] — one pip install, gives wound + body masks in the full scene; quickest signal and its body class separates Bob from the mattress.
8. **MedSAM / LiteMedSAM (box-prompted)** [EDGE, Lite] — A/B against generic SAM 2; sidesteps recognition, but its radiology training may underperform on RGB — test, don't assume.
9. **CLIP/BiomedCLIP region rescorer** [EDGE] — cheap accuracy win: rescore each candidate box to suppress orange shorts and red marker dots.
10. **FusionVision template (swap in open-vocab detector + SAM)** [EDGE] — the detect→segment→RealSense-depth back-projection glue we need for 3D wound localization and arm targeting.
11. **RTMW / RTMO via rtmlib (TensorRT)** [EDGE] — best deployable whole-body pose; RTMO's one-stage design avoids the horizontal-bbox failure mode on supine Bob.
12. **ViTPose++ (base/large)** [EDGE] — top keypoint accuracy to validate clean, L/R-correct limbs on the supine footage; pick the pose winner from this vs RTMW.
13. **Sapiens-0.6B (pose + body-part seg)** [OFFLINE/HEAVY → use 0.3B/0.6B] — most robust to weird orientations; part masks localize "forearm/thigh" and depth/normals corroborate ROS depth.
14. **YOLO11-pose** [EDGE] (`yolo11x-pose.pt` local) — immediate working pose baseline + keypoint source for the posture rule.
15. **Torso-axis + bed-plane posture rule (ArUco+depth), optional tiny SVM** [EDGE] — trivial, explainable lying/sitting/standing classifier that sidesteps COCO upright bias entirely.
16. **Autodistill (Grounded-SAM ontology) → distilled YOLO-seg** [EDGE] — if zero-shot recall is insufficient, auto-label ~50–200 Bob frames and distill a fast custom model.
17. **Auto-WCEBleedGen (YOLOv8-X bleeding)** [EDGE] — bleeding-specific detector to test the "blood as a class" idea; expect weak transfer from GI mucosa, use as a vote only.
18. **Skin-Burn YOLOv7 / Roboflow injury YOLO** [EDGE] — fast box proposers / structural template; **use boxes, discard class labels**.

### What likely WON'T work on moulage wounds (be honest)
- **DFU/chronic-wound and ISIC dermoscopy models used as standalone scene detectors** (FUSegNet, HarDNet-DFUS, TransNeXt, ISIC nets): trained on cropped, centered real-tissue close-ups; expect them to under-fire at full-body scale on featureless plastic. Refiners on tight crops only.
- **Burn classifiers' degree labels** (Skin-Burn YOLOv7): burns ≠ bleeding cuts; the class will be wrong even when the box is right.
- **MedSAM / MedSAM2 / MedCLIP** as RGB skin segmenters: radiology/ultrasound/CT training, no dermatology claim; likely beaten by generic SAM on our RGB frames.
- **Brain/GI hemorrhage and ALS/UV bruise models**: wrong modality entirely; not even worth a download for our RGB-D rig.
- **SAM "everything"/auto mode and point prompts on plastic skin**: texture-starved surface causes under-segmentation or whole-limb grabs — always drive SAM with a *box*.
- **BlazePose/MoveNet as the primary detector for supine Bob**: upright-torso assumption degrades on a horizontal subject — sanity checks only.
- **API-gated models** (Grounding DINO 1.5/1.6 Pro, DINO-X): not downloadable, so they fail the "run on Jetson" bar — cloud sanity-checks at most.
- **End-to-end robot-wound papers** (PMC10058897, Dr. Spot, DARPA Triage data): blueprints, not repos — no code/weights to deploy; the DARPA Data Competition public dataset link is unverified.
