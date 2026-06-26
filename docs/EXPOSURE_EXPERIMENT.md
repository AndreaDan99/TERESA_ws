# TERESA Exposure Experiment — Offline Protocol

## Prerequisites

- Jetson Orin powered on, Spot connected
- RealSense D415 unplugged from Z1 arm, held by hand (USB to Jetson)
- Mannequin supino on a table
- Fake wounds applied (silicone, makeup, bandages, etc.)

## Quick Reference — Direttamente sulla Jetson

Apri 3 terminali direttamente sul desktop della Jetson.

```bash
# === TERMINAL 1: RealSense camera (tienilo aperto) ===
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p enable_color:=true -p rgb_camera.color_profile:=640x480x30 \
  -p depth_module.depth_profile:=640x480x30 -p enable_depth:=true \
  -p align_depth.enable:=true -p pointcloud.enable:=false -p enable_infra:=false

# === TERMINAL 2 (optional): preview ===
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
rqt_image_view /camera/camera/color/image_raw

# === TERMINAL 3: Capture ===
docker exec -it teresa_gpu bash
mkdir -p /work/exposure/exp_01
python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
#   w+ENTER = wide shot  |  ENTER = close-up  |  q+ENTER = quit

# === After capture: NLF + GDINO ===
docker exec teresa_gpu python3 /ros2_ws/scripts/run_exposure_offline.py \
  --exp_dir /work/exposure/exp_01 \
  --nlf_model /ros2_ws/nlf_s_multi.torchscript \
  --cache_dir /work/hf_cache

# === After manual review: metrics ===
docker exec teresa_gpu python3 /ros2_ws/scripts/compute_exposure_metrics.py \
  --predictions /work/exposure/exp_01/predictions.json
```

## Quick Reference — Via SSH da Mac

Apri 3 terminali sul Mac.

```bash
# === TERMINAL 1: RealSense camera ===
ssh orin
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p enable_color:=true -p rgb_camera.color_profile:=640x480x30 \
  -p depth_module.depth_profile:=640x480x30 -p enable_depth:=true \
  -p align_depth.enable:=true -p pointcloud.enable:=false -p enable_infra:=false

# === TERMINAL 2 (optional): preview (needs XQuartz on Mac) ===
ssh -X orin
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
rqt_image_view /camera/camera/color/image_raw

# === TERMINAL 3: Capture ===
ssh orin
docker exec -it teresa_gpu bash
mkdir -p /work/exposure/exp_01
python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
#   w+ENTER = wide shot  |  ENTER = close-up  |  q+ENTER = quit

# === After capture: NLF + GDINO ===
ssh orin "docker exec teresa_gpu python3 /ros2_ws/scripts/run_exposure_offline.py \
  --exp_dir /work/exposure/exp_01 \
  --nlf_model /ros2_ws/nlf_s_multi.torchscript \
  --cache_dir /work/hf_cache"

# === After manual review: metrics ===
ssh orin "docker exec teresa_gpu python3 /ros2_ws/scripts/compute_exposure_metrics.py \
  --predictions /work/exposure/exp_01/predictions.json"

# === Copy photos to Mac for review ===
scp -r orin:/ssd/andrea_deploy/exposure/exp_01 ~/Desktop/
```

---

## Step-by-Step

### 1. Prepare the mannequin

Place the mannequin supine on a table. Apply fake wounds in known positions:

| ID | Type | Location | Notes |
|----|------|----------|-------|
| w1 | laceration | upper left chest | ~5 cm cut |
| w2 | burn | right forearm | red/brown patch |
| w3 | bruise | left thigh | dark purple |
| w4 | abrasion | right knee | scraped skin |
| ... | ... | ... | ... |

Measure the 3D position of each wound relative to a fixed reference point on the mannequin (e.g., sternum or belly button). Use a ruler or the RealSense depth viewer.

### 2. Pull latest code + start containers

**Sulla Jetson:**
```bash
cd ~/AndreaDantonaTeresa/TERESA_ws
git pull
bash teresa_start.sh
```

**Via SSH:**
```bash
ssh orin
cd ~/AndreaDantonaTeresa/TERESA_ws && git pull && bash teresa_start.sh
```

### 3. Launch RealSense camera — TERMINAL 1 (keep it running)

> **Nota:** `teresa_core.launch.py` richiede l'Orbbec connesso. Per l'esperimento offline usiamo il nodo RealSense standalone.

```bash
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p enable_color:=true -p rgb_camera.color_profile:=640x480x30 \
  -p depth_module.depth_profile:=640x480x30 -p enable_depth:=true \
  -p align_depth.enable:=true -p pointcloud.enable:=false -p enable_infra:=false
```

Aspetta ~5 secondi. Per verificare che i topic siano attivi:

```bash
# In un altro terminale
docker exec teresa_gpu bash -c "source /opt/ros/humble/install/setup.bash && ros2 topic list | grep camera/color/image_raw"
```

Output atteso:
```
/camera/camera/color/image_raw
```

### 4. Preview RealSense — TERMINAL 2 (optional)

**Sulla Jetson:**
```bash
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
rqt_image_view /camera/camera/color/image_raw
```

**Via SSH (serve XQuartz sul Mac):**
```bash
ssh -X orin
docker exec -it teresa_core bash
source /ros2_ws/install/setup.bash
rqt_image_view /camera/camera/color/image_raw
```

### 5. Capture photos — TERMINAL 3

**Sulla Jetson:**
```bash
docker exec -it teresa_gpu bash
mkdir -p /work/exposure/exp_01
python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
```

**Via SSH:**
```bash
ssh orin
docker exec -it teresa_gpu bash
mkdir -p /work/exposure/exp_01
python /ros2_ws/scripts/capture_exposure.py --out_dir /work/exposure/exp_01
```

Capture sequence:
1. **Wide shot**: hold RealSense at Orbbec height (~0.5 m from ground) pointing at mannequin. Press `w` + ENTER.
2. **Close-up 1**: hold RealSense above the head/torso. Press ENTER.
3. **Close-up 2**: above the left arm. Press ENTER.
4. **Close-up 3**: above the right arm. Press ENTER.
5. **Close-up 4**: above the legs. Press ENTER.
6. **Close-up 5**: above the feet. Press ENTER (optional).
7. Press `q` + ENTER to quit.

### 6. Run NLF + GroundingDINO

```bash
docker exec teresa_gpu python3 /ros2_ws/scripts/run_exposure_offline.py \
  --exp_dir /work/exposure/exp_01 \
  --nlf_model /ros2_ws/nlf_s_multi.torchscript \
  --cache_dir /work/hf_cache
```

Output files:
- `nlf_keypoints.json` — 24 SMPL joints
- `wide_overlay.jpg` — wide shot with GDINO boxes
- `close_up/overlays/` — each close-up with detection IDs (D0, D1, …)
- `predictions.json` — all detections with 3D positions

### 7. Manual review

Open `close_up/overlays/` and `wide_overlay.jpg`. Edit `predictions.json`:

```json
{
  "detections": [
    {
      "id": 0,
      "box": [212, 156, 278, 201],
      "score": 0.87,
      "label": "laceration",
      "source": "02_color.png",
      "position_3d": [0.12, -0.05, 0.68],

      "verified": "tp",
      "wound_id": "w1"
    },
    {
      "id": 1,
      "box": [312, 200, 350, 240],
      "score": 0.45,
      "label": "bruise",
      "source": "02_color.png",
      "position_3d": [0.15, 0.10, 0.72],

      "verified": "fp"
    }
  ],

  "ground_truth": [
    {"id": "w1", "type": "laceration", "position_mm": [120, -45, 680], "notes": "left upper chest"},
    {"id": "w2", "type": "burn", "position_mm": [-80, 200, 550], "notes": "right forearm"},
    {"id": "w3", "type": "bruise", "position_mm": [60, 350, 400], "notes": "left thigh"}
  ],

  "missed": ["w3"]
}
```

Rules:
- `verified: "tp"` = real wound correctly detected
- `verified: "fp"` = false alarm
- `wound_id` = which real wound this detection belongs to (same ID = same wound seen in multiple photos)
- `missed` = wound IDs placed but NOT detected by GDINO (false negatives)

### 8. Compute metrics

```bash
docker exec teresa_gpu python3 /ros2_ws/scripts/compute_exposure_metrics.py \
  --predictions /work/exposure/exp_01/predictions.json
```

Output: recall, FP/scan, precision, F1, 3D localisation error, LaTeX table row.

### 9. Repeat for 5 experiments

Change wound positions, create new folders:

```
/work/exposure/exp_01/   ← wounds on torso + arms
/work/exposure/exp_02/   ← wounds on legs + feet
/work/exposure/exp_03/   ← various burn types
/work/exposure/exp_04/   ← covered wounds (bandages)
/work/exposure/exp_05/   ← mixed injuries
```

Repeat steps 5-8 for each.

### 10. Aggregate results

Average the metrics across all 5 experiments to fill the paper table:

| Metric | exp_01 | exp_02 | exp_03 | exp_04 | exp_05 | Mean ± Std |
|--------|--------|--------|--------|--------|--------|------------|
| Scan points | 6 | 6 | 5 | 6 | 5 | 5.6 ± 0.5 |
| Scan duration (s) | 12 | 12 | 10 | 12 | 10 | 11.2 ± 1.1 |
| Wound recall (%) | 75 | 80 | 100 | 60 | 80 | 79.0 ± 14.3 |
| FP per scan | 0.8 | 0.5 | 1.0 | 0.7 | 0.6 | 0.7 ± 0.2 |
| 3D loc error (mm) | 28 | 35 | 22 | 30 | 25 | 28.0 ± 5.1 |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `No Orbbec frame yet` | Check Orbbec USB + 12V power. Relaunch `teresa_core.launch.py`. |
| `No RealSense frame yet` | Check RealSense USB. `rs-enumerate-devices` in teresa_core. |
| `No depth frame` | RealSense D415 needs stereo module enabled: `ros2 param set /camera/camera enable_depth true`. |
| GDINO OOM | Reduce photo resolution or use `--device cpu` (slow but works). |
| NLF timeout | The Jetson workaround is applied. If still hangs, use `--skip_nlf`. |
| X11 not available | The script auto-detects and falls back to TTY mode (terminal keys). |

---

## Scripts Reference

| Script | Purpose | Run in |
|--------|---------|--------|
| `scripts/capture_exposure.py` | Capture RGB-D photos from RealSense + Orbbec | `teresa_gpu` |
| `scripts/run_exposure_offline.py` | NLF + GDINO batch inference | `teresa_gpu` |
| `scripts/compute_exposure_metrics.py` | Compute metrics from reviewed predictions | `teresa_gpu` |
| `teresa_start.sh` | Start/stop Docker containers | Jetson host |
