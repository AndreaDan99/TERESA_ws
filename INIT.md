# INIT.md

This file provides guidance to Claude Code when working with this repository.

## Startup — always do this first

```bash
# Check latest commits to know what changed last
git log -3 --format="%h %s (%an, %ad)"

# Check working tree status
git status --short
```

---

## Quick Reference

| Document | Contents |
|----------|----------|
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 6 June 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (exposure, injury detection, refactoring) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (9 June 2026)

### NLF Burst Streaming
- NLF trigger redesigned from one-shot to multi-frame burst
- Collects 2 valid detections (lying + torso non-NaN), EMA accumulation, timeout 30s
- Publishes refined prior on `/exposure/nlf_prior` and confidence on `/exposure/nlf_confidence`
- LOCKING blocks until NLF burst completes (or times out)
- EXCELLENT confidence tier: if NLF bbox_score ≥ 0.80 → 100% NLF blending

### SEARCHING — Timed Open-Loop with 6 Symmetric Poses
- 6 symmetric mathematically-generated poses (3 forward + 3 look-behind)
- Orientation computed via `compute_ee_orientation()` — no FK-reader quaternions
- Refinement best pitch saved and applied on ALL LOCKING entry paths

### Search Poses
- 6 symmetric mathematically-generated poses (3 forward + 3 look-behind)
- Orientation computed via compute_ee_orientation() — no FK-reader quaternions

### Exposure NLF Grid
- exposure_scanner uses NLF prior for body grid when available
- Falls back to YOLO keypoints if NLF prior not captured

### Z1 WBC Dependency
- Z1 homes on startup, then waits indefinitely for WBC coordinator
- No standalone operation possible

### Launch Fix
- `nlf_skeleton_node` only launches with `perception_backend:=nlf`
- With `perception_backend:=yolo` (default): NLF not started at all

### Other Fixes
- Publish suppression: `/human_pose/points_3d` blocked during active NLF burst
- Coordinator no longer publishes `Bool(False)` on NLF timeout (NLF self-manages)
- `nlf_timeout` extended from 10s to 30s

---

## Quick Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select z1_vision spot_control spot_perception teresa_demo
source install/setup.bash
```

---

## Running

See [`DESCRIPTION.md`](DESCRIPTION.md) for the full flow (5 terminals).

```
T1: ros2 launch spot_control teresa_core.launch.py         # driver + TF + monitor
T2: ros2 launch spot_control teresa_perception.launch.py   # Orbbec + RealSense
T3: ros2 launch z1_vision z1_control.launch.py use_impedance:=false  # IK + FSM
T4: ros2 launch spot_control wbc.launch.py                 # WBC + navigator + scanner + exposure
T5: ros2 run spot_control wbc_keyboard_node                # tastiera
T6: ros2 run spot_control experiment_logger                # logger metriche (opzionale)
```

---

## Linting / Tests

```bash
colcon test --packages-select z1_vision spot_control spot_perception
colcon test-result --verbose
```
