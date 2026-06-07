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

## Current State (7 June 2026)

### NLF Prior at LOCKING — single-frame prior with binary fallback

- **NLF single-frame prior** triggered at LOCKING, 10s timeout
- **Binary fallback**: if NLF fails → entire system = 6 June 2026 behavior (YOLO-only)
- **Gate**: `_nlf_prior_valid()` controls all branches (PRE_APPROACH, APPROACHING, LOOKAT)
- **PRE_APPROACH**: 1s safety gate with NLF, legacy sliding window without
- **APPROACHING**: unified 6-pose grid centered on torso, tight offsets with NLF, wide with YOLO
- **LOOKAT**: blended NLF(70%)+YOLO(30%) when HIGH coherence, YOLO 100% when LOW
- **CPU saving**: NLF streaming paused after prior capture
- **24 pytest tests**, 3 new test files
- Files: `nlf_skeleton.py` (+76), `wbc_coordinator.py` (+217), `wbc_qp_controller.py` (+44), `wbc_params.yaml` (+7), `body_search_params.yaml` (+7)

### Exposure Body Scanning — full-body + simultaneous skeleton refinement

- **exposure_scanner.py** (650 righe, riscritto): full-body grid 14 punti su 7 regioni (HEAD, TORSO, L/R ARM, L/R LEG, FEET). Look-at dinamico EE X verso corpo. Standoff orizzontale 0.50m. TF Orbbec→world. Running-average scheletro raffinato su `/exposure/refined_skeleton`. Accumulo keypoint RealSense da `/exposure/body_keypoints` durante dwell. JSON output. Head stima da spalle se naso occluso.
- **z1_yolo_torso_tracker.py**: publisher `/exposure/body_keypoints` (PoseArray, 17 kp COCO in scan mode). Nuovo metodo `_extract_all_body_keypoints`.
- **wbc_coordinator.py**: `_cb_next_point` esteso a EXPOSURE_SCANNING, `_apply_exposure_body_pose`. PRE_APPROACH: Z offset +0.40m su fallback goal, sliding window ≥1/5 ESTIMATING/LOCKED tick.
- **exposure_snapshot.py** (nuovo, 128 righe): snapshot RealSense in EXPOSURE_REVIEW. Trigger `/exposure/goto_point` + `/ik_done`, delay 1s, pubblica `/exposure/snapshot`, salva JPEG.
- **Web UI**: Grid toggle + legenda 7 colori, click-to-revisit, Body Map (🗺 / tasto `m`), snapshot freeze + badge "📸" + Close button, gate toggle sempre visibile.
- **Grid generation**: i keypoint Orbbec vengono pubblicati già dall'inizio (SEARCHING) su `/human_pose/points_3d`, quindi quando si entra in EXPOSURE_SCANNING il buffer `_keypoints` è già popolato. Stima per keypoint mancanti (es. head da spalle).

### FSM States (11 total)
- `EXPOSURE_SCANNING`: body scan full-body, per-point Spot reconfiguration
- `EXPOSURE_REVIEW`: click-to-revisit interattivo
- `WAITING_EXPOSURE`: manual gate
- `WAITING_FAST`: manual gate
- 7 stati esistenti invariati

### Web Interface
- **teresa_control.html**: Grid toggle su RealSense con legenda colori (7 regioni). Click su marker → `/exposure/goto_point`. Body Map panel (🗺 o tasto `m`): canvas top-down con scheletro progressivo (17 kp + linee COCO) + griglia exposure.
- **camera_view.html** invariato

### Nuovi topic
| Topic | Publisher | Subscriber | Type |
|-------|-----------|------------|------|
| `/exposure/body_keypoints` | z1_yolo_torso_tracker | exposure_scanner | PoseArray (17 kp) |
| `/exposure/refined_skeleton` | exposure_scanner | web UI (Body Map) | PoseArray (17 kp, running avg) |

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
