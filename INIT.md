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
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 5 June 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (exposure, injury detection, refactoring) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (5 June 2026)

### Exposure Body Scanning (NEW)
- **exposure_scanner.py**: nodo dedicato per body scan con camera RealSense
- Griglia punti adattiva dai keypoint COCO sul corpo del paziente
- Pattern per-punto: body_pose(h,p) → settle → IK goal → ik_done → dwell 2s
- Salvataggio IK goals per replay durante review interattiva
- Pubblica `/exposure/grid_markers` (MarkerArray) per overlay web
- Pubblica `/exposure/ready` al completamento
- Subscriber `/exposure/goto_point` per re-inspection click-to-revisit

### FSM States (13 total)
- `EXPOSURE_SCANNING`: body scan automatico con camera
- `EXPOSURE_REVIEW`: fase interattiva, click su punti griglia per re-inspect
- `WAITING_EXPOSURE`: manual gate prima dell'exposure scan
- `WAITING_FAST`: manual gate prima del FAST ultrasound
- 9 stati esistenti invariati

### Manual Scan Gate
- Parametro `manual_scan_gate` (default true)
- Quando true: missione in pausa a WAITING_EXPOSURE e WAITING_FAST
- Conferma via tasto `n` (keyboard) o pulsante web UI
- Toggle MANUAL/AUTO in `teresa_control.html`
- Publisher `/wbc/manual_scan_gate` + subscriber `/wbc/set_manual_scan_gate`

### Web Interface
- **camera_view.html**: overlay griglia exposure (punti blu) su RealSense
  - Click su punto → `/exposure/goto_point` → Spot torna a inquadrarlo
  - Pulsante Terminate durante EXPOSURE_REVIEW
  - Toggle `Exposure` nella barra overlay
- **teresa_control.html**: toggle MANUAL/AUTO scan gate
  - Pulsanti STEP contestuali: "▶ Expose" / "▶ FAST"

### Experiment Logger
- Traccia EXPOSURE_SCANNING, EXPOSURE_REVIEW nel timeline
- Colonna CSV: `exposure_duration_s`
- Metriche JSON: `t_exposure_start`, `t_review_start`

### Paper (TERESA_RAL/)
- 8 pagine, 42 reference, 5 figure, 0 errori compilazione
- Introduction + Related Work unificate in sezione unica
- Sezione IV.D: Exposure Body Scanning and Injury Detection
- Griglia posture-adaptive (supino 15pt, seduto 29, in piedi 49)
- Modelli wound/burn citati come footnote (non in bibliografia)
- FSM diagram: 13 stati, 4 colonne, nuovi codici colore
- System block diagram: +Injury Detection, +Web UI, +Exp. Scanner
- Fig. 4: placeholder per screenshot interfaccia web

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
