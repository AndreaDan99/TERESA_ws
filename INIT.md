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
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 28 May 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (refactoring launch, body pose optimization, paper) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (29 May 2026)

- **Active Perception Cartesian Scanning**: il QP controller genera waypoint **Cartesiani** (non più null-space SVD). `ACTIVE_SEARCH` (9 pose + sweep polso ±15° in SEARCHING) e `PERCEPTUAL_SCAN` (6 pose multi-angolo in APPROACHING). Movimenti prevedibili, Z ≥ home (0.44m). Rotazione polso combinata alla traslazione per amplificare la copertura (fino a ±26°/lato a 1m).
- **Semi-lock rilassato**: il tracker accetta `ESTIMATING` oltre a `LOCKED` — basta vedere 3+ keypoint qualsiasi per guidare Spot verso il corpo. `/torso_target_ee` pubblicato anche durante `ESTIMATING`.
- **Web Control Panel**: interfaccia web (`web/teresa_control.html`) con pulsanti, stato WBC, navigazione RETURN via P-controller JS. Camera view (`web/camera_view.html`) con feed live Orbbec/RealSense + overlay scheletro YOLO.
- **Step mode debugging**: `step_mode:=true` blocca le transizioni FSM automatiche. Conferma con tasto `n` o pulsante STEP.
- **Ricerca ibrida 360°**: Spot + braccio coordinati in SEARCHING. 18 posizioni Spot (6 yaw × 3 pitch). Braccio esplora con 9 pose Cartesiane in loop (ACTIVE_SEARCH, sweep Y ±0.20m, rotazione polso ±15°). Semi-lock: RealSense triggera da `ESTIMATING` (basta vedere keypoint, anche solo gambe). WBC spento immediatamente all'ingresso in LOCKING. PRE_APPROACH con timeout 5s.
- **QP Controller — 3 modalità**: ACTIVE_SEARCH, LOOKAT, PERCEPTUAL_SCAN. Arm-only.
- **Spot P-controller rimosso dal QP**. Spot mosso solo da navigatore e coordinator.
- **Paper reframing**: Whole-Body Active Perception for Emergency Assessment.

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
T4: ros2 launch spot_control wbc.launch.py                 # WBC + navigator + scanner
T5: ros2 run spot_control wbc_keyboard_node                # tastiera
```

---

## Linting / Tests

```bash
colcon test --packages-select z1_vision spot_control spot_perception
colcon test-result --verbose
```
