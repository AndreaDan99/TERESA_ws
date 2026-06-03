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
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 2 June 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (refactoring launch, body pose optimization, paper) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (2 June 2026)

- **Ricerca adattiva coarse + refinement**: SEARCHING riscritto. Spot ruota con `cmd_vel.angular.z` P-control (6 posizioni coarse ×60° = 360°, dwell 5s). Durante il dwell, se una camera vede qualcosa (RealSense tracker `GUIDING` o Orbbec conf ≥ 0.30) → **refinement**: sweep pitch [0°,5°,10°] (dwell 4s), traccia best Orbbec conf. `best_conf ≥ 0.70` → LOCKING, altrimenti prossimo yaw. Niente più griglia fissa 18 posizioni.
- **Tracker state GUIDING**: nuovo stato del torso tracker (giallo) oltre a `IDLE/ESTIMATING/LOCKED`. In guidance mode (SEARCHING) qualsiasi keypoint → GUIDING. Triggera refinement e guida il SEMI_LOCKING.
- **Braccio QP ACTIVE_SEARCH = 3 pose**: HOME/LEFT/RIGHT, tilt fisso -15°, sweep Y ±0.28m, X +0.20m, Z=0.42m. Loop infinito. No wrist sweep.
- **Active Perception Cartesian Scanning**: `PERCEPTUAL_SCAN` (6 pose multi-angolo in APPROACHING, step 0.12m). Movimenti prevedibili, Z ≥ home (0.44m).
- **Semi-lock rilassato**: il tracker accetta `GUIDING`/`ESTIMATING` oltre a `LOCKED` — basta vedere keypoint qualsiasi per guidare Spot verso il corpo.
- **Web Control Panel**: interfaccia web (`web/teresa_control.html`) con pulsanti, stato WBC, navigazione RETURN via P-controller JS. Camera view (`web/camera_view.html`) con feed live Orbbec/RealSense + overlay scheletro YOLO.
- **Step mode debugging**: `step_mode:=true` blocca le transizioni FSM automatiche. Conferma con tasto `n` o pulsante STEP.
- **WBC spento** immediatamente all'ingresso in LOCKING. PRE_APPROACH con timeout 5s.
- **QP Controller — 3 modalità**: ACTIVE_SEARCH (3 pose), LOOKAT, PERCEPTUAL_SCAN (6 pose). Arm-only.
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
