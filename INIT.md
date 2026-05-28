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

## Current State (26 May 2026)

- **Web Control Panel**: interfaccia web (`web/teresa_control.html`) con pulsanti, stato WBC, navigazione RETURN via P-controller JS. Camera view (`web/camera_view.html`) con feed live Orbbec/RealSense + overlay scheletro YOLO. Comunicazione via rosbridge WebSocket, nessuna connessione diretta a Spot. Vedi [`web/README.md`](web/README.md).
- **Step mode debugging**: `step_mode:=true` blocca le transizioni FSM automatiche. Conferma con tasto `n` o pulsante STEP.
- **Ricerca ibrida 360°**: Spot + braccio coordinati in SEARCHING. 18 posizioni Spot (6 yaw × 3 pitch, 360° completi). Braccio esplora con 7 pose QP-based in loop (SEARCH_GRID mode, δ=0.15, safe joint limits). Lock ibrido: Orbbec diretto o RealSense semi-lock (guida Spot, braccio congelato, 3s finestra). Lock confidence: 70%.
- **Nuovi stati FSM**: SEMI_LOCKING (braccio in pausa, Orbbec cerca) e LOCKING (braccio in home, 5 campioni in parallelo, tolleranza 1s assenza Orbbec, rientro senza azzerare posizione).
- **FSM a 10 Hz** (era 5 Hz). SEMI_LOCKING: early exit se RealSense perde torso.
- **QP Controller — 3 modalità**: SEARCH_GRID, LOOKAT, SCAN_SEQ. Arm-only, Spot mai controllato dal QP.
- **Spot P-controller rimosso dal QP**. Spot mosso solo da navigatore e coordinator.
- **wbc_approach_scanner deprecato** (stub). Tutta la logica nel QP controller.
- **wbc_math.py**: `damped_pinv()`, `null_space_projector()`. Vecchie `wbc_split` deprecated.
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
