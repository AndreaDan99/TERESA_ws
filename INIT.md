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
| [`docs/CHANGELOG.md`](docs/CHANGELOG.md) | Changelog storico (6 May – 10 June 2026) |
| [`docs/DESCRIPTION.md`](docs/DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`docs/PLAN.md`](docs/PLAN.md) | Piano futuro (exposure, injury detection, refactoring) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (12 June 2026)

### Body Pose Optimizer Node
- **Nuovo nodo**: `body_pose_optimizer.py` (~600 righe). Ottimizzazione 2D (h,p), 3D (dy_body,h,p), 4D (dx_body,dy_body,h,p) con retry loop IK-driven
- **Retry IK-driven**: 2D→3D→4D basato su `/ik_done` timeout (2s), non su distanza euristica
- **FAST + Exposure**: entrambi i path WBC ora usano lo stesso optimizer
- **Topic interface**: `~/optimize_request` (PoseArray) → `~/optimize_result` (PoseArray)

### Y-Walking Simulation
- `test_exposure_poses.py`: Spot cammina lungo Y per ogni punto (3D grid search spot_y×h×p, 600 combo)
- Corpo virtuale a grandezza 1.0 (1.70m reale) in frame odom, non più link00
- Navigazione cmd_vel.linear.y + TF feedback chiuso, NavState machine, safety guards

### Patient Body TF
- **Nuova TF**: `my_spot/odom` → `patient_body` pubblicata da `laying_human_detector.py`
- Body frame da keypoint detector: X=attraverso corpo, Y=testa→piedi, Z=UP
- `body_pose_optimizer` usa TF per convertire dy_body/dx_body in odom
- `wbc_coordinator` usa TF per yaw corpo e approccio (sostituisce 3 subscriber)

### WBC Refactoring
- **Rimossi 11 metodi**: `_optimize_body_poses`, `_optimize_exposure_body_poses`, `_optimize_ws_extension`, `_drive_ws_ext_position`, `_tick_ws_ext_drive`, `_simulate_link00`, `_link00_to_odom_vec`, `_odom_to_link00_vec`, `_apply_fast_body_pose`, `_apply_exposure_body_pose`
- **-340 righe nette**, FSM preservato
- **Bug fix**: `_navigator_timeout` assegnato (era dichiarato ma mai inizializzato)
- Sostituiti 3 subscriber (`/approach_point`, `/body_axis`, `/body_center`) con TF lookup

### Modified files
| File | +/- | Changes |
|------|-----|---------|
| `body_pose_optimizer.py` | +600 | Nuovo nodo: 2D/3D/4D + retry + TF |
| `wbc_coordinator.py` | -340 | Rimosse ottimizzazioni interne, integrato optimizer, TF lookup |
| `laying_human_detector.py` | +77 | Aggiunto TransformBroadcaster, pubblica patient_body TF |
| `test_exposure_poses.py` | +190 | Y-walking 3D, corpo odom, NavState, safety guards |
| `setup.py` | +1 | Entry point body_pose_optimizer |

---

## Quick Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select z1_vision spot_control spot_perception teresa_demo
source install/setup.bash
```

---

## Running

See [`docs/DESCRIPTION.md`](docs/DESCRIPTION.md) for the full flow (5 terminals).

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
