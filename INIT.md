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

- **Ricerca adattiva coarse + refinement**: SEARCHING riscritto. Spot ruota con `cmd_vel.angular.z` P-control (6 posizioni coarse ×60° = 360°, dwell 5s). Durante il dwell, se una camera vede qualcosa (RealSense tracker `GUIDING` o Orbbec conf ≥ 0.30) → **refinement**: sweep pitch [0°,5°,10°] (dwell 4s), traccia best Orbbec conf. `best_conf ≥ 0.70` → LOCKING, altrimenti prossimo yaw.
- **GUIDING strict**: `guidance_min_conf` 0.3→0.5, minimo keypoint 1→2. Riduce falsi positivi.
- **SEMI_LOCKING**: Pitch flush via `Twist()` quando solo pitch non OK. QP LOOKAT subito attivo (`_end_search(re_enable=True)`).
- **PRE_APPROACH**: LOOKAT verso `/laying_human/body_center` (torso centroid). Soglia ESTIMATING/LOCKED ×3 tick. `ik_done` gate per transizione da LOCKING. Home lock Z=0.60.
- **APPROACHING**: Griglia Cartesiana adattiva (2 pose se 4 keypoint conf≥0.6, 4 con HOME transit altrimenti). Advance X=0.10m. `_do_set_state(APPROACHING)` pulizia stato. Timeout 60s → IDLE.
- **SCANNING**: 📝 Da analizzare.
- **9 stati FSM**: WAITING_TF → IDLE → SEARCHING → SEMI_LOCKING → LOCKING → PRE_APPROACH → APPROACHING → SCANNING ↔ WS_EXT.
- **Paper aggiornato**: abstract/introduction senza numeri, active_perception (hybrid lock + adaptive grid), system_architecture (9 stati, frame tree colorato). Sezioni TODO: experiments, results, conclusion.
- **Web Control Panel**: interfaccia web (`web/teresa_control.html`) con pulsanti, stato WBC, navigazione RETURN via P-controller JS.
- **Step mode debugging**: `step_mode:=true` blocca le transizioni FSM automatiche.
- **WBC spento** immediatamente all'ingresso in LOCKING.

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
