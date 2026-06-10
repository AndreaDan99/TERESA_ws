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
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 10 June 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (exposure, injury detection, refactoring) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (10 June 2026)

### WBC — LOCKING Deadlock Fixed
- NLF trigger/timeout now runs at top of `_tick_locking()` — no longer skipped by early return
- Lock home sent only once (prev guard on LOCKING state change)
- Throttled debug logs show what's blocking LOCKING → PRE_APPROACH
- QP `/wbc/state` debug log only fires on actual state change (no 10Hz spam)
- `ik_done` arrival logged in coordinator callback

### SEARCHING — 6 Symmetric Poses with 10° Downward Tilt
- 6 symmetric mathematically-generated poses (3 forward + 3 look-behind)
- Camera tilted 10° downward for better torso view
- search_timeout_per_point: 1.2s (was 5.0s)
- Orientation via `compute_ee_orientation()` — no FK-reader quaternions

### Web Dashboard
- Component status grid: IK, Orbbec, RealSense, QP with colored dots (green/yellow/gray)
- One-time event logging (no spam)
- `/wbc/qp_mode` topic for QP controller mode
- Works independently of camera panel

### Paper (TERESA_RAL)
- Bibliography: 8 fixes (gu2024vttb type, xie2024capm authors, DOIs, orphan entries removed, rozycki1996 cited)
- FSM diagram redesigned larger for readability

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
