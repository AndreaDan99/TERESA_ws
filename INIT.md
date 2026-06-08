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

## Current State (8 June 2026)

### Web Joystick Control Panel
- Two-panel layout: left log terminal, right joystick D-pad (↑↓←→)
- Drive mode: arrow buttons for cmd_vel (forward/back/strafe)
- Body mode: ↑↓ for height (±5cm), ←→ for pitch (±5°)
- Speed +/- buttons (0.1 to 1.0 m/s)
- Height slider (0-20cm, 5cm steps) and pitch slider (0°-15°, 5° steps)
- 🏠 HOME and 🔌 PARK buttons for arm presets
- STOP now also disables IK (stops arm)
- Joystick auto-disabled during autonomous WBC mission

### SEARCHING — Timed Open-Loop 7-Pose Search
- 7 hardcoded manual poses (3 forward + 4 look-behind from FK reader)
- Spot: ±30° yaw rotation (timed open-loop, no TF needed)
- Sequential: rotate → wait for 7 arm poses → rotate other way → HOME → step 20cm forward → repeat
- Rotation speed: 0.2 rad/s (gentle). Each 30° step = ~2.6s
- Arm speed: max_joint_vel 0.4 rad/s
- Dwell replaced by ik_done count (wait for all 7 poses, not timer)
- `_search_ik_done_count` tracks per-position completions

### Perception Pipeline — YOLO Default, NLF Idle
- Default perception_backend changed from `nlf` to `yolo` (40 FPS vs 2.5 FPS)
- YOLO on both cameras during SEARCHING
- NLF skeleton always runs but starts in paused mode (`_streaming_paused = True`)
- NLF triggered at LOCKING with 3s delay for model loading
- NLF publishes to `/exposure/nlf_prior` (different topic, no conflict with YOLO)
- Both YOLO and NLF can coexist without conflicts

### Other Fixes
- Removed dead `/torso_sm_state` subscription from both trackers
- Fixed missing closing brace in `camera_view.html` drawRealSenseOverlay
- LOCKING home now uses search pose 1 instead of raised Z position

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
