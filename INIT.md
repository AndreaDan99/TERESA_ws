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
| [`CHANGELOG.md`](CHANGELOG.md) | Changelog storico (6 May – 24 May 2026) |
| [`DESCRIPTION.md`](DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`PLAN.md`](PLAN.md) | Piano futuro (refactoring launch, body pose optimization, paper) |

---

## Current State (26 May 2026)

- **QP Controller refactored: arm-only WBC**. LOOKAT mode in PRE_APPROACH (ω_des + null-space joint centering). SCAN_SEQ mode in APPROACHING (genera 11 pose dal null-space del look-at, le sequenzia con BodySearchScanner, fonde stime, pubblica FAST points).
- **Spot P-controller rimosso dal QP**. Spot mosso solo dal navigatore (rotate → drive → stop) e dal coordinator (body pose).
- **wbc_approach_scanner deprecato** (stub). Tutta la logica di scansione nel QP controller.
- **wbc_math.py**: nuove `damped_pinv()` e `null_space_projector()`. Vecchie `wbc_split` spostate in fondo come deprecated.
- **wbc.launch.py**: lancia solo 3 nodi (QP controller + coordinator + navigator). Rimosso scanner, rimossi z1_mount args.
- **Paper reframing**: Whole-Body Active Perception for Emergency Assessment. Sezioni I–VI scritte.

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
