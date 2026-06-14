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
| [`docs/CHANGELOG.md`](docs/CHANGELOG.md) | Changelog storico (6 May – 14 June 2026) |
| [`docs/DESCRIPTION.md`](docs/DESCRIPTION.md) | Architettura sistema, frame tree, FSM, build/run |
| [`docs/PLAN.md`](docs/PLAN.md) | Piano futuro (exposure, injury detection, refactoring) |
| [`web/README.md`](web/README.md) | Web control panel + camera view con YOLO overlay |

---

## Current State (14 June 2026)

### SEARCHING — Pitch-Based Design (completely redesigned)
- **NO yaw rotation** — Spot stays at current yaw throughout search
- **Pitch cycling**: Spot tilts +10° → +5° → 0° (nose down). Each pitch: 2s body pose settle → arm does 7 poses
- **7 arm search poses**: 3 forward (FWD-C, FWD-L, FWD-R) + 3 behind (BWD-L, BWD-C, BWD-R) with transit + final return
- **Forward poses**: 10° camera tilt downward (FWD-C, FWD-L). Behind poses: original quaternions, no tilt
- **After all 3 pitches**: arm HOME → Spot steps forward **50cm** (was 20cm) → repeat
- **Search positions**: `[{yaw:0, pitch:+10°}, {yaw:0, pitch:+5°}, {yaw:0, pitch:0°}]`
- TF `odom→body` no longer needed for search rotation

### SEMI_LOCKING Improvements
- **RealSense gate**: dwell starts ONLY if RealSense still sees person (GUIDING/ESTIMATING/LOCKED). If lost → skip dwell, return to SEARCHING
- Same check at settle timeout
- **Yaw restoration**: after failed semi-lock, Spot returns to original yaw via body_pose
- **Orbbec dwell**: 3s → 5s
- **Cooldown**: 3 ticks (0.3s) after SEARCHING entry before semi-lock can fire; prevents immediate re-trigger loop

### TF Fixes
- `_tf_lookup()` uses `rclpy.time.Time()` instead of `get_clock().now()` to avoid extrapolation errors with DDS latency
- Timeout: 1s → 10s

### NLF Changes
- NLF skeleton **always launched** but model loads **lazily** only on `/nlf/trigger` — near-zero CPU until LOCKING
- YOLO remains default perception backend

### New / Changed Features
- `/wbc/perception_enable` publisher (transient_local QoS) — enables posture classifier and torso tracker on SEARCHING entry, disables on IDLE
- **spot_control gating**: navigator disabled in WAITING_TF and SEARCHING, re-enabled in IDLE
- **`Twist()` after every `body_pose`** as workaround for spot_ros2 actuation bug
- **Dead code removed**: `wbc_approach_scanner.py` (deprecated), `test_legacy/` directory, `SEARCH_HOME_POS`/`SEARCH_HOME_ORI` constants, `_pub_debug_marker()` stub

### Coordinator FSM Changes
- Refinement mode (pitch sweep) REMOVED — pitch is now part of main search cycle
- Semi-lock gated by `_search_position_start is None and not _search_settling`

### Modified files
| File | +/- | Changes |
|------|-----|---------|
| `wbc_coordinator.py` | ~+200/−150 | Pitch-based search, semi-lock RealSense gate, yaw restore, TF fix, perception_enable, spot_control gating, dead code removal |
| `wbc_qp_controller.py` | ~+50/−30 | 7 search poses (3 forward 10° tilt + 3 behind + return), ACTIVE_SEARCH mode updated |
| `nlf_skeleton.py` | ~+20/−5 | Lazy model loading on `/nlf/trigger`, always launched |
| `wbc_params.yaml` | ~+10/−5 | Search params updated (pitch angles, step 0.50m, semi-lock cooldown) |
| `spot_perception.launch.py` | ~+2/−1 | NLF always launched (no condition) |
| `wbc_approach_scanner.py` | −40 | Removed (deprecated) |
| `test_legacy/` | −all | Removed |

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
