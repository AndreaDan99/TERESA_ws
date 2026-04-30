# PLAN.md — TERESA Project Roadmap & Test Guide

---

## Quick Test Procedures

### Prerequisites
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Z1 Standalone (no Spot)
```bash
# T1: Hardware + RealSense + YOLO
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# T2: Perception
ros2 launch z1_vision z1_perception.launch.py

# T3: Control (FSM, homing in 5s → body_scan → FAST)
ros2 launch z1_vision z1_control.launch.py
```

### WBC Dry-Run (all code runs, nothing moves)
Prerequisites: Spot in **sit**, `spot_ros2` on SpotCore.

```bash
# T1: Orbbec perception
ros2 launch spot_perception spot_perception.launch.py test_mode:=true

# T2: Z1 hardware + RealSense
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# T3: Z1 perception
ros2 launch z1_vision z1_perception.launch.py

# T4: Z1 control (FSM, body scan gated on WBC=SCANNING)
ros2 launch z1_vision z1_control.launch.py

# T5: WBC dry-run (debug topics only)
ros2 launch spot_control wbc.launch.py dry_run:=true
```

### WBC Full (real Spot movement)
Prerequisites: Spot in **sit**, `spot_ros2` on SpotCore.

```bash
# T1–T4: same as dry-run

# T5: WBC live (arm + Spot move)
ros2 launch spot_control wbc.launch.py
```

### Build
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select z1_vision spot_control spot_perception
```

### Lint / Test
```bash
colcon test --packages-select z1_vision spot_control spot_perception
colcon test-result --verbose
```

---

## Recent Changes (30 Apr 2026)

### WBC-as-Master handoff (simplified FSM)

**Before:** Z1 FSM started body scan autonomously; WBC waited in HANDOFF for Z1 to enter APPROACHING. Race condition + deadlock risk.

**After:** WBC is the master controller. When Spot reaches approach point, WBC transitions directly `APPROACHING → SCANNING`, disables WBC control, and signals Z1 FSM to begin body scan.

**Files changed:**
- `src/z1_vision/z1_vision/z1_FSM.py` — WAITING gate: `_wbc_state_str == 'SCANNING'` before BODY_SCANNING
- `src/spot_control/spot_control/wbc_coordinator.py` — removed HANDOFF state, direct APPROACHING→SCANNING

**New flow:**
```
WBC: IDLE → APPROACHING (arm look-at + Spot navigation)
       → SCANNING  (Spot reached, WBC disables, body height adjusted)
       → WS_EXTENSION (Z1 can't reach → Spot micro-step)
       
Z1:  HOMING → WAITING (gate: wait for WBC=SCANNING or standalone)
       → BODY_SCANNING → CHECKING_WORKSPACE → APPROACHING → FAST
```

| State | Before | After |
|-------|--------|-------|
| WAITING → BODY_SCAN | Immediately (autonomous) | Gate on WBC=SCANNING (or standalone) |
| APPROACHING → ??? | → HANDOFF → SCANNING (waited for Z1) | → SCANNING (WBC decides) |
| HANDOFF | WBC wait state | **Removed** |
| Z1 FSM ↔ WBC | Independent FSMs | WBC master, Z1 waits for signal |

---

## Planned: WBC Pitch + Height Integration

### Current limitation
WBC `J_base` is **6×2**: only `[vx, wz]` — forward velocity + yaw. Spot cannot use pitch (forward tilt) or body height (squat/stand) as part of the holistic controller.

### Proposal: extend `J_base` to **6×4**

| New DOF | Spot command | Effect on EE |
|---------|-------------|--------------|
| `vy` (pitch) | `cmd_vel.angular.y` | Pitch forward → EE lowers; pitch backward → EE raises |
| `vz` (height) | `cmd_vel.linear.z` | Squat → EE lowers; stand → EE raises |

New solver: 6 equations × (6 arm + 4 base) = **10 DOF**.

### Challenge: pitch and height are Z-redundant
Both `vy` and `vz` affect the EE **vertical** position. Without a second task, the solver mixes them arbitrarily.

Two strategies:
1. **Add secondary tasks**: like yaw, add rows for preferred pitch (keep Spot level) and preferred height (keep Spot at nominal squat), weighted low so arm takes priority when well-conditioned
2. **Mutual exclusion**: prefer height adjustment first (more stable), use pitch only when height alone isn't enough

### Files to touch
- `src/spot_control/spot_control/wbc_math.py` — extend `compute_j_base` to 6×4, extend `W` to 10×10, add 2 secondary task rows (→ 8×10)
- `src/spot_control/spot_control/wbc_qp_controller.py` — add `vy`/`vz` to cmd_vel, new ROS params for pitch/height weights
- `src/spot_control/spot_control/wbc_coordinator.py` — desired pitch/height, new WS_EXTENSION limits for pitch/height
- `src/spot_control/config/wbc_params.yaml` — new params: `pitch_weight`, `height_weight`, `pitch_max`, `height_max`

### Next steps
1. Derive analytical `J_base` for pitch + height (kinematic chain: Spot base → body frame → Z1 base → EE)
2. Implement and test in dry-run first
3. Add safety limits: max pitch angle, max height delta, per-direction bounding box in WS_EXTENSION

---

## Notes

- WS_EXTENSION bounding box: forward 0.20 m, lateral 0.20 m, backward 0.50 m — anchored at WS_EXTENSION entry
- `wbc_coordinator.py` has a dormant `_cb_z1_state` subscription (no-op after HANDOFF removal) — kept for future monitoring
- The `_cb_z1_state` subscription can be removed or repurposed later
