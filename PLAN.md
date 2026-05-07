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

## Planned: Early Body Scan during WBC APPROACHING

### Idea
Il braccio esegue il body scan **mentre** Spot copre gli ultimi ~65 cm di avvicinamento.
Quando Spot arriva a 5 cm, il centro torso è già noto → si salta la fase BODY_SCANNING
dopo l'handoff. Guadagno: 5-10 secondi risparmiati.

### Flusso
```
t=0:  dist=2m     → WBC APPROACHING, braccio look-at
t=5s: dist=0.7m   → scan_distance raggiunta!
                    QP sospende ik_enable → MUX lascia passare FSM
                    FSM esegue body scan (phase 1→2→3)
                    Spot continua a ricevere cmd_vel (QP non si ferma)
t=11s: scan done  → QP riprende ik_enable
t=13s: dist=5cm   → handoff! FSM va diretto a CHECKING_WORKSPACE
```

### File da toccare
- `wbc_coordinator.py` — param `scan_distance`, topic `/wbc/publish_arm`
- `wbc_qp_controller.py` — flag `_publish_arm`, smette di pubblicare ik_enable quando False
- `z1_FSM.py` — segnale `early_scan` in WAITING, skip body scan dopo handoff
- `wbc_params.yaml` — `scan_distance: 0.7`

### Sfide
- Coordinazione FSM↔coordinator: chi segnala inizio/fine scan
- Fallback se scan fallisce → comportamento attuale (body scan dopo handoff)
- Timing stretto: body scan ~6-8s, Spot percorre 0.65m a ~0.1 m/s → 6-7s
- Se Spot arriva a 5cm prima che scan finisca → attendere o forzare handoff?

### Pre-requisito
Validare prima le modifiche WBC attuali (goal in odom, 10 Hz, look-at stabile, QualityMonitor).

---

## Planned: WBC Pitch + Height Integration

### Current limitation
WBC `J_base` is **6×2**: only `[vx, wz]` — forward velocity + yaw. Spot cannot use pitch (forward tilt) or body height (squat/stand) as part of the holistic controller.

### Strategy (updated 5 May 2026)

**Phase 1 — Pitch as discrete compensation** (current focus):
- Pitch is NOT integrated into the continuous WBC Jacobian (6×4)
- Instead, pitch is a discrete command (like `SetStandHeight` for body height)
- Applied during WS_EXTENSION: when arm can't reach target, Spot tilts forward to shift workspace
- Delivery mechanism: Spot API service (to be verified via `ros2 service list | grep my_spot`). Candidates: `/my_spot/robot_command` or a new `SetBodyPitch.srv` in `spot_msgs`. NOT `cmd_vel.angular.y` (likely not processed by standard `spot_driver`).

**Phase 2 — Full WBC 6×4** (future, after Phase 1 validated):
| New DOF | Spot command | Effect on EE |
|---------|-------------|--------------|
| `vy` (pitch) | discrete service or `cmd_vel.angular.y` | Pitch forward → EE lowers + forward; pitch backward → EE raises |
| `vz` (height) | `cmd_vel.linear.z` or `SetStandHeight` | Squat → EE lowers; stand → EE raises |

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
1. Test WBC without pitch — verify current vx+wz+body_height is sufficient
2. Verify Spot pitch API: `ros2 service list | grep my_spot` on SpotCore
3. Implement Phase 1: discrete pitch compensation in `wbc_coordinator.py` + new service if needed
4. Derive analytical `J_base` for pitch + height (kinematic chain: Spot base → body frame → Z1 base → EE)
5. Implement and test in dry-run first
6. Add safety limits: max pitch angle, max height delta, per-direction bounding box in WS_EXTENSION

---

## Notes

- WS_EXTENSION bounding box: forward 0.20 m, lateral 0.20 m, backward 0.50 m — anchored at WS_EXTENSION entry
- `wbc_coordinator.py` has a dormant `_cb_z1_state` subscription (no-op after HANDOFF removal) — kept for future monitoring
- Target paziente fissato in odom (media prime 3 misure) — mai più ricambiato durante APPROACHING
- QualityMonitor: qualità = `max_q * (1 - posture_confidence)` + crescita lineare senza misure
