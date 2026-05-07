# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Startup — always do this first

```bash
# Check latest commits to know what changed last
git log -3 --format="%h %s (%an, %ad)"

# Check working tree status
git status --short
```

---

## Recent Changes (6 May 2026)

- **Arm twist fix (WBC)**: EE orientation now computed geometrically (X_ee toward target, Y_ee from home via Gram-Schmidt) instead of using the approach_point yaw orientation that caused a roll twist around the X axis. Same algorithm as `z1_FSM._orientation_for_xee()`.
- **Shared utilities**: new `teresa_utils` package with `orientation.py` — `compute_ee_orientation`, `quat_to_rot`, `rot_to_quat`, `normalize_angle`. Eliminates duplicate code across 4 files.
- **`workspace_safety_margin` unified**: all defaults aligned to 0.05 m (were 0.05 in YAML but 0.30 in code).
- **`REQUESTING_WS_EXT` race fixed**: FSM now proceeds to CHECKING_WORKSPACE on SCANNING even if WS_EXTENSION was missed between ticks.
- **`wbc_startup_timeout` configurable**: 30s default (was hardcoded 10s). Parameter in `z1_fsm_params.yaml`.
- **`wait_ik_timeout_s` robustness**: declared in FSM (not only via ScanManager) — no crash if `from_params()` fails.

## Recent Changes (7 May 2026)

### WBC refactoring — goal in odom, 10 Hz, stable look-at

**Before:** WBC approach broken:
- Goal in camera frame → target "scappa" con Spot, errore non cala mai
- `update_period` 1.5s → `cmd_vel` troppo rado, Spot non reagisce fluidamente
- look-at: `x_ee = clipped_pos - ee_cur` → instabile, polso oscilla a ogni ciclo
- Kalman dead zone → `sigma_max` collassa subito (2mm), inutile
- Handoff puramente a 5cm, nessun controllo qualità

**After:**
- Goal **fissato in odom** (media prime 3 misure, `_QualityMonitor`) → target fermo nel mondo
- **10 Hz** (`update_period: 0.1`) → Spot fluido come `spot_goal_navigator`
- look-at: `x_ee = target_link00 - clipped_pos` → coerente con posizione IK (stesso orizzonte temporale `q_new`)
- `compute_ee_orientation_minrot()` — rotazione minima da home X a x_ee, polso rilassato
- `ik_rot_weight: 0.7` (era 0.3) — IK rispetta l'orientazione
- `orientation_mode: "minrot"` default in `wbc_params.yaml` (fallback: `"gram_schmidt"`)

### QualityMonitor (sostituisce `_PositionKalman`)
- `target` = media prime `quality_buf_size=3` misure in odom → inizializzato
- `target` aggiornato solo se `posture_confidence > best_conf + confidence_margin` (0.10)
- `quality` = `max_q * (1 - posture_confidence)` + crescita lineare senza confidence
  - conf=0.80 → quality=0.10m, conf=0.60 → quality=0.20m
- Pubblicato su `/wbc/target_uncertainty` in **metri** (non più sigma)
- `v_scale = v_min + (1 - v_min) / (1 + quality / quality_ref)` → **mai zero**
- Spot si ferma **solo** a 5cm (handoff), quality riduce velocità ma non blocca
- Target converge sempre sulla migliore vista del paziente

### Nuovi parametri WBC
| Parametro | Valore | Significato |
|-----------|--------|-------------|
| `update_period` | 0.1s | WBC a 10 Hz |
| `quality_ref` | 0.05m | Soglia qualità per `v_scale = (1+v_min)/2` |
| `v_min` | 0.15 | Velocità minima mai zero |
| `confidence_margin` | 0.10 | Min incremento confidenza per aggiornare target |
| `quality_growth` | 0.05 m/s | Crescita qualità senza dati posture_confidence |
| `quality_min/max` | 0.01/0.50 | Floor/ceiling qualità [m] |
| `quality_buf_size` | 3 | Misure per inizializzare target |
| `orientation_mode` | "minrot" | Min-rotation quaternion (vs gram_schmidt) |

### Parametri rimossi
- `z_delta` (chance-constraint dead zone) — sostituito da velocity scaling
- `approach_kf_process_noise`, `approach_kf_meas_noise` — Kalman rimosso

### Files modificati
`wbc_coordinator.py`, `wbc_qp_controller.py`, `wbc_params.yaml`,
`teresa_utils/orientation.py`, `z1_ik_jtc_params.yaml`

---

## System overview

Two main pipelines coexist:

| Pipeline | Robot | Camera | Role |
|----------|-------|--------|------|
| **Z1 standalone** | Unitree Z1 arm | RealSense D435 | FAST ultrasound scanning |
| **Spot + Z1 (WBC)** | Boston Dynamics Spot + Z1 arm | Orbbec Femto Bolt | Spot navigates to patient, Z1 performs ultrasound |

---

## Build & Run

```bash
# Source ROS2 first (required every shell)
source /opt/ros/humble/setup.bash

# Build the full workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific packages
colcon build --packages-select z1_vision spot_control spot_perception

# Source install after build
source install/setup.bash
```

### Running Z1 standalone (three terminals)

```bash
# Terminal 1: Robot hardware + RealSense camera
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# Terminal 2: Vision pipeline
ros2 launch z1_vision z1_perception.launch.py

# Terminal 3: Control (FSM starts after 5s)
ros2 launch z1_vision z1_control.launch.py
```

### Running Spot + Z1 WBC (SpotCore + 5 PC terminals)

**Prerequisites on Spot:**
- `spot_ros2` running on SpotCore (publishes `my_spot/odom → my_spot/body` TF)
- Spot in **sit** position (ignores `/my_spot/cmd_vel` for safety)

**PC terminals:**

```bash
# 1: Orbbec driver + perception (no Spot navigation)
ros2 launch spot_perception spot_perception.launch.py test_mode:=true

# 2: Z1 hardware + RealSense + YOLO tracker
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# 3: Surface normals + signed distance
ros2 launch z1_vision z1_perception.launch.py

# 4: Z1 control (FSM + IK + impedance)
ros2 launch z1_vision z1_control.launch.py

# 5: WBC holistic controller
ros2 launch spot_control wbc.launch.py

# Optional: dry-run mode (no arm movement, debug topics only)
ros2 launch spot_control wbc.launch.py dry_run:=true
```

### Linting / tests

```bash
colcon test --packages-select z1_vision spot_control spot_perception
colcon test-result --verbose
```

---

## Architecture

### Z1 standalone pipeline

```
RealSense D435
  └─► z1_yolo_torso_tracker  (YOLO11 + Kalman 3D)
        ├─► /torso_target_ee_locked  (PoseStamped, only when LOCKED)
        └─► /torso_tracker_state

  └─► realsense_surface_node  (depth ROI → PCA plane fit)
        ├─► /torso_surface_frame   (PoseStamped, surface normal)
        └─► /surface_signed_distance (Float32)

/torso_target_ee_locked
  └─► z1_FSM  (main orchestrator)
        ├─► /ik_goal_pose + /ik_enable
        └─► /z1_fsm/state

/ik_goal_pose + /ik_enable
  └─► z1_ik_to_jtc  (Pinocchio damped pseudo-inverse IK → JTC action)
        └─► /ik_done

z1_FSM ──(Trigger srv)──► safe_controller_switch  ←──► impedance_controller_realsense
```

### Spot + Z1 WBC pipeline

```
Orbbec Femto Bolt (Jetson → PC)
  └─► yolo_skeleton_spot (YOLO11 pose)
        └─► posture_classifier (posture + confidence)
              └─► laying_human_detector
                    ├─► /laying_human/approach_point   (PoseStamped, camera frame)
                    ├─► /laying_human/body_axis         (asse testa-piedi)
                    └─► /human_pose/posture, /human_pose/posture_confidence

/laying_human/approach_point
  └─► wbc_coordinator  (FSM: IDLE → APPROACHING → SCANNING → WS_EXTENSION)
        │  Trasforma approach_point camera → odom via TF
        │  QualityMonitor: target best-confidence + quality = max_q*(1-conf)
        ├─► /wbc/ee_goal             (target fisso in odom frame)
        ├─► /wbc/enable              (True = WBC priority su MUX)
        ├─► /wbc/desired_yaw         (Spot ⊥ body_axis)
        ├─► /wbc/target_uncertainty  (quality [m], non sigma)
        └─► /wbc/state               (IDLE/APPROACHING/SCANNING/WS_EXTENSION)

/wbc/ee_goal (odom) + /wbc/enable + /wbc/desired_yaw + /wbc/target_uncertainty
  └─► wbc_qp_controller  (10 Hz, holistic WBC: arm q_dot + base vx·wz)
        │  dp = goal_odom - ee_odom  (errore cala quando Spot avanza)
        │  WBC split → q_dot, vx, wz
        │  v_scale = v_min + (1-v_min)/(1 + quality/ref)  → vx,wz ridotti
        │  x_ee = target_link00 - clipped_pos  → orientazione stabile (minrot)
        ├─► /wbc/ik_goal_pose  → ik_goal_mux → /ik_goal_pose → z1_ik_to_jtc
        ├─► /wbc/ik_enable     → ik_goal_mux → /ik_enable → z1_ik_to_jtc
        └─► /my_spot/cmd_vel   → Spot base velocity (10 Hz)

ik_goal_mux:
  Z1 FSM ──[/z1/ik_goal_pose]──┐
                                   ├──► /ik_goal_pose → z1_ik_to_jtc
  WBC QP ──[/wbc/ik_goal_pose]──┘
             priority controlled by /wbc/enable
```

### WBC coordinator FSM states

```
IDLE ──(posture=LYING & confidence≥0.5 & target_odom ready)──► APPROACHING
APPROACHING ──(dist<handoff_distance=5cm)──► SCANNING
SCANNING ──(/wbc/ws_request)──► WS_EXTENSION
WS_EXTENSION ──(/ik_done)──► SCANNING
any ──(posture≠LYING for >lying_timeout)──► IDLE
```

**APPROACHING details:**
- Target inizializzato: media prime 3 misure in odom
- Target aggiornato se posture_confidence migliora ≥ confidence_margin (0.10)
- Quality = max_q * (1 - posture_confidence) su `/wbc/target_uncertainty`
- Spot naviga con `v_scale` proporzionale alla quality (mai zero)
- Braccio look-at: orientazione stabile via min-rotation quaternion
- Handoff a 5 cm di distanza (non basato su incertezza)
- Senza dati posture_confidence, quality cresce linearmente → Spot rallenta

**Handoff logic:** WBC master controller. Spot raggiunge target → `APPROACHING → SCANNING`, disabilita WBC, segnala Z1 FSM via `/wbc/state='SCANNING'`. Z1 FSM in WAITING attende questo segnale prima di BODY_SCANNING. In standalone mode (no WBC), body scan parte immediatamente.


### Dry-run mode

When `dry_run:=true` on WBC launch, all outputs go to debug topics:

| Normal (arm + Spot move) | Dry-run (nothing moves) |
|---|---|
| `/ik_goal_pose` → `z1_ik_to_jtc` | `/wbc/ik_goal_pose_debug` |
| `/ik_enable` → `z1_ik_to_jtc` | `/wbc/ik_enable_debug` |
| `/my_spot/cmd_vel` → Spot | `/wbc/cmd_vel_debug` |

---

### Packages

| Package | Role |
|---------|------|
| `src/teresa_utils/` | Shared orientation & transform utilities (no ROS node) |
| `src/z1_vision/` | Z1 arm: FSM, IK, impedance, YOLO tracking, workspace checker |
| `src/spot_control/` | Spot navigation, WBC coordinator, WBC QP controller, ik_goal_mux |
| `src/spot_perception/` | Orbbec perception: YOLO skeleton, posture classifier, laying detector |
| `src/spot_msgs/` | Custom ROS2 messages for Spot (SetStandHeight srv, etc.) |
| `src/z1_ros2/` | Unitree Z1 hardware interface, URDF, MoveIt2, bringup configs |
| `src/realsense-ros/` | Intel RealSense ROS2 driver |
| `src/orbbec_camera/` | Orbbec camera driver |

### Main modules: `src/z1_vision/z1_vision/`

| Module | Role |
|--------|------|
| `z1_FSM.py` | Top-level FSM: HOMING → WAITING → BODY_SCANNING → APPROACHING → IMPEDANCE_RUNNING → SCAN_PRELIFT → … |
| `z1_yolo_torso_tracker.py` | YOLO11 pose estimation, Kalman 3D smoothing, lock detection |
| `realsense_surface_node.py` | Depth ROI → PCA plane → surface normal & signed distance |
| `z1_ik_to_jtc.py` | Pinocchio IK (damped Jacobian, LOCAL frame), smoothstep quintic trajectory → JTC |
| `impedance_controller_realsense.py` | Cartesian impedance at 500 Hz (torque_controller), contact detection, gravity compensation |
| `z1_scan_manager.py` | FAST anatomical scan sequence (Hub, Subxiphoid, RUQ, LUQ, Suprapubic) |
| `body_search_scanner.py` | Multi-phase body search: wrist sweep → arc → adaptive refinement |
| `safe_controller_switch.py` | JTC ↔ torque_controller switching via `/controller_manager/switch_controller` |
| `kalman_filter.py` | 6-state Kalman [x,y,z,vx,vy,vz], adaptive dt, vel_damping=0.9 |
| `workspace_checker.py` | Reachability check via Pinocchio (shared by both FSM and WBC) |

### Main modules: `src/spot_control/spot_control/`

| Module | Role |
|--------|------|
| `wbc_coordinator.py` | Phase FSM for Spot+Z1: QualityMonitor (target fisso in odom + quality tracking), triggers SCANNING handoff |
| `wbc_qp_controller.py` | Holistic WBC at 10 Hz: damped pseudo-inverse split of arm joints + base velocity, quality-based v_scale, stable look-at orientation |
| `wbc_math.py` | Pure math: J_base, J_holistic, manipulability, WBC split, WBC split with yaw |
| `ik_goal_mux.py` | Priority mux: WBC goals override Z1 FSM goals |
| `spot_goal_navigator.py` | Spot point-to-point navigation |

### Main modules: `src/spot_perception/spot_perception/`

| Module | Role |
|--------|------|
| `laying_human_detector.py` | Detects laying person, publishes approach_point |
| `posture_classifier.py` | Classifies posture (STANDING/SITTING/LYING) with confidence |
| `yolo_skeleton_spot.py` | YOLO11 pose estimation for Spot's Orbbec |

### Multi-controller architecture

The arm alternates between two ROS2 controllers:

- **joint_trajectory_controller (JTC)** — position control, used for homing / approaching
- **torque_controller** — effort control, used during impedance-based ultrasound contact

`safe_controller_switch` exposes `/safe_switch/to_torque` and `/safe_switch/to_jtc` Trigger services. The FSM calls these before entering/leaving `IMPEDANCE_RUNNING`. JTC is the safe default; the system always returns to it on error.

### Z1 FSM states (z1_FSM.py)

`HOMING → WAITING → BODY_SCANNING → CHECKING_WORKSPACE → APPROACHING → WAIT_IK_DONE → SWITCHING_TO_TORQUE → IMPEDANCE_RUNNING → SWITCHING_TO_JTC → SCAN_PRELIFT → (next FAST point or HOMING) | EMERGENCY`

### IK conventions

- Solver: Pinocchio damped pseudo-inverse Jacobian, `LOCAL_WORLD_ALIGNED` frame
- Trajectory interpolation: smoothstep quintic (10t³−15t⁴+6t⁵), zero vel/acc at endpoints
- Timing: `T = max_joint_displacement / max_joint_vel`, clipped to `[traj_min_time, traj_max_time]`
- Joint unwrapping: `_make_target_near()` prevents >π rotations between waypoints
- URDF path: auto-resolved via ament_index (fallback in `z1_ik_jtc_params.yaml`)

### World frame convention

```
X → toward patient (approach direction)
Y → head to feet
Z → right to left
```

### Config files

| File | Package | Governs |
|------|---------|---------|
| `z1_fsm_params.yaml` | z1_vision | FSM topics, home pose, approach offset, FAST point ratios, workspace safety margin, WBC startup timeout |
| `z1_yolo_torso_params.yaml` | z1_vision | YOLO model path, confidence, Kalman gains, lock threshold |
| `z1_ik_jtc_params.yaml` | z1_vision | URDF path, IK tol/damping, max_joint_vel (0.2 rad/s), trajectory timing (max 15s) |
| `impedance_control_params.yaml` | z1_vision | K_p [150,150,300], K_d, K_i, approach speed, contact threshold |
| `surface_params.yaml` | z1_vision | Depth ROI size, PCA config, frame names |
| `body_search_params.yaml` | z1_vision | Scan extents, wrist angles, early-stop threshold |
| `camera_params.yaml` | z1_vision | Camera TF offset relative to EE (link06 → camera_link) |
| `wbc_params.yaml` | spot_control | WBC QP weights, handoff distance (0.05), quality params (ref, alpha, growth, min, max, v_min), orientation_mode, workspace safety margin |

### Key shared parameters (keep in sync)

- `workspace_safety_margin: 0.05` — in both `z1_fsm_params.yaml` and `wbc_params.yaml`, both use the same `WorkspaceChecker` class
- `orbbec_confidence_threshold: 0.5` — in `wbc_params.yaml` and `laying_human_detector` (min_detection_confidence). Both must match.
- `ik_goal_topic` / `ik_enable_topic` — FSM code defaults are `/z1/ik_goal_pose` and `/z1/ik_enable` (go through `ik_goal_mux`). YAML must NOT override these to `/ik_*` directly or the mux will be bypassed.
- `home_orientation: [-0.0062, 0.4107, 0.0021, 0.9118]` — must be identical in `z1_fsm_params.yaml` and `wbc_params.yaml`

### YOLO model

`yolo11n-pose.pt` lives at the workspace root. Used by both `z1_yolo_torso_tracker` (RealSense) and `yolo_skeleton_spot` (Orbbec).

### Legacy code

Superseded files have been moved to `src/z1_vision/z1_vision/Old/` — do not reference them unless explicitly debugging history.
