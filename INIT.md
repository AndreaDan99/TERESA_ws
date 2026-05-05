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

## Recent Changes (5 May 2026)

- **WBC pitch exploration**: `J_base` is 6×2 (only `vx` + `wz`). No body pitch control mechanism exists in `spot_msgs` or this workspace — `cmd_vel.angular.y` is likely not processed by the standard `spot_driver`. Spot API for pitch still to be verified via `ros2 service list | grep my_spot` on SpotCore.
- **Pitch strategy**: first test WBC without pitch. If arm reach is insufficient, integrate pitch as a discrete compensation (service call, like `SetStandHeight`) during WS_EXTENSION, NOT as part of the continuous WBC 6×4 Jacobian.
- `skip_impedance: true` for WBC testing (impedance disabled).

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
                    ├─► /laying_human/approach_point  (PoseStamped)
                    └─► /human_pose/posture, /human_pose/posture_confidence

/laying_human/approach_point
  └─► wbc_coordinator  (FSM: IDLE → APPROACHING → SCANNING → WS_EXTENSION)
        ├─► /wbc/ee_goal  (filtered approach point via Kalman)
        ├─► /wbc/enable   (True = WBC takes over from Z1 FSM)
        └─► /wbc/desired_yaw  (Spot ⊥ patient body axis)

/wbc/ee_goal + /wbc/enable
  └─► wbc_qp_controller  (holistic WBC: arm q_dot + base vx·wz)
        ├─► /wbc/ik_goal_pose  → ik_goal_mux → /ik_goal_pose → z1_ik_to_jtc
        ├─► /wbc/ik_enable     → ik_goal_mux → /ik_enable → z1_ik_to_jtc
        └─► /my_spot/cmd_vel   → Spot base velocity

ik_goal_mux:
  Z1 FSM ──[/z1/ik_goal_pose]──┐
                                  ├──► /ik_goal_pose → z1_ik_to_jtc
  WBC QP ──[/wbc/ik_goal_pose]──┘
            priority controlled by /wbc/enable
```

### WBC coordinator FSM states

```
IDLE ──(posture=LYING & confidence≥0.5)──► APPROACHING
APPROACHING ──(dist<handoff_distance)──► SCANNING
SCANNING ──(/wbc/ws_request)──► WS_EXTENSION
WS_EXTENSION ──(/ik_done)──► SCANNING
any ──(posture≠LYING for >lying_timeout)──► IDLE
```

**Handoff logic (updated):** WBC is the master. When Spot reaches the approach point, it transitions directly `APPROACHING → SCANNING`, disables WBC control (`/wbc/enable=False`), and signals the Z1 FSM to begin its body scan. The Z1 FSM gate in `WAITING` state waits for `/wbc/state == 'SCANNING'` before starting `BODY_SCANNING` (standalone mode is unchanged: if no WBC is present, body scan starts immediately).


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
| `wbc_coordinator.py` | Phase FSM for Spot+Z1: parses posture, triggers SCANNING handoff |
| `wbc_qp_controller.py` | Holistic WBC: damped pseudo-inverse split of arm joints + base velocity |
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
| `z1_fsm_params.yaml` | z1_vision | FSM topics, home pose, approach offset, FAST point ratios, workspace safety margin |
| `z1_yolo_torso_params.yaml` | z1_vision | YOLO model path, confidence, Kalman gains, lock threshold |
| `z1_ik_jtc_params.yaml` | z1_vision | URDF path, IK tol/damping, max_joint_vel (0.2 rad/s), trajectory timing (max 15s) |
| `impedance_control_params.yaml` | z1_vision | K_p [150,150,300], K_d, K_i, approach speed, contact threshold |
| `surface_params.yaml` | z1_vision | Depth ROI size, PCA config, frame names |
| `body_search_params.yaml` | z1_vision | Scan extents, wrist angles, early-stop threshold |
| `camera_params.yaml` | z1_vision | Camera TF offset relative to EE (link06 → camera_link) |
| `wbc_params.yaml` | spot_control | WBC QP weights, handoff distance, confidence threshold (0.5), workspace safety margin (0.30) |

### Key shared parameters (keep in sync)

- `workspace_safety_margin: 0.05` — in both `z1_fsm_params.yaml` and `wbc_params.yaml`, both use the same `WorkspaceChecker` class
- `orbbec_confidence_threshold: 0.5` — in `wbc_params.yaml` and `laying_human_detector` (min_detection_confidence). Both must match.
- `ik_goal_topic` / `ik_enable_topic` — FSM code defaults are `/z1/ik_goal_pose` and `/z1/ik_enable` (go through `ik_goal_mux`). YAML must NOT override these to `/ik_*` directly or the mux will be bypassed.

### YOLO model

`yolo11n-pose.pt` lives at the workspace root. Used by both `z1_yolo_torso_tracker` (RealSense) and `yolo_skeleton_spot` (Orbbec).

### Legacy code

Superseded files have been moved to `src/z1_vision/z1_vision/Old/` — do not reference them unless explicitly debugging history.
