# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Source ROS2 first (required every shell)
source /opt/ros/humble/setup.bash

# Build the full workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build only the main package
colcon build --packages-select z1_vision

# Source install after build
source install/setup.bash
```

### Running the system (three-terminal launch)

```bash
# Terminal 1: Robot hardware + RealSense camera
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# Terminal 2: Vision pipeline
ros2 launch z1_vision z1_perception.launch.py

# Terminal 3: Control (FSM starts after fsm_delay seconds)
ros2 launch z1_vision z1_control.launch.py fsm_delay:=5.0
```

### Linting / tests

```bash
colcon test --packages-select z1_vision
colcon test-result --verbose
```

---

## Architecture

The system implements autonomous FAST (Focused Assessment with Sonography in Trauma) ultrasound scanning on a Unitree Z1 arm mounted on TERESA.

### Pipeline overview

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

### Main package: `src/z1_vision`

All application logic lives here (ament_python). Key modules:

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
| `workspace_checker.py` | Reachability check using Pinocchio before approach |

### Multi-controller architecture

The arm alternates between two ROS2 controllers:

- **joint_trajectory_controller (JTC)** — position control, used for homing / approaching
- **torque_controller** — effort control, used during impedance-based ultrasound contact

`safe_controller_switch` exposes `/safe_switch/to_torque` and `/safe_switch/to_jtc` Trigger services. The FSM calls these before entering/leaving `IMPEDANCE_RUNNING`. JTC is the safe default; the system always returns to it on error.

### FSM states (z1_FSM.py)

`HOMING → WAITING → BODY_SCANNING → CHECKING_WORKSPACE → APPROACHING → WAIT_IK_DONE → SWITCHING_TO_TORQUE → IMPEDANCE_RUNNING → SWITCHING_TO_JTC → SCAN_PRELIFT → (next FAST point or HOMING) | EMERGENCY`

### IK conventions

- Solver: Pinocchio damped pseudo-inverse Jacobian, `LOCAL_WORLD_ALIGNED` frame
- Trajectory interpolation: smoothstep quintic (10t³−15t⁴+6t⁵), zero vel/acc at endpoints
- Joint unwrapping: `_make_target_near()` prevents >π rotations between waypoints
- URDF path (hardcoded in z1_ik_jtc_params.yaml): `/home/andrea/Ros2_repositories/unitree_z1_ws/install/z1_description/share/z1_description/urdf/z1.urdf`

### World frame convention

```
X → toward patient (approach direction)
Y → head to feet
Z → right to left
```

### Config files (`src/z1_vision/config/`)

| File | Governs |
|------|---------|
| `z1_fsm_params.yaml` | FSM topics, home pose, approach offset, FAST point ratios |
| `z1_yolo_torso_params.yaml` | YOLO model path, confidence, Kalman gains, lock threshold |
| `z1_ik_jtc_params.yaml` | URDF path, IK tol/damping, trajectory timing |
| `impedance_control_params.yaml` | K_p [150,150,300], K_d, K_i, approach speed, contact threshold |
| `surface_params.yaml` | Depth ROI size, PCA config, frame names |
| `body_search_params.yaml` | Scan extents, wrist angles, early-stop threshold |
| `camera_params.yaml` | Camera TF offset relative to EE (link06 → camera_link) |

### Supporting packages

- `src/z1_ros2/` — Unitree Z1 hardware interface, URDF, MoveIt2, bringup configs
- `src/z1_control/` — Legacy alternative control nodes (mostly superseded by z1_vision)
- `src/realsense-ros/` — Intel RealSense ROS2 driver
- `src/orbbec_camera/` — Orbbec camera driver (alternative)

### YOLO model

`yolo11n-pose.pt` lives at the workspace root. Path is referenced in `z1_yolo_torso_params.yaml`.

### Legacy code

Superseded files have been moved to `src/z1_vision/z1_vision/Old/` — do not reference them unless explicitly debugging history.
