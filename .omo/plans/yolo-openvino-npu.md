# Convert YOLO11 to OpenVINO NPU — TERESA Integration

## TL;DR

> **Quick Summary**: Export `yolo11n-pose.pt` to OpenVINO INT8 format for Intel Core Ultra 7 NPU acceleration. Update the 2 YOLO-based tracker nodes to load the OpenVINO model instead of PyTorch. Expected speedup: **3-5x** on inference (10-20 FPS → 60-100 FPS).
>
> **Deliverables**:
> - `yolo11n-pose_openvino_model/` — OpenVINO IR (INT8 quantized)
> - `z1_yolo_torso_tracker.py` — updated model_path default
> - `yolo_skeleton_spot.py` — updated model_path default
>
> **Estimated Effort**: Quick
> **Parallel Execution**: YES — 2 tasks, same wave
> **Critical Path**: Task 1 (export) → Task 2 & 3 (parallel)

---

## Context

### Original Request
Convert YOLO11n-pose (already used in TERESA) to OpenVINO to leverage the Intel Core Ultra 7 NPU. No model change — same weights, same output, just faster inference via Intel AI Boost NPU.

### Current State
- **2 nodes** load YOLO via `ultralytics.YOLO(model_path)`:
  - `z1_yolo_torso_tracker.py` (RealSense torso tracking, default: `yolo11n-pose.pt`)
  - `yolo_skeleton_spot.py` (Orbbec skeleton via YOLO backend, default: `yolo11n-pose.pt`)
- Both accept `model_path` as a ROS2 parameter (configurable via YAML or launch args)
- NLF tracker uses torchscript, NOT YOLO — not affected
- YOLO11n-pose.pt is 6.3 MB at workspace root

### Hardware
- Intel Core Ultra 7 (Meteor Lake) with integrated NPU (Intel AI Boost)
- OpenVINO 2025.0 already installed on the HP machine

---

## Work Objectives

### Core Objective
Export YOLO11n-pose to OpenVINO INT8 format and configure the trackers to load it.

### Concrete Deliverables
- `yolo11n-pose_openvino_model/` directory (3 files: .xml, .bin, metadata.yaml)
- Updated `model_path` defaults in both tracker Python files

### Definition of Done
- [ ] OpenVINO export completes without errors
- [ ] `YOLO("yolo11n-pose_openvino_model")` loads and runs inference
- [ ] Both tracker nodes accept the OpenVINO model path
- [ ] OpenVINO `.xml/.bin` files committed to repo (or `.gitignore`d if preferred)

### Must NOT Have
- Do NOT modify the NLF tracker (nlf_torso_tracker.py) — not YOLO-based
- Do NOT change tracker logic, only model loading
- Do NOT remove the PyTorch model (keep as fallback)

---

## Verification Strategy

- **Automated tests**: N/A (model export + config change)
- **Agent-Executed QA**: Verify export succeeds, model loads, inference produces valid output

---

## Execution Strategy

```
Wave 1:
├── Task 1: Export YOLO11n-pose to OpenVINO INT8 [quick]
│
Wave 2 (parallel, after Task 1):
├── Task 2: Update z1_yolo_torso_tracker.py model_path default [quick]
└── Task 3: Update yolo_skeleton_spot.py model_path default [quick]
```

---

## TODOs

- [ ] 1. Export `yolo11n-pose.pt` to OpenVINO INT8 format

  **What to do**:
  - Run `yolo export model=yolo11n-pose.pt format=openvino int8=True` from workspace root
  - Verify output directory `yolo11n-pose_openvino_model/` contains xml, bin, metadata.yaml
  - Quick inference test: `YOLO("yolo11n-pose_openvino_model")` on a sample image

  **Must NOT do**:
  - Do NOT delete the original .pt file

  **Recommended Agent Profile**: `quick`
  - Reason: single command, no code changes

  **Parallelization**: Wave 1 (blocks Tasks 2 & 3)

  **Acceptance Criteria**:
  - [ ] `ls yolo11n-pose_openvino_model/yolo11n-pose.xml` exists
  - [ ] `ls yolo11n-pose_openvino_model/yolo11n-pose.bin` exists
  - [ ] Quick Python test: `YOLO("yolo11n-pose_openvino_model")` loads without error

  **QA Scenarios**:
  ```
  Scenario: Export succeeds and model loads
    Tool: Bash + Python
    Steps:
      1. cd workspace root
      2. yolo export model=yolo11n-pose.pt format=openvino int8=True
      3. ls -la yolo11n-pose_openvino_model/
      4. python3 -c "from ultralytics import YOLO; m=YOLO('yolo11n-pose_openvino_model'); print('OK')"
    Expected Result: Export completes, files exist, model loads
    Evidence: .omo/evidence/task-1-export.txt
  ```

  **Commit**: YES
  - Message: `feat(yolo): add OpenVINO INT8 export of yolo11n-pose`
  - Files: `yolo11n-pose_openvino_model/`

- [ ] 2. Update `z1_yolo_torso_tracker.py` to default to OpenVINO model

  **What to do**:
  - In `__init__`, change the default value of `model_path` parameter:
    from: `self.declare_parameter('model_path', 'yolo11n-pose.pt')`
    to: `self.declare_parameter('model_path', 'yolo11n-pose_openvino_model')`
  - The YOLO() call already supports OpenVINO — `YOLO(model_path)` automatically detects the directory format

  **Must NOT do**:
  - Do NOT change any tracker logic or other parameters

  **Recommended Agent Profile**: `quick`
  - Reason: 1-line change in parameter default

  **Parallelization**: Wave 2 with Task 3 (independent files)

  **Acceptance Criteria**:
  - [ ] Default `model_path` parameter points to OpenVINO directory
  - [ ] File still parses correctly

  **Commit**: YES (groups with Task 3)
  - Message: `refactor(z1_vision): default to OpenVINO YOLO model for NPU`
  - Files: `src/z1_vision/z1_vision/z1_yolo_torso_tracker.py`

- [ ] 3. Update `yolo_skeleton_spot.py` to default to OpenVINO model

  **What to do**:
  - In `__init__`, change the default value of `model_path` parameter:
    from: `self.declare_parameter("model_path", "yolo11n-pose.pt")`
    to: `self.declare_parameter("model_path", "yolo11n-pose_openvino_model")`

  **Must NOT do**:
  - Do NOT change any tracker logic or other parameters

  **Recommended Agent Profile**: `quick`
  - Reason: 1-line change in parameter default

  **Parallelization**: Wave 2 with Task 2 (independent files)

  **Acceptance Criteria**:
  - [ ] Default `model_path` parameter points to OpenVINO directory
  - [ ] File still parses correctly

  **Commit**: YES (groups with Task 2)
  - Message: `refactor(spot_perception): default to OpenVINO YOLO model for NPU`
  - Files: `src/spot_perception/spot_perception/yolo_skeleton_spot.py`

---

## Commit Strategy

- **Wave 1**: `feat(yolo): add OpenVINO INT8 export of yolo11n-pose`
- **Wave 2**: `refactor: default YOLO trackers to OpenVINO model for Intel NPU`
  - Groups commit for Tasks 2 & 3

---

## Success Criteria

### Verification Commands
```bash
# Export
yolo export model=yolo11n-pose.pt format=openvino int8=True

# Verify files
ls yolo11n-pose_openvino_model/

# Quick inference test
python3 -c "from ultralytics import YOLO; m=YOLO('yolo11n-pose_openvino_model'); r=m('test.jpg'); print(r[0].keypoints)"
```

### Fallback
If OpenVINO doesn't work on the HP machine, both trackers accept `model_path` as a parameter:
```bash
# Launch with PyTorch fallback:
ros2 run z1_vision z1_yolo_torso_tracker --ros-args -p model_path:=yolo11n-pose.pt
```
