# NLF Burst Streaming — Learnings (Task 3)

## Changes Made
File: `src/spot_perception/spot_perception/nlf_skeleton.py`

### 1. Import
- Added `import time` for burst timeout tracking

### 2. `__init__` additions
- Burst state variables: `_burst_active`, `_burst_detection_count`, `_burst_start_time`, `_latest_raw_torso`
- 3 new parameters declared with defaults: `burst_min_detections: 2`, `burst_timeout_s: 30.0`, `burst_throttle_frames: 10`

### 3. `_cb_trigger` redesign
- Bool(True): activates burst (unpauses streaming, resets counters, clears smoothed_kp)
- Bool(False): pauses streaming, but IGNORED during active burst
- No longer runs one-shot inference — delegates to `_cb_color` streaming pipeline

### 4. `_cb_color` additions
- **Throttle guard** at top: skips frames when `_frame_count % burst_throttle_frames != 0` during burst
- `_frame_count` incremented at very top of callback (before all guards) to ensure throttle works
- **Burst detection counting**: after target selection, validates 4 torso joints (SPINE1, SPINE2, SPINE3, PELVIS) non-NaN, increments counter, stores raw torso for 1-detection fallback
- **Auto-finish**: calls `_finish_burst()` when `burst_min_detections` (2) reached OR `burst_timeout_s` (30s) elapsed
- **Publish suppression**: all normal publish calls (`_publish_target_pose`, `_publish_all_markers`, mesh) suppressed during burst

### 5. `_finish_burst()` new method
- Publishes EMA-refined skeleton to `/exposure/nlf_prior` (≥2 detections)
- Falls back to raw single detection (1 detection)
- Falls back to empty PoseArray (0 detections)
- Sets `_streaming_paused=True`, `_burst_active=False`

## Verification
- `python3 -c "import ast; ast.parse(...)"` → OK

## YAML
- `nlf_params.yaml` already has `burst_min_detections: 2`, `burst_timeout_s: 30.0`, `burst_throttle_frames: 10` (from Task 1)
