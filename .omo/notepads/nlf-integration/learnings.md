# NLF Integration — Implementation Learnings

## Architecture Decisions

### Timer-driven FSM vs frame-synchronous
- Chose timer (20 Hz) to decouple NLF inference rate from FSM tick rate.
- NLF inference runs in `_cb_image` at camera frame rate (typically 15-30 Hz).
- FSM tick at 20 Hz consumes the latest detection; re-evaluates same detection if no new image.
- Kalman dt computed from real tick intervals for accurate prediction.

### No depth subscription
- NLF regresses 3D SMPL joints directly from monocular RGB — no depth image needed.
- Removed ApproximateTimeSynchronizer, message_filters, and depth subscriber entirely.
- Removed sync_slop and sync_queue_size parameters.

### Camera frame
- NLF 3D output is in the RGB camera frame.
- Default: `camera_color_optical_frame` — configurable via `camera_frame` parameter.
- TF lookup uses this frame for camera->world transforms.

### SMPL indices (not COCO)
- Torso: SHOULDER_LEFT(16), SHOULDER_RIGHT(17), HIP_LEFT(1), HIP_RIGHT(2)
- Face/head for fallback: HEAD(15), NECK(12)
- Body keypoints: all 24 SMPL joints (PoseArray of 24, not 17)

### NLF model stub
- `torch.jit.load()` commented out with clear TODO markers.
- `_nlf_infer()` returns None until model is available.
- Per-joint confidence: hardcoded 1.0 (NLF does not provide conf scores).
  TODO: add confidence estimation when NLF provides uncertainty output.

### Parameter compatibility
- All z1_yolo_torso_tracker params declared for backward compatibility.
- New NLF-specific params: `camera_frame`, `tick_rate_hz`.

### 22-float scan point format
- Identical layout to z1_yolo_torso_tracker, SMPL indices in positions 6-21:
  [score, n_kp, conf, x, y, z, kp16_conf, kp17_conf, kp1_conf, kp2_conf, kp16_xyz, kp17_xyz, kp1_xyz, kp2_xyz]

### PersonTrack: 24 Kalman filters (was 17)
- `person_tracking.py` now uses `NUM_JOINTS` (24) from `sml_pose_indices` instead of hardcoded 17.
- All list allocations (`kf`, `visible`, `missing_count`, `_cached_pts`) use `NUM_JOINTS`.
- Joint group sets (`TORSO_JOINTS`, `ARM_JOINTS`, `LEG_JOINTS`) imported from `sml_pose_indices`.
- `NOSE_JOINTS = {0}` kept as PELVIS proxy; `SKIP_JOINTS` is empty (all 24 joints tracked).

### Per-joint Kalman tuning for new SMPL joints
- SPINE1/SPINE2/SPINE3: `Q*=0.25, R*=0.125` (very stable, tight column)
- HEAD/NECK: `Q*=0.75, R*=0.375` (like nose)
- FOOT_LEFT/FOOT_RIGHT: `Q*=0.35, R*=0.175` (like torso)
- HAND_LEFT/HAND_RIGHT: `Q*=0.5, R*=0.25` (default-ish)

### Torso angle uses SPINE vector
- `torso_angle_deg()` now computes angle between `SPINE3→SPINE1` vector and world-up.
- Replaced old shoulder-hip midpoint vector approach.
- Only requires SPINE1 and SPINE3 to be available (2 joints instead of 4).

### TORSO_length_constraint uses SMPL indices
- Updated from hardcoded `[5,6,11,12]` to `[SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT]`.

## Topics (unchanged from z1_yolo_torso_tracker)
/torso_target_ee, /torso_target_ee_locked, /torso_tracker_state,
/torso_markers, /torso_scan_point, /torso_keypoint_conf, /exposure/body_keypoints

## TODO for real NLF integration
1. Download nlf_s.torchscript from https://github.com/isarandi/nlf/releases
2. Place in package share directory or configure model_path param
3. Uncomment torch.jit.load() block in __init__
4. Uncomment preprocessing + inference block in _nlf_infer()
5. Add per-joint confidence estimation
6. Test FSM transitions with real detections
