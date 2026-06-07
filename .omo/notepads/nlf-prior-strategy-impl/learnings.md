# NLF Prior Strategy — Learnings

## Task 6 — Quality Monitor NLF Delta + LOOKAT Target Blending

### Changes Made
- **`_check_nlf_delta(torso_yolo)`**: new coordinator method (line 1284) comparing YOLO torso vs NLF prior center
  - Uses `nlf_coherence_threshold` (0.15m) and `nlf_divergence_threshold` (0.30m) from params
  - Returns `('HIGH'|'MEDIUM'|'LOW', delta)` tuple
- **`_tick_pre_approach()` first-tick block**: blends NLF+YOLO based on quality label
  - HIGH → 70% NLF / 30% YOLO, MEDIUM → 50/50, LOW → YOLO 100%
  - Falls back to NLF-only when no RealSense data
- **`_tick_pre_approach()` safety gate check**: replaced inline delta check with `_check_nlf_delta()`
  - Added LOW tick tracking (patient movement detection after >30 consecutive LOW ticks)
- **`_filtered_goal()`**: NLF blending applied to QualityMonitor position in APPROACHING
  - Same blending weights as PRE_APPROACH
  - Independent LOW tick tracking
- **Parameters**: `nlf_coherence_threshold` (0.15) and `nlf_divergence_threshold` (0.30) declared
- **`_nlf_low_ticks`**: initialized in `__init__` for tracking consecutive LOW coherence ticks

### Verification
- AST parse: OK
- Python compile: OK
- `colcon build`: not available on macOS dev machine; deferred to CI/robot hardware
