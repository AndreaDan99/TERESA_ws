# TERESA — Changelog

Storico completo delle modifiche dal 6 maggio 2026 al 6 giugno 2026.
Per la descrizione del sistema corrente vedi [`DESCRIPTION.md`](DESCRIPTION.md).

---

## 9 June 2026 — NLF Burst Streaming + Confidence Gate + LOCKING Blocking

### Overview
- **NLF Burst Streaming**: NLF trigger redesigned from one-shot to multi-frame burst with EMA accumulation. Collects 2 valid detections (lying person + 4 torso joints non-NaN), publishes refined skeleton to `/exposure/nlf_prior`, auto-pauses. Timeout: 30s.
- **EXCELLENT Confidence Tier**: after burst, publishes mean bbox_score on `/exposure/nlf_confidence`. If NLF confidence ≥ 0.80 → 100% NLF blending (skip positional delta check). Otherwise existing HIGH/MEDIUM/LOW tiers unchanged.
- **LOCKING Blocking**: coordinator waits for NLF prior (or timeout) before transitioning to PRE_APPROACH. Condition: 5 samples + ik_done + (NLF valid or timeout).
- **Best Pitch on LOCKING**: refinement best pitch saved during SEARCHING is now applied on ALL LOCKING entry paths (direct lock, semi-lock), not just refinement lock. Ensures Orbbec has optimal camera angle during NLF burst.
- **Launch Fix**: `nlf_skeleton_node` now has `IfCondition` — only launches with `perception_backend:=nlf`.
- **Publish Suppression**: `/human_pose/points_3d` suppressed during active NLF burst to avoid YOLO conflict.

### Modified files
| File | +/− | Changes |
|------|-----|---------|
| `nlf_skeleton.py` | +117/−34 | Burst state machine: `_burst_active`, `_finish_burst()` (3 fallback branches), `_cb_trigger` redesign, EMA accumulation, publish suppression, `/exposure/nlf_confidence` publisher |
| `wbc_coordinator.py` | +11/−11 | LOCKING blocking gate, timeout 10→30s, removed `Bool(False)` publish on timeout, `_cb_nlf_confidence`, EXCELLENT tier in `_filtered_goal()`, best pitch on LOCKING entry |
| `nlf_params.yaml` | +3 | `burst_min_detections`, `burst_timeout_s`, `burst_throttle_frames` |
| `wbc_params.yaml` | +2/−1 | `nlf_timeout` 10→30s, `nlf_excellent_confidence: 0.80` |
| `spot_perception.launch.py` | +2/−1 | `condition=IfCondition` on `nlf_skeleton_node` |

### New topics
| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/exposure/nlf_confidence` | Float32 | nlf_skeleton | wbc_coordinator | Mean NLF bbox_score after burst |

---

## 8 June 2026 — Web Joystick, Timed Search, YOLO Default

### Overview
- **Web Joystick Control Panel**: D-pad with 5 buttons, two modes (Drive/Body), speed control, height/pitch sliders, HOME/PARK arm buttons
- **SEARCHING rewrite**: timed open-loop rotation (no TF), 7 hardcoded manual poses, ±30° yaw, step forward after cycle
- **Perception defaults**: YOLO (40 FPS) for SEARCHING, NLF idle until LOCKING trigger with 3s delay
- **Multiple fixes**: dead code removal, IK stability tuning, arm wait logic, rotation speed

### SEARCHING changes
| File | +/− | Changes |
|------|-----|---------|
| `wbc_coordinator.py` | +114/−66 | Timed rotation, 7-pose wait, HOME+step forward cycle, NLF trigger delay, refinement trigger |
| `wbc_qp_controller.py` | +15/−20 | 7 hardcoded manual poses (FK reader), LOCKING home to search pose 1 |
| `wbc_params.yaml` | +4/−7 | search_yaw_angles [30,-30], step_forward 0.20m, step_speed 0.3m/s |
| `z1_ik_jtc_params.yaml` | +2/−2 | ik_damping 1e-2, max_joint_vel 0.4 |

### Web interface
| File | +/− | Changes |
|------|-----|---------|
| `teresa_control.html` | +240/−6 | D-pad, mode toggle, speed control, sliders, HOME/PARK buttons, ik_enable on STOP, joystick always enabled |

### Perception
| File | +/− | Changes |
|------|-----|---------|
| `nlf_skeleton.py` | +1/−1 | _streaming_paused = True by default |
| `spot_perception.launch.py` | +2/−3 | NLF always runs (no condition), YOLO default |
| `z1_perception.launch.py` | +1/−1 | default yolo |
| `teresa_perception.launch.py` | +1/−1 | default yolo |

### Fixes
| File | +/− | Changes |
|------|-----|---------|
| `z1_yolo_torso_tracker.py` | −11 | Removed dead /torso_sm_state subscription |
| `nlf_torso_tracker.py` | −11 | Removed dead /torso_sm_state subscription |
| `camera_view.html` | +1 | Fixed missing closing brace |

---

## 7 June 2026 — NLF Prior at LOCKING (single-frame, binary fallback)

### Overview
- **NLF single-frame prior** triggered at LOCKING entry, 10s timeout for NLF to produce a valid prior
- **Binary fallback**: if NLF prior fails → entire system reverts to 6 June 2026 behavior (YOLO-only)
- **Gate**: `_nlf_prior_valid()` controls all downstream branches
- **PRE_APPROACH**: 1s safety gate with NLF prior active; legacy sliding window fallback without
- **APPROACHING**: unified 6-pose Cartesian grid centered on torso. Tight offsets with NLF prior, wide offsets with YOLO-only
- **LOOKAT**: blended NLF(70%) + YOLO(30%) when HIGH coherence; YOLO 100% when LOW coherence
- **CPU saving**: NLF streaming paused after prior capture

### Modified files
| File | +/− | Changes |
|------|-----|---------|
| `nlf_skeleton.py` | +76 | Single-frame prior capture at LOCKING, `_nlf_prior_valid()` gate, streaming pause |
| `wbc_coordinator.py` | +217 | NLF prior FSM integration: PRE_APPROACH safety gate, APPROACHING grid selection, LOOKAT blend, binary fallback |
| `wbc_qp_controller.py` | +44 | Unified 6-pose grid centered on torso, tight/wide offset selection based on NLF prior validity |
| `wbc_params.yaml` | +7 | NLF prior params: timeout, coherence thresholds, blend ratios |
| `body_search_params.yaml` | +7 | Grid offset parameters for NLF-prior vs YOLO-only modes |

### Tests
- 24 pytest tests, 3 new test files covering NLF prior capture, binary fallback, and blended LOOKAT

---

## 6 June 2026 — NLF Integration (24 SMPL Joints)

### Overview
- **NLF (Neural Localizer Fields)** replaces YOLO11n-pose as default perception backend
- **24 SMPL joints** natively published on `/human_pose/points_3d` (up from 17 COCO)
- **YOLO fallback** via `perception_backend:=yolo` — publishes 24 joints with NaN padding
- **New topic**: `/human_pose/smpl_mesh` (SMPL mesh vertices, decimated)
- **New nodes**: `nlf_skeleton.py` (Orbbec), `nlf_torso_tracker.py` (RealSense)
- **Updated consumers**: posture_classifier, laying_human_detector, person_tracking, z1_scan_manager, exposure_scanner, body_search_scanner — all migrated to SMPL-24 indices
- **New module**: `sml_pose_indices.py` — shared SMPL-24 joint constants

### New files
- `src/spot_perception/spot_perception/sml_pose_indices.py` — SMPL-24 constants
- `src/spot_perception/spot_perception/yolo_to_smpl_pad.py` — YOLO→SMPL adapter
- `src/spot_perception/spot_perception/nlf_skeleton.py` — NLF Orbbec node
- `src/z1_vision/z1_vision/nlf_torso_tracker.py` — NLF RealSense node
- `src/spot_perception/config/nlf_params.yaml`, `src/z1_vision/config/nlf_torso_params.yaml`
- `scripts/download_nlf_models.sh`

### Modified files
- Launch files: `spot_perception.launch.py`, `z1_perception.launch.py`, `z1_torso_surface.launch.py` — added `perception_backend` (default: `nlf`)
- YOLO nodes updated to publish 24 joints: `yolo_skeleton_spot.py`, `z1_yolo_torso_tracker.py`
- Consumer updates: `person_tracking.py`, `posture_classifier.py`, `laying_human_detector.py`, `z1_scan_manager.py`, `exposure_scanner.py`, `body_search_scanner.py`

---

## 6 June 2026 (cont.) — Always exposure before FAST

### Modifica flusso handoff

- **Prima**: paziente LYING → direttamente a SCANNING (FAST); paziente non-LYING → EXPOSURE_SCANNING → SCANNING
- **Ora**: TUTTI i pazienti fanno sempre EXPOSURE_SCANNING → EXPOSURE_REVIEW → SCANNING (FAST)
- Motivazione clinica: la body map di esposizione è utile sempre, indipendentemente dalla postura

### File modificati
`wbc_coordinator.py` — rimosso il ramo condizionale `if self._posture == 'LYING'` in `_tick_approaching()`

---

## 6 June 2026 (cont.) — Exposure snapshot + PRE_APPROACH fixes

### New node: exposure_snapshot.py

- Dedicated node for capturing RealSense snapshots during EXPOSURE_REVIEW
- Triggers on `/exposure/goto_point` + `/ik_done`, waits 1s settle, publishes `/exposure/snapshot`
- Saves JPEG to disk: `/tmp/exposure_snapshot_{region}_{idx}_{timestamp}.jpg`
- Uses `/exposure/grid_markers` to determine region label from marker namespace

### Web UI: snapshot display

- Subscription to `/exposure/snapshot` freezes the RealSense feed when received
- Semi-transparent badge overlay: "📸 torso [3]" on the camera image
- "📸 Close" button in RealSense overlay bar to return to live feed
- Live feed blocked while snapshot is active

### PRE_APPROACH fixes (wbc_coordinator.py)

- **Goal target Z offset**: when `_body_center_odom` is None, fallback `_filtered_goal()` now adds +0.40m Z offset so the QP LOOKAT points at the torso instead of the ground
- **Sliding window confirmation**: changed from "3 consecutive ESTIMATING/LOCKED ticks" to "≥1 in last 5 ticks". Tolerates tracker flickering, proceeds immediately on first detection.

### Web UI: gate toggle always visible

- MANUAL/AUTO buttons now visible by default (instead of hidden until TF ready)
- Disabled state follows the same lifecycle as other buttons (disableAll / enableMissionButtons)
- Visibility controlled by `_topicsReady` flag (set when rosbridge connects)

### New topics

| Topic | Publisher | Subscriber | Purpose |
|-------|-----------|------------|---------|
| `/exposure/snapshot` | exposure_snapshot | web UI | JPEG snapshot on goto_point |

### Files

| File | Action | +/− |
|------|--------|-----|
| `exposure_snapshot.py` | **New** | +128 |
| `wbc_coordinator.py` | PRE_APPROACH fixes | +15/−10 |
| `wbc.launch.py` | +snapshot node | +7 |
| `setup.py` | +entry point | +1 |
| `web/teresa_control.html` | Snapshot + gate toggle fixes | +40/−10 |

---

## 6 June 2026 — Full-body exposure scanner + simultaneous skeleton refinement

### exposure_scanner.py — rewrite completo (310 → 650 righe)

- **Full-body grid**: 14 punti su 7 regioni (HEAD=2, TORSO=4, ARM×2=2+2, LEG×2=2+2, FEET=2)
- **Dynamic look-at orientation**: EE X-axis points toward body via `compute_ee_orientation`, not hardcoded. Same approach as FAST ultrasound.
- **Horizontal standoff**: 0.50 m in X (toward Spot), not vertical +Z. Arm stays in natural configuration.
- **Orbbec keypoints transformed to world frame** via TF (`orbbec_color_optical_frame → world`). Head position estimated from shoulders when nose occluded.
- **Running-average refined skeleton**: accumulates `/exposure/body_keypoints` (17 COCO kp from RealSense) during dwell, running average (α=0.5), publishes on `/exposure/refined_skeleton` (PoseArray, NaN for unobserved kp). Progressive fill: 0/17 → 17/17 at scan end.
- **JSON output**: per-region camera poses + scan data frames + dwell duration. Saved to `/tmp/exposure_scan_YYYYMMDD_HHMMSS.json`.
- **Region-color-coded markers**: `/exposure/grid_markers` (MarkerArray) — current point larger + glow, visited dimmed, future semi-transparent. Legend in web UI.

### z1_yolo_torso_tracker.py — full keypoint extraction in scan mode

- **New publisher** `/exposure/body_keypoints` (PoseArray, 17 kp world frame, NaN for undetected).
- **New method** `_extract_all_body_keypoints()`: extracts all 17 COCO keypoints (not just torso+face) during scan mode. Same projection + depth logic as existing `_extract_guidance()`.
- **New method** `_publish_body_keypoints()`: transforms camera-frame kp to world frame via existing TF lookup and publishes.

### wbc_coordinator.py — exposure scan per-point support

- **`_cb_next_point`** extended: handles `EXPOSURE_SCANNING` state in addition to `SCANNING`.
- **New method** `_apply_exposure_body_pose(idx)`: sets handoff body height + starts settle timer. Falls back to handoff height until full pose optimisation is plumbed in.

### Web UI — teresa_control.html

- **Grid toggle on RealSense overlay**: new `[Grid]` button projects 14 exposure markers onto camera image with region colors. Current point has white glow.
- **Color legend bar**: appears below yolo-bar when Grid active. 7 colored swatches + region labels.
- **Click-to-revisit**: click any grid marker on RealSense overlay → publishes `/exposure/goto_point(id)` → Spot repositions to that point.
- **Body Map panel**: toggle via `&#128506;` button or `m` key. Top-down (X-Y world) canvas showing progressive 17-keypoint skeleton (from `/exposure/refined_skeleton`) + exposure grid markers (from `/exposure/grid_markers`). Auto-scaled, color-coded keypoints. Skeleton edges (COCO). Toggle overlays: Skeleton + Grid. Visibility synced with camera panel.
- **`m` key** added to keyboard shortcuts.
- **Legend hide on camera close**: Grid toggle + legend reset on camera panel close.

### Paper (paper/sections/)

| File | Change |
|------|--------|
| `abstract.tex` | +4 lines: body exposure scanning mode + "one click" closing sentence |
| `introduction.tex` | +1 contribution (#5: exposure scanning + simultaneous skeleton refinement) |
| `active_perception.tex` | +80 lines: new subsection IV.C "Body Exposure Scanning with Simultaneous Skeleton Refinement" — full-body grid, per-point protocol, simultaneous refinement |
| `system_architecture.tex` | FSM 9→11 states (added EXPOSURE_SCANNING, EXPOSURE_REVIEW). Table updated. Two new bullet points. |

### Fixes

| # | Fix | File |
|---|------|------|
| 1 | `x_ee = ep.look_dir` (removed minus sign) — EE X now points toward body | `exposure_scanner.py:488` |
| 2 | Orbbec keypoints transformed to world frame via TF | `exposure_scanner.py:_cb_skeleton` |
| 3 | Head position estimated from shoulders when nose occluded | `exposure_scanner.py:_gen_head` |
| 4 | Removed dead code `REGION_KEYPOINTS` dict | `exposure_scanner.py:71-79` |
| 5 | Removed unused `field` import | `exposure_scanner.py:30` |

### New topics

| Topic | Publisher | Subscriber | Purpose |
|-------|-----------|------------|---------|
| `/exposure/body_keypoints` | z1_yolo_torso_tracker | exposure_scanner | 17 COCO kp in world frame during scan |
| `/exposure/refined_skeleton` | exposure_scanner | web UI (Body Map) | Progressive running-average skeleton |

### Files

| File | +/− |
|------|-----|
| `exposure_scanner.py` | +360/−20 (rewrite) |
| `z1_yolo_torso_tracker.py` | +55 |
| `wbc_coordinator.py` | +15 |
| `web/teresa_control.html` | +120 |
| `paper/sections/abstract.tex` | +7 |
| `paper/sections/introduction.tex` | +8 |
| `paper/sections/active_perception.tex` | +80 |
| `paper/sections/system_architecture.tex` | +12 |

---

## 5 June 2026 — Exposure Body Scanning + Interactive Review

### New node: exposure_scanner.py
- Posture-adaptive body scan with camera grid over patient body
- 5-phase state machine: request_body_pose → wait_ik → dwell → advance
- Per-point Spot body reconfiguration via same protocol as FAST scanning
- Saved IK goals for click-to-revisit replay during review phase
- Publishes `/exposure/grid_markers` (MarkerArray) for web overlay
- Publishes `/exposure/ready` on scan completion
- Subscriber `/exposure/goto_point` for interactive re-inspection

### New FSM states in wbc_coordinator.py
- `EXPOSURE_SCANNING`: automated body scan with RealSense camera
- `EXPOSURE_REVIEW`: interactive phase, click grid points to re-inspect
- `WAITING_EXPOSURE`: manual gate before exposure scan (step_confirm)
- `WAITING_FAST`: manual gate before FAST ultrasound (step_confirm)

### Manual scan gate (wbc_coordinator.py + web UI)
- New parameter `manual_scan_gate` (default true)
- Subscriber `/wbc/set_manual_scan_gate` (Bool) — from web UI
- Publisher `/wbc/manual_scan_gate` (Bool) — feedback to web UI
- When true: mission pauses at WAITING_EXPOSURE and WAITING_FAST
- When false: mission advances automatically
- Keyboard 'n' key publishes `/wbc/step_confirm` (same as web button)

### Web interface updates
- `camera_view.html`: exposure grid overlay (blue dots) on RealSense
  - Current point highlighted with blue ring
  - Visited points dimmed, pending points medium opacity
  - Click on any grid marker → publishes `/exposure/goto_point`
  - Terminate button during EXPOSURE_REVIEW phase
  - Toggle button 'Exposure' in RealSense overlay bar
- `teresa_control.html`: MANUAL/AUTO scan gate toggle
  - Buttons appear when TF is ready
  - Contextual STEP button labels: "▶ Expose" / "▶ FAST"
  - Step pending banner shows contextual messages

### Experiment logger
- Tracks EXPOSURE_SCANNING, EXPOSURE_REVIEW states in timeline
- New CSV column: `exposure_duration_s`
- New trial field: `t_exposure_start`, `t_review_start`

### New topics
| Topic | Publisher | Subscriber | Purpose |
|-------|-----------|------------|---------|
| `/exposure/grid_markers` | exposure_scanner | camera_view.html | Grid points overlay |
| `/exposure/goto_point` | camera_view.html | exposure_scanner | Re-inspect point |
| `/exposure/terminate` | camera_view.html / keyboard | wbc_coordinator | End review |
| `/exposure/ready` | exposure_scanner | wbc_coordinator | Scan complete |
| `/wbc/set_manual_scan_gate` | teresa_control.html | wbc_coordinator | Toggle gate |
| `/wbc/manual_scan_gate` | wbc_coordinator | teresa_control.html | Gate status |

### Paper (TERESA_RAL/)
- Merged Introduction + Related Work into single I. Introduction
- Added IV.D: Exposure Body Scanning and Injury Detection
- Posture-adaptive grid (supine 15 pts, sitting 29, standing 49)
- Pre-trained YOLOv8 wound + YOLOv7 burn models (footnoted)
- Web-based interactive review with click-to-revisit
- Updated FSM diagram: 13 states, 4 columns, new colour codes
- Updated system block diagram: +Injury Detection, +Web UI, +Exp. Scanner
- Placeholder figure for web interface screenshot
- 8 pages, 42 references, 0 compilation errors

### Files
| File | Action |
|------|--------|
| `exposure_scanner.py` | New (~290 lines) |
| `wbc_coordinator.py` | +95/−15 (4 states, manual gate, review) |
| `experiment_logger.py` | +25 (exposure tracking) |
| `camera_view.html` | +100 (grid overlay, click, terminate) |
| `teresa_control.html` | +55 (gate toggle, step buttons) |
| `wbc_params.yaml` | +12 (exposure + manual gate params) |
| `wbc.launch.py` | +8 (exposure_scanner node) |
| `setup.py` | +2 (entry points) |

---

## 3 June 2026 (cont.) — SCANNING robustness fixes

### Fixes

| Fix | File | Descrizione |
|-----|------|-------------|
| Global timeout | `wbc_coordinator.py:532` | `_tick_scannning()` ora controlla timeout `scan_timeout=120s` → IDLE. Protegge da FSM bloccato, IK infinito, CHECKING_WORKSPACE perenne. |
| Param `max_workspace_reach` | `wbc_coordinator.py:1116` | Hardcoded `0.60` → parametro YAML `max_workspace_reach`. |
| Param `ws_ext_goal_tolerance` | `wbc_coordinator.py:1322` | Hardcoded `0.15` → parametro YAML `ws_ext_goal_tolerance`. |
| body_ready timeout safe | `z1_FSM.py:1671` | Invece di forzare `body_ready=True` (procedere con postura errata), **salta il punto** (`scan_mgr.advance()`) e pubblica `next_point_idx`. Se tutti i punti saltati, scan termina normalmente. |

### Nuovi parametri YAML

```yaml
ws_ext_goal_tolerance: 0.15
max_workspace_reach: 0.60
scan_timeout: 120.0
```

### File modificati

| File | +/− |
|------|-----|
| `wbc_coordinator.py` | +15/−6 |
| `z1_FSM.py` | +8/−3 |
| `wbc_params.yaml` | +3 |

## 3 June 2026 (cont.) — Paper update: FSM + adaptive perception sections

### Sections rewritten

| File | Changes |
|------|---------|
| `abstract.tex` | Rewritten without explicit numbers. Coarse-to-fine search + hybrid dual-sensor lock + adaptive anticipatory scanner. |
| `introduction.tex` | 4 principles updated: hybrid confidence-gated search, adaptive anticipatory body scanner. Contributions aligned with new terminology. |
| `active_perception.tex` | IV.A: coarse rotation + refinement + dual-sensor lock (Orbbec + RealSense). IV.B: adaptive Cartesian scan grid replacing ARC_GRID + phase-three. IV.C unchanged. |
| `system_architecture.tex` | FSM: 7→9 states (added SEMI\_LOCKING, LOCKING). Updated PRE\_APPROACH (ESTIMATING/LOCKED ×3, body\_center) and APPROACHING (adaptive grid, timeout 60s). `description` replaced with `itemize`. Frame tree extracted to `figures/frame_tree.tex`. Added hardware camera justification (why two RGBD over 360° and Spot built-in). |

### Figures

| File | Changes |
|------|---------|
| `figures/fsm.tex` | Redrawn from 5 to 9 states. Clean 3-column layout, orthogonal transitions, dashed return paths, diagonal timeout arc. |
| `figures/frame_tree.tex` | **New**. Colored TF tree: yellow (Spot), gray (Z1), red (Orbbec), teal (RealSense). Full chain `link00 → ... → link06 → camera_link → camera_color`. |
| `figures/system_block.tex` | "(5 states)" → "(9 states)". |

## 3 June 2026 — FSM phase-by-phase analysis & fixes (SEMI_LOCKING → APPROACHING)

### SEMI_LOCKING fixes

| Fix | File | Descrizione |
|-----|------|-------------|
| Pitch flush | `wbc_coordinator.py:620` | `elif not yaw_ok` → `else` + `Twist()` flush: se yaw ok ma pitch no, pubblica cmd_vel zero per applicare il body_pose pendente |
| GUIDING strict | `z1_yolo_torso_tracker.py:95,553` + `z1_yolo_torso_params.yaml:53` | `guidance_min_conf` 0.3→0.5, minimo keypoint 1→2. Riduce falsi positivi. |
| `_end_search(re_enable)` | `wbc_qp_controller.py:501,223` | IK riacceso subito nella transizione ACTIVE_SEARCH→LOOKAT, nessuna finestra di 100ms a braccio fermo. |

### PRE_APPROACH fixes

| Fix | File | Descrizione |
|-----|------|-------------|
| Body center LOOKAT | `laying_human_detector.py` + `wbc_coordinator.py` | Nuovo topic `/laying_human/body_center` (torso centroid 3D). In PRE_APPROACH il LOOKAT punta al corpo invece che a `approach_point` (punto a terra). |
| Soglia conferma | `wbc_coordinator.py:938` | `ESTIMATING` o `LOCKED` ×3 tick invece di `LOCKED` ×5. Più rapido, tollera distanza. |
| ik_done gate | `wbc_coordinator.py:1406` | La transizione LOCKING→PRE_APPROACH aspetta `/ik_done` (braccio in home prima di partire). |
| Home elevata | `wbc_qp_controller.py:511` | `_send_home()` usa Z=0.60 invece di 0.44. Braccio più alto, RealSense ha vista migliore. |

### APPROACHING fixes

| Fix | File | Descrizione |
|-----|------|-------------|
| Griglia adattiva | `wbc_qp_controller.py:521` | `_gen_cartesian_scan_grid()`: 2 pose (HOME, +X+Z) se i 4 keypoint torso conf≥0.6, altrimenti 4 pose (HOME, +X+Y, +X-Y, +X+Z) con HOME transit tra i waypoint. Advance X=0.10m su tutta la griglia. |
| `_do_set_state` pulizia | `wbc_coordinator.py:1408` | Case `CoordState.APPROACHING`: ferma cmd_vel, spegne guidance mode, disabilita navigator. |
| Timeout navigator | `wbc_coordinator.py:538` | Dopo 60s senza raggiungere handoff → IDLE (goal irraggiungibile). |
| Keypoint conf pre-scan | `z1_yolo_torso_tracker.py` + `wbc_qp_controller.py` | Nuovo topic `/torso_keypoint_conf` pubblicato in ESTIMATING/LOCKED. Usato per decidere griglia adattiva. |

### Nuovi parametri

```yaml
# wbc_params.yaml
body_center_topic: '/laying_human/body_center'
ik_done_topic: '/ik_done'
home_lock_z: 0.60
cartesian_x_advance: 0.10
pre_scan_conf_thr: 0.6
approach_timeout: 60.0

# z1_yolo_torso_params.yaml
guidance_min_conf: 0.5  # was 0.3
```

### File modificati

| File | +/− |
|------|-----|
| `laying_human_detector.py` | +12 |
| `z1_yolo_torso_tracker.py` | +18 |
| `wbc_qp_controller.py` | +62/−28 |
| `wbc_coordinator.py` | +69/−17 |
| `wbc_params.yaml` | +9 |
| `z1_yolo_torso_params.yaml` | +1/−1 |
| `PLAN.md` | +42 |

## 2 June 2026 — Adaptive Coarse + Refinement Search + GUIDING tracker state

Riscrittura completa della fase **SEARCHING**. Sostituisce la griglia fissa (18 posizioni Spot body_pose + 9 pose braccio + wrist sweep) con una ricerca adattiva a due livelli.

### Spot: coarse rotation via cmd_vel (non più body_pose yaw)

**Before:** Spot ruotava cambiando `body_pose(yaw)` su 18 posizioni fisse (6 yaw × 3 pitch).

**After:** Spot ruota con `cmd_vel.angular.z` P-control (`search_yaw_kp=0.8`, max `0.5 rad/s`):
- 6 posizioni coarse (yaw step ≈60°, 360° totali)
- Ad ogni posizione: dwell `search_coarse_dwell=5s` fermo, Orbbec/RealSense osservano
- Rotazione yaw assoluta in odom via TF `odom→body`, tolleranza `search_yaw_tolerance=0.08`
- Fallback su `_last_yaw_error` se TF non disponibile durante la rotazione

### Refinement: pitch sweep adattivo (trigger-based)

Durante il dwell coarse, se una camera vede qualcosa → entra in **refinement** (`_tick_refinement`):
- **Trigger** (`_should_refine`): RealSense tracker `== GUIDING` (qualsiasi keypoint) **oppure** Orbbec conf `≥ search_refine_trigger_orb_conf=0.30`
- Sweep pitch `search_pitch_angles=[0°, 5°, 10°]`, dwell `search_refine_dwell=4s` per pitch
- Traccia la migliore Orbbec conf + relativo `approach_point`
- `best_conf ≥ search_lock_confidence=0.70` → **LOCKING** (`_finish_refinement_lock`, fornisce già 1 campione)
- altrimenti → resume coarse dal prossimo yaw (`_finish_refinement_fail`)

### Nuovo tracker state: GUIDING

Il torso tracker (`z1_yolo_torso_tracker.py`) ha un nuovo stato **GUIDING** (giallo) oltre a `IDLE/ESTIMATING/LOCKED`. In guidance mode (attivo durante SEARCHING via `/tracker_guidance_mode`), qualsiasi keypoint valido → `GUIDING`. Usato per triggerare il refinement e per guidare il SEMI_LOCKING.

### Braccio: QP ACTIVE_SEARCH ridotto a 3 pose

**Before:** 9 pose Cartesiane + sweep polso ±15°.

**After:** `_gen_cartesian_search_grid()` genera **3 pose wide** (HOME, LEFT, RIGHT) con tilt fisso -15°, sweep Y ±0.28m, X +0.20m, Z=0.42m. Nessun wrist sweep. Loop infinito via BodySearchScanner.

### Fix

- Rimosso `_set_body_pose` dalle fasi ROTATING/DWELLING — azzerava `cmd_vel.angular.z` (commit dbf02ae, 9cb84bf)
- Guard contro `_search_initial_yaw=None` al primo TF lookup (a64f19f)
- GUIDING accettato in semi-lock + TF retry + body_pose refresh periodico (b7cb956)

### Nuovi parametri YAML (`wbc_params.yaml`)

```yaml
search_coarse_dwell: 5.0              # [s] dwell per posizione coarse
search_refine_dwell: 4.0             # [s] dwell per pitch in refinement
search_yaw_kp: 0.8                   # P-gain rotazione yaw via cmd_vel
search_yaw_tolerance: 0.08           # [rad] tolleranza yaw raggiunto
search_max_angular_vel: 0.5          # [rad/s] velocità angolare max
search_refine_trigger_orb_conf: 0.30 # [0-1] soglia Orbbec trigger refinement
search_pitch_angles: [0.0, 0.087, 0.17]  # [rad] 0°,5°,10° — usati nel refinement
```

### Files modificati

| File | Modifica |
|------|----------|
| `wbc_coordinator.py` | SEARCHING coarse cmd_vel rotation + refinement pitch sweep. `_tick_search_positions`, `_should_refine`, `_start_refinement`, `_tick_refinement`, `_finish_refinement_lock/fail`. SEMI_LOCKING accetta GUIDING. |
| `wbc_qp_controller.py` | `_gen_cartesian_search_grid` ridotto a 3 pose, tilt fisso -15°, no wrist sweep |
| `z1_yolo_torso_tracker.py` | Nuovo stato GUIDING + guidance mode (`/tracker_guidance_mode`) |
| `wbc_params.yaml` | Nuovi parametri coarse+refinement, rimossi parametri griglia fissa |

---


## 29 May 2026 — Active Perception: Cartesian Scanning + Semi-lock Relaxed

### Cartesian Scanning (sostituisce null-space SVD)

**Before:** `SEARCH_GRID` (7 pose joint-space) e `SCAN_SEQ` (11 pose joint-space) generavano waypoint via SVD del null-space projector — imprevedibili, space-giunti, nessuna relazione percezione→azione.

**After:** generazione **Cartesiana** attorno alla posizione corrente dell'EE con rotazione polso combinata:

| Fase | Modo QP | Pose | Pattern |
|------|---------|:---:|---------|
| **SEARCHING** | `ACTIVE_SEARCH` | **9** | Home, ±Z, ±Y(w=0.20m), 4 diagonali, 3 pose in +X |
| **APPROACHING** | `PERCEPTUAL_SCAN` | **6** | Home, ±Y, +Z, +X, +X+Y |
| Rotazione polso | sweep ±15° | | Amplifica copertura di ±15° per lato |

**Vincoli:** Z ≥ HOME_POS[2] (0.44m, mai sotto la home). Look-at verso body-X in SEARCHING, verso target in APPROACHING. Workspace clipping automatico.

### Semi-lock tracker state rilassato

**Before:** semi-lock si attivava solo su `/torso_tracker_state == 'LOCKED'` (torso completo, 4 keypoint stabilizzati).

**After:** accetta anche `ESTIMATING` (basta vedere 3+ keypoint qualsiasi — anche solo gambe). Il tracker pubblica `/torso_target_ee` anche durante `ESTIMATING` (prima solo in `LOCKED`) per guidare Spot.

| File | Modifica |
|------|----------|
| `z1_yolo_torso_tracker.py:606` | `_publish_target_world(interpolated)` anche in `ESTIMATING` |
| `wbc_coordinator.py:543,556,611` | Stato tracker: `'LOCKED'` → `in ('ESTIMATING', 'LOCKED')` |

### Fix: WBC spento all'ingresso in LOCKING

Quando Orbbec rileva LYING, il coordinator chiama `_set_wbc_enabled(False)` **prima** di cambiare stato in `LOCKING`. Il braccio si blocca immediatamente nella posa corrente, senza aspettare che il QP processi il messaggio di stato. In `_do_set_state(LOCKING)` il WBC viene riattivato per il movimento home.

### Fix: PRE_APPROACH timeout fallback

Aggiunto timeout in `_tick_pre_approach()`: se dopo `pre_approach_duration` (5s) RealSense non ha ancora dato 5 LOCKED consecutivi, forza comunque APPROACHING con un warning.

### Nuovi parametri YAML (sostituiscono `search_delta` / `scan_delta`)

```yaml
cartesian_step: 0.12          # [m] passo base
cartesian_step_wide: 0.20     # [m] sweep Y ampio (compensa rotazione Spot)
search_sweep_angle: 0.26      # [rad] ≈15° rotazione polso
search_timeout_per_point: 2.0 # [s] SEARCHING
scan_timeout_per_point: 3.0   # [s] APPROACHING
scan_adaptive_iters: 3        # max iterazioni adattive
kp_confidence_ok: 0.4         # soglia keypoint
```

### Files modificati

| File | Modifica |
|------|----------|
| `wbc_qp_controller.py` | Rinominati modi (`ACTIVE_SEARCH`, `PERCEPTUAL_SCAN`). Rimossi `_gen_search_poses`, `_gen_scan_poses` (null-space SVD). Aggiunti `_gen_cartesian_search_grid`, `_gen_cartesian_scan_grid` con rotazione polso combinata. Rimossi `SAFE_Q_LOW`/`SAFE_Q_HIGH`. Nuovi parametri Cartesiani. |
| `wbc_coordinator.py` | Semi-lock trigger accetta `ESTIMATING`. FASE 2 (SEARCHING→LOCKING): `_set_wbc_enabled(False)` prima del cambio stato. `_do_set_state(LOCKING)`: riattiva WBC. PRE_APPROACH timeout fallback. |
| `wbc_params.yaml` | -2 parametri (`search_delta`, `scan_delta`), +7 parametri Cartesiani |
| `z1_yolo_torso_tracker.py` | +1 riga: pubblica `/torso_target_ee` durante `ESTIMATING` |

---


## 28 May 2026 — Web Control Panel + Step Mode Debugging

### Web Control Panel (`web/`)

Interfaccia web per controllo remoto e debug via rosbridge WebSocket. Due pagine:
- **`teresa_control.html`** — pannello comandi con pulsanti, stato WBC, log eventi, navigazione RETURN con P-controller JS
- **`camera_view.html`** — feed live Orbbec + RealSense con overlay scheletro YOLO (proiezione 3D→2D), stato posture e tracker

**Comandi replicati dal keyboard controller:**
START, RETURN (con navigazione P-controller 10 Hz), UPDATE start pose, STAND, SIT, STOP, STEP (step mode)

**Camera view:**
- Scheletro YOLO 17 keypoint (COCO) proiettato 3D→2D con CameraInfo
- Colori: giallo (naso), ciano (viso), verde (busto), arancione (gambe)
- Bordo RealSense dinamico: verde LOCKED, giallo TRACKING
- Pulsanti Hide indipendenti per Orbbec e RealSense — fermano lo stream immagine ma mantengono l'overlay scheletro
- FPS counter per ogni camera
- Barra YOLO: postura, confidenza, stato tracker, joint visibili

**Nessuna dipendenza lato client** — `roslibjs` da CDN. Comunicazione solo via topic ROS attraverso rosbridge. Nessuna connessione diretta browser↔Spot.

### Step-by-Step Debug Mode

Nuovo parametro `step_mode` (default `false`) in `wbc_params.yaml`. Quando attivo, il coordinator blocca ogni transizione automatica tra stati mission e richiede conferma esplicita via `/wbc/step_confirm` (pulsante STEP o tasto `n`).

Stati sempre liberi (non gate-ati): `IDLE → SEARCHING` (manuale), `any → IDLE/WAITING_TF` (emergenza/TF loss/ESC).

### Files

| File | Modifica |
|------|----------|
| `web/teresa_control.html` | **Nuovo** — 622 righe, pannello controllo web |
| `web/camera_view.html` | **Nuovo** — 641 righe, feed telecamere + YOLO |
| `web/README.md` | **Nuovo** — documentazione interfaccia web |
| `wbc_coordinator.py` | Step mode gating: `_set_state(force)`, `_do_set_state`, `/wbc/step_pending`, `/wbc/step_confirm` |
| `wbc_keyboard_controller.py` | Tasto `n` per step confirm, subscriber `/wbc/step_pending` |
| `wbc_params.yaml` | +`step_mode: false` |

### Uso

```bash
# rosbridge
ros2 run rosbridge_server rosbridge_websocket

# HTTP server per file statici
cd web && python3 -m http.server 8000
```

Apri `http://localhost:8000/teresa_control.html`.

---

## 26 May 2026 (cont.) — Hybrid Search 360°

### Coordinated Spot + arm SEARCHING
- **Nuovi stati FSM**: `SEMI_LOCKING` e `LOCKING`. Rimosso `HANDOFF` (mai usato).
- **SEARCHING**: ricerca ibrida Spot + braccio. Spot: 18 posizioni incrementali (6 yaw × 3 pitch = 360°). Braccio: 7 pose QP-based esplorative in loop (SEARCH_GRID mode, δ=0.15, safe joint limits). Due sensori in parallelo: Orbbec (postura LYING) e RealSense (torso 3D).
- **Lock ibrido**: Orbbec full lock (conf ≥ 70%) o RealSense semi-lock (guida Spot verso il torso, braccio congelato, 3s finestra pulita per Orbbec).
- **FSM tick**: 5 Hz → 10 Hz. Lock confidence: 85% → 70%.
- **LOCKING**: braccio torna in home + 5 campioni approach_point in parallelo. Tolleranza 1s (10 tick) se Orbbec perde momentaneamente LYING. Rientro in SEARCHING dalla posizione corrente (non da zero).
- **SEMI_LOCKING**: early exit se RealSense perde il torso durante l'attesa.

### QP Controller — SEARCH_GRID mode
- `_gen_search_poses()`: 7 pose dal null-space con target virtuale body-X, δ=0.15, safe joint limits.
- `_pause_search()` / `_resume_search()` per SEMI_LOCKING.
- `_send_home()` per LOCKING.
- `_cb_wbc_state()` esteso per SEARCHING, SEMI_LOCKING, LOCKING.
- Delta resi parametrizzabili: `search_delta=0.15`, `scan_delta=0.12` (riduzione movimento per sicurezza).

### Parametri ricerca
- `search_yaw_increment=1.05` (≈60°), `search_yaw_steps=6` (=360°)
- `search_dwell=15.0`, `search_semi_lock_pause=3.0`
- Rimossi: `search_pause_per_point`, `search_yaw_offsets` (griglia fissa 3×3)

### Fix resilienza
- SEMI_LOCKING: se RealSense perde torso → torna subito a SEARCHING (non aspetta 3s inutilmente)
- LOCKING: tolleranza 10 tick (1s a 10 Hz) di assenza Orbbec prima di arrendersi
- LOCKING → SEARCHING: riprende dalla posizione corrente (non azzera idx)
- `_set_state(SEARCHING)`: full reset solo da IDLE, resume da LOCKING/SEMI_LOCKING

---

## 26 May 2026

### WBC QP Controller — refactoring arm-only + QP-based scanning

**QP Controller — due modalità:**
- `LOOKAT` (PRE_APPROACH): ω_des da errore orientamento X_ee→target, damped pseudo-inverse su J_task (3×6, solo angolare), null-space joint centering con `N @ k_null * (q_mid - q)`, FK prediction, workspace clipping, pubblica IK goal a 10 Hz. Arm-only: nessuna dipendenza da J_base.
- `SCAN_SEQ` (APPROACHING): genera 11 pose dal null-space del look-at (1 home + 6 assi ±δ + 4 diagonali), le sequenzia con BodySearchScanner (SEND_IK → wait ik_done → collect data 4s → next), fonde stime 3D (outlier rejection 0.15m), calcola e pubblica 5 FAST points.

**Rimosso P-controller Spot dal QP:**
- Il QP non pubblica più `/my_spot/cmd_vel`. Analisi fase per fase ha confermato che il P-controller era dead code in tutte le fasi e confliggeva col navigatore in WS_EXTENSION.
- Spot mosso solo da `wbc_spot_navigator` (rotate → drive → stop) e dal `wbc_coordinator` (body pose in SEARCHING/SCANNING).
- Rimossi parametri: `kp_lin_base`, `kp_ang_base`, `quality_ref`, `v_min`, `vx_max`, `wz_max`, `cmd_vel_topic`.
- Rimossi subscriber `/wbc/spot_control` e callback `_cb_spot_control`.

**wbc_approach_scanner deprecato:**
- Ridotto a stub di 40 righe. Tutta la logica di generazione pose (ARC_GRID, wrist sweep, phase 3), sequencing e FAST publishing è nel QP controller.
- Rimosso da `wbc.launch.py`.

**wbc_math.py:**
- Nuove `damped_pinv(J, damping)` e `null_space_projector(J, J_pinv)`: funzioni pure, nessuna dipendenza ROS.
- `manipulability()` mantenuta (usata per damping adattivo).
- Vecchie `compute_j_base`, `compute_j_holistic`, `wbc_split`, `wbc_split_with_yaw` spostate in fondo con marker deprecated.

**wbc.launch.py:**
- Lancia solo 3 nodi: `wbc_qp_controller` + `wbc_coordinator` + `wbc_spot_navigator`.
- Rimossi z1_mount_x/y/z launch args (non più usati senza J_base).

**Documentazione:**
- `DESCRIPTION.md`: fasi 2-3 riscritte con LOOKAT/SCAN_SEQ mode, tabella riassuntiva aggiornata, architettura WBC aggiornata.
- `INIT.md`: current state aggiornato.
- `CHANGELOG.md`: questa entry.

---

## 24 May 2026

### Paper reframing — Whole-Body Active Perception for Emergency Assessment

Il paper è stato completamente rifocalizzato. Il contributo centrale non è più il WBC ma **l'active perception-driven whole-body reconfiguration**.

**Titolo**: *TERESA: Whole-Body Active Perception for Legged Robot-Assisted Emergency Assessment*

**Dominio**: allargato da FAST ultrasound a emergency assessment (ABCDE + FAST).

**Nuovo indice**:
```
I.   Introduction                                    ✅
II.  Related Work (34 ref, 4 aree: ultrasound, WBC, active perception, posture) ✅
III. Problem Formulation (h, φ nel sistema; perception model; 5 obiettivi)      ✅
IV.  Active Perception (confidence-gated search + anticipatory scan + quality)  ✅
V.   Whole-Body Reconfiguration (WBC look-at + body reconfig + impedance)      ✅
VI.  System Architecture (7 stati FSM, frame tree, nodi, Z1 integration)       ✅
VII. Experiments                                                                 📝
VIII.Results                                                                     📝
IX.  Conclusion                                                                  📝
```

**Sezioni riscritte**:
| File | Modifica |
|------|----------|
| `paper/sections/abstract.tex` | Riscritto: active perception framework, 4 pilastri, emergency assessment |
| `paper/sections/introduction.tex` | Riscritto: ABCDE+FAST, active perception > WBC puro, 4 contributi |
| `paper/sections/related_work.tex` | Riscritto: 4 sottosezioni (US, WBC, active perception, posture), +26 ref |
| `paper/sections/problem_formulation.tex` | Riscritto: h, φ nel sistema, perception model, N target, 5 obiettivi |
| `paper/sections/active_perception.tex` | **Nuovo**: confidence-gated search, anticipatory scan + WBC, QualityMonitor |
| `paper/sections/method.tex` | Riscritto: WBC look-at (non olistico), body reconfig grid search, impedance Z1 |
| `paper/sections/system_architecture.tex` | Riscritto: 7 stati, frame tree TiKZ, tabella nodi, Z1 integration, 5 terminali |
| `paper/TERESA.tex` | Nuovo titolo, nuove keywords, aggiunto `\input{active_perception}` |
| `paper/references.bib` | +26 nuove reference (totale 45) |
| `paper/sections/acronyms.tex` | Aggiunto ABCDE |
| `INIT.md` | Questa sezione |
| `PLAN.md` | Aggiunta sezione paper + cosa manca |

**WBC nel nuovo framing**: il WBC è posizionato come tool al servizio dell'active perception (arm look-at durante l'anticipatory scanning), non come contributo centrale. L'impedance control custom per Z1 (che non ha impedance nativo) è descritto come contributo tecnico abilitante.

**Narrativa**: TERESA non è "un WBC applicato all'ecografia" — è un legged robot che usa l'active perception per riconfigurare il corpo e massimizzare la qualità percettiva, con WBC e impedance come strumenti al servizio di questo obiettivo.

---

### WS_EXTENSION redesign + Body Pose Optimization implementata + CHECKING_WORKSPACE ovunque

**Before:**
- WS_EXTENSION usava il vecchio WBC QP per muovere Spot con un bounding box (forward +20cm, lateral ±20cm, back -50cm) e salvava un'ancora di posizione
- Il FSM triggerava WS_EXTENSION via `/wbc/ws_request` e aspettava `/ik_done` per tornare in SCANNING
- Il FSM aveva uno stato dedicato `REQUESTING_WS_EXT` con retry fino a 3 volte
- `_optimize_body_poses()` era uno stub che pubblicava solo un marker di debug — il grid search non funzionava
- `_apply_fast_body_pose(idx)` e `_tick_fast_settle()` non esistevano (chiamate ma mai definite, causavano crash)
- `self._mount_x/y/z` erano usati ma mai dichiarati
- `/wbc/body_ready` veniva pubblicato solo a fine ciclo FAST, mai per singolo punto
- `CHECKING_WORKSPACE` veniva eseguito solo per il centro hub (idx=0), i punti 1-4 andavano direttamente `SCAN_PAUSE → APPROACHING`
- `_workspace_future` non veniva mai resettato, causando riutilizzo di risultati vecchi

**After:**
- **WS_EXTENSION come grid search matematico**: invece di usare il QP per muovere Spot, il coordinator esegue un grid search 4D (altezza, pitch, dx, dy) per trovare la combinazione ottimale. Spot viene guidato dal `wbc_spot_navigator` verso il goal calcolato (timeout 5s). Nessun bounding box, nessun `/wbc/ws_request`, nessun QP coinvolto.
- **Body Pose Optimization implementata**: `_optimize_body_poses()` ora esegue un grid search reale (3 altezze × 4 pitch = 12 combinazioni) per ogni punto FAST. Per ogni combinazione simula matematicamente dove si troverebbe link00 in odom e calcola la distanza dal sweet spot `[0.35, 0, 0.30]` in frame link00.
- **WS_EXTENSION fallback automatico**: se dopo l'ottimizzazione (h,p) il punto è ancora fuori workspace, il coordinator esegue automaticamente il grid search 4D (h, p, dx, dy). Se il target è ancora irraggiungibile dopo il WS_EXT, il coordinator pubblica comunque `body_ready` e il FSM deciderà in `CHECKING_WORKSPACE`.
- **`_apply_fast_body_pose(idx)`**: applica body_pose (h,p) per il punto corrente, o se necessario avvia WS_EXT (navigator drive + body_pose). Poi avvia il timer di settle.
- **`_tick_fast_settle()`**: monitora il settle time (1.5s) o il completamento del navigator drive (distanza < 0.15m o timeout 5s). Pubblica `body_ready` quando completato.
- **`_mount_x/y/z`**: parametri dichiarati e letti dal YAML.
- **`_current_body_height`**: tracciato in `_set_body_pose()` per permettere la simulazione della posizione nominale del corpo Spot.
- **`CHECKING_WORKSPACE` ovunque**: ora eseguito per OGNI punto FAST (non solo idx=0):
  - `SCAN_PAUSE → CHECKING_WORKSPACE → (OK) APPROACHING | (was_clipped) skip o procedi`
  - Per idx=0: usa il tracker live (comportamento invariato)
  - Per idx>0: calcola il target da `center_approach_pose + offset`, nessuna dipendenza dal tracker
  - `_workspace_future` resettato a `None` in tutti i rami di uscita
- **Skip logica**: se `was_clipped`:
  - idx=0: procede con target clippato (il centro hub serve per salvare `center_approach_pose`)
  - idx>0: salta il punto, avanza al successivo via `scan_mgr.advance()`, pubblica `next_point_idx`, torna in `SCAN_PAUSE`
- **Vecchio WS_EXTENSION rimosso**: stati `WS_EXTENSION` e `REQUESTING_WS_EXT`, callback `_cb_ws_req`/`_cb_ik_done` (per WS_EXT), `_save_ws_ext_anchor`, `_tick_ws_extension`, bounding box params, publishers `pub_wbc_ws_request`/`pub_wbc_ee_goal`, variabili `_ws_ext_retries`/`_ws_ext_confirmed` — tutto rimosso.
- **Nuovi parametri YAML**: `z1_mount_x/y/z` nel coordinator, `ws_ext_dx_steps/max`, `ws_ext_dy_fwd/bwd_max`, `navigator_timeout`.

### Files modificati

| File | Modifica |
|------|----------|
| `wbc_coordinator.py` | Body pose optimization implementata, WS_EXTENSION grid search 4D + navigator drive, settle monitor, mount params dichiarati, vecchio WS_EXTENSION rimosso |
| `z1_FSM.py` | CHECKING_WORKSPACE per tutti i punti, target computation per idx>0, skip logic, vecchio REQUESTING_WS_EXT rimosso, _workspace_future reset |
| `wbc_params.yaml` | +z1_mount_x/y/z (coordinator section), +ws_ext_dx_*, +navigator_timeout, -ws_ext_fwd/lat/bwd_limit |
| `INIT.md` | Questo changelog |
| `DESCRIPTION.md` | Aggiornato con architettura corrente |

---

## Recent Changes (23 May 2026)

### WBC refactoring — Spot/braccio decoupled + body scan anticipato

**Before:** WBC olistico fragile — arm e Spot dipendevano da `wbc_split(J_hol, v_des)` con TF cross-machine. Errori TF (clock desync SpotCore/PC) bloccavano sia braccio che Spot. Body scan eseguito dopo handoff (10-15s extra).

**After:**
- **Spot P-controller**: `wbc_qp_controller` ora usa un P-controller indipendente (`vx = kp_lin_base × dist`, `wz = kp_ang_base × angle`) basato solo su `odom→body` (1 TF hop). Il braccio mantiene il WBC look-at (`J_arm` damped pseudo-inverse). Quality scaling invariato.
- **Cmd_vel cache**: quando un TF lookup fallisce, il QP ripubblica l'ultimo `cmd_vel` valido invece di andare muto. Spot non perde mai il contatto.
- **PRE_APPROACH active perception**: il coordinator ora attende 5 tick consecutivi di RealSense `LOCKED` (invece di timer fisso 5s). Timeout fallback 5s.
- **`wbc_spot_navigator.py`**: navigatore semplificato per APPROACHING. Legge `/wbc/ee_goal` in odom, rotate → drive → stop. P-controller robusto. Il WBC non muove più Spot (`/wbc/spot_control=False` in APPROACHING).
- **`wbc_approach_scanner.py`**: body scan eseguito **durante APPROACHING** invece che dopo. ARC_GRID (8 pose: fase 1 home × wrist ±8° + fase 2 arc ±4cm × wrist). BodySearchScanner reale con feed da `/torso_scan_point`. Pubblica `/z1/fast_points` e `/z1/fast_ready`.
- **Fase 3 condizionale**: se keypoint hanno confidenza < 0.50 dopo ARC_GRID, si esegue fase 3 adattiva in SCANNING. Altrimenti skip.
- **Z1 FSM saltata**: la FSM riceve `/z1/fast_ready=True` + `/z1/fast_points` → salta `BODY_SCANNING` → va direttamente a `CHECKING_WORKSPACE → FAST`.
- **Riduzione movimenti**: wrist ±8° (era ±12°), arc grid ±4cm (era ±6cm). Meno movimento durante navigazione.
- **TF monitor continuo**: pubblica `/wbc/tf_ready` True/False a ogni tick (2s). Se TF degradano → coordinator torna in `WAITING_TF` → keyboard blocca `s`.
- **ESC emergency stop**: tasto ESC sul keyboard → `/wbc/restart=False` + `cmd_vel=0`, stop immediato.

### Files nuovi

| File | Ruolo |
|------|-------|
| `spot_control/wbc_spot_navigator.py` | Navigator semplificato per Spot in APPROACHING |
| `spot_control/wbc_approach_scanner.py` | Body scan multi-view + WBC look-at + FAST points |
| `rviz/wbc_debug.rviz` | RViz config: TF tree + goal marker + debug line body→goal |

### Files modificati

| File | Modifica |
|------|----------|
| `wbc_qp_controller.py` | P-controller Spot (1 TF) + cache cmd_vel |
| `wbc_coordinator.py` | PRE_APPROACH active perception, APPROACHING spot_control=False, SCANNING con WBC enabled per fase 3 |
| `wbc_params.yaml` | +kp_lin_base, +kp_ang_base; quality_max 0.50→0.20 |
| `wbc.launch.py` | +wbc_spot_navigator + wbc_approach_scanner |
| `setup.py` | +entry points per nuovi nodi |
| `z1_FSM.py` | +subscriber /z1/fast_ready, skip BODY_SCANNING |
| `teresa_core.launch.py` | static_transform_publisher in formato Jazzy; realsense2_camera_node lanciato direttamente (no nesting) |
| `tf_monitor.py` | Monitor continuo (no _done flag), world→link06 check, topic corretto /camera/camera/color/image_raw |
| `wbc_keyboard_controller.py` | ESC stop, display pulito, gestione TF loss |
| `z1_realsense.launch.py` | +log_level:info per compatibilità Jazzy |
| `DESCRIPTION.md` | Aggiornato con architettura corrente |

### FAST Body Pose Optimization (23 May)

**Obiettivo:** Spot non resta fermo a `handoff_height=-0.15m` per tutti i 5 punti FAST. Per ogni punto, il coordinator esegue un grid search offline su 3 altezze × 4 pitch per trovare la combinazione che porta il target più vicino al centro workspace Z1 (`sweet_spot: [0.35, 0, 0.30]` in link00). Spot si aggiusta **prima** che il braccio esegua il punto.

**Modifiche:**
- `wbc_coordinator.py`: subscriber `/z1/fast_points` (PoseArray dal wbc_approach_scanner). Grid search `_optimize_body_poses()` — 1 TF lookup world→odom, poi matematica locale su 3×4 combinazioni. Publisher `/wbc/body_ready` dopo settle 1.5s.
- `z1_FSM.py`: dopo ogni punto FAST, pubblica `/z1/next_point_idx` per segnalare al coordinator il prossimo punto. In `SCAN_PAUSE`, attende `/wbc/body_ready=True` prima di procedere. Timeout fallback 3s.
- `wbc_params.yaml`: `body_grid_heights: [-0.20, -0.18, -0.15]`, `body_grid_pitches: [0.0, 0.087, 0.17, 0.26]`, `body_sweet_spot: [0.35, 0.0, 0.30]`, `body_settle_time: 1.5`.

**Vincoli:** altezza [-0.20, -0.15] m; pitch [0°, 15°]; yaw mai cambiato.

**Paper note:** questa non è WBC in senso stretto (Spot e braccio non si muovono simultaneamente). È più precisamente **whole-body planning** o **cooperative mobile manipulation**. Vedi `PLAN.md` per i paper angle suggeriti.

### Soft Handoff a 20cm + Risoluzione conflitto IK goal

**Soft handoff:** Spot non arriva subito a 5cm. A 20cm si ferma (navigator in pausa via `/wbc/spot_control=False`) finché lo scanner non completa ARC_GRID. Quando i FAST points sono pronti, il navigator viene sbloccato e Spot completa gli ultimi 15cm.

**Conflitto IK goal risolto:** il WBC QP e il `wbc_approach_scanner` pubblicavano entrambi su `/wbc/ik_goal_pose`, causando conflitti. Ora:
- **PRE_APPROACH**: solo WBC QP pubblica look-at (scanner non ancora attivo)
- **APPROACHING**: solo scanner pubblica grid+look-at (WBC QP non esegue IK)
- **SCANNING**: solo scanner per fase 3 (attivata via `/wbc/state='SCANNING'`, non via `/wbc/enable`)

**Modifiche:**
- `wbc_approach_scanner.py`: si attiva su `/wbc/state='APPROACHING'` invece che `/wbc/enable=True`
- `wbc_coordinator.py`: non abilita WBC in APPROACHING; soft handoff a 20cm se scanner non ancora pronto
- `wbc_spot_navigator.py`: subscriber `/wbc/spot_control` — se False, zero cmd_vel (pausa)
- `wbc_params.yaml`: `soft_handoff_distance: 0.20`

---

## Recent Changes (21 May 2026)

### `teresa_perception.launch.py` — unified perception launch

**Before:** 2 terminali separati per `spot_perception` (Orbbec) e `z1_perception` (RealSense).

**After:** `teresa_perception.launch.py` usa `IncludeLaunchDescription` per richiamare entrambi i launch originali in un unico terminale. Comportamento identico, zero duplicazione.

```bash
ros2 launch spot_control teresa_perception.launch.py
ros2 launch spot_control teresa_perception.launch.py use_orbbec_driver:=true
```

Argomenti: `use_orbbec_driver` (default `false`, driver già in `teresa_core`), `test_mode`, `use_tracker`, `use_surface`.

### `teresa_demo` package — visitor demonstration

Nuovo package standalone per dimostrazioni senza telecamere/WBC/percezione. Spot e Z1 si muovono contemporaneamente in pattern di searching.

**Files:** `src/teresa_demo/`
| File | Ruolo |
|------|-------|
| `visitor_demo_node.py` | Orchestratore con due loop paralleli: Spot griglia 3×3 body_pose + Z1 arm state machine |
| `visitor_demo.launch.py` | Z1 bringup + `z1_ik_to_jtc` + demo node (nessuna TF statica, nessuna telecamera) |
| `demo_params.yaml` | Griglia Spot (9 punti) + 4 pose braccio + topic IK |

**Comportamento:**
- **Spot**: griglia 3×3: 3 yaw × 3 pitch, 3s per punto, loop continuo (stessi parametri del WBC SEARCHING)
- **Braccio**: home → front-left → home → front-right → home → front-up → home → front-down → loop
- A **Ctrl-C**: entrambi tornano in posizione di partenza (Spot in piedi, braccio in home)

**Uso:**
```bash
ros2 launch teresa_demo visitor_demo.launch.py
# Richiede spot_ros2 su SpotCore (per body_pose topic)
```

### Compatibilità ROS 2 Jazzy

Il workspace gira su **Jazzy** (non Humble). Due fix necessari:
- **Liste annidate YAML**: `[[x,y,z,...], [...]]` non supportato dal parser parametri Jazzy → flattenato a lista singola con parsing a blocchi di 7 nel codice
- **Default `[]`**: interpretato come `BYTE_ARRAY` invece di `DOUBLE_ARRAY` → usare `[0.0]` per fissare il tipo

---

## Recent Changes (20 May 2026)

### TF monitor & keyboard-controlled startup — 7 catene TF + 3 topic

**Before:** WBC coordinator partiva subito in `SEARCHING` all'avvio. Se i TF SpotCore non erano ancora disponibili (DDS non pronto, clock desincronizzato), i nodi fallivano silenziosamente con errori TF criptici. L'utente doveva indovinare il problema.

**After:**
- **Nuovo nodo `tf_monitor.py`** (lanciato da `teresa_core.launch.py`): controlla ogni secondo 7 catene TF + 3 topic hardware. Appena TUTTI sono pronti pubblica `/wbc/tf_ready = True` e logga un banner di conferma. Non si limita più al solo `odom→body`.
- **Nuovo stato `WAITING_TF` nel coordinator**: il WBC parte in `WAITING_TF`, aspetta `/wbc/tf_ready`, poi passa a `IDLE`. Solo da `IDLE` il keyboard controller può farlo partire (`/wbc/restart → SEARCHING`).
- **Keyboard controller blocca `s`**: se premi "s" prima che i TF siano pronti, il nodo ti avverte e non fa nulla. Quando `/wbc/tf_ready` arriva, stampa `[TF READY] SpotCore connesso — premi "s" per iniziare`.
- **Messaggi di errore TF esplicativi**: tutti i `lookup_transform` ora loggano diagnostica chiara. I messaggi sono throttled: ogni 5s quando TF mai visto, ogni 2s se perso a runtime.
- **Script `tf_diag.sh`**: diagnostica standalone (basta eseguirlo senza far girare i nodi TERESA).
- **Helper `_tf_lookup()` / `_tf_transform()`**: introdotti in tutti i nodi (`wbc_qp_controller`, `wbc_coordinator`, `spot_goal_navigator`) per centralizzare la gestione errori TF.

### teresa_core refactoring (20 May 2026 — same day)

**Obiettivo:** ridurre i 6 terminali a 3 separando driver hardware dalla logica applicativa.

**Before:** ogni launch file lanciava i propri driver + le proprie TF statiche. `tf_monitor` in `wbc.launch.py` controllava solo `odom→body`. `body→link00` pubblicato dal WBC a runtime.

**After:**
- **`teresa_core.launch.py`** (nuovo) — unico terminale per TUTTI i driver hardware:
  - Orbbec Femto Bolt driver
  - Z1 bringup + RealSense via `z1_realsense.launch.py` (con `use_rviz:=false`, `use_camera_tf:=false`)
  - **4 TF statiche** centralizzate nel core
  - `tf_monitor` (spostato da `wbc.launch.py`)
- **`tf_monitor`** ora controlla **8 catene TF** + 3 topic
- **`z1_realsense.launch.py`**: nuovo arg `use_camera_tf` (default `true`)
- **`spot_perception.launch.py`**: arg `use_orbbec_driver` (default `true`)
- **`wbc_qp_controller.py`**: rimosso `StaticTransformBroadcaster` di `body→link00`
- **`wbc.launch.py`**: rimosso `tf_monitor`, lancia solo `wbc_qp_controller` + `wbc_coordinator`

### Files modificati in questo commit

| File | Modifica |
|------|----------|
| `teresa_core.launch.py` | **Nuovo** |
| `tf_monitor.py` | 7 catene TF + 3 topic (prima: solo `odom→body`) |
| `wbc_qp_controller.py` | Rimosso `StaticTransformBroadcaster` per `body→link00` |
| `wbc.launch.py` | Rimosso `tf_monitor` |
| `spot_perception.launch.py` | Aggiunto arg `use_orbbec_driver` |
| `z1_realsense.launch.py` | Aggiunto arg `use_camera_tf` |

---

## Recent Changes (12 May 2026)

### SEARCHING grid search + confidence lock + body_pose fix

**Before:**
- SEARCHING continuous rotation with pitch ramp
- `quaternion_from_euler(pitch, 0.0, 0.0)` → pitch applied as roll (tilted sideways)
- `body_pose` published without `cmd_vel` flush → spot_driver never applied it
- IDLE→APPROACHING shortcut bypassed SEARCHING

**After:**
- SEARCHING: **3×3 grid** — 3 yaw positions (center, +10°, -10°) × 3 pitch angles (5°, 10°, 15°). At each point Spot pauses 3s for the camera to observe, then moves to the next via `body_pose`. Grid completes after all 9 points (~27s) → IDLE.
- **Confidence lock**: when `confidence ≥ 0.85`, Spot freezes (no pose changes) and collects 10 approach_point samples in odom (~2s @5Hz). Target = mean of 10 samples → `QualityMonitor.set_target()` → PRE_APPROACH. If confidence drops < 0.85 during sampling → lock lost, resume grid from current point.
- `quaternion_from_euler(0.0, pitch, yaw)` → pitch on Y axis (nose-down), yaw on Z (orientation)
- Every `_set_body_pose()` call publishes a zero `Twist` on `/my_spot/cmd_vel` to flush to spot_driver
- PRE_APPROACH entry resets body_pose to (0,0) → Spot stands upright for stable approach
- IDLE→APPROACHING shortcut **removed** — all approaches go through SEARCHING
- `_check_lying_timeout` now excludes APPROACHING — Spot never aborts approach once committed
- `_cb_approach` skips `QualityMonitor.try_init()` during SEARCHING (target set only via lock)

### New parameters
| Parameter | Value | Meaning |
|-----------|-------|---------|
| `search_pitch_angles` | [0.087, 0.17, 0.26] | 5°, 10°, 15° pitch grid |
| `search_yaw_offsets` | [0.0, 0.17, -0.17] | center, +10°, -10° yaw grid |
| `search_pause_per_point` | 3.0s | Pause per grid point |
| `search_lock_confidence` | 0.85 | Confidence to freeze and sample |
| `search_lock_samples` | 10 | Samples averaged as target |

### Removed parameters
- `search_timeout`, `search_angular_speed`, `search_pitch_max`, `search_pitch_min`, `search_pitch_steps`, `search_detection_frames`, `orbbec_confidence_threshold`

### Keyboard controller + WBC restart
- New node `wbc_keyboard_controller.py`: keyboard-driven Spot control with WBC integration
- Keys: `s`=start (save pose + SEARCHING), `r`/`q`=return to start, `u`=update start pose, `c`/`a`=sit/stand
- WBC gains `/wbc/restart` subscriber (Bool): True → IDLE→SEARCHING, False → any→IDLE
- During return navigation, keyboard node takes over `/my_spot/cmd_vel`

### PRE_APPROACH exits on IK_done + QP publishes mount TF
- PRE_APPROACH now exits when `/ik_done` = True (arm completed look-at), not on 5s timer
- `pre_approach_duration` parameter kept for backward compat but no longer used
- QP controller now publishes `my_spot/body → link00` static TF at 10 Hz via `StaticTransformBroadcaster`
- `search_lock_samples` reduced 10 → 5 (faster lock, 1s instead of 2s)

### Files modified
`wbc_coordinator.py`, `wbc_qp_controller.py`, `wbc_params.yaml`, `wbc.launch.py`, `wbc_keyboard_controller.py` (new), `setup.py`

---

## Recent Changes (11 May 2026)

### Orbbec TF collision fix — camera renamed to `orbbec`

**Before:** Orbbec and RealSense both published TF frames `camera_link`, `camera_color_optical_frame`. The approach_point (Orbbec, on tripod) was transformed through the RealSense chain (`link06 → camera_link`) instead of the static tripod TF.

**After:**
- Orbbec driver launched with `camera_name: 'orbbec'` → TF frames become `orbbec_link`, `orbbec_color_optical_frame`
- Static TF chain: `my_spot/body → orbbec_link → orbbec_color_optical_frame` (separate from RealSense)
- YOLO skeleton topics: `/camera/*` → `/orbbec/*`

### Handoff distance analysis — offset already present
Approach point computed in `laying_human_detector.py` already includes offset: `dist = bbox_half(≥0.30) + approach_margin(0.05) + spot_front_offset(0.50) = 0.85m`. At handoff (5cm from approach_point), Spot front ~5cm from patient bbox edge. Arm reach covers the rest (~60cm).

### Files modificati
`spot_perception.launch.py`, `yolo_skeleton_spot.py`

---

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
- **10 Hz** (`update_period: 0.1`) → Spot fluido
- look-at: `x_ee = target_link00 - clipped_pos` → coerente con posizione IK
- `compute_ee_orientation_minrot()` — rotazione minima da home X a x_ee, polso rilassato
- `ik_rot_weight: 0.7` (era 0.3) — IK rispetta l'orientazione
- `orientation_mode: "minrot"` default in `wbc_params.yaml`

### QualityMonitor (sostituisce `_PositionKalman`)
- `target` = media prime `quality_buf_size=3` misure in odom → inizializzato
- `target` aggiornato solo se `posture_confidence > best_conf + confidence_margin` (0.10)
- `quality` = `max_q * (1 - posture_confidence)` + crescita lineare senza confidence
- Pubblicato su `/wbc/target_uncertainty` in **metri** (non più sigma)
- `v_scale = v_min + (1 - v_min) / (1 + quality / quality_ref)` → **mai zero**

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
`z_delta` (chance-constraint dead zone), `approach_kf_process_noise`, `approach_kf_meas_noise`

### Files modificati
`wbc_coordinator.py`, `wbc_qp_controller.py`, `wbc_params.yaml`, `teresa_utils/orientation.py`, `z1_ik_jtc_params.yaml`

---

## Recent Changes (6 May 2026)

- **Arm twist fix (WBC)**: EE orientation now computed geometrically (X_ee toward target, Y_ee from home via Gram-Schmidt) instead of using the approach_point yaw orientation that caused a roll twist around the X axis. Same algorithm as `z1_FSM._orientation_for_xee()`.
- **Shared utilities**: new `teresa_utils` package with `orientation.py` — `compute_ee_orientation`, `quat_to_rot`, `rot_to_quat`, `normalize_angle`. Eliminates duplicate code across 4 files.
- **`workspace_safety_margin` unified**: all defaults aligned to 0.05 m (were 0.05 in YAML but 0.30 in code).
- **`REQUESTING_WS_EXT` race fixed**: FSM now proceeds to CHECKING_WORKSPACE on SCANNING even if WS_EXTENSION was missed between ticks.
- **`wbc_startup_timeout` configurable**: 30s default (was hardcoded 10s). Parameter in `z1_fsm_params.yaml`.
- **`wait_ik_timeout_s` robustness**: declared in FSM (not only via ScanManager) — no crash if `from_params()` fails.
