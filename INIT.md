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

### Flusso WBC attuale (5 terminali, senza `teresa_app`)

`teresa_app.launch.py` NON esiste ancora. Il flusso attuale usa i singoli launch:

```
T0: SpotCore — spot_ros2 attivo (TF odom→body)
T1: ros2 launch spot_control teresa_core.launch.py        (driver + TF + tf_monitor)
T2: ros2 launch spot_control teresa_perception.launch.py  (Orbbec + RealSense perception)
T3: ros2 launch z1_vision z1_control.launch.py            (IK + switch + mux + FSM)
T4: ros2 launch spot_control wbc.launch.py                (WBC QP + coordinator)
T5: ros2 run spot_control wbc_keyboard_node               (tastiera)
```

### Files nuovi/modificati

| File | Modifica |
|------|----------|
| `spot_control/launch/teresa_perception.launch.py` | **Nuovo** — unifica spot_perception + z1_perception |
| `teresa_demo/` | **Nuovo** — intero package demo visitatori |
| `teresa_demo/visitor_demo_node.py` | **Nuovo** — orchestratore Spot + Z1 parallelo |
| `teresa_demo/launch/visitor_demo.launch.py` | **Nuovo** — launch self-contained |
| `teresa_demo/config/demo_params.yaml` | **Nuovo** — parametri demo |

---

### TF monitor & keyboard-controlled startup — 7 catene TF + 3 topic

**Before:** WBC coordinator partiva subito in `SEARCHING` all'avvio. Se i TF SpotCore non erano ancora disponibili (DDS non pronto, clock desincronizzato), i nodi fallivano silenziosamente con errori TF criptici. L'utente doveva indovinare il problema.

**After:**
- **Nuovo nodo `tf_monitor.py`** (lanciato da `teresa_core.launch.py`): controlla ogni secondo 7 catene TF + 3 topic hardware. Appena TUTTI sono pronti pubblica `/wbc/tf_ready = True` e logga un banner di conferma. Non si limita più al solo `odom→body`.
- **Nuovo stato `WAITING_TF` nel coordinator**: il WBC parte in `WAITING_TF`, aspetta `/wbc/tf_ready`, poi passa a `IDLE`. Solo da `IDLE` il keyboard controller può farlo partire (`/wbc/restart → SEARCHING`).
- **Keyboard controller blocca `s`**: se premi "s" prima che i TF siano pronti, il nodo ti avverte e non fa nulla. Quando `/wbc/tf_ready` arriva, stampa `[TF READY] SpotCore connesso — premi "s" per iniziare`.
- **Messaggi di errore TF esplicativi**: tutti i `lookup_transform` ora loggano diagnostica chiara:
  ```
  TF my_spot/odom → my_spot/body non disponibile.
    Diagnostica: ros2 topic list | grep tf
    Verifica: 1) spot_ros2 attivo su SpotCore?  2) ROS_DOMAIN_ID uguale?  3) spot_name='my_spot'?
  ```
  I messaggi sono throttled: ogni 5s quando TF mai visto, ogni 2s se perso a runtime.
- **Script `tf_diag.sh`**: diagnostica standalone (basta eseguirlo senza far girare i nodi TERESA).
- **Helper `_tf_lookup()` / `_tf_transform()`**: introdotti in tutti i nodi (`wbc_qp_controller`, `wbc_coordinator`, `spot_goal_navigator`) per centralizzare la gestione errori TF.

### Nuovo flusso operativo (3 terminali via teresa_core)

```
1. SpotCore acceso con spot_ros2
2. PC: 3 terminali in ordine:
   T1: ros2 launch spot_control teresa_core.launch.py     (driver + TF + monitor)
   T2: ros2 launch spot_control teresa_app.launch.py      (percezione + controllo + WBC)
   T3: ros2 run spot_control wbc_keyboard_node             (tastiera)
3. Aspettare "[TUTTO PRONTO]" da T1, poi "[TF READY] premi s" da T2
4. Premere "s" in T3 → missione parte
```

**Nota:** i 6 terminali standalone restano disponibili per debug (vedi sezione "Running Spot + Z1 WBC").

### FSM aggiornato

```
WAITING_TF ──(/wbc/tf_ready)──► IDLE ──(/wbc/restart, tastiera)──► SEARCHING ──► ...
```

### Files nuovi/modificati

| File | Modifica |
|------|----------|
| `spot_control/tf_monitor.py` | **Nuovo** — nodo monitor TF standalone |
| `scripts/tf_diag.sh` | **Nuovo** — script diagnostico TF |
| `wbc_coordinator.py` | Stato `WAITING_TF`, subscriber `/wbc/tf_ready`, helper `_tf_lookup/transform`, errori esplicativi |
| `wbc_qp_controller.py` | Helper `_tf_lookup/transform`, flag `_tf_ready`, errori esplicativi |
| `spot_goal_navigator.py` | Helper `_tf_lookup/transform`, flag `_tf_ready`, errori esplicativi |
| `wbc_keyboard_controller.py` | Subscriber `/wbc/tf_ready`, blocca `s` se TF non pronto |
| `wbc.launch.py` | Aggiunto `tf_monitor` al launch |
| `setup.py` | Entry point `tf_monitor`, data `scripts/*.sh` |

### teresa_core refactoring (20 May 2026 — same day)

**Obiettivo:** ridurre i 6 terminali a 3 separando driver hardware dalla logica applicativa.

**Before:** ogni launch file lanciava i propri driver + le proprie TF statiche. `tf_monitor` in `wbc.launch.py` controllava solo `odom→body`. `body→link00` pubblicato dal WBC a runtime.

**After:**

- **`teresa_core.launch.py`** (nuovo) — unico terminale per TUTTI i driver hardware:
  - Orbbec Femto Bolt driver (stessi parametri di prima)
  - Z1 bringup + RealSense via `z1_realsense.launch.py` (con `use_rviz:=false`, `use_camera_tf:=false`)
  - **4 TF statiche** centralizzate nel core:
    - `my_spot/body → orbbec_link` (0.30, 0, 0.15)
    - `orbbec_link → orbbec_color_optical_frame` (-1.5708, 0, -1.5708)
    - `my_spot/body → link00` (z1_mount_x/y/z, default 0.20/0.0/0.20)
    - `link06 → camera_link` (0, 0, 0.05)
  - `tf_monitor` (spostato da `wbc.launch.py`): 4 condizioni → `/wbc/tf_ready`

- **`tf_monitor`** ora controlla **7 catene TF** (non più solo `odom→body`):
  1. `my_spot/odom → my_spot/body` (SpotCore DDS)
  2. `my_spot/body → link00` (Z1 mount on Spot)
  3. `my_spot/body → orbbec_link` (Orbbec mount)
  4. `orbbec_link → orbbec_color_optical_frame` (Orbbec optical)
  5. `link00 → link06` (Z1 arm, robot_state_publisher)
  6. `link06 → camera_link` (RealSense mount)
  7. `camera_link → camera_color_optical_frame` (RealSense optical)
  - Più 3 topic: `/joint_states`, `/orbbec/color/image_raw`, `/camera/color/image_raw`
  - Solo quando TUTTE vere → `/wbc/tf_ready = True`

- **`z1_realsense.launch.py`**: nuovo arg `use_camera_tf` (default `true`). Quando `false` (da `teresa_core`), salta la pubblicazione della TF `link06→camera_link`.
- **`spot_perception.launch.py`**: arg `use_orbbec_driver` (default `true`). Quando `false` salta driver Orbbec + TF statiche.
- **`wbc_qp_controller.py`**: rimosso `StaticTransformBroadcaster` di `body→link00` (ora è una static TF del core).
- **`wbc.launch.py`**: rimosso `tf_monitor` (ora nel core), lancia solo `wbc_qp_controller` + `wbc_coordinator`.

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

## Recent Changes (11 May 2026)

### Orbbec TF collision fix — camera renamed to `orbbec`

**Before:** Orbbec and RealSense both published TF frames `camera_link`, `camera_color_optical_frame`. The approach_point (Orbbec, on tripod) was transformed through the RealSense chain (`link06 → camera_link`) instead of the static tripod TF. WBC coordinator stayed in IDLE because `_cb_approach` couldn't resolve `approach_point` → `my_spot/odom` correctly.

**After:**
- Orbbec driver launched with `camera_name: 'orbbec'` → TF frames become `orbbec_link`, `orbbec_color_optical_frame`
- Static TF chain: `my_spot/body → orbbec_link → orbbec_color_optical_frame` (separate from RealSense)
- YOLO skeleton topics: `/camera/*` → `/orbbec/*`
- Perception nodes `frame_id`: `orbbec_color_optical_frame`

### Handoff distance analysis — offset already present

Approach point computed in `laying_human_detector.py` already includes offset:
```
dist = bbox_half(≥0.30) + approach_margin(0.05) + spot_front_offset(0.50) = 0.85m
```
At handoff (5cm from approach_point), Spot front ~5cm from patient bbox edge. Arm reach covers the rest (~60cm). Approach is lateral (Spot ⊥ patient).

### Files modificati
`spot_perception.launch.py`, `yolo_skeleton_spot.py`

---

## Recent Changes (12 May 2026)

### SEARCHING grid search + confidence lock + body_pose fix

**Before:**
- SEARCHING continuous rotation with pitch ramp
- `quaternion_from_euler(pitch, 0.0, 0.0)` → pitch applied as roll (tilted sideways)
- `body_pose` published without `cmd_vel` flush → spot_driver never applied it
- IDLE→APPROACHING shortcut bypassed SEARCHING

**After:**
- SEARCHING: **3×3 grid** — 3 yaw positions (center, +10°, -10°) × 3 pitch angles (5°, 10°, 15°).
  At each point Spot pauses 3s for the camera to observe, then moves to the next via `body_pose`.
  Grid completes after all 9 points (~27s) → IDLE.
- **Confidence lock**: when `confidence ≥ 0.85`, Spot freezes (no pose changes) and collects 10 approach_point samples in odom (~2s @5Hz). Target = mean of 10 samples → `QualityMonitor.set_target()` → PRE_APPROACH.
  If confidence drops < 0.85 during sampling → lock lost, resume grid from current point.
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
- `search_timeout` — grid completes when all points visited
- `search_angular_speed` — no continuous rotation, yaw via body_pose
- `search_pitch_max`, `search_pitch_min`, `search_pitch_steps` — replaced by explicit pitch array
- `search_detection_frames` — replaced by confidence lock + sample count
- `orbbec_confidence_threshold` — replaced by `search_lock_confidence`

### Keyboard controller + WBC restart

- New node `wbc_keyboard_controller.py`: keyboard-driven Spot control with WBC integration
- Keys: `s`=start (save pose + SEARCHING), `r`/`q`=return to start, `u`=update start pose, `c`/`a`=sit/stand
- WBC gains `/wbc/restart` subscriber (Bool): True → IDLE→SEARCHING, False → any→IDLE
- During return navigation, keyboard node takes over `/my_spot/cmd_vel` (no conflict: WBC disables on IDLE)
- Displays WBC state changes from `/wbc/state`

### PRE_APPROACH exits on IK_done + QP publishes mount TF

- PRE_APPROACH now exits when `/ik_done` = True (arm completed look-at), not on 5s timer
- `pre_approach_duration` parameter kept for backward compat but no longer used
- QP controller now publishes `my_spot/body → link00` static TF at 10 Hz via `StaticTransformBroadcaster`
- `static_tf_mount` node removed from `wbc.launch.py` — no race conditions at startup
- `search_lock_samples` reduced 10 → 5 (faster lock, 1s instead of 2s)

### Files modified
`wbc_coordinator.py`, `wbc_qp_controller.py`, `wbc_params.yaml`, `wbc.launch.py`, `wbc_keyboard_controller.py` (new), `setup.py`

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
source /opt/ros/jazzy/setup.bash

# Build the full workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific packages
colcon build --packages-select z1_vision spot_control spot_perception teresa_demo

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

### Running Spot + Z1 WBC (SpotCore + 5 terminali)

**Prerequisites on Spot:**
- `spot_ros2` running on SpotCore (publishes `my_spot/odom → my_spot/body` TF)
- Spot in **stand** position (body_pose richiede Spot in piedi)

**PC terminals (in order):**

```bash
# T1: Core — tutti i driver hardware + TF statiche + tf_monitor
ros2 launch spot_control teresa_core.launch.py
# Aspettare: [TUTTO PRONTO] /wbc/tf_ready = True

# T2: Perception — Orbbec + RealSense YOLO perception (unico launch)
ros2 launch spot_control teresa_perception.launch.py
# Aspettare: nodi perception pronti

# T3: Z1 Control — IK + safe switch + impedance + mux + FSM
ros2 launch z1_vision z1_control.launch.py
# FSM parte dopo 5s → HOMING → WAITING

# T4: WBC — QP controller + coordinator
ros2 launch spot_control wbc.launch.py
# Aspettare: "[TF READY] SpotCore connesso — premi s per iniziare"

# T5: Keyboard controller (richiede TTY interattivo)
ros2 run spot_control wbc_keyboard_node
# Premere "s" → missione parte

# Optional: dry-run mode (no arm movement, debug topics only)
ros2 launch spot_control wbc.launch.py dry_run:=true

# Optional: override Z1 mount position
ros2 launch spot_control teresa_core.launch.py z1_mount_x:=0.25 z1_mount_z:=0.15
```

**Nota:** i sub-launch (`z1_realsense`, `z1_perception`, `z1_control`, `spot_perception`, `wbc`) restano comunque utilizzabili standalone per debug. Tutti i parametri hardware (Orbbec, RealSense, Z1) sono invariati.

### Keyboard controller keys

| Key | Action |
|-----|--------|
| `s` | Save start pose (first press) + trigger WBC SEARCHING (only if TF ready) |
| `r` | Stand + navigate back to start pose + realign yaw |
| `q` | Same as `r` (restart: interrupt WBC, return to start) |
| `u` | Update start pose to current position + yaw |
| `c` / `a` | Sit / Stand |

The keyboard node subscribes to `/wbc/tf_ready` (Bool) to know when SpotCore is connected.
Pressing `s` before TF is ready shows a warning and does nothing.
When TF becomes available, the node prints `[TF READY] SpotCore connesso — premi "s" per iniziare`.

The keyboard node publishes to `/wbc/restart` (Bool) to control the WBC coordinator FSM:
- `True` → WBC transitions from IDLE → SEARCHING
- `False` → WBC transitions from any state → IDLE

During return navigation, the keyboard node takes control of `/my_spot/cmd_vel` directly (no conflict: WBC disables cmd_vel on IDLE).

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
tf_monitor (da teresa_core.launch.py)
  │  Check: 7 catene TF + 3 topic (1 Hz)
  │    TF: odom→body, body→link00, body→orbbec_link, orbbec→optical,
  │         link00→link06, link06→camera_link, camera_link→camera_optical
  │    Topic: /joint_states, /orbbec/color/image_raw, /camera/color/image_raw
  └─► /wbc/tf_ready  (Bool, True when ALL conditions met)
        ├─► wbc_coordinator  (WAITING_TF → IDLE)
        └─► wbc_keyboard_controller  (enables "s" key)

Orbbec Femto Bolt (Jetson → PC)
  └─► yolo_skeleton_spot (YOLO11 pose)
        └─► posture_classifier (posture + confidence)
              └─► laying_human_detector
                    ├─► /laying_human/approach_point   (PoseStamped, camera frame)
                    ├─► /laying_human/body_axis         (asse testa-piedi)
                    └─► /human_pose/posture, /human_pose/posture_confidence

/laying_human/approach_point
  └─► wbc_coordinator  (FSM: WAITING_TF → IDLE → SEARCHING → PRE_APPROACH → APPROACHING → SCANNING → WS_EXTENSION)
        │  Trasforma approach_point camera → odom via TF
        │  QualityMonitor: target best-confidence + quality = max_q*(1-conf)
        ├─► /wbc/ee_goal             (target fisso in odom frame)
        ├─► /wbc/enable              (True = WBC priority su MUX)
        ├─► /wbc/desired_yaw         (Spot ⊥ body_axis)
        ├─► /wbc/target_uncertainty  (quality [m], non sigma)
        ├─► /wbc/state               (WAITING_TF/SEARCHING/PRE_APPROACH/IDLE/APPROACHING/SCANNING/WS_EXTENSION)
        └─► /wbc/spot_control        (False = sopprime cmd_vel, braccio arm-only)

/wbc/ee_goal (odom) + /wbc/enable + /wbc/desired_yaw + /wbc/target_uncertainty
  └─► wbc_qp_controller  (10 Hz, holistic WBC: arm q_dot + base vx·wz)
        │  dp = goal_odom - ee_odom  (errore cala quando Spot avanza)
        │  WBC split → q_dot, vx, wz
        │  v_scale = v_min + (1-v_min)/(1 + quality/ref)  → vx,wz ridotti
        │  x_ee = target_link00 - clipped_pos  → orientazione stabile (minrot)
        │  cmd_vel pubblicato solo se /wbc/spot_control=True
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
WAITING_TF ──(/wbc/tf_ready)──► IDLE
IDLE       ──(/wbc/restart=True from keyboard)──► SEARCHING
SEARCHING  ──(posture=LYING & conf≥0.85 & lock: 5 samples)──► PRE_APPROACH
SEARCHING  ──(grid complete: 3 yaw × 3 pitch visited)──► IDLE (dead-end)
any        ──(/wbc/restart=False from keyboard)──► IDLE
PRE_APPROACH ──(/ik_done=True, arm oriented)──► APPROACHING
APPROACHING  ──(dist<handoff_distance=5cm)──► SCANNING
SCANNING     ──(/wbc/ws_request)──► WS_EXTENSION
WS_EXTENSION ──(/ik_done)──► SCANNING
```

**SEARCHING details:**
- Griglia 3×3: 3 yaw (center, +10°, -10°) × 3 pitch (5°, 10°, 15°)
- A ogni punto griglia: `body_pose(height=-0.20, pitch, yaw)` + `Twist()` flush, poi pausa 3s
- Yaw capture: all'ingresso SEARCHING, TF `body→odom` fornisce lo yaw di riferimento
- **Lock**: quando conf ≥ 0.85 → Spot FREEZE (nessun cambio body_pose), raccoglie 5 sample, media → `QualityMonitor.set_target()` → PRE_APPROACH
- Se conf scende < 0.85 durante lock → riprende griglia dal punto corrente
- `_cb_approach` salta `try_init` durante SEARCHING (target viene solo dal lock)
- Griglia completata (~27s) → IDLE (dead-end)

**PRE_APPROACH details:**
- Spot si RADDRIZZA (body_pose → 0,0), WBC abilitato
- `/wbc/spot_control=False` → braccio look-at, Spot FERMO (nessuna rotazione)
- Pausa 5s per far assestare il braccio verso il target
- Scaduto il timer → APPROACHING con WBC pieno (Spot cammina)

**APPROACHING details:**
- Target già fissato dal lock in SEARCHING — `QualityMonitor.set_target` già chiamato
- `_check_lying_timeout` **non abortisce MAI** APPROACHING — Spot raggiunge sempre il target
- `try_best_update` può raffinare il target se confidence migliora ≥ confidence_margin (0.10)
- Quality = max_q * (1 - posture_confidence) su `/wbc/target_uncertainty`
- Spot naviga con `v_scale` proporzionale alla quality (mai zero, v_min=0.15)
- Senza dati posture_confidence, quality cresce linearmente → Spot rallenta
- Braccio look-at: orientazione stabile via min-rotation quaternion
- Handoff a 5 cm di distanza (non basato su quality)

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
| `src/spot_control/` | Spot navigation, WBC coordinator, WBC QP controller, ik_goal_mux, perception launcher |
| `src/spot_perception/` | Orbbec perception: YOLO skeleton, posture classifier, laying detector |
| `src/teresa_demo/` | Visitor demo: Spot + Z1 simultaneous search movements (no cameras/WBC) |
| `src/spot_msgs/` | Custom ROS2 messages (Trajectory action only; SetStandHeight deprecated in favor of body_pose topic) |
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
| `wbc_coordinator.py` | Phase FSM for Spot+Z1: WAITING_TF→IDLE→SEARCHING→PRE_APPROACH→APPROACHING→SCANNING→WS_EXTENSION, QualityMonitor (target fisso in odom + quality tracking), body height control |
| `wbc_qp_controller.py` | Holistic WBC at 10 Hz: damped pseudo-inverse split of arm joints + base velocity, quality-based v_scale, stable look-at orientation, `/wbc/spot_control` gates cmd_vel |
| `wbc_math.py` | Pure math: J_base, J_holistic, manipulability, WBC split, WBC split with yaw |
| `tf_monitor.py` | Monitora 7 catene TF + 3 topic hardware. Pubblica `/wbc/tf_ready` quando tutto è pronto. Lanciato da `teresa_core.launch.py`. |
| `ik_goal_mux.py` | Priority mux: WBC goals override Z1 FSM goals |
| `spot_goal_navigator.py` | Spot point-to-point navigation |
| `wbc_keyboard_controller.py` | Keyboard-driven Spot control: start/return/restart WBC via `/wbc/restart`, blocca start se TF non pronto |

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
| `wbc_params.yaml` | spot_control | WBC QP weights, handoff distance (0.05), quality params, search params (body_height=-0.20, body_pitch=0.26), pre_approach_duration (5s), orientation_mode, workspace safety margin |

### Key shared parameters (keep in sync)

- `workspace_safety_margin: 0.05` — in both `z1_fsm_params.yaml` and `wbc_params.yaml`, both use the same `WorkspaceChecker` class
- `ik_goal_topic` / `ik_enable_topic` — FSM code defaults are `/z1/ik_goal_pose` and `/z1/ik_enable` (go through `ik_goal_mux`). YAML must NOT override these to `/ik_*` directly or the mux will be bypassed.
- `home_orientation: [-0.0062, 0.4107, 0.0021, 0.9118]` — must be identical in `z1_fsm_params.yaml` and `wbc_params.yaml`
- Body control: `/my_spot/body_pose` (Pose topic, nativo spot_driver) + `/my_spot/cmd_vel` (Twist). Il body_pose è "lazy": spot_driver salva i parametri internamente e li applica solo al prossimo cmd_vel. Il coordinator usa `_pub_cmd_vel` per pubblicare Twist() zero come flush dopo ogni `_set_body_pose()`.

### YOLO model

`yolo11n-pose.pt` lives at the workspace root. Used by both `z1_yolo_torso_tracker` (RealSense) and `yolo_skeleton_spot` (Orbbec).

### Orbbec Femto Bolt — power considerations

- Point cloud e colored point cloud **disabilitate** nel launch file (`spot_perception.launch.py`) — nessun nodo le usa, risparmiano CPU e banda USB
- Dispositivi disabilitati: IR, accelerometro, giroscopio, TF automatico
- Abilitato: RGB 1280×720 @15fps MJPG + Depth 1024×1024 @15fps Y16 + depth registration
- La camera può freezare se alimentata solo via USB-C (potenza insufficiente). Serve alimentatore 12V DC per stabilità.
- **SEARCHING**: Spot usa `/my_spot/body_pose` (Pose topic nativo spot_driver) per abbassarsi (-0.20m) e inclinarsi in avanti (~15° pitch) così l'Orbbec punta verso il suolo

### Body pose control

- **Topic**: `/my_spot/body_pose` (tipo `geometry_msgs/Pose`) — nativo dello `spot_driver` ufficiale
- **`position.z`** → altezza corpo (offset da nominale, negativo = abbassato)
- **`orientation`** → quaternione per pitch/roll del corpo
- **`_set_body_pose(height, pitch)`** nel coordinator pubblica su questo topic
- Sostituisce il vecchio `SetStandHeight` service (custom `spot_msgs`) — più semplice, nessuna dipendenza extra

### Legacy code

Superseded files have been moved to `src/z1_vision/z1_vision/Old/` — do not reference them unless explicitly debugging history.
