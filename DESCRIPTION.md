# TERESA — Whole Body Control (Spot + Z1)

Architettura del sistema TERESA per navigazione autonoma Spot + ecografia Z1.

> **Nota:** questo documento descrive il sistema **come deve funzionare**. Per il changelog storico vedi `INIT.md`.

---

## Overview

Due pipeline coesistono:

| Pipeline | Robot | Camera | Ruolo |
|----------|-------|--------|-------|
| **Z1 standalone** | Unitree Z1 arm | RealSense D435 | FAST ultrasound scanning (no Spot) |
| **Spot + Z1 (WBC)** | Boston Dynamics Spot + Z1 arm | Orbbec Femto Bolt + RealSense D435 | Spot naviga verso il paziente, Z1 esegue ecografia |

---

## Frame Tree

```
my_spot/odom                        ← world-fixed odometry (spot_ros2 su SpotCore)
    └── my_spot/body                ← Spot body frame (dinamico: segue body_pose)
            ├── orbbec_link         ← TF statica (0.30, 0, 0.15)
            │     └── orbbec_color_optical_frame  ← TF statica (-1.5708, 0, -1.5708)
            └── world               ← TF statica (z1_mount_x, 0, z1_mount_z) = Z1 base
                  └── link00        ← Z1 URDF root (robot_state_publisher, fixed joint)
                        └── link01 ... link06  ← Z1 arm chain
                              └── camera_link  ← TF statica (0, 0, 0.05)
                                    └── camera_color_optical_frame  ← RealSense driver
```

**Key points:**
- `my_spot/odom` NON si muove con Spot (frame world-fixed, pubblicato da spot_ros2)
- `my_spot/body` si muove con Spot (height, pitch, yaw sono campi di `body_pose`)
- `world` è il frame root del modello cinematico Z1 nell'URDF — figlio di `body` via TF statica
- `link00` è figlio di `world` (fixed joint nell'URDF, robot_state_publisher)
- `link00` = `'world'` nell'IK solver — sono coincidenti (joint fixed con offset zero)
- Le TF statiche `body → orbbec_link` e `body → world` sono pubblicate da `teresa_core.launch.py`

---

## Flusso operativo completo

```bash
# T1: Core — driver hardware + TF statiche + tf_monitor
ros2 launch spot_control teresa_core.launch.py

# T2: Perception — Orbbec + RealSense YOLO
ros2 launch spot_control teresa_perception.launch.py

# T3: Z1 Control — IK + switch + mux + FSM
ros2 launch z1_vision z1_control.launch.py use_impedance:=false

# T4: WBC — QP + coordinator + navigator + approach scanner
ros2 launch spot_control wbc.launch.py

# T5: Keyboard controller
ros2 run spot_control wbc_keyboard_node
```

---

## Fase 0 — Avvio e connessione TF

### tf_monitor (da teresa_core)
Controlla 8 catene TF e 3 topic hardware ogni 2s:
- TF: `odom→body`, `body→world`, `world→link00`, `body→orbbec_link`, `orbbec_link→optical`,
  `world→link06`, `link06→camera_link`, `camera_link→camera_optical`
- Topic: `/joint_states`, `/orbbec/color/image_raw`, `/camera/camera/color/image_raw`

Pubblica `/wbc/tf_ready = True/False` a ogni tick. QoS normale (no latched).
Se TF degradano → pubblica `False` → coordinator torna in `WAITING_TF` → keyboard blocca `s`.

### Coordinator FSM
```
WAITING_TF ──(/wbc/tf_ready)──► IDLE
```

### Z1 FSM
```
HOMING → WAITING (aspetta segnale WBC + FAST points per BODY_SCANNING)
```

---

## Fase 1 — SEARCHING (Spot cerca il paziente)

`IDLE → SEARCHING` (premi `s` sulla tastiera, `/wbc/restart = True`)

**Cosa fa Spot:**
- Si abbassa: `body_pose(height=-0.20m)` così l'Orbbec punta verso il suolo
- Griglia 3×3: 3 yaw (center, +10°, -10°) × 3 pitch (5°, 10°, 15°)
- A ogni punto: `body_pose(height, pitch, yaw)` + `cmd_vel=Twist()` flush, pausa 3s
- Yaw di riferimento catturato all'ingresso da TF `body→odom`

**Cosa fa Orbbec:**
- YOLO11 → posture classifier → laying_human_detector
- Pubblica `/human_pose/posture`, `/human_pose/posture_confidence`,
  `/laying_human/approach_point` (in frame camera)
- Il coordinator trasforma l'approach_point in **odom** via TF

**Lock:** quando `confidence ≥ 0.85`:
- Spot si blocca (nessun cambio body_pose)
- Raccolta di 5 sample approach_point in odom
- Media → `QualityMonitor.set_target()` → target fissato in **odom**
- `SEARCHING → PRE_APPROACH`

**Braccio:** non coinvolto. Solo Spot + Orbbec.

---

## Fase 2 — PRE_APPROACH (RealSense active detection)

`SEARCHING → PRE_APPROACH`

- Spot si raddrizza: `body_pose(height=0.0, pitch=0.0)`
- RealSense YOLO tracker già attivo (da T2), pubblica `/torso_tracker_state` e `/torso_target_ee`
- Coordinator conta 5 tick consecutivi di `LOCKED` da RealSense
- Timeout 5s → APPROACHING comunque (fallback)
- `LOCKED ×5 → APPROACHING`
- `/wbc/enable=True` → `wbc_approach_scanner` inizia ARC_GRID

---

## Fase 3 — APPROACHING (navigator + scanner + WBC look-at)

`PRE_APPROACH → APPROACHING`

### Spot: wbc_spot_navigator
Navigator semplificato: riceve `/wbc/ee_goal` in odom, trasforma in body frame, rotate → drive → stop. P-controller robusto (1 TF hop `odom→body`), indipendente dal WBC.

### Braccio: WBC QP + wbc_approach_scanner

**WBC QP Controller:**
- Arm: damped pseudo-inverse (J_arm) per orientamento look-at verso target odom
- Spot: P-controller indipendente — `vx = kp_lin_base × dist`, `wz = kp_ang_base × angle`
- Quality scaling: `v_scale = v_min + (1-v_min)/(1 + quality/ref)`
- `/wbc/spot_control=False` — WBC non muove Spot (lo fa il navigator)
- Cache cmd_vel: se TF lookup fallisce, ripubblica l'ultimo comando valido

**wbc_approach_scanner:**
- ARC_GRID (8 pose): Fase 1 (home × wrist ±8°, 4 pose) + Fase 2 (arc ±4cm × wrist, 4 pose)
- BodySearchScanner reale con feed da `/torso_scan_point` (score, confidenze, keypoint 3D)
- Ogni punto combinato con WBC look-at orientation
- Pubblica `/wbc/ik_goal_pose` e `/wbc/ik_enable`

**Handoff:**
- Soft handoff a 20cm: se scanner non ha finito ARC_GRID → Spot aspetta
- Hard handoff a 5cm: `APPROACHING → SCANNING`. WBC viene disabilitato (scanner fase 3 gestita via `/wbc/state`).

---

## Fase 4 — SCANNING (FAST points + fase 3 condizionale)

`APPROACHING → SCANNING`

- WBC rimane **enabled** per consentire allo scanner di eseguire fase 3
- Spot si abbassa: `body_pose(height=-0.15m)`

### wbc_approach_scanner
ARC_GRID completato → `fused_torso_xyz()` + `kp_visibility_stats()`:

- **Se tutti i keypoint visibili** (conf > 0.50): pubblica subito `/z1/fast_points` + `/z1/fast_ready=True`
- **Se keypoint deboli** (spalle o fianchi < 0.50): attende SCANNING → PHASE_3 adattiva:
  - Fianchi nascosti: +P3_EXT_Y body-axis, +P3_FAR_HIP_Z verticale
  - Spalle asimmetriche: ±P3_EXT_X laterale
  - Al termine: pubblica `/z1/fast_points` + `/z1/fast_ready=True`

### Coordinator
- Riceve `/z1/fast_ready=True` → `_set_wbc_enabled(False)` (scanner completato)

### Z1 FSM
- In WAITING, riceve `/wbc/state='SCANNING'` + `/z1/fast_ready=True` + `/z1/fast_points`
- → **Salta BODY_SCANNING** (già fatto dallo scanner)
- → `HOMING → WAITING → CHECKING_WORKSPACE → FAST`

---

## Fase 5 — Ciclo FAST con Body Pose Optimization

`skip_impedance = true` — nessun contatto, solo posizionamento.

Il coordinator riceve i 5 FAST points dal `wbc_approach_scanner`. Per ogni punto esegue un grid search offline (3 altezze × 4 pitch) per trovare la combinazione che porta il target più vicino al centro del workspace Z1 (`sweet_spot: [0.35, 0, 0.30]` in link00).

Per ognuno dei 5 punti (Hub, Subxiphoid, RUQ, LUQ, Suprapubic):
```
Coordinator:     _set_body_pose(h*, p*) → attendere settle 1.5s
                 → /wbc/body_ready = True
FSM:             SCAN_PAUSE attende body_ready
                 → CHECKING_WORKSPACE → APPROACHING → WAIT_IK_DONE
                 → SCAN_PRELIFT → pub /z1/next_point_idx → SCAN_PAUSE
```

Vincoli Spot: altezza [-0.20, -0.15] m, pitch [0°, 15°], yaw invariato.

Dopo tutti i 5 punti: Spot torna a `handoff_height (-0.15m)`, FSM → `HOMING → WAITING`.

I target FSM sono in world frame — quando Spot cambia body_pose, il target si aggiorna automaticamente via TF tree.

---

## Tabella riassuntiva

| Fase | Spot | Braccio | Orbbec | RealSense |
|------|:----:|:-------:|:------:|:---------:|
| **SEARCHING** | Grid 3×3 body_pose | Fermo in home | ✅ Attiva | In attesa |
| **PRE_APPROACH** | Raddrizzato | Fermo in home | — | ✅ Tracker LOCKED×5 |
| **APPROACHING** | Navigator → goal | ARC_GRID (8 pose) + look-at | — | ✅ Raccolta 3D |
| **SCANNING** | Body pose per FAST (grid search) | Fase 3 (se necessaria) | — | ✅ FAST points |
| **FAST** | Grid search ottimizza (h,p) per ogni punto | 5 punti ecografici | — | — |

---

## Architettura WBC

### Componenti (lanciati da `wbc.launch.py`)

| Nodo | Ruolo |
|------|-------|
| `wbc_coordinator` | FSM: WAITING_TF → IDLE → SEARCHING → PRE_APPROACH → APPROACHING → SCANNING → WS_EXTENSION. QualityMonitor. Body pose control. FAST body pose grid search. |
| `wbc_qp_controller` | Arm: WBC look-at (J_arm damped pseudo-inverse). Spot: P-controller (1 TF `odom→body`). Quality-based velocity scaling. |
| `wbc_spot_navigator` | Navigatore semplificato per APPROACHING. Legge `/wbc/ee_goal` in odom, rotate → drive → stop. P-controller robusto. |
| `wbc_approach_scanner` | Body scan durante APPROACHING. ARC_GRID (8 pose, ±8° wrist, ±4cm grid) con BodySearchScanner + ScanManager. Fase 3 condizionale in SCANNING. Pubblica `/z1/fast_points` e `/z1/fast_ready`. |

### Coordinator FSM
```
WAITING_TF → IDLE → SEARCHING → PRE_APPROACH → APPROACHING → SCANNING → WS_EXTENSION
                ↑         ↑                                    │            │
                └─────────┴────────────────────────────────────┘            │
                (/wbc/restart=False da keyboard o TF loss)                  │
                                                                           │
                └──────────────────────────────────────────────────────────┘
                                        (/wbc/ws_request)
```

### Z1 FSM states
```
HOMING → WAITING → BODY_SCANNING → CHECKING_WORKSPACE → APPROACHING
    ↑                                                        ↓
    └──── SCAN_PRELIFT ← ... ← IMPEDANCE_RUNNING ← WAIT_IK_DONE
                                          ↕
                                    REQUESTING_WS_EXT  (→ CHECKING_WORKSPACE)
```

### WBC look-at + body scan (APPROACHING)

Durante PRE_APPROACH solo il WBC QP pubblica look-at (scanner non attivo).
In APPROACHING il `wbc_approach_scanner` prende il controllo completo del braccio via
`/wbc/state='APPROACHING'`, pubblicando posizioni griglia + orientamento look-at.
Il WBC QP non esegue IK goals in APPROACHING — evita conflitti sullo stesso topic.

```
PRE_APPROACH:
  WBC QP → /wbc/ik_goal_pose (look-at puro, braccio fermo in home)

APPROACHING:
  wbc_spot_navigator     → /my_spot/cmd_vel → Spot
  wbc_approach_scanner   → /wbc/ik_goal_pose (pos grid + ori torso)
                           BodySearchScanner + ScanManager
                           feed da /torso_scan_point (RealSense)
```

---

## QualityMonitor

- **Target**: media prime `quality_buf_size=3` misure in odom → fissato
- **Aggiornamento**: solo se `posture_confidence > best_conf + confidence_margin` (0.10)
- **Quality [m]**: `max_q * (1 - posture_confidence)` → cresce linearmente senza dati
- **v_scale**: `v_min + (1 - v_min) / (1 + quality / quality_ref)` — mai zero
- Pubblicato su `/wbc/target_uncertainty`

---

## Multi-controller Z1

L'alternanza tra due controller ROS2:
- **joint_trajectory_controller (JTC)** — controllo posizione (homing, approaching)
- **torque_controller** — controllo sforzo (impedance durante contatto ecografico)

`safe_controller_switch` espone servizi `/safe_switch/to_torque` e `/safe_switch/to_jtc`.
JTC è il default di sicurezza.

---

## File chiave

| File | Ruolo |
|------|-------|
| `teresa_core.launch.py` | Core launch: driver Orbbec+RealSense+Z1 + TF statiche + tf_monitor |
| `teresa_perception.launch.py` | Perception launch: Orbbec YOLO posture + RealSense YOLO torso tracker |
| `wbc.launch.py` | WBC launch: coordinator + QP + navigator + approach scanner |
| `wbc_coordinator.py` | FSM Spot+Z1: WAITING_TF→IDLE→SEARCHING→PRE_APPROACH→APPROACHING→SCANNING→WS_EXTENSION, QualityMonitor, active perception |
| `wbc_qp_controller.py` | WBC: arm damped pseudo-inverse look-at + Spot P-controller, quality scaling |
| `wbc_spot_navigator.py` | Navigator semplificato: rotate → drive → stop verso goal odom |
| `wbc_approach_scanner.py` | Body scan durante APPROACHING: ARC_GRID (8 pose) + fase 3 condizionale |
| `wbc_math.py` | Matematica pura: J_base, J_holistic, manipulability, WBC split |
| `z1_FSM.py` | FSM braccio: HOMING→WAITING→BODY_SCANNING→CHECKING_WORKSPACE→APPROACHING→FAST cycle |
| `z1_ik_to_jtc.py` | Pinocchio IK + smoothstep trajectory → JTC action |
| `z1_scan_manager.py` | Calcolo 5 punti FAST da keypoint torso |
| `ik_goal_mux.py` | Priority mux: WBC goals vs Z1 FSM goals |
| `tf_monitor.py` | Monitor continuo 8 TF + 3 topic → `/wbc/tf_ready` True/False |
| `wbc_keyboard_controller.py` | Keyboard: `s`=start, `r`=return, ESC=emergency stop |
