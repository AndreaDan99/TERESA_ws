# TERESA — Whole Body Control (Spot + Z1)

Architettura del sistema TERESA per navigazione autonoma Spot + ecografia Z1.

> **Nota:** questo documento descrive il sistema **come deve funzionare**. Per il changelog storico vedi [`CHANGELOG.md`](CHANGELOG.md). Per il piano futuro vedi [`PLAN.md`](PLAN.md).

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

# T4: WBC — QP controller (arm-only) + coordinator + navigator
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
HOMING → WAITING (aspetta segnale WBC + FAST points pre-calcolati dal QP)
```

---

## Fase 1 — SEARCHING (ricerca ibrida Spot + braccio, 360°)

`IDLE → SEARCHING` (premi `s` sulla tastiera)

### Spot: ricerca incrementale
- Altezza nominale (0m). Non più abbassato.
- Sequenza di 18 posizioni: 6 yaw (0°, +60°, -60°, +120°, -120°, +180°) × 3 pitch (0°, 5°, 10°)
- A ogni posizione: `body_pose(height=0, pitch, yaw)`, pausa 15s, poi prossima
- Copertura totale: 360° con overlap di 10° a ogni giunzione

### Braccio: QP Controller — SEARCH_GRID mode
- Genera 7 pose esplorative dal null-space del "guarda avanti" (δ=0.15 rad ≈9°)
- Safe joint limits per evitare pose estreme (non colpire Spot, non toccare terra)
- BodySearchScanner in loop infinito: per ogni posa → 2s raccolta dati → prossima
- I movimenti cambiano automaticamente quando Spot ruota (FK ricalcolata)

### Lock ibrido — due sensori in parallelo

**Full lock (Orbbec diretto):**
- Posture = `LYING` + confidence ≥ 70%
- `approach_point` disponibile → `LOCKING`

**Semi-lock (RealSense guida Spot):**
- Torso tracker = `LOCKED`, torso 3D disponibile
- Coordinator calcola yaw e pitch ottimali per puntare l'Orbbec al torso
- Spot ruota e si inclina verso il torso
- Braccio congelato (QP in pausa)
- 3 secondi di finestra pulita per l'Orbbec
- Se Orbbec conferma → `LOCKING`
- Se timeout (3s) o RealSense perde il torso → riprende ricerca dalla posizione corrente

### Lock: raccolta e conferma
- **LOCKING**: braccio torna in home, coordinator raccoglie 5 campioni `approach_point` in odom (10 Hz, ~0.5s)
- Tolleranza 1s se Orbbec perde momentaneamente LYING
- 5 campioni raccolti + braccio in home → media → target fissato → `PRE_APPROACH`
- Se Orbbec persa per >1s → riprende ricerca dalla posizione corrente (non da zero)

### Sensori coinvolti
- **Orbbec** (su Spot): YOLO11 → posture classifier → `approach_point` laterale in odom
- **RealSense** (sul polso): YOLO torso tracker → posizione 3D del torso

---

## Fase 2 — PRE_APPROACH (WBC LOOKAT mode)

`LOCKING → PRE_APPROACH` (dopo 5 campioni + braccio in home)

- Spot si raddrizza: `body_pose(height=0.0, pitch=0.0)`
- **WBC QP Controller — LOOKAT mode**: calcola ω_des (errore orientamento X_ee → target), risolve il task con damped pseudo-inverse sul Jacobiano angolare J_task (3×6), applica joint centering nel null-space (N @ k_null * (q_mid - q_current)), integra q_dot in FK prediction, pubblica il goal di posa (posizione predetta + orientamento minrot verso il target) all'IK solver. Loop a 10 Hz: l'orientamento si adatta in tempo reale.
- RealSense YOLO tracker già attivo (da T2). Coordinator conta 5 tick consecutivi di `LOCKED` da RealSense. Timeout 5s → APPROACHING comunque (fallback).
- `LOCKED ×5 → APPROACHING`

---

## Fase 3 — APPROACHING (navigator + QP SCAN_SEQ)

`PRE_APPROACH → APPROACHING`

### Spot: wbc_spot_navigator
Navigatore semplificato: riceve il goal in odom, trasforma in body frame, rotate → drive → stop. P-controller robusto (1 TF hop `odom→body`), indipendente dal QP.

### Braccio: QP Controller — SCAN_SEQ mode
Il QP riceve il segnale di APPROACHING e passa in modalità SCAN_SEQ:

1. **Generazione griglia QP-based** (`_gen_scan_poses`): FK + Jacobiano alla configurazione corrente del braccio. Dal Jacobiano angolare J_task (3×6) calcola il proiettore null-space N = I - J_task⁺·J_task. SVD di N → 3 direzioni ortonormali del null-space. Genera 11 pose: 1 home + 6 (±δ lungo ogni direzione, δ=0.12rad ≈7°) + 4 diagonali (v1±v2, v2±v3). Ogni posa è generata via FK di `q + δ·basis_vector` nel null-space → garantisce look-at per costruzione (il null-space preserva ω=0) ed è sempre raggiungibile.

2. **Sequencing** (`BodySearchScanner`): per ogni posa pubblica goal all'IK solver → attende `ik_done` → raccoglie dati di detection per 4s (minimo 5 frame, early stop se score ≥ 0.95) → passa alla posa successiva.

3. **Fusione + FAST points**: al termine fonde le stime 3D di tutte le pose (outlier rejection a 0.15m). Calcola i 5 punti FAST (Hub, Subxiphoid, RUQ, LUQ, Suprapubic) con offset fissi dal torso stimato.

### Handoff
- Soft handoff a 20cm: se il QP non ha ancora finito la scansione → Spot aspetta
- Hard handoff a 5cm e FAST points pubblicati → `APPROACHING → SCANNING`

---

## Fase 4 — SCANNING (FAST points + ottimizzazione body pose)

`APPROACHING → SCANNING`

- WBC viene disabilitato (`/wbc/enable=False`), Spot si abbassa a handoff height (-0.15m)
- Il QP Controller ha già pubblicato `/z1/fast_points` (5 Pose in frame link00) durante APPROACHING

### Body Pose Optimization (per punto)
Il coordinator riceve i 5 FAST points ed esegue un **grid search matematico** offline:
- **Primaria**: 3 altezze [-0.20, -0.18, -0.15] × 4 pitch [0°, 5°, 10°, 15°] = 12 combinazioni
- Per ogni combinazione simula matematicamente dove si troverebbe link00 in odom
- Calcola la distanza tra il target FAST e il sweet spot `[0.35, 0, 0.30]` in link00
- Seleziona la combinazione (h, p) che minimizza la distanza dal sweet spot

### WS_EXTENSION (fallback, solo se necessario)
Se dopo l'ottimizzazione primaria il target è ancora fuori dal workspace del braccio:
- Grid search 4D: 3×4×5×5 = 300 combinazioni (altezza × pitch × dx × dy)
- Il navigator guida Spot alla posizione (dx, dy) calcolata — timeout 5s
- Si applica la body_pose ottimale (h, p) e dopo 1.5s di settle si segnala `body_ready`

### Per-punto application
Il FSM segnala al coordinator l'indice del prossimo punto via `/z1/next_point_idx`:
1. Coordinator applica body_pose (h, p) ottimale (eventualmente con WS_EXT drive)
2. Dopo settle 1.5s → pubblica `/wbc/body_ready = True`
3. FSM in SCAN_PAUSE riceve body_ready → entra in CHECKING_WORKSPACE

### Z1 FSM
- In WAITING, riceve `/wbc/state='SCANNING'` + `/z1/fast_ready=True` + `/z1/fast_points`
- → **Salta BODY_SCANNING** (già fatto dal QP durante APPROACHING)
- → `HOMING → WAITING → CHECKING_WORKSPACE → FAST cycle`

---

## Fase 5 — Ciclo FAST con CHECKING_WORKSPACE per ogni punto

`skip_impedance = true` — nessun contatto, solo posizionamento.

Per ognuno dei 5 punti (Hub, Subxiphoid, RUQ, LUQ, Suprapubic):
```
SCAN_PRELIFT → pub /z1/next_point_idx → SCAN_PAUSE
     │                                        │
     │                          attende /wbc/body_ready dal coordinator
     │                                        │
     └────────────────────────────────────────┘
                                              ▼
                                     CHECKING_WORKSPACE
                                        │           │
                                   target OK    was_clipped
                                        │           │
                                        │     idx==0: procede clippato
                                        │     idx>0:  SKIP → advance()
                                        │              pub next_point_idx
                                        │              → SCAN_PAUSE
                                        ▼
                                   APPROACHING → WAIT_IK_DONE
                                        │
                                        ▼
                                   SCAN_PRELIFT → (prossimo punto)
```

**CHECKING_WORKSPACE per ogni punto:**
- Per idx=0 (centro hub): usa il tracker torso live per il workspace check
- Per idx>0 (punti FAST): calcola il target da `center_approach_pose + offset`, nessuna dipendenza dal tracker
- Se `was_clipped` dopo tutte le ottimizzazioni (body pose + eventuale WS_EXT):
  - idx=0: procede comunque (il centro hub salva `center_approach_pose` usata dai punti successivi)
  - idx>0: **salta il punto**, avanza al successivo, pubblica `next_point_idx`, torna in SCAN_PAUSE

Vincoli Spot: altezza [-0.20, -0.15] m, pitch [0°, 15°], yaw invariato.
WS_EXT: dx ±0.20m, dy -0.30/+0.20m, navigator timeout 5s.

Dopo tutti i 5 punti: Spot torna a `handoff_height (-0.15m)`, FSM → `HOMING → WAITING`.

I target FSM sono in world frame — quando Spot cambia body_pose, il target si aggiorna automaticamente via TF tree.

---

## Tabella riassuntiva

| Fase | Spot | Braccio | WBC/QP |
|------|:----:|:-------:|:------:|
| **SEARCHING** | 18 posizioni (6 yaw × 3 pitch) | 7 pose QP-based in loop (SEARCH_GRID) | QP-based grid |
| **SEMI_LOCKING** | Ruotato+inclinato verso torso | Congelato (QP in pausa) | Off |
| **LOCKING** | Fermo | Va in home | Off |
| **PRE_APPROACH** | Dritto, fermo | LOOKAT (ω_des + null-space joint centering) | Arm-only WBC |
| **APPROACHING** | Navigatore → goal | SCAN_SEQ (11 pose null-space + BodySearchScanner) | QP-based grid |
| **SCANNING** | Body pose (h,p) + WS_EXT (h,p,dx,dy) | z1_FSM: FAST cycle 5 punti | Off |

---

## Architettura WBC

### Componenti (lanciati da `wbc.launch.py`)

| Nodo | Ruolo |
|------|-------|
| `wbc_coordinator` | FSM: WAITING_TF → IDLE → SEARCHING → SEMI_LOCKING → LOCKING → PRE_APPROACH → APPROACHING → SCANNING. Hybrid lock (Orbbec full + RealSense semi-lock). QualityMonitor. Body pose control. FAST body pose grid search (h,p) + WS_EXT fallback (h,p,dx,dy) con navigator drive. |
| `wbc_qp_controller` | **Arm-only WBC, 3 modalità**: SEARCH_GRID (7 pose esplorative QP-based, δ=0.15, loop infinito), LOOKAT (ω_des + null-space joint centering), SCAN_SEQ (11 pose grid, δ=0.12, BodySearchScanner, FAST points). Safe joint limits in SEARCH_GRID. |
| `wbc_spot_navigator` | Navigatore semplificato per APPROACHING e WS_EXT. Legge il goal in odom, rotate → drive → stop. P-controller robusto. Spot non è mai controllato dal QP. |

### Coordinator FSM
```
WAITING_TF → IDLE → SEARCHING → SEMI_LOCKING → LOCKING → PRE_APPROACH → APPROACHING → SCANNING
                ↑         ↑              ↑            ↑
                │         └──────────────┴────────────┘  (timeout / RealSense lost / Orbbec lost)
                │
                └── restart (keyboard)
```

### Z1 FSM states
```
HOMING → WAITING → BODY_SCANNING → CHECKING_WORKSPACE ──────────────────► APPROACHING
    ↑                      ▲              ↑    ↑                              ↓
    │                      │              │    └── SCAN_PAUSE ←────────── WAIT_IK_DONE
    │                      │              │              ↑                    ↓
    │                      │              │              └── SCAN_PRELIFT ←── ┘
    │                      │              │                       ↑
    └──────────────────────┴──────────────┴───────────────────────┘
              (ciclo completato o FAULT → HOMING → WAITING)
```
**CHECKING_WORKSPACE** viene eseguito per **ogni punto FAST**:
- Entrando da WAITING (idx=0, centro hub): usa il tracker live
- Entrando da SCAN_PAUSE (idx>0): calcola target da center_approach_pose + offset
- Se was_clipped: idx=0 procede clippato, idx>0 skips al punto successivo

### WBC QP modes (SEARCHING → PRE_APPROACH → APPROACHING)

Durante **SEARCHING** il QP opera in **SEARCH_GRID mode**: target virtuale "body X avanti", δ=0.15 rad, safe joint limits, 7 pose esplorative in loop infinito. Il braccio esplora senza un target reale.

Durante **PRE_APPROACH** il QP opera in **LOOKAT mode**: calcola l'errore di orientamento tra X_ee e target (`ω_des = kp_ang * angle * axis`), risolve con damped pseudo-inverse su J_task (3×6, solo parte angolare), proietta joint centering nel null-space (`N @ k_null * (q_mid - q)`), integra in FK prediction, pubblica il goal all'IK solver. Loop a 10 Hz.

In **APPROACHING** il QP passa in **SCAN_SEQ mode**: genera 11 pose dal null-space del look-at verso il target reale (1 home + 6 assi ±δ + 4 diagonali, δ=0.12 rad), le sequenzia con BodySearchScanner, raccoglie dati di detection per ogni posa, fonde le stime 3D, pubblica i 5 punti FAST.

Spot è sempre controllato dal navigatore (APPROACHING) o dal coordinator (body pose in SEARCHING/SCANNING), mai dal QP.

#### SEMI_LOCKING e LOCKING (gestione QP)
- **SEMI_LOCKING**: il QP va in pausa — il braccio si blocca nella posa corrente. L'Orbbec ha 3s di finestra pulita.
- **LOCKING**: il QP esce da SEARCH_GRID e manda il braccio in home. Il coordinator raccoglie 5 campioni in parallelo.
- Se RealSense perde il torso durante SEMI_LOCKING, o se Orbbec perde LYING per >1s durante LOCKING: si riprende la ricerca dalla posizione corrente.
```
SEARCHING:
  QP Controller (SEARCH_GRID) → 7 pose esplorative, loop infinito
  Coordinator → body pose cycling (18 posizioni)

SEMI_LOCKING:
  QP Controller → PAUSA (braccio congelato)
  Coordinator → Spot ruotato+inclinato verso torso

LOCKING:
  QP Controller → end search + home pose
  Coordinator → 5 campioni approach_point

PRE_APPROACH:
  QP Controller (LOOKAT) → IK goal (ω_des + null-space joint centering)
  Navigatore: fermo

APPROACHING:
  Navigatore → rotate → drive → stop
  QP Controller (SCAN_SEQ) → 11 pose null-space → BodySearchScanner → fuses → FAST points
```

---

## QualityMonitor

- **Target**: media prime `quality_buf_size=3` misure in odom → fissato
- **Aggiornamento**: solo se `posture_confidence > best_conf + confidence_margin` (0.10)
- **Quality [m]**: `max_q * (1 - posture_confidence)` → cresce linearmente senza dati
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
| `wbc_coordinator.py` | FSM Spot+Z1: WAITING_TF→IDLE→SEARCHING→PRE_APPROACH→APPROACHING→SCANNING. QualityMonitor. Body pose grid search (h,p) + WS_EXT fallback. Per-point body_ready. |
| `wbc_qp_controller.py` | WBC: arm damped pseudo-inverse look-at + Spot P-controller, quality scaling |
| `wbc_spot_navigator.py` | Navigator semplificato: rotate → drive → stop verso goal odom. Usato anche per WS_EXT drive. |
| `wbc_approach_scanner.py` | Body scan durante APPROACHING: ARC_GRID (8 pose) + fase 3 condizionale |
| `wbc_math.py` | Matematica pura: J_base, J_holistic, manipulability, WBC split |
| `z1_FSM.py` | FSM braccio: HOMING→WAITING→BODY_SCANNING→CHECKING_WORKSPACE (per ogni punto)→APPROACHING→FAST cycle. Skip per punti irraggiungibili. |
| `z1_ik_to_jtc.py` | Pinocchio IK + smoothstep trajectory → JTC action |
| `z1_scan_manager.py` | Calcolo 5 punti FAST da keypoint torso |
| `ik_goal_mux.py` | Priority mux: WBC goals vs Z1 FSM goals |
| `tf_monitor.py` | Monitor continuo 8 TF + 3 topic → `/wbc/tf_ready` True/False |
| `wbc_keyboard_controller.py` | Keyboard: `s`=start, `r`=return, ESC=emergency stop |

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select z1_vision spot_control spot_perception teresa_demo
source install/setup.bash
```

---

## Running

### Z1 standalone (3 terminali)

```bash
# Terminal 1: Robot hardware + RealSense camera
ros2 launch z1_vision z1_realsense.launch.py use_rviz:=true

# Terminal 2: Vision pipeline
ros2 launch z1_vision z1_perception.launch.py

# Terminal 3: Control (FSM starts after 5s)
ros2 launch z1_vision z1_control.launch.py
```

### Spot + Z1 WBC (5 terminali)

**Prerequisites on Spot:**
- `spot_ros2` running on SpotCore (publishes `my_spot/odom → my_spot/body` TF)
- Spot in **stand** position

```bash
# T1: Core — driver hardware + TF statiche + tf_monitor
ros2 launch spot_control teresa_core.launch.py
# Aspettare: [TUTTO PRONTO] /wbc/tf_ready = True

# T2: Perception — Orbbec + RealSense YOLO
ros2 launch spot_control teresa_perception.launch.py

# T3: Z1 Control — IK + switch + mux + FSM
ros2 launch z1_vision z1_control.launch.py use_impedance:=false
# FSM parte dopo 5s → HOMING → WAITING

# T4: WBC — QP + coordinator + navigator + scanner
ros2 launch spot_control wbc.launch.py

# T5: Keyboard controller
ros2 run spot_control wbc_keyboard_node
# Premere "s" → missione parte

# Optional: dry-run mode
ros2 launch spot_control wbc.launch.py dry_run:=true

# Optional: override Z1 mount
ros2 launch spot_control teresa_core.launch.py z1_mount_x:=0.25 z1_mount_z:=0.15
```

### Keyboard Controller Keys

| Key | Action |
|-----|--------|
| `s` | Save start pose (first press) + trigger WBC SEARCHING (only if TF ready) |
| `r` | Stand + navigate back to start pose + realign yaw |
| `q` | Same as `r` (restart: interrupt WBC, return to start) |
| `u` | Update start pose to current position + yaw |
| `c` | Sit |
| `a` | Stand |
| `ESC` | Emergency stop: `/wbc/restart=False` + `cmd_vel=0` |

The keyboard node subscribes to `/wbc/tf_ready` (Bool) to know when SpotCore is connected. Pressing `s` before TF is ready shows a warning. When TF becomes available, prints `[TF READY] SpotCore connesso — premi "s" per iniziare`.

---

## Packages

| Package | Ruolo |
|---------|-------|
| `src/teresa_utils/` | Shared orientation & transform utilities (no ROS node) |
| `src/z1_vision/` | Z1 arm: FSM, IK, impedance, YOLO tracking, workspace checker |
| `src/spot_control/` | Spot navigation, WBC coordinator, WBC QP controller, ik_goal_mux, perception launcher |
| `src/spot_perception/` | Orbbec perception: YOLO skeleton, posture classifier, laying detector |
| `src/teresa_demo/` | Visitor demo: Spot + Z1 simultaneous search movements (no cameras/WBC) |
| `src/spot_msgs/` | Custom ROS2 messages (Trajectory action only) |
| `src/z1_ros2/` | Unitree Z1 hardware interface, URDF, MoveIt2, bringup configs |
| `src/realsense-ros/` | Intel RealSense ROS2 driver |
| `src/orbbec_camera/` | Orbbec camera driver |

---

## Config Files

| File | Package | Governs |
|------|---------|---------|
| `z1_fsm_params.yaml` | z1_vision | FSM topics, home pose, approach offset, FAST point ratios, workspace safety margin, WBC startup timeout |
| `z1_yolo_torso_params.yaml` | z1_vision | YOLO model path, confidence, Kalman gains, lock threshold |
| `z1_ik_jtc_params.yaml` | z1_vision | URDF path, IK tol/damping, max_joint_vel (0.2 rad/s), trajectory timing |
| `impedance_control_params.yaml` | z1_vision | K_p [150,150,300], K_d, K_i, approach speed, contact threshold |
| `surface_params.yaml` | z1_vision | Depth ROI size, PCA config, frame names |
| `body_search_params.yaml` | z1_vision | Scan extents, wrist angles, early-stop threshold |
| `camera_params.yaml` | z1_vision | Camera TF offset relative to EE (link06 → camera_link) |
| `wbc_params.yaml` | spot_control | WBC QP weights, handoff distance, quality params, search grid, body pose optimization |

### Key Shared Parameters

- **`workspace_safety_margin: 0.05`** — in both `z1_fsm_params.yaml` and `wbc_params.yaml`
- **`ik_goal_topic` / `ik_enable_topic`** — defaults are `/z1/ik_goal_pose` and `/z1/ik_enable` (go through `ik_goal_mux`). YAML must NOT override these to `/ik_*` directly.
- **`home_orientation: [-0.0062, 0.4107, 0.0021, 0.9118]`** — must be identical in `z1_fsm_params.yaml` and `wbc_params.yaml`
- **Body control**: `/my_spot/body_pose` (Pose topic, nativo spot_driver) + `/my_spot/cmd_vel` (Twist). `body_pose` is "lazy": spot_driver saves params internally and applies them only on the next `cmd_vel`. The coordinator publishes `Twist()` zero as flush after every `_set_body_pose()`.

---

## Conventions

### World Frame Convention

```
X → toward patient (approach direction)
Y → head to feet
Z → right to left
```

### IK Conventions

- Solver: Pinocchio damped pseudo-inverse Jacobian, `LOCAL_WORLD_ALIGNED` frame
- Trajectory interpolation: smoothstep quintic (10t³−15t⁴+6t⁵), zero vel/acc at endpoints
- Timing: `T = max_joint_displacement / max_joint_vel`, clipped to `[traj_min_time, traj_max_time]`
- Joint unwrapping: `_make_target_near()` prevents >π rotations between waypoints
- URDF path: auto-resolved via ament_index (fallback in `z1_ik_jtc_params.yaml`)

---

## Hardware Notes

### YOLO Model

`yolo11n-pose.pt` lives at the workspace root. Used by both `z1_yolo_torso_tracker` (RealSense) and `yolo_skeleton_spot` (Orbbec).

### Orbbec Femto Bolt — Power

- Point cloud and colored point cloud **disabled** in launch file — no node uses them, save CPU and USB bandwidth
- Disabled devices: IR, accelerometer, gyroscope, auto TF
- Enabled: RGB 1280×720 @15fps MJPG + Depth 1024×1024 @15fps Y16 + depth registration
- The camera can freeze if powered only via USB-C (insufficient power). Use 12V DC power supply for stability.

### Body Pose Control

- **Topic**: `/my_spot/body_pose` (type `geometry_msgs/Pose`) — native to `spot_driver`
- **`position.z`** → body height (offset from nominal, negative = lowered)
- **`orientation`** → quaternion for body pitch/roll
- SEARCHING: Spot lowers (`-0.20m`) and tilts forward (up to 15° pitch) so Orbbec points toward the ground

### Dry-run Mode

When `dry_run:=true` on WBC launch, all outputs go to debug topics:

| Normal (arm + Spot move) | Dry-run (nothing moves) |
|---|---|
| `/ik_goal_pose` → `z1_ik_to_jtc` | `/wbc/ik_goal_pose_debug` |
| `/ik_enable` → `z1_ik_to_jtc` | `/wbc/ik_enable_debug` |
| `/my_spot/cmd_vel` → Spot | `/wbc/cmd_vel_debug` |
