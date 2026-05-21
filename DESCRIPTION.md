# TERESA — Whole Body Control (Spot + Z1)

Architettura del sistema TERESA per navigazione autonoma Spot + ecografia Z1.

> **Nota:** questo documento descrive il sistema **come deve funzionare**. Per il changelog storico vedi `INIT.md`.

---

## Overview

Due pipeline coesistono:

| Pipeline | Robot | Camera | Ruolo |
|----------|-------|--------|-------|
| **Z1 standalone** | Unitree Z1 arm | RealSense D435 | FAST ultrasound scanning (no Spot) |
| **Spot + Z1 (WBC)** | Boston Dynamics Spot + Z1 arm | Orbbec Femto Bolt | Spot naviga verso il paziente, Z1 esegue ecografia |

---

## Frame Tree

```
my_spot/odom                        ← world-fixed odometry (spot_ros2 su SpotCore)
    └── my_spot/body                ← Spot body frame (dinamico: segue body_pose)
            ├── orbbec_link         ← TF statica (0.30, 0, 0.15)
            │     └── orbbec_color_optical_frame  ← TF statica (-1.5708, 0, -1.5708)
            └── link00              ← TF statica (z1_mount_x, 0, z1_mount_z) = Z1 base
                  └── link01 ... link06  ← Z1 arm chain (robot_state_publisher)
                        └── camera_link  ← TF statica (0, 0, 0.05)
                              └── camera_color_optical_frame  ← intrinseca RealSense
```

**Key points:**
- `my_spot/odom` NON si muove con Spot (frame world-fixed, pubblicato da spot_ros2)
- `my_spot/body` si muove con Spot (height, pitch, yaw sono campi di `body_pose`)
- `link00` = `'world'` nell'IK solver — è il frame base del modello cinematico Z1
- Le TF statiche `body → orbbec_link` e `body → link00` sono pubblicate da `teresa_core.launch.py`

---

## Flusso operativo completo

```bash
# T1: Core — driver hardware + TF statiche + tf_monitor
ros2 launch spot_control teresa_core.launch.py

# T2: Perception — Orbbec + RealSense YOLO perception
ros2 launch spot_control teresa_perception.launch.py

# T3: Z1 Control — IK + safe switch + impedance + mux + FSM
ros2 launch z1_vision z1_control.launch.py

# T4: WBC — QP controller + coordinator
ros2 launch spot_control wbc.launch.py

# T5: Keyboard controller
ros2 run spot_control wbc_keyboard_node
```

---

## Fase 1 — Avvio e connessione

### tf_monitor (da teresa_core)
Controlla 7 catene TF e 3 topic hardware a 1 Hz:
- TF: `odom→body`, `body→link00`, `body→orbbec_link`, `orbbec_link→optical`,
  `link00→link06`, `link06→camera_link`, `camera_link→camera_optical`
- Topic: `/joint_states`, `/orbbec/color/image_raw`, `/camera/color/image_raw`

Quando tutto pronto → `/wbc/tf_ready = True`

### Coordinator FSM
```
WAITING_TF ──(/wbc/tf_ready)──► IDLE
```

### Z1 FSM
```
HOMING → WAITING (aspetta segnale WBC per BODY_SCANNING)
```

---

## Fase 2 — SEARCHING (Spot cerca il paziente)

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

**Bracico:** non coinvolto. Solo Spot + Orbbec.

---

## Fase 3 — PRE_APPROACH (braccio si orienta, Spot fermo)

`SEARCHING → PRE_APPROACH`

- Spot si raddrizza: `body_pose(height=0.0, pitch=0.0)`
- WBC abilitato: `/wbc/enable = True`, `/wbc/spot_control = False` (solo braccio)

### Catena goal PRE_APPROACH

```
wbc_coordinator:    _filtered_goal()  →  /wbc/ee_goal  [frame: my_spot/odom]
wbc_qp_controller:  _update() riceve goal in odom
                      │
                      ├─ goal_odom = goal (odom, invariato)
                      ├─ TF odom→link00 → goal_link00 (per orientazione look-at)
                      ├─ dp = goal_odom - ee_odom (posizione in odom)
                      └─ WBC split → /wbc/ik_goal_pose → mux → z1_ik_to_jtc
```

Il braccio punta verso il target in look-at. Spot non cammina.
Quando `/ik_done = True` → `PRE_APPROACH → APPROACHING`.

---

## Fase 4 — APPROACHING (Spot cammina + braccio punta)

`PRE_APPROACH → APPROACHING`

- `/wbc/spot_control = True`: Spot cammina e ruota
- WBC QP produce: `[q_dot(6), vx, wz]` a 10 Hz
- `v_scale` basato su `QualityMonitor.quality` (mai zero, `v_min=0.15`)

### Catena goal APPROACHING (identica a PRE_APPROACH)

```
wbc_coordinator:    _filtered_goal()  →  /wbc/ee_goal  [frame: my_spot/odom]
wbc_qp_controller:  _update()
                      │
                      ├─ goal_odom = goal (odom — target fissato, non cambia)
                      ├─ TF odom→link00 → goal_link00 (si aggiorna automaticamente
                      │   mentre Spot cammina, perché la TF odom→body cambia)
                      ├─ dp = goal_odom - ee_odom (errore cala quando Spot avanza)
                      └─ WBC split → /wbc/ik_goal_pose + /my_spot/cmd_vel
```

**Perché funziona:** il target è fissato in odom (world-fixed). Mentre Spot cammina, la TF `odom→link00` aggiorna automaticamente la posizione del target nel frame del braccio. L'errore `dp` cala genuinamente perché Spot si avvicina fisicamente al target.

Quando `distanza < handoff_distance (0.05m)` → `APPROACHING → SCANNING`.

---

## Fase 5 — SCANNING (Spot fermo, Z1 prende il controllo)

`APPROACHING → SCANNING`

- **WBC disabilitato**: `/wbc/enable = False`
- Coordinator pubblica `/wbc/state = 'SCANNING'`
- Spot si abbassa per handoff: `body_pose(height=-0.15m)`
- Z1 FSM in WAITING vede `wbc_state='SCANNING'` → `BODY_SCANNING`

### BODY_SCANNING (Z1 FSM)

Multi-fase con body_search_scanner:
1. **Fase 1** — wrist sweep in posizione home
2. **Fase 2** — arc grid + wrist sweep con look-at dinamico
3. **Fase 3** — raffinamento adattivo sui keypoint

Il torso tracker YOLO (RealSense) pubblica `/torso_scan_point`.
Alla fine: `_finish_body_scan()` calcola:

- Centro torso 3D fuso (media pesata di tutti i punti scan)
- 4 keypoint 3D (spalle, fianchi)
- **5 punti FAST** come offset relativi al `torso_center` in frame **`world`** (= link00):
  - idx 0: Hub (centro, offset `(0,0,0)`)
  - idx 1: Subxiphoid
  - idx 2: RUQ (Morrison's pouch)
  - idx 3: LUQ (Koller's pouch)
  - idx 4: Suprapubic

FSM torna: `HOMING → WAITING` (con `_body_scan_done = True`).

---

## Fase 6 — Ciclo FAST: un punto alla volta

Per ognuno dei 5 punti, FSM esegue:

### 6a. Path normale: target nel workspace

```
CHECKING_WORKSPACE
  ├─ WorkspaceChecker verifica: |target_link00| ≤ max_reach − safety_margin?
  ├─ Se SI → was_clipped=False → APPROACHING
  └─ APPROACHING: pub_ik_goal → /z1/ik_goal_pose [frame: world/link00]
```

**Catena frame:**
```
Z1_FSM:               _make_approach_pose() → /z1/ik_goal_pose  [world]
ik_goal_mux:           inoltra a /ik_goal_pose (priorità WBC se abilitato)
z1_ik_to_jtc:          usa posizione raw come coordinate link00/Pinocchio
```

**Perché funziona:** FSM pubblica in `world` (= link00). L'IK solver (`z1_ik_to_jtc`) ignora `header.frame_id` e interpreta sempre la posizione come coordinate nel frame base Pinocchio (= `world`/`link00`). Frame coerente.

Dopo APPROACHING: `WAIT_IK_DONE → ... → IMPEDANCE_RUNNING → ... → SCAN_PRELIFT → next point`.

### 6b. Path alternativo: target fuori workspace → WS_EXTENSION

Se `was_clipped = True` (target oltre `max_reach - safety_margin`):

```
CHECKING_WORKSPACE (was_clipped=True)
  │
  ├─ FSM pubblica approach_goal su /wbc/ee_goal [frame: world/link00]
  ├─ FSM pubblica /wbc/ws_request = True
  └─ FSM → REQUESTING_WS_EXT
```

```
REQUESTING_WS_EXT
  │
  ├─ Coordinator riceve /wbc/ws_request → SCANNING → WS_EXTENSION
  │   _set_wbc_enabled(True) — riattiva WBC
  │
  ├─ WBC QP _update():
  │     goal_in = self._goal  da /wbc/ee_goal
  │     goal_in.header.frame_id = 'world'  ← pubblicato da FSM
  │     │
  │     ├─ Il QP riconosce frame_id = 'world' → goal è già in link00
  │     ├─ Trasforma world→odom per il calcolo dp (posizione goal in odom)
  │     ├─ Usa goal direttamente in link00 per orientazione look-at
  │     └─ WBC split → /wbc/ik_goal_pose + /my_spot/cmd_vel
  │
  ├─ Coordinator _tick_ws_extension(): monitora bounding box di sicurezza
  │   (ancorato alla posizione Spot all'ingresso WS_EXTENSION)
  │
  └─ Quando /ik_done = True → WS_EXTENSION → SCANNING
       WBC disabilitato, FSM → CHECKING_WORKSPACE (riprova)
```

**Frame handling in WS_EXTENSION:**
- Il FSM pubblica in `world`/`link00` (il suo frame nativo)
- Il WBC QP controlla `header.frame_id`: se è `world`/`link00`, sa che il goal è già nel frame braccio e lo trasforma solo per `dp` (in odom)
- Se invece il goal arriva in `odom` (da coordinator durante PRE_APPROACH/APPROACHING), lo trasforma a `link00` come al solito

Questo preserva la separazione: FSM lavora sempre in `world`, coordinator lavora sempre in `odom`, il QP fa da ponte accettando entrambi.

---

## Catena pubblicazione goal — tabella riassuntiva

| Fase | Publisher | Topic | Frame | Consumer | Frame usato |
|------|-----------|-------|-------|----------|-------------|
| PRE_APPROACH | Coordinator | `/wbc/ee_goal` | `my_spot/odom` | WBC QP | odom → link00 |
| APPROACHING | Coordinator | `/wbc/ee_goal` | `my_spot/odom` | WBC QP | odom → link00 |
| FAST (normale) | Z1 FSM | `/z1/ik_goal_pose` | `world` | z1_ik_to_jtc | usato come link00 |
| WS_EXTENSION | Z1 FSM | `/wbc/ee_goal` | `world` | WBC QP | link00 diretto (riconosciuto da frame_id) |

Routing:
- `/z1/ik_goal_pose` (FSM) → `ik_goal_mux` → `/ik_goal_pose` → `z1_ik_to_jtc`
- `/wbc/ik_goal_pose` (QP) → `ik_goal_mux` → `/ik_goal_pose` → `z1_ik_to_jtc`
- Il mux dà priorità al WBC quando `/wbc/enable = True`

---

## Convenzioni IK

- **Solver**: Pinocchio damped pseudo-inverse Jacobian, `LOCAL_WORLD_ALIGNED` frame
- **Frame base**: `'world'` = `link00` (dal modello URDF Z1)
- **`z1_ik_to_jtc` ignora `header.frame_id`**: usa la posizione raw del goal come coordinate Pinocchio
- **Traiettoria**: smoothstep quintic (10t³−15t⁴+6t⁵), zero vel/acc ai capi
- **Timing**: `T = max_joint_displacement / max_joint_vel`, clippato a `[traj_min_time, traj_max_time]`
- **Joint unwrapping**: `_make_target_near()` evita rotazioni >π tra waypoint
- **URDF**: auto-risolto via `ament_index` (fallback in `z1_ik_jtc_params.yaml`)

### Coordinate choice — perché due frame diversi

| Publisher | Frame | Motivazione |
|-----------|-------|-------------|
| Coordinator → WBC QP | `odom` | Target world-fixed, invariante ai movimenti Spot. WBC usa `dp` per navigazione |
| Z1 FSM → z1_ik_to_jtc | `world`/`link00` | L'IK risolve rispetto alla base del braccio. Naturale per il FSM |

Il WBC QP accetta entrambi i frame in ingresso e li risolve internamente:
- Goal in `odom` → lo trasforma a `link00` per la cinematica del braccio
- Goal in `world`/`link00` → lo trasforma a `odom` per il calcolo dell'errore di posizione

---

## Parametri critici (da tenere sincronizzati)

| Parametro | File | File | Valore |
|-----------|------|------|--------|
| `workspace_safety_margin` | `wbc_params.yaml` | `z1_fsm_params.yaml` | `0.05` m |
| `home_orientation` | `wbc_params.yaml` | `z1_fsm_params.yaml` | `[-0.0062, 0.4107, 0.0021, 0.9118]` |
| `ik_goal_topic` | `wbc_params.yaml` | `z1_fsm_params.yaml` | `/z1/ik_goal_pose` (passa da mux, non diretto!) |

---

## FSM stati

### WBC Coordinator FSM
```
WAITING_TF → IDLE → SEARCHING → PRE_APPROACH → APPROACHING → SCANNING → WS_EXTENSION
                                              ↑                            │
                                              └────────────────────────────┘
```

### Z1 FSM states
```
HOMING → WAITING → BODY_SCANNING → CHECKING_WORKSPACE → APPROACHING
    ↑                                                        ↓
    └──── SCAN_PRELIFT ← ... ← IMPEDANCE_RUNNING ← WAIT_IK_DONE
                                          ↕
                                    REQUESTING_WS_EXT  (→ CHECKING_WORKSPACE)
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

## World frame convention

```
X → verso il paziente (direzione approccio)
Y → testa-piedi (asse corpo paziente)
Z → destra-sinistra (laterale)
```

L'orientamento EE (X_ee) punta sempre verso il target. Calcolato via rotazione minima da home (Rodrigues) preservando la configurazione del polso.

---

## File chiave

| File | Ruolo |
|------|-------|
| `wbc_coordinator.py` | FSM Spot+Z1: WAITING_TF→IDLE→SEARCHING→PRE_APPROACH→APPROACHING→SCANNING→WS_EXTENSION, QualityMonitor |
| `wbc_qp_controller.py` | WBC olistico 10 Hz: split braccio + base, quality scaling, gestione multi-frame goal |
| `wbc_math.py` | Matematica pura: J_base (6×2), J_holistic (6×8), WBC split, WBC split with yaw |
| `z1_FSM.py` | FSM braccio: HOMING→WAITING→BODY_SCANNING→CHECKING_WORKSPACE→APPROACHING→IMpedance→FAST cycle |
| `z1_ik_to_jtc.py` | Pinocchio IK + smoothstep trajectory → JTC action |
| `z1_scan_manager.py` | Calcolo 5 punti FAST da keypoint torso |
| `ik_goal_mux.py` | Priority mux: WBC goals vs Z1 FSM goals |
| `tf_monitor.py` | Monitor 7 TF + 3 topic → `/wbc/tf_ready` |
