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

### Perception Backend

Two perception backends are available, selectable via the `perception_backend` launch parameter:

- **`yolo` (default since 8 June 2026)**: YOLO11n-pose — 2D keypoints + depth back-projection + Kalman filtering. 24 joints published (17 COCO mapped + 7 NaN for SMPL-only joints). Runs at ~40 FPS, used during SEARCHING.
- **`nlf`**: NLF (Neural Localizer Fields) — direct 3D SMPL joints from RGB, no depth back-projection needed. 24 joints published. Runs at ~2.5 FPS. Starts in paused mode (`_streaming_paused = True`). Triggered at LOCKING con burst multi-frame (2 detection valide, EMA smoothing, timeout 30s). Pubblica prior raffinato su `/exposure/nlf_prior` e confidence su `/exposure/nlf_confidence`.

Switch at launch:
```bash
ros2 launch spot_control teresa_perception.launch.py perception_backend:=yolo
ros2 launch spot_control teresa_perception.launch.py perception_backend:=nlf
```

The `/human_pose/points_3d` topic always carries 24 SMPL joints regardless of backend. Both YOLO and NLF can coexist without conflicts — YOLO handles SEARCHING at high FPS, NLF provides refined priors at LOCKING.

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

# T2: Perception — Orbbec NLF + RealSense NLF (default)
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

## Fase 1 — SEARCHING (rewritten 8 June 2026)

`IDLE → SEARCHING` (premi `s` sulla tastiera)

### SEARCHING (rewritten 8 June 2026)

**Coarse search**: Spot alternates ±30° yaw (timed open-loop, no TF dependency). Each yaw position: arm cycles through 6 symmetric mathematically-generated poses (3 forward X=+0.12 + 3 look-behind X=-0.15, all Z=0.53, Y=±0.20). Orientation computed via `compute_ee_orientation()` — no forced FK-reader quaternions. After both yaws complete: arm returns HOME, Spot steps forward 20cm, cycle repeats.

- **Rotation**: timed `cmd_vel.angular.z = 0.2 rad/s` for ~2.6s per 30° step — no TF `odom→body` required
- **Arm poses**: 6 pose simmetriche generate matematicamente (3 forward X=+0.12 + 3 look-behind X=-0.15, Z=0.53, Y=±0.20). Orientamento calcolato da `compute_ee_orientation()` — X_ee punta avanti/dietro, Y_ee vicino a home. Nessun quaternione FK-reader forzato.
- **Wait logic**: coordinator counts 6 `ik_done` events before advancing to next yaw
- **Step forward**: 20cm at 0.3 m/s (~0.67s) after each full cycle
- **Refinement**: triggers when Orbbec posture confidence ≥ 0.30 during arm wait
- **Lock**: confidence ≥ 0.70 → direct LOCKING (no SEMI_LOCKING needed)

### Sensori coinvolti
- **Orbbec** (su Spot): YOLO11 → posture classifier → `approach_point` laterale in odom
- **RealSense** (sul polso): YOLO torso tracker → posizione 3D del torso

---

## Fase 2 — PRE_APPROACH (WBC LOOKAT mode)

`LOCKING → PRE_APPROACH` (dopo 5 campioni + `/ik_done`)

- Spot si raddrizza: `body_pose(height=0.0, pitch=0.0)`
- **Target LOOKAT**: `/laying_human/body_center` (torso centroid 3D, pubblicato dal laying_human_detector). Fallback a `approach_point` se non disponibile.
- **WBC QP Controller — LOOKAT mode**: calcola ω_des (errore orientamento X_ee → target), risolve il task con damped pseudo-inverse sul Jacobiano angolare J_task (3×6), applica joint centering nel null-space (N @ k_null * (q_mid - q_current)), integra q_dot in FK prediction, pubblica il goal di posa all'IK solver. Loop a 10 Hz.
- Coordinator conta 3 tick consecutivi di `ESTIMATING` o `LOCKED` da RealSense. **Timeout 5s** → APPROACHING comunque (fallback con warning).
- `ESTIMATING/LOCKED ×3 → APPROACHING`
- **NLF confidence gate**: se la confidence NLF post-burst è ≥ 0.80 (pubblicata su `/exposure/nlf_confidence`), il blend LOOKAT usa **100% NLF** (skip del delta check posizionale). Sotto 0.80, blend standard NLF(70%)+YOLO(30%) per HIGH coherence, YOLO 100% per LOW coherence.

---

## Fase 3 — APPROACHING (navigator + QP PERCEPTUAL_SCAN)

`PRE_APPROACH → APPROACHING`

- Entry: ferma cmd_vel, spegne guidance mode, disabilita navigator fino al primo tick
- Timeout globale `approach_timeout=60s`: se Spot non raggiunge handoff → `IDLE`

### Spot: wbc_spot_navigator
Navigatore semplificato: riceve il goal in odom, trasforma in body frame, rotate → drive → stop. P-controller robusto (1 TF hop `odom→body`), indipendente dal QP.

### Braccio: QP Controller — PERCEPTUAL_SCAN mode
Il QP riceve il segnale di APPROACHING e passa in modalità PERCEPTUAL_SCAN:

1. **Griglia Cartesiana adattiva** (`_gen_cartesian_scan_grid`): usa le confidence pre-scan dei 4 keypoint torso (`/torso_keypoint_conf`).
   - Se tutti ≥ `pre_scan_conf_thr=0.6` → **griglia ridotta**: HOME, +X+Z (2 pose, ~6s)
   - Altrimenti → **griglia completa**: HOME, +X+Y, HOME transit, +X-Y, HOME transit, +X+Z (4 pose raccolta + 2 transit, ~12s)
   - Tutte le pose hanno advance X=`cartesian_x_advance=0.10`m verso il paziente. Look-at verso target con minrot. Z ≥ 0.44m, workspace clipping.

2. **Sequencing** (`BodySearchScanner`): per ogni posa pubblica goal all'IK solver → attende `ik_done` → raccoglie dati di detection per 3s (minimo 5 frame, early stop se score ≥ 0.95) → passa alla posa successiva.

3. **Fusione + FAST points**: al termine fonde le stime 3D di tutte le pose. Calcola i 5 punti FAST (Hub, Subxiphoid, RUQ, LUQ, Suprapubic) con offset fissi dal torso stimato.

### Handoff
- Soft handoff a 0.50m: se il QP non ha ancora finito la scansione → Spot aspetta
- Hard handoff a 0.05m → `APPROACHING → WAITING_EXPOSURE` (se manual gate) o `APPROACHING → EXPOSURE_SCANNING` (se auto)

---

## Fase 3b — EXPOSURE_SCANNING (body scan + camera)

`WAITING_EXPOSURE → EXPOSURE_SCANNING` (conferma manuale o auto)

### exposure_scanner node
- Nodo dedicato (`exposure_scanner.py`, 650 righe), stesso pattern per-punto del FAST
- **Full-body grid**: 14 punti su 7 regioni (HEAD=2, TORSO=4, ARM×2=2+2, LEG×2=2+2, FEET=2) generate dai 17 keypoint COCO Orbbec trasformati in world frame via TF
- **NLF prior grid**: la griglia exposure usa preferenzialmente lo skeleton NLF (24 SMPL, EMA-raffinato) quando disponibile. Fallback ai keypoint YOLO se il prior NLF non è stato catturato.
- **Stima head da spalle**: se naso occluso nell'Orbbec, posizione testa stimata da (spalla_sx + spalla_dx) / 2 + offset verticale
- **Look-at dinamico**: EE X verso corpo via `compute_ee_orientation`, stessa funzione del FAST ultrasound
- **Standoff orizzontale**: 0.50 m verso Spot (X negativo), non verticale. Il braccio resta in configurazione naturale.
- **Scheletro raffinato progressivo**: accumula `/exposure/body_keypoints` (17 kp RealSense) durante il dwell, running average (α=0.5), pubblica su `/exposure/refined_skeleton` — da 0/17 a 17/17 durante lo scan
- Per ogni punto: body_pose(h,p) → settle 1.5s → body_ready → IK goal → ik_done → dwell 2s → running average kp → next point
- Pubblica `/exposure/grid_markers` (MarkerArray, color-coded per regione) per overlay web
- **JSON output**: `/tmp/exposure_scan_YYYYMMDD_HHMMSS.json` con per-regione camera pose, surface position, scan data frames
- Al completamento pubblica `/exposure/ready`

### Spot
- **NON cammina** — solo body_pose (height + pitch) per spostare il workspace del braccio lungo il corpo
- Pattern identico al FAST: `_optimize_body_poses()` + `_set_body_pose()` + settle

### Braccio
- Posiziona la RealSense sui punti della griglia (nessun contatto, nessuna sonda)
- Look-at verso il corpo, standoff ~0.50m
- Nessun impedance controller (solo posizionamento camera)

### Web UI
- **Grid toggle su RealSense**: pulsante `[Grid]` nella barra overlay proietta i 14 marker griglia sull'immagine RealSense con colori per regione
- **Legenda colori**: barra sotto yolo-bar con 7 swatch: HEAD (giallo), TORSO (blu), L-ARM (rosso), R-ARM (arancione), L-LEG (verde), R-LEG (verde chiaro), FEET (viola)
- Punto corrente: marker grande + glow bianco. Visitati: piccoli e trasparenti. Da visitare: medi e semi-trasparenti.
- **Click-to-revisit**: click su marker → `/exposure/goto_point(id)` → Spot riposiziona il braccio su quel punto
- **Body Map panel**: toggle via `&#128506;` o tasto `m`. Canvas top-down (X-Y world) con scheletro progressivo (17 kp + linee COCO) e griglia exposure. Auto-scalato, auto-fit.

---

## Fase 3c — EXPOSURE_REVIEW (interattiva)

`EXPOSURE_SCANNING → EXPOSURE_REVIEW` (dopo `/exposure/ready`)

### Interazione
- L'operatore clicca su qualsiasi punto blu della griglia nella web UI
- Click → `/exposure/goto_point` (Int32: indice punto)
- Spot torna alla body_pose ottimizzata per quel punto
- Braccio riproduce il tracciato IK salvato
- Camera inquadra la regione finché l'operatore non clicca altro

### Terminazione
- Pulsante `Terminate` nella web UI o tasto `n` sulla tastiera
- `/exposure/terminate` (Bool) → `WAITING_FAST` (se manual gate) o `SCANNING` (se auto)

### Manual scan gate
- Parametro `manual_scan_gate` (default true) in `wbc_params.yaml`
- Quando true: il FSM si ferma a WAITING_EXPOSURE e WAITING_FAST
- Conferma via tasto `n` (keyboard) o pulsante STEP (web UI)
- Toggle MANUAL/AUTO nella web UI (`teresa_control.html`)
- Quando false: avanzamento automatico

### Topic nuovi per exposure
| Topic | Tipo | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/exposure/grid_markers` | MarkerArray | exposure_scanner | camera_view.html |
| `/exposure/goto_point` | Int32 | camera_view.html | exposure_scanner |
| `/exposure/terminate` | Bool | camera_view.html / keyboard | wbc_coordinator |
| `/exposure/ready` | Bool | exposure_scanner | wbc_coordinator |
| `/exposure/refined_skeleton` | PoseArray | exposure_scanner | web UI (Body Map) |
| `/exposure/body_keypoints` | PoseArray | z1_yolo_torso_tracker | exposure_scanner |
| `/wbc/set_manual_scan_gate` | Bool | teresa_control.html | wbc_coordinator |
| `/wbc/manual_scan_gate` | Bool | wbc_coordinator | teresa_control.html |

---

## Fase 4 — SCANNING (FAST points + ottimizzazione body pose)

`WAITING_FAST → SCANNING` (conferma manuale o auto)

- WBC viene disabilitato (`/wbc/enable=False`), Spot si abbassa a handoff height (-0.15m)
- Il QP Controller ha già pubblicato `/z1/fast_points` (5 Pose in frame link00) durante APPROACHING
- Timeout globale `scan_timeout=120s`: se la scansione si blocca → IDLE
- Parametri parametrizzati: `max_workspace_reach=0.60` (soglia WS_EXT), `ws_ext_goal_tolerance=0.15`

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
| **SEARCHING** | ±30° yaw timed open-loop + step forward 20cm | 6 pose simmetriche generate matematicamente in loop | Rotation + arm poses |
| **SEMI_LOCKING** | Ruotato+inclinato verso torso | LOOKAT attivo subito (re_enable=True) | Arm LOOKAT |
| **LOCKING** | Fermo, al miglior pitch del refinement. Attende NLF burst (2 detection o 30s timeout). | Prima posa di search. Resta fermo. | Off → search pose. NLF burst pubblica prior raffinato. |
| **PRE_APPROACH** | Dritto, fermo | LOOKAT verso body_center (ω_des + joint centering) | Arm-only WBC |
| **APPROACHING** | Navigatore → goal (timeout 60s) | PERCEPTUAL_SCAN (2-4 pose adattive + advance X) | Cartesian grid adattiva |
| **SCANNING** | Body pose (h,p) + WS_EXT (h,p,dx,dy), timeout 120s | z1_FSM: FAST cycle 5 punti, body_ready safe skip | Off |

---

## Architettura WBC

### Componenti (lanciati da `wbc.launch.py`)

| Nodo | Ruolo |
|------|-------|
| `wbc_coordinator` | FSM (11 stati). PRE_APPROACH: LOOKAT verso body_center con Z offset +0.40m fallback, sliding window (≥1 ESTIMATING/LOCKED su 5 tick). |
| `exposure_scanner` | Full-body exposure scan: 14-pose grid su 7 regioni, look-at dinamico, standoff orizzontale 0.50m, TF Orbbec→world, running-average scheletro raffinato su `/exposure/refined_skeleton`, JSON output. |
| `exposure_snapshot` | Snapshot RealSense su click in EXPOSURE_REVIEW. Trigger `/exposure/goto_point` + `/ik_done`, delay 1s, pubblica `/exposure/snapshot`, salva JPEG su disco. |
| `wbc_qp_controller` | **Arm-only WBC, 3 modalità**: ACTIVE_SEARCH (6 symmetric mathematically-generated poses), LOOKAT (ω_des + joint centering), PERCEPTUAL_SCAN (griglia adattiva 2-4 pose). |
| `wbc_spot_navigator` | Navigatore semplificato per APPROACHING e WS_EXT. |

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

Durante **SEARCHING** il QP opera in **ACTIVE_SEARCH mode**: 6 pose simmetriche generate matematicamente (3 forward X=+0.12 + 3 look-behind X=-0.15, Z=0.53, Y=±0.20), eseguite in loop mentre Spot ruota ±30° yaw. Orientamento calcolato da `compute_ee_orientation()` invece di quaternioni FK-reader forzati. Il braccio esplora lo spazio senza un target reale, compensando la rotazione di Spot. Spot ruota via cmd_vel a tempo (open-loop, no TF); il refinement (sweep pitch) avviene a livello body_pose nel coordinator.

Durante **PRE_APPROACH** il QP opera in **LOOKAT mode**: calcola l'errore di orientamento tra X_ee e target (`ω_des = kp_ang * angle * axis`), risolve con damped pseudo-inverse su J_task (3×6), proietta joint centering nel null-space (`N @ k_null * (q_mid - q)`), integra in FK prediction, pubblica il goal all'IK solver. Loop a 10 Hz.

In **APPROACHING** il QP passa in **PERCEPTUAL_SCAN mode**: 6 pose Cartesiane multi-angolo attorno alla posizione corrente EE (home, ±Y, +Z, +X, +X+Y). Passo 0.12m, look-at verso il target reale. Le sequenzia con BodySearchScanner, raccoglie dati di detection per ogni posa (3s, min 5 frame), fonde le stime 3D, pubblica i 5 punti FAST.

Spot è sempre controllato dal navigatore (APPROACHING) o dal coordinator (body pose in SEARCHING/SCANNING), mai dal QP.

#### SEMI_LOCKING e LOCKING (gestione QP)
- **SEMI_LOCKING**: il QP va in pausa — il braccio si blocca nella posa corrente. Triggerato da RealSense `ESTIMATING` o `LOCKED`. L'Orbbec ha 3s di finestra pulita.
- **LOCKING**: il WBC viene spento immediatamente, poi riattivato per mandare il braccio alla prima posa di search. Spot applica il best pitch dal refinement per la miglior visuale Orbbec. NLF esegue un burst multi-frame (2 detection valide con EMA, timeout 30s). Il coordinator raccoglie 5 campioni in parallelo. La transizione a PRE_APPROACH è bloccante: richiede (5 campioni + ik_done + NLF valido o timeout).
- Se RealSense perde il segnale durante SEMI_LOCKING, o se Orbbec perde LYING per >1s durante LOCKING: si riprende la ricerca dalla posizione corrente.
```
SEARCHING:
  QP Controller (ACTIVE_SEARCH) → 6 symmetric mathematically-generated poses, loop
  Coordinator → ±30° yaw timed rotation (open-loop) + step forward 20cm

SEMI_LOCKING:
  QP Controller → PAUSA (braccio congelato)
  Coordinator → Spot ruotato+inclinato verso torso

LOCKING:
  WBC spento → QP Controller → home pose
  Coordinator → 5 campioni approach_point

PRE_APPROACH:
  QP Controller (LOOKAT) → IK goal (ω_des + joint centering)
  Navigatore: fermo

APPROACHING:
  Navigatore → rotate → drive → stop
  QP Controller (PERCEPTUAL_SCAN) → 6 pose Cartesiane → BodySearchScanner → fuses → FAST points
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
| `teresa_perception.launch.py` | Perception launch: Orbbec NLF skeleton + RealSense NLF torso tracker (default); YOLO fallback via `perception_backend:=yolo` |
| `wbc.launch.py` | WBC launch: coordinator + QP + navigator |
| `wbc_coordinator.py` | FSM Spot+Z1: WAITING_TF→IDLE→SEARCHING→SEMI_LOCKING→LOCKING→PRE_APPROACH→APPROACHING→SCANNING. Coarse rotation + refinement pitch. QualityMonitor. Body pose grid search (h,p) + WS_EXT fallback. Per-point body_ready. |
| `wbc_qp_controller.py` | WBC arm-only, 3 modalità: ACTIVE_SEARCH / LOOKAT (damped pseudo-inverse) / PERCEPTUAL_SCAN. Mai muove Spot. |
| `wbc_spot_navigator.py` | Navigator semplificato: rotate → drive → stop verso goal odom. Usato anche per WS_EXT drive. |
| `wbc_math.py` | Matematica pura: `damped_pinv`, `null_space_projector`, manipulability (J_base/J_holistic/wbc_split deprecati) |
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

# T2: Perception — Orbbec NLF + RealSense NLF (default)
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
| `src/z1_vision/` | Z1 arm: FSM, IK, impedance, NLF (default) / YOLO torso tracking, workspace checker |
| `src/spot_control/` | Spot navigation, WBC coordinator, WBC QP controller, ik_goal_mux, perception launcher |
| `src/spot_perception/` | Orbbec perception: NLF skeleton (default) / YOLO skeleton fallback, posture classifier, laying detector |
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

### Perception Models

Two model families are used depending on the `perception_backend` parameter:

- **NLF (default)**: Neural Localizer Fields models downloaded via `scripts/download_nlf_models.sh` to `models/nlf/`. Used by `nlf_skeleton.py` (Orbbec) and `nlf_torso_tracker.py` (RealSense).
- **YOLO fallback**: `yolo11n-pose.pt` lives at the workspace root. Used by `z1_yolo_torso_tracker` (RealSense) and `yolo_skeleton_spot` (Orbbec).

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
