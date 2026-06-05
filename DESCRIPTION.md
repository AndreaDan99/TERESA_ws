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

## Fase 1 — SEARCHING (ricerca adattiva coarse + refinement, 360°)

`IDLE → SEARCHING` (premi `s` sulla tastiera)

Ricerca a due livelli: scansione **coarse** 360° con rotazione cmd_vel, e **refinement** locale (sweep pitch) trigger-based appena una camera vede qualcosa.

### Spot: coarse rotation via cmd_vel
- Altezza nominale (`search_body_height=0m`)
- 6 posizioni coarse: yaw step ≈60° (`search_yaw_increment=1.05`, `search_yaw_steps=6`) → 360°
- Rotazione con `cmd_vel.angular.z` P-control (`search_yaw_kp=0.8`, max `search_max_angular_vel=0.5 rad/s`) — **non** body_pose yaw
- Target yaw assoluto in odom via TF `odom→body`, tolleranza `search_yaw_tolerance=0.08` (~4.6°)
- Fallback su `_last_yaw_error` se TF non disponibile durante la rotazione
- A ogni posizione raggiunta: dwell `search_coarse_dwell=5s` fermo, le camere osservano

### Refinement: pitch sweep adattivo (trigger-based)
Durante il dwell coarse, `_should_refine()` controlla se una camera vede qualcosa:
- **Trigger**: RealSense tracker `== GUIDING` (qualsiasi keypoint) **oppure** Orbbec conf `≥ search_refine_trigger_orb_conf=0.30`
- Entra in refinement (`_tick_refinement`): sweep pitch `search_pitch_angles=[0°, 5°, 10°]`, dwell `search_refine_dwell=4s` per pitch
- Traccia la migliore Orbbec conf + relativo `approach_point`
- `best_conf ≥ search_lock_confidence=0.70` → `_finish_refinement_lock` → **LOCKING** (fornisce già 1 campione)
- altrimenti → `_finish_refinement_fail` → resume coarse dal prossimo yaw
- Sequenza coarse esaurita senza lock → `IDLE`

### Braccio: QP Controller — ACTIVE_SEARCH mode
- `_gen_cartesian_search_grid()`: **3 pose wide** attorno alla home — HOME, LEFT, RIGHT
- Tilt fisso **-15°** pitch down (no wrist sweep), sweep Y ±0.28m, X +0.20m, Z=0.42m
- Z mai sotto la home (0.44m), workspace clipping automatico
- BodySearchScanner in loop infinito: per ogni posa → raccolta dati → prossima

### Tracker state GUIDING
Il torso tracker RealSense ha 4 stati: `IDLE → ESTIMATING → LOCKED` + **GUIDING** (giallo).
In guidance mode (attivo durante SEARCHING via `/tracker_guidance_mode`), qualsiasi keypoint valido → `GUIDING`. Serve a triggerare il refinement e a guidare il SEMI_LOCKING.

### SEMI_LOCKING (RealSense guida Spot)
- Trigger: tracker in `GUIDING`/`ESTIMATING`/`LOCKED` (`_check_realsense_guidance`)
- **GUIDING**: richiede ≥2 keypoint con conf ≥ 0.5 (rafforzato per ridurre falsi positivi)
- Coordinator calcola yaw e pitch ottimali per puntare l'Orbbec al torso, Spot ruota+inclina
- Braccio in LOOKAT mode attivo (IK acceso subito, `_end_search(re_enable=True)`)
- Settle TF-based (tolleranza yaw 0.05, pitch 0.03). Pitch riceve flush cmd_vel per applicazione body_pose
- Dwell `search_semi_lock_dwell=3s` di finestra pulita per l'Orbbec, timeout settle 5s
- Se Orbbec conferma → `LOCKING`; se dwell timeout → riprende ricerca dalla posizione corrente

### Lock: raccolta e conferma
- **LOCKING**: braccio torna in home rialzata (`home_lock_z=0.60`, vista migliore per RealSense)
- Coordinator raccoglie `search_lock_samples=5` campioni `approach_point` in odom (10 Hz)
- Tolleranza 1s se Orbbec perde momentaneamente LYING
- 5 campioni raccolti + braccio in home (`/ik_done`) → media → target fissato → `PRE_APPROACH`
- Se Orbbec persa per >1s → riprende ricerca dalla posizione corrente (non da zero)

### Sensori coinvolti
- **Orbbec** (su Spot): YOLO11 → posture classifier → `approach_point` laterale in odom
- **RealSense** (sul polso): YOLO torso tracker → posizione 3D del torso (GUIDING/ESTIMATING/LOCKED)

---

## Fase 2 — PRE_APPROACH (WBC LOOKAT mode)

`LOCKING → PRE_APPROACH` (dopo 5 campioni + `/ik_done`)

- Spot si raddrizza: `body_pose(height=0.0, pitch=0.0)`
- **Target LOOKAT**: `/laying_human/body_center` (torso centroid 3D, pubblicato dal laying_human_detector). Fallback a `approach_point` se non disponibile.
- **WBC QP Controller — LOOKAT mode**: calcola ω_des (errore orientamento X_ee → target), risolve il task con damped pseudo-inverse sul Jacobiano angolare J_task (3×6), applica joint centering nel null-space (N @ k_null * (q_mid - q_current)), integra q_dot in FK prediction, pubblica il goal di posa all'IK solver. Loop a 10 Hz.
- Coordinator conta 3 tick consecutivi di `ESTIMATING` o `LOCKED` da RealSense. **Timeout 5s** → APPROACHING comunque (fallback con warning).
- `ESTIMATING/LOCKED ×3 → APPROACHING`

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
- Nodo dedicato (`exposure_scanner.py`), stesso pattern per-punto del FAST
- Genera griglia punti 3D sul corpo del paziente dai keypoint COCO
- Per ogni punto: body_pose(h,p) → settle 1.5s → body_ready → IK goal → ik_done → dwell 2s
- Pubblica `/exposure/grid_markers` (MarkerArray) per overlay web
- Salva gli IK goals per replay durante la review
- Al completamento pubblica `/exposure/ready`

### Spot
- **NON cammina** — solo body_pose (height + pitch) per spostare il workspace del braccio lungo il corpo
- Pattern identico al FAST: `_optimize_body_poses()` + `_set_body_pose()` + settle

### Braccio
- Posiziona la RealSense sui punti della griglia (nessun contatto, nessuna sonda)
- Look-at verso il corpo, standoff ~0.50m
- Nessun impedance controller (solo posizionamento camera)

### Web UI
- Overlay blu sulla RealSense: marker per ogni punto griglia
- Punto corrente: anello blu brillante. Visitati: blu trasparente. Da visitare: blu medio.
- Toggle `Exposure` nella barra overlay RealSense

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
| **SEARCHING** | 6 yaw coarse (cmd_vel) + refinement pitch | 3 pose Cartesiane in loop (ACTIVE_SEARCH) | Coarse rotation + refinement |
| **SEMI_LOCKING** | Ruotato+inclinato verso torso | LOOKAT attivo subito (re_enable=True) | Arm LOOKAT |
| **LOCKING** | Fermo | Va in home Z=0.60 | Off → home |
| **PRE_APPROACH** | Dritto, fermo | LOOKAT verso body_center (ω_des + joint centering) | Arm-only WBC |
| **APPROACHING** | Navigatore → goal (timeout 60s) | PERCEPTUAL_SCAN (2-4 pose adattive + advance X) | Cartesian grid adattiva |
| **SCANNING** | Body pose (h,p) + WS_EXT (h,p,dx,dy), timeout 120s | z1_FSM: FAST cycle 5 punti, body_ready safe skip | Off |

---

## Architettura WBC

### Componenti (lanciati da `wbc.launch.py`)

| Nodo | Ruolo |
|------|-------|
| `wbc_coordinator` | FSM: WAITING_TF → IDLE → SEARCHING → SEMI_LOCKING → LOCKING → PRE_APPROACH → APPROACHING → SCANNING. Hybrid lock (Orbbec full + RealSense semi-lock da GUIDING/ESTIMATING/LOCKED con ≥2 kp a conf≥0.5). WBC spento immediatamente all'ingresso in LOCKING. QualityMonitor. Body pose control. FAST body pose grid search (h,p) + WS_EXT fallback. PRE_APPROACH: LOOKAT verso body_center, ESTIMATING/LOCKED ×3 tick. APPROACHING: timeout 60s. |
| `wbc_qp_controller` | **Arm-only WBC, 3 modalità**: ACTIVE_SEARCH (3 pose Cartesiane wide, tilt fisso -15°, loop infinito), LOOKAT (ω_des + joint centering, LOOKAT subito attivo dopo `_end_search(re_enable=True)`), PERCEPTUAL_SCAN (griglia adattiva 2-4 pose con advance X, HOME transit, BodySearchScanner, FAST points). |
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

Durante **SEARCHING** il QP opera in **ACTIVE_SEARCH mode**: 3 pose Cartesiane wide attorno alla home (HOME/LEFT/RIGHT, sweep Y ±0.28m, X +0.20m, Z=0.42m, tilt fisso -15°). Il braccio esplora lo spazio senza un target reale, compensando la rotazione lenta di Spot. Spot ruota in coarse via cmd_vel; il refinement (sweep pitch) avviene a livello body_pose nel coordinator.

Durante **PRE_APPROACH** il QP opera in **LOOKAT mode**: calcola l'errore di orientamento tra X_ee e target (`ω_des = kp_ang * angle * axis`), risolve con damped pseudo-inverse su J_task (3×6), proietta joint centering nel null-space (`N @ k_null * (q_mid - q)`), integra in FK prediction, pubblica il goal all'IK solver. Loop a 10 Hz.

In **APPROACHING** il QP passa in **PERCEPTUAL_SCAN mode**: 6 pose Cartesiane multi-angolo attorno alla posizione corrente EE (home, ±Y, +Z, +X, +X+Y). Passo 0.12m, look-at verso il target reale. Le sequenzia con BodySearchScanner, raccoglie dati di detection per ogni posa (3s, min 5 frame), fonde le stime 3D, pubblica i 5 punti FAST.

Spot è sempre controllato dal navigatore (APPROACHING) o dal coordinator (body pose in SEARCHING/SCANNING), mai dal QP.

#### SEMI_LOCKING e LOCKING (gestione QP)
- **SEMI_LOCKING**: il QP va in pausa — il braccio si blocca nella posa corrente. Triggerato da RealSense `ESTIMATING` o `LOCKED`. L'Orbbec ha 3s di finestra pulita.
- **LOCKING**: il WBC viene spento immediatamente, poi riattivato per mandare il braccio in home. Il coordinator raccoglie 5 campioni in parallelo.
- Se RealSense perde il segnale durante SEMI_LOCKING, o se Orbbec perde LYING per >1s durante LOCKING: si riprende la ricerca dalla posizione corrente.
```
SEARCHING:
  QP Controller (ACTIVE_SEARCH) → 3 pose Cartesiane wide, loop infinito
  Coordinator → coarse rotation cmd_vel (6 yaw) + refinement pitch trigger-based

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
| `teresa_perception.launch.py` | Perception launch: Orbbec YOLO posture + RealSense YOLO torso tracker |
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
