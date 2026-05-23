# TERESA — Piano rifattorizzazione launch (Core + App)

## Obiettivo

Ridurre i 6 terminali attuali a 3, separando i driver hardware (Orbbec, RealSense, Z1) dalla logica applicativa (percezione, controllo, WBC). Il Core segnala quando tutto è attivo via `/core/ready`.

```
T1: ros2 launch spot_control teresa_core.launch.py
T2: ros2 launch spot_control teresa_app.launch.py
T3: ros2 run spot_control wbc_keyboard_node
```

---

## Architettura

### `teresa_core.launch.py` — nuovo file in `spot_control/launch/`

| Cosa | Provenienza |
|---|---|
| Orbbec Femto Bolt driver | `spot_perception.launch.py` → estrarre `IncludeLaunchDescription` di `femto_bolt.launch.py` |
| TF statica `my_spot/body → orbbec_link` | `spot_perception.launch.py` → spostare `static_transform_publisher` |
| TF statica `orbbec_link → orbbec_color_optical_frame` | `spot_perception.launch.py` → spostare |
| Z1 bringup (`z1.launch.py`, robot_state_publisher + JTC + `/joint_states`) | `z1_realsense.launch.py` → estrarre `IncludeLaunchDescription` di `z1.launch.py` |
| RealSense driver | `z1_realsense.launch.py` → estrarre `IncludeLaunchDescription` di `rs_launch.py` |
| TF statica `link06 → camera_link` | `z1_realsense.launch.py` → spostare `static_transform_publisher` |
| Nodo `core_ready` | **Nuovo**: monitora `/joint_states` + `/orbbec/color/image_raw` + TF `link00→link06`. Quando tutti attivi, pubblica `/core/ready` Bool True una volta sola. |

### `teresa_app.launch.py` — nuovo file in `spot_control/launch/`

Include tramite `IncludeLaunchDescription`:
- `spot_perception.launch.py` con `use_orbbec_driver:=false`
- `z1_perception.launch.py`
- `z1_control.launch.py`
- `wbc.launch.py`

### `spot_perception.launch.py` — modifica

Aggiungere argomento `use_orbbec_driver` (default `true` per retrocompatibilità):

```python
use_orbbec_driver_arg = DeclareLaunchArgument(
    'use_orbbec_driver', default_value='true',
    description='Lancia Orbbec driver + TF statiche. false se già lanciato da teresa_core')
```

Quando `false`:
- **Salta** `orbbec_launch` (IncludeLaunchDescription di femto_bolt)
- **Salta** `static_tf_body_camera` e `static_tf_camera_optical`
- **Riduce** i `TimerAction`: yolo+posture+bbox+laying a t=1s invece di t=4s (Orbbec già attivo dal core)
- **Lancia** solo: yolo_skeleton, posture_analyzer, bbox_visualizer, laying_human_detector

### `core_ready` node — nuovo file `spot_control/spot_control/core_ready.py`

Nodo ROS 2 minimale:
1. Si sottoscrive a `/joint_states` → ricevuto almeno un messaggio = Z1 driver attivo
2. Si sottoscrive a `/orbbec/color/image_raw` (o un topic equivalente) → Orbbec attivo
3. Controlla TF `link00 → link06` con `lookup_transform` → robot_state_publisher funzionante
4. Quando tutti e 3 OK → pubblica `/core/ready` = True (una volta), logga conferma, esce

### `setup.py` — modifica

Aggiungere entry point per `core_ready`:
```python
'core_ready = spot_control.core_ready:main',
```

I nuovi launch file sono già coperti da `glob('launch/*.py')`.

---

## Terminali finali

```bash
# T1: Core (driver hardware + TF statiche)
ros2 launch spot_control teresa_core.launch.py
# Aspettare: [core_ready] Core pronto — tutti i driver attivi.

# T2: App (percezione + controllo + WBC)
ros2 launch spot_control teresa_app.launch.py
# Aspettare: [TF READY] SpotCore connesso — premi "s" per iniziare.

# T3: Keyboard
ros2 run spot_control wbc_keyboard_node
# Premere "s" → missione parte
```

---

## File da creare

| File | Ruolo |
|---|---|
| `src/spot_control/launch/teresa_core.launch.py` | Core launch: driver + TF statiche + core_ready |
| `src/spot_control/launch/teresa_app.launch.py` | App launch: include i 4 sub-launch applicativi |
| `src/spot_control/spot_control/core_ready.py` | Nodo monitor: controlla driver attivi → pubblica `/core/ready` |

## File da modificare

| File | Modifica |
|---|---|
| `src/spot_perception/launch/spot_perception.launch.py` | Aggiungere arg `use_orbbec_driver`; quando false, salta driver e TF, riduce TimerAction |
| `src/spot_control/setup.py` | Aggiungere entry point `core_ready` |
| `INIT.md` | Aggiornare sezione "Running Spot + Z1 WBC" con i 3 terminali |

## File invariati

`z1_realsense.launch.py`, `z1_perception.launch.py`, `z1_control.launch.py`, `wbc.launch.py` restano come sono — usabili standalone per debug, inclusi da `teresa_app.launch.py`.

---

# TERESA — Body Height/Pitch Optimization per FAST Points

## Obiettivo

Aggiungere controllo di altezza e inclinazione (pitch) del corpo di Spot durante la fase FAST, per ridurre l'estensione del braccio Z1 e mantenere configurazioni più naturali (miglior manipulability). Spot si adatta a ogni punto FAST invece di restare fermo all'handoff_height (-0.15m) per tutti i 5 punti.

## Contesto

**Problema attuale:** dopo BODY_SCANNING, i 5 punti FAST sono calcolati in frame `world` (= link00). Per i punti non-centro (idx 1-4), il FSM usa la posa centro salvata al punto 0 + offset relativo. Se Spot cambiasse altezza tra un punto e l'altro, la posa centro salvata diventerebbe stale (calcolata con la vecchia posizione link00). Inoltre la camera RealSense (su link06) non vede più il torso dopo il punto 0 (braccio vicino al paziente), quindi non si può ricalcolare live.

**Soluzione:** pre-calcolare tutti i target in frame **odom** (world-fixed, invariante ai movimenti Spot) durante BODY_SCANNING quando la camera ha piena visibilità. Poi, per ogni punto, trasformare il target da odom al link00 corrente (che riflette la nuova postura Spot) e usarlo come IK goal.

## Vincoli

- **Altezza:** `[-0.20, -0.15]` m (5 cm di range, Spot resta basso)
- **Pitch:** `[0°, 15°]` (0-0.26 rad)
- **Nessun cambio yaw** (lo yaw è gestito dal WBC durante APPROACHING)
- **Nessun impedance control** per questa fase di test

## Architettura

### Nuovi topic

| Topic | Direction | Tipo | Contenuto |
|-------|-----------|------|-----------|
| `/z1/fast_points` | FSM → Coordinator | `PoseArray` | 5 target FAST in frame `my_spot/odom`, più surface Z per ciascuno |
| `/z1/fast_ready` | Coordinator → FSM | `Bool` | Conferma: ottimizzazione completata, body_pose impostato per punto 0 |
| `/z1/approach_target` | Coordinator → FSM | `PoseStamped` | Target corrente in frame `world`/link00 (già trasformato da odom) |
| `/z1/next_point_idx` | FSM → Coordinator | `Int32` | Richiesta: prepara body_pose per il prossimo punto FAST (idx 1-4) |
| `/wbc/body_ready` | Coordinator → FSM | `Bool` | Spot ha raggiunto la postura richiesta, target disponibile su `/z1/approach_target` |

### Flusso

```
BODY_SCANNING (camera RealSense ha piena visibilità del torso)
  │
  ├─ BodySearchScanner completa 3 fasi → fused keypoints + torso_center
  ├─ ScanManager.set_fast_points() → 5 target FAST in world/link00
  ├─ Cattura surface frame (p_surf, normal) da realsense_surface_node
  ├─ Trasforma ogni target + surface da world/link00 → my_spot/odom via TF
  └─ Pubblica /z1/fast_points (PoseArray in odom, 5 pose + surface z)
  │
  ▼
FSM: HOMING → WAITING (aspetta /z1/fast_ready)
  │
Coordinator riceve /z1/fast_points:
  ├─ Per ogni idx 0..4, grid search su (height ∈ [-0.20,-0.15], pitch ∈ [0°,15°]):
  │     Trasforma target_odom[idx] → link00 simulando body_pose (h, p)
  │     Score = -‖target_in_link00 - sweet_spot(0.35, 0, 0.30)‖
  │     Salva (h*, p*) ottimale per ogni idx
  ├─ Applica body_pose per punto 0: _set_body_pose(h*[0], p*[0])
  ├─ Aspetta body_settle_time (1.5s)
  ├─ Trasforma target_odom[0] → link00 corrente via TF live
  ├─ Pubblica /z1/approach_target [frame: world, posizione in link00]
  └─ Pubblica /z1/fast_ready = True
  │
  ▼
Punto 0 (Hub):
  FSM: Riceve /z1/approach_target → CHECKING_WORKSPACE → APPROACHING
       IK done → WAIT (nessun impedance) → fine punto 0
       Pubblica /z1/next_point_idx = 1
  │
  ▼
Punto 1 (Subxiphoid):
  Coordinator: _set_body_pose(h*[1], p*[1]) → settle 1.5s
              Trasforma target_odom[1] → link00 via TF
              Pubblica /z1/approach_target
              Pubblica /wbc/body_ready = True
  FSM:        Riceve /z1/approach_target → CHECKING_WORKSPACE → APPROACHING
              IK done → WAIT → /z1/next_point_idx = 2
  ...
  │
  ▼
Punto 4 (Suprapubic) — ultimo punto:
  Uguale ai precedenti
  Dopo IK done → FSM pubblica /z1/next_point_idx = -1 (fine)
  Coordinator: _set_body_pose(-0.15, 0°) → torna in handoff
```

### Ottimizzazione postura (grid search nel coordinator)

```python
def _optimize_body_poses(self, fast_targets_odom):
    """Grid search: per ogni punto FAST, trova (h, p) che minimizza distanza dal sweet spot."""
    sweet_spot = np.array([0.35, 0.0, 0.30])  # centro workspace Z1 in link00
    heights = self._body_grid_heights    # [-0.20, -0.18, -0.15]
    pitches = self._body_grid_pitches    # [0.0, 0.087, 0.17, 0.26]
    
    for idx, target_odom in enumerate(fast_targets_odom):
        best_score = float('-inf')
        best_h, best_p = heights[0], pitches[0]
        
        for h in heights:
            for p in pitches:
                # Simula dove sarebbe link00 con questa postura
                link00_in_odom = self._simulate_link00_pose(h, p)
                # Trasforma target da odom a link00 simulato
                target_link00 = self._transform_odom_to_link00(target_odom, link00_in_odom)
                # Score: vicinanza al sweet spot (negativo = meglio)
                dist = np.linalg.norm(target_link00 - sweet_spot)
                score = -dist
                if score > best_score:
                    best_score = score
                    best_h, best_p = h, p
        
        self._optimal_height[idx] = best_h
        self._optimal_pitch[idx] = best_p
```

### Modifiche FSM — path APPROACHING unificato

**Prima (due rami):**
```python
if idx == 0:
    target = self._make_approach_pose()     # live surface + torso
else:
    c = self._scan_mgr._center_approach_pose  # salvata al punto 0 (stale se body cambia)
    target = c + offset_relative
```

**Dopo (ramo unico):**
```python
# Target pre-calcolato dal coordinator, già nel giusto frame link00
target = self._latest_approach_target  # da /z1/approach_target
```

La surface frame non serve più per i punti successivi: il target è già completo (posizione + orientamento) quando arriva dal coordinator.

Il FSM deve acquisire un TF buffer (`tf2_ros.Buffer` + `TransformListener`) per:
- Durante BODY_SCANNING: trasformare i target da world/link00 → odom
- Non serve per APPROACHING (il target arriva già in link00 dal coordinator)

### Casi edge

| Scenario | Comportamento |
|----------|---------------|
| **Standalone Z1 (no WBC/coordinator)** | `/z1/fast_ready` mai ricevuto → timeout `fast_ready_timeout` (10s) → procede senza body optimization |
| **Coordinator non disponibile a metà scan** | `/wbc/body_ready` timeout (3s) per punto → procede con target salvato in link00 originale |
| **Grid search non trova miglioramento** | Usa handoff_height (-0.15m) e pitch 0° come fallback |
| **idx = -1 (fine scan)** | Coordinator riporta Spot a handoff_height (-0.15m, 0°) |

## File da creare

Nessuno. La grid search è un metodo privato del coordinator (~40 righe).

## File da modificare

| File | Modifica |
|------|----------|
| **`z1_FSM.py`** | Aggiungere `tf2_ros` Buffer + TransformListener; publisher `/z1/fast_points` (PoseArray), `/z1/next_point_idx` (Int32); subscriber `/z1/fast_ready` (Bool), `/z1/approach_target` (PoseStamped), `/wbc/body_ready` (Bool); in `_finish_body_scan()`: trasformare target in odom e pubblicare `/z1/fast_points`; in WAITING: attendere `/z1/fast_ready`; in APPROACHING: usare `_latest_approach_target` (path unico); tra punti: pubblicare `/z1/next_point_idx` e attendere `/wbc/body_ready` |
| **`wbc_coordinator.py`** | Subscriber `/z1/fast_points` (PoseArray), `/z1/next_point_idx` (Int32); publisher `/z1/fast_ready` (Bool), `/z1/approach_target` (PoseStamped), `/wbc/body_ready` (Bool); metodo `_optimize_body_poses()` grid search; metodo `_simulate_link00_pose(h, p)`; logica per body_settle timer; lookup TF odom→link00 per pubblicare target corrente |
| **`wbc_params.yaml`** | `body_grid_heights: [-0.20, -0.18, -0.15]`, `body_grid_pitches: [0.0, 0.087, 0.17, 0.26]`, `body_sweet_spot: [0.35, 0.0, 0.30]`, `body_settle_time: 1.5` |
| **`z1_fsm_params.yaml`** | `fast_ready_timeout: 10.0`, `body_ready_timeout: 3.0` |
| **`setup.py` (spot_control)** | Verificare che eventuali nuovi entry point siano coperti (nessuno previsto) |

## File invariati

`wbc_math.py`, `wbc_qp_controller.py`, `z1_scan_manager.py`, `z1_ik_to_jtc.py`, `ik_goal_mux.py`.

---

## Nota (23 May 2026) — Paper angle e semplificazioni

### Body Pose Optimization: Whole-Body Planning, non WBC

L'ottimizzazione body pose per FAST points è più precisamente un **whole-body planning** o **cooperative mobile manipulation**, non un WBC in senso stretto:

| | WBC classico | Body Pose Optimization |
|---|:---:|:---:|
| Movimento | Braccio + Spot **contemporaneamente** | Spot si muove **prima**, braccio **dopo** |
| Matematica | Jacobian olistica in tempo reale | Grid search offline |
| Obiettivo | Minimizzare errore EE | Trovare configurazione ottimale |

**Paper angle suggerito:**

> **"Hierarchical Whole-Body Framework for Autonomous Ultrasound"**
> 
> Livello 1 (reactive): APPROACHING — arm look-at + Spot P-controller
> Livello 2 (planning): FAST — body pose optimization per ogni punto

Oppure più onesto: **"Decoupled Mobile Manipulation with Pre-Planned Body Reconfiguration"**.

### Nota implementativa

Il FSM pubblica target in `world` frame. Quando Spot cambia body_pose, `world` segue automaticamente via TF — l'IK solver riceve il target corretto **senza nessuna modifica ai target FSM**. Il FSM non deve leggere target dal coordinator — deve solo segnalare quando è pronto per il prossimo punto (`/z1/next_point_idx`) e attendere che Spot si sia assestato (`/wbc/body_ready`).

Il grid search è completamente offline: 1 solo TF lookup per trasformare i 5 punti da world a odom, poi tutta matematica locale. Nessun problema di clock desync.

### Vincoli di movimento

| Parametro | Range | Note |
|-----------|-------|------|
| Altezza | `[-0.20, -0.15]` m | 5 cm range, Spot resta basso |
| Pitch | `[0°, 15°]` | Massimo 15° |
| Yaw | **mai cambiato** | Fissato dal WBC during APPROACHING |
| Settle time | 1.5s | Tra body_pose e inizio punto FAST |
