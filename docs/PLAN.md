# TERESA — Piani futuri

---

## ✅ Pitch-Based Search + NLF Lazy-Load + Semi-Lock Robustness — IMPLEMENTED (14 June 2026)

### Fatto
- ✅ **SEARCHING redesign**: pitch-based (+10°/+5°/0°), no yaw rotation, 50cm step forward (was 20cm)
- ✅ **7 arm search poses**: 3 forward with 10° camera tilt + 3 behind (original quaternions) + return
- ✅ **SEMI_LOCKING**: RealSense gate (dwell only if person still visible), yaw restoration on fail, Orbbec dwell 3s→5s, cooldown 3 ticks
- ✅ **TF fixes**: `_tf_lookup()` uses `rclpy.time.Time()` instead of `get_clock().now()`, timeout 1s→10s
- ✅ **NLF lazy-load**: skeleton always launched, model loads only on `/nlf/trigger`
- ✅ **`/wbc/perception_enable`**: transient_local QoS, enables perception on SEARCHING, disables on IDLE
- ✅ **spot_control gating**: navigator disabled in WAITING_TF and SEARCHING
- ✅ **Dead code removed**: `wbc_approach_scanner.py`, `test_legacy/`, `SEARCH_HOME_POS`/`SEARCH_HOME_ORI`, `_pub_debug_marker()`
- ✅ **Refinement mode removed**: pitch sweep is now part of main search cycle

### File modificati
| File | +/- |
|------|-----|
| `wbc_coordinator.py` | ~+200/−150 |
| `wbc_qp_controller.py` | ~+50/−30 |
| `nlf_skeleton.py` | ~+20/−5 |
| `wbc_params.yaml` | ~+10/−5 |
| `spot_perception.launch.py` | ~+2/−1 |
| `wbc_approach_scanner.py` | −40 |
| `test_legacy/` | −all |

---

## ✅ Body Pose Optimizer + Y-Walking + Patient Body TF — IMPLEMENTED (12 June 2026)

### Fatto
- ✅ `body_pose_optimizer.py`: nuovo nodo (~600 righe). 2D/3D/4D grid search + IK-driven retry loop
- ✅ `test_exposure_poses.py`: Y-walking 3D (spot_y×h×p), corpo virtuale 1.70m in odom
- ✅ `laying_human_detector.py`: pubblica TF `patient_body` (body frame da keypoint)
- ✅ `wbc_coordinator.py`: refactoring (-340 righe), integrato optimizer, TF lookup per yaw/approccio
- ✅ IK-driven retry: 2D→3D→4D basato su `/ik_done` timeout, non su soglia distanza

### File modificati
| File | +/- |
|------|-----|
| `body_pose_optimizer.py` | +600 |
| `wbc_coordinator.py` | -340 |
| `laying_human_detector.py` | +77 |
| `test_exposure_poses.py` | +190 |
| `setup.py` | +1 |

---

## ✅ NLF Burst Streaming + Confidence Gate — IMPLEMENTED (9 June 2026)

### Fatto
- ✅ NLF trigger: one-shot → burst multi-frame con EMA (2 detection valide, timeout 30s)
- ✅ EXCELLENT confidence tier: ≥ 0.80 → 100% NLF
- ✅ LOCKING bloccante: attende NLF prima di PRE_APPROACH
- ✅ Best pitch applicato su tutti i path di ingresso LOCKING
- ✅ Launch fix: nlf_skeleton_node solo con perception_backend:=nlf
- ✅ Publish suppression su /human_pose/points_3d durante burst
- ✅ `/exposure/nlf_confidence` topic per confidence NLF

### File modificati
| File | +/− |
|------|-----|
| `nlf_skeleton.py` | +117/−34 |
| `wbc_coordinator.py` | +11/−11 |
| `nlf_params.yaml` | +3 |
| `wbc_params.yaml` | +2/−1 |
| `spot_perception.launch.py` | +2/−1 |

---

## ✅ Exposure Body Scanning — IMPLEMENTED (5 June 2026)

L'exposure body scanning è stato implementato con differenze rispetto al piano originale:

### Fatto (5-6 June 2026)
- ✅ `exposure_scanner.py`: riscritto da 310 a 650 righe. Full-body grid 14 punti su 7 regioni, look-at dinamico, standoff orizzontale 0.50m, TF Orbbec→world, running-average scheletro raffinato, JSON output
- ✅ `exposure_snapshot.py`: nuovo nodo (128 righe). Snapshot RealSense su click in EXPOSURE_REVIEW, JPEG su disco, pubblica `/exposure/snapshot`
- ✅ `z1_yolo_torso_tracker.py`: publisher `/exposure/body_keypoints` (17 kp COCO in scan mode)
- ✅ `wbc_coordinator.py`: FSM 11 stati, `_cb_next_point` esteso, PRE_APPROACH fix (Z offset +0.40m, sliding window ≥1/5 tick)
- ✅ Web UI: Grid toggle + legenda, click-to-revisit, Body Map (🗺, tasto `m`), snapshot freeze + badge + Close, gate toggle sempre visibile
- ✅ Paper: abstract, introduction, active perception (IV.C), system architecture aggiornati
- ✅ Head stima da spalle, fix segno x_ee, rimozione dead code

### NON ancora fatto (rispetto al PLAN.md sottostante)
- ❌ `injury_detector.py` (nodo ROS per YOLO ferite/bruciature)
- ❌ `human_approach_detector.py` (approccio frontale SITTING/STANDING)
- ❌ `InjuryDetection.msg` + `InjuryDetectionArray.msg`
- ❌ Download modelli YOLO (`best_yolov8n_roboV3.pt`, `skin_burn_2022_8_21.pt`)
- ❌ Supporto reale keypoint 3D per SITTING/STANDING (usa placeholder)
- ❌ Il paper usa footnote per i modelli invece di reference bibliografiche

### File creati/modificati
| File | +/− |
|------|-----|
| `exposure_scanner.py` | +290 (nuovo) |
| `wbc_coordinator.py` | +95/−15 |
| `experiment_logger.py` | +25 |
| `camera_view.html` | +100 |
| `teresa_control.html` | +55 |
| `wbc_params.yaml` | +12 |
| `wbc.launch.py` | +8 |
| `setup.py` | +2 |

---

## ✅ NLF Prior at LOCKING — IMPLEMENTED (7 June 2026)

### Fatto
- ✅ NLF single-frame prior triggered at LOCKING, 10s timeout
- ✅ Binary fallback: if NLF fails → entire system = 6 June 2026 behavior (YOLO-only)
- ✅ Gate: `_nlf_prior_valid()` controls all branches (PRE_APPROACH, APPROACHING, LOOKAT)
- ✅ PRE_APPROACH: 1s safety gate with NLF, legacy sliding window without
- ✅ APPROACHING: unified 6-pose grid centered on torso, tight offsets with NLF, wide with YOLO
- ✅ LOOKAT: blended NLF(70%)+YOLO(30%) when HIGH coherence, YOLO 100% when LOW
- ✅ CPU saving: NLF streaming paused after prior capture
- ✅ 24 pytest tests, 3 new test files

### File modificati
| File | +/− |
|------|-----|
| `nlf_skeleton.py` | +76 |
| `wbc_coordinator.py` | +217 |
| `wbc_qp_controller.py` | +44 |
| `wbc_params.yaml` | +7 |
| `body_search_params.yaml` | +7 |

---

# ⭐ Exposure Body Scanning (Sitting + Standing) — PLAN ORIGINALE

## Obiettivo

Estendere TERESA per gestire pazienti **non supini**. Oggi il sistema cerca solo `LYING` (pancia in su) per FAST ultrasound.
Con questa estensione, se il paziente è seduto a terra (`SITTING`) o in piedi (`STANDING`), il robot esegue una
**scansione corporea completa** per rilevare ferite, bruciature ed emorragie visibili (copre la **"E"** di ABCDE — Exposure).

La fase SEARCHING (coarse rotation cmd_vel + refinement pitch, Orbbec/RealSense cercano il corpo) **rimane invariata**. La differenza è dopo il lock.

## Architettura generale

```
SEARCHING (Orbbec trova corpo)
    │
    ├── posture == LYING ──► FAST ultrasound (percorso esistente, INVARIATO)
    │    LOCKING → PRE_APPROACH → APPROACHING → SCANNING
    │
    └── posture in [SITTING, STANDING] ──► Exposure body scan (NUOVO)
         LOCKING → PRE_APPROACH → APPROACHING → EXPOSURE_SCANNING
```

PRE_APPROACH e APPROACHING servono anche per l'exposure (Spot deve comunque navigare verso il paziente e pre-orientarsi).
In APPROACHING non si attiva SCAN_SEQ (non servono le 11 pose QP per FAST), si attende solo handoff.

## Lock generalizzato

**Oggi:** il lock richiede `posture == 'LYING' + confidence >= 0.70`.
**Dopo:** il lock accetta qualsiasi postura con soglia differenziata:
- `posture == 'LYING'` → soglia 0.70 (esistente, invariata)
- `posture in ['SITTING', 'STANDING']` → soglia 0.55 (`exposure_lock_confidence`)
- `posture == 'UNKNOWN'` con conf > 0.40 → trattato come SITTING (fail-safe)

## Approccio Spot: frontale per corpi eretti

A differenza del LYING (approccio laterale), per corpi eretti Spot si mette **davanti** al paziente:

```
     Paziente seduto/standing
          ┌───┐
          │ T │  testa
          ├───┤
          │   │  torso
          │   │
          ├───┤
          │   │  gambe
          └───┘
            ↑
            │  body_axis (alto→basso, parallelo a world Z)
            │
     ┌──────┴──────┐
     │    Spot     │
     │  camera →   │  RealSense punta al torso
     └─────────────┘
         
   distanza = standoff_distance (~0.80m)
   Spot altezza = adattata (seduto: -0.05m, standing: 0.0m)
```

L'`approach_point` frontale viene calcolato da un **nuovo nodo** `human_approach_detector.py`,
che generalizza `laying_human_detector.py` per posture non-LYING.

### Geometria approach_point per SITTING/STANDING

```python
# kp da /human_pose/points_3d (Orbbec, in camera_optical_frame)
torso_center = mean([kp5, kp6, kp11, kp12])    # spalle + anche
head_center  = mean([kp0, kp1, kp2, kp3, kp4]) # naso + occhi + orecchie

# Approccio frontale: Spot si posiziona davanti al paziente
# Nel frame camera Orbbec: Z = profondità (avanti verso il corpo)
# Spot deve stare a distanza standoff lungo Z negativa (indietro dalla camera)
approach_pos = torso_center + np.array([0, 0, -standoff_distance])

# Orientamento: Spot guarda il torso (yaw = atan2(dx, dz))
```

**Nuovo topic:** `/human/approach_point` (PoseStamped) pubblicato da `human_approach_detector`.
Il `laying_human_detector` esistente rimane invariato e continua a pubblicare su `/laying_human/approach_point`.
Il coordinator si subscribe ad entrambi.

## Griglia di punti: adattiva dai keypoint YOLO

**Principio:** invece di una griglia fissa 3×2, i punti di scansione sono calcolati **dai 17 keypoint COCO** di YOLO.
Ogni segmento anatomico ha la sua griglia, proporzionale alla taglia del corpo.

### Segmenti anatomici e generazione griglia

```
kp0  (nose)    ──►  HEAD segment     (griglia 2×2 centrata sul naso)
kp1-4 (occhi/orecchie)
    │
kp5  (spalla sx) ──┐
kp6  (spalla dx) ──┤  TORSO segment   (griglia bilineare 3×3 tra spalle e anche)
kp7  (gomito sx) ──┤  LEFT_ARM seg.   (polyline spalla→gomito→polso, N punti × 2 laterali)
kp8  (gomito dx) ──┤  RIGHT_ARM seg.  (polyline spalla→gomito→polso, N punti × 2 laterali)
kp9  (polso sx)  ──┘
kp10 (polso dx)  ──┘
    │
kp11 (anca sx)   ──┐
kp12 (anca dx)   ──┤  LEFT_LEG seg.   (polyline anca→ginocchio→caviglia, N punti × 2. Solo STANDING)
kp13 (ginocchio sx)─┤  RIGHT_LEG seg.  (polyline anca→ginocchio→caviglia, N punti × 2. Solo STANDING)
kp14 (ginocchio dx)─┤
kp15 (caviglia sx) ─┘
kp16 (caviglia dx) ─┘
```

### Metodo di generazione per segmento

**TORSO** (griglia bilineare 3×3 = 9 punti):
- 4 corner = kp5, kp6, kp11, kp12 (3D, camera_optical_frame)
- Interpolazione bilineare: `point(row, col) = lerp(lerp(tl, tr, u), lerp(bl, br, u), v)`
- `u = col/2`, `v = row/2`

**HEAD** (griglia 2×2 = 4 punti):
- Centro = kp0 (naso)
- Raggio = `|kp0 - kp1| * 1.2` (distanza naso→orecchio × margine)
- 4 punti a ±radius su X e Y

**ARMS** (polyline 4 punti × 2 laterali = 8 punti per braccio):
- 3 anchor = spalla → gomito → polso
- Interpolazione lineare: 4 punti lungo la spezzata
- Per ogni punto, offset perpendicolare ±0.05m per coprire larghezza braccio

**LEGS** (solo per STANDING, 5 punti × 2 laterali = 10 punti per gamba):
- 3 anchor = anca → ginocchio → caviglia
- Stessa logica delle braccia, 5 punti lungo (gambe più lunghe)

### Punti totali

| Postura | Segmenti | Punti griglia |
|---------|----------|:------------:|
| **SITTING** | HEAD + TORSO + L_ARM + R_ARM | 4 + 9 + 8 + 8 = **29** |
| **STANDING** | + L_LEG + R_LEG | 29 + 10 + 10 = **49** |

## Per-point Spot + Arm coordination (pattern FAST riutilizzato)

Per **ogni punto della griglia**, Spot e braccio si coordinano con lo stesso pattern dei 5 punti FAST:

```
Per ogni punto griglia (N punti):

  1. Ottimizza body_pose(h, p)
     Grid search su 3×4 (altezza × pitch).
     Score = -‖target_in_link00 - sweet_spot‖
     sweet_spot = [0.35, 0.0, 0.30] (centro workspace Z1)

  2. _set_body_pose(h*, p*) + cmd_vel flush

  3. Attendi settle 1.5s → body_ready

  4. Trasforma grid_point da odom → link00 via TF live

  5. Calcola IK goal:
     posizione = grid_point in link00 + standoff lungo Z camera
     orientamento = look-at (X_ee punta al grid_point)
     clippato al workspace con WorkspaceChecker

  6. Pubblica IK goal via ik_goal_mux → /z1/ik_goal_pose

  7. Attendi /ik_done (timeout 3s)

  8. Raccogli detection da /injury/detections (dwell_time secondi)

  9. Associa detection al segmento + coordinate UV + posizione 3D in odom

  10. NMS 3D intra-segmento (distanza < 0.10m → stessa detection, tieni max conf)

  11. Salva foto JPEG se detection trovata

  12. Avanza al prossimo punto
```

### Meccanismi riusati (zero codice nuovo per Spot movement)

| Meccanismo | File | Riuso |
|-----------|------|:----:|
| Grid search body_pose (h×p) | `wbc_coordinator._optimize_body_poses()` | 100% |
| Applica body_pose + cmd_vel flush | `wbc_coordinator._set_body_pose()` | 100% |
| Settle timer 1.5s | `wbc_coordinator._tick_fast_settle()` | 100% |
| Topic body_ready | `/wbc/body_ready` | 100% |
| IK goal via mux | `ik_goal_mux` | 100% |
| Topic ik_done | `/ik_done` | 100% |
| Transform odom→link00 | `wbc_coordinator._tf_transform()` | 100% |
| Workspace checking | `WorkspaceChecker.clip_target()` | 100% |

## FSM modificato

```
WAITING_TF
    │
    └── tf_ready ──► IDLE
                        │
                        └── keyboard 's' ──► SEARCHING
                                               │
                    ┌──────────────────────────┼──────────────────────┐
                    ▼                          ▼                      ▼
              Orbbec lock               RealSense semi-lock     Sequenza esausta
              (full lock)              (guida Spot → Orbbec)    → IDLE
                    │                          │
                    └──────────┬───────────────┘
                               ▼
                           LOCKING   ←── accetta LYING, SITTING, STANDING
                               │
                    ┌──────────┴──────────┐
                    ▼                     ▼
            posture == LYING      posture in [SITTING, STANDING]
                    │                     │
                    ▼                     ▼
            PRE_APPROACH           PRE_APPROACH
            APPROACHING            APPROACHING
            SCANNING (FAST)        EXPOSURE_SCANNING  ← NUOVO STATO
                    │                     │
                    │                     ├── _tick_exposure()
                    │                     │   │
                    │                     │   ├─ _gen_exposure_grid()
                    │                     │   │   da keypoint 3D YOLO
                    │                     │   │
                    │                     │   ├─ per ogni punto:
                    │                     │   │    body_pose(h,p) → settle
                    │                     │   │    IK goal → ik_done
                    │                     │   │    raccolta /injury/detections
                    │                     │   │    associazione segmento + UV
                    │                     │   │    NMS 3D
                    │                     │   │    salva foto
                    │                     │   │
                    │                     │   └─ _finalize_exposure_report()
                    │                     │       report JSON su /exposure/report
                    │                     │
                    ▼                     ▼
              HOMING → WAITING      IDLE
```

### Nuovo stato: `EXPOSURE_SCANNING`

Metodo `_tick_exposure()` nel coordinator. Logica:

```
Fase 0 — Init: genera griglia punti da keypoint YOLO in odom, idx=0
Fase 1 — Fine: idx >= len(points) → report finale → IDLE
Fase 2 — Body pose: grid search (h,p) per punto corrente, _set_body_pose, avvia settle timer
Fase 3 — Settle: attesa 1.5s
Fase 4 — IK goal: calcola look-at + standoff, pubblica via ik_goal_mux
Fase 5 — IK wait: attesa /ik_done (timeout 3s)
Fase 6 — Collect: raccogli da /injury/detections per dwell_time secondi
      — Associa a segmento + coordinate UV
      — NMS 3D intra-segmento
      — Salva foto JPEG se detection presente
      — Avanza idx
```

### Modifiche a `_tick_locking` (branching dopo lock)

```python
# Oggi: 5 campioni → sempre PRE_APPROACH
# Domani: 5 campioni → PRE_APPROACH, ma ricorda la postura
if len(self._search_lock_buffer) >= self._search_lock_samples:
    target = np.mean(self._search_lock_buffer, axis=0)
    self._quality.set_target(target, lock_confidence)
    self._exposure_mode = (self._posture in ['SITTING', 'STANDING'])
    self._set_state(CoordState.PRE_APPROACH)
```

### Modifiche a `_tick_approaching` (handoff)

```python
# Dopo handoff (dist < handoff_distance):
if self._exposure_mode:
    self._set_state(CoordState.EXPOSURE_SCANNING)
elif self._fast_points is not None:
    self._set_state(CoordState.SCANNING)
```

### Modifiche a `_set_state`

```python
if new_state == CoordState.EXPOSURE_SCANNING:
    self._init_exposure()  # resetta buffer, genera griglia
```

## Nuovi componenti

### injury_detector.py — nodo detection ferite/ustioni

```
Package: spot_perception
Nodo:    injury_detector

Subscribers:
  /camera/camera/color/image_raw              (RealSense RGB)
  /camera/camera/aligned_depth_to_color/image_raw  (depth allineato)
  /camera/camera/color/camera_info

Publishers:
  /injury/detections       (InjuryDetectionArray)
  /injury/detection_image  (CompressedImage, solo se ci sono detection)

Parametri:
  wound_model_path:  'best_yolov8n_roboV3.pt'     # YOLOv8 nano ferite (6MB)
  burn_model_path:   'skin_burn_2022_8_21.pt'     # YOLOv7 bruciature (~40MB)
  conf_threshold:    0.30
  depth_valid_range: [0.2, 3.0]                   # range profondità valido [m]
  frame_stride:      1                             # processa 1 frame ogni N

Logica inferenza:
  1. Frame RGB → resize 640×640
  2. Frame pari: YOLOv8 ferite. Frame dispari: YOLOv7 bruciature. (alternanza 5 Hz eff)
  3. Per ogni detection con conf > threshold:
     a. Centro bbox → profondità (mediana 5×5 da depth frame)
     b. De-proiezione 2D→3D via camera_info → camera_optical_frame
     c. Trasforma in my_spot/odom via TF lookup
     d. Accoda a InjuryDetectionArray
  4. Se detection presenti → salva frame JPEG compresso → pub /injury/detection_image
  5. Pubblica /injury/detections
```

### human_approach_detector.py — approccio frontale

```
Package: spot_perception
Nodo:    human_approach_detector

Subscribers:
  /human_pose/points_3d          (da yolo_skeleton_spot)
  /human_pose/posture            (String)
  /human_pose/posture_confidence (Float32)

Publishers:
  /human/approach_point   (PoseStamped, per SITTING/STANDING)
  /human/body_axis        (Vector3Stamped)

Gating:
  posture in ['SITTING', 'STANDING'] AND conf >= 0.5 AND valid_keypoints >= 4

Geometria (frontale, non laterale):
  - torso_center = mean([kp5, kp6, kp11, kp12])
  - approach_pos = torso_center + [0, 0, -standoff_distance]
    (davanti al corpo, lungo Z negativa camera Orbbec)
  - Orientamento: Spot guarda torso_center (yaw = atan2(dx, dz))
```

**Nota:** `laying_human_detector.py` esistente **rimane invariato**. Continua a pubblicare
su `/laying_human/approach_point` per LYING. Il coordinator si subscribe a entrambi i topic.

## Messaggi custom

### InjuryDetection.msg (nuovo, spot_msgs/)

```
std_msgs/Header header
string injury_class              # 'wound' | 'burn_1st' | 'burn_2nd' | 'burn_3rd'
float32 confidence
float32[4] bbox_pixels           # [x1, y1, x2, y2]
geometry_msgs/Pose pose          # posizione 3D in my_spot/odom
string body_segment              # 'HEAD' | 'TORSO' | 'LEFT_ARM' | 'RIGHT_ARM' | 'LEFT_LEG' | 'RIGHT_LEG'
float32 ratio_u                  # 0.0 → 1.0 orizzontale nel segmento
float32 ratio_v                  # 0.0 → 1.0 verticale nel segmento
int32 source_point_idx           # indice del punto griglia che l'ha rilevata
```

### InjuryDetectionArray.msg (nuovo, spot_msgs/)

```
std_msgs/Header header
InjuryDetection[] detections
```

## Topic

### Nuovi topic

| Topic | Tipo | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/human/approach_point` | `PoseStamped` | `human_approach_detector` | `wbc_coordinator` |
| `/injury/detections` | `InjuryDetectionArray` | `injury_detector` | `wbc_coordinator` |
| `/injury/detection_image` | `CompressedImage` | `injury_detector` | RViz / log |
| `/exposure/report` | `String` (JSON) | `wbc_coordinator` | log / external |
| `/exposure/scan_state` | `String` | `wbc_coordinator` | monitor |

### Topic esistenti riusati

| Topic | Scopo esposizione |
|-------|-------------------|
| `/wbc/state` | Pubblica `'EXPOSURE_SCANNING'` |
| `/wbc/body_ready` | Settle completato per punto corrente |
| `/wbc/ik_goal_pose` | IK goal via ik_goal_mux |
| `/wbc/ik_enable` | Abilita IK solver |
| `/ik_done` | Conferma completamento traiettoria |
| `/my_spot/body_pose` | Comandi altezza/pitch/yaw Spot |
| `/human_pose/points_3d` | Keypoint YOLO per generare griglia |
| `/human_pose/posture` | String (SITTING/STANDING) |
| `/human_pose/posture_confidence` | Confidenza postura |

## Report finale

Al termine della scansione, il coordinator pubblica un JSON su `/exposure/report`:

```json
{
  "timestamp": "2026-05-29T14:30:00.000Z",
  "patient_posture": "SITTING",
  "segments_scanned": ["HEAD", "TORSO", "LEFT_ARM", "RIGHT_ARM"],
  "total_grid_points": 29,
  "points_visited": 29,
  "detections": [
    {
      "id": 0,
      "injury_class": "burn_2nd",
      "confidence": 0.67,
      "body_segment": "TORSO",
      "ratio_u": 0.35,
      "ratio_v": 0.60,
      "position_odom": {"x": 1.23, "y": 0.45, "z": 0.12},
      "photo_path": "/tmp/teresa_exposure/burn_2nd_0_20260529_143005.jpg"
    }
  ],
  "summary": {
    "total_wounds": 2,
    "total_burns_1st": 0,
    "total_burns_2nd": 1,
    "total_burns_3rd": 0,
    "total_unknown": 0
  },
  "scan_duration_s": 145.2
}
```

## Salvataggio foto

Ogni detection attiva un salvataggio JPEG:
- Directory: `/tmp/teresa_exposure/` (configurabile via `exposure_photo_dir`)
- Nome file: `{class}_{id}_{timestamp}.jpg`
- L'immagine è il frame RGB compresso con bounding box disegnata

## Parametri YAML (nuovo blocco in wbc_params.yaml)

```yaml
# ── wbc_coordinator: EXPOSURE body scanning ───────────────────────────────

# Lock per posture non-LYING
exposure_lock_confidence: 0.55       # [0-1] soglia confidenza per lock SITTING/STANDING

# Body pose Spot per inquadrare il corpo
exposure_body_height_sitting: -0.05  # [m] Spot leggermente abbassato per paziente seduto
exposure_body_pitch_sitting: 0.10    # [rad] ≈6° inclinazione verso busto
exposure_body_height_standing: 0.0   # [m] altezza nominale per paziente in piedi
exposure_body_pitch_standing: 0.15   # [rad] ≈8.6° inclinazione verso l'alto

# Griglia punti — generazione adattiva dai keypoint YOLO
exposure_grid_torso_rows: 3          # righe griglia TORSO
exposure_grid_torso_cols: 3          # colonne griglia TORSO
exposure_grid_head_size: 2           # griglia HEAD: N×N centrata sul naso
exposure_grid_limb_along: 4          # punti lungo braccio (spalla→polso)
exposure_grid_limb_across: 2         # punti trasversali braccio (± offset)
exposure_grid_leg_along: 5           # punti lungo gamba (anca→caviglia), solo STANDING
exposure_grid_leg_across: 2          # punti trasversali gamba
exposure_limb_offset: 0.05           # [m] offset laterale per braccia/gambe
exposure_head_radius_scale: 1.2      # fattore scala raggio testa (× distanza naso→orecchio)

# Timing
exposure_dwell_per_point: 3.0        # [s] tempo raccolta detection per punto
exposure_ik_timeout: 3.0             # [s] timeout attesa ik_done

# Camera
exposure_standoff_distance: 0.80     # [m] distanza camera→superficie corporea

# Detection
exposure_detection_conf: 0.30        # [0-1] soglia minima YOLO
exposure_3d_nms_dist: 0.10           # [m] distanza NMS 3D tra detection duplicate
exposure_min_detections: 3           # frame minimi per early-stop (non implementato in v1)

# Segmenti da scansionare (ordinati per priorità)
exposure_segments: ['TORSO', 'HEAD', 'LEFT_ARM', 'RIGHT_ARM', 'LEFT_LEG', 'RIGHT_LEG']

# Modelli YOLO
wound_model_path: 'best_yolov8n_roboV3.pt'      # path al modello ferite
burn_model_path:  'skin_burn_2022_8_21.pt'      # path al modello bruciature

# Salvataggio
exposure_save_photos: True
exposure_photo_dir: '/tmp/teresa_exposure'
```

## File inventory

### File nuovi

| # | File | Package | Ruolo |
|---|------|---------|-------|
| **N1** | `injury_detector.py` | `spot_perception` | Nodo ROS: carica 2 modelli YOLO (ferite+bruciature), subscribe RealSense RGB+Depth, pubblica detection 3D in odom con riferimento anatomico |
| **N2** | `human_approach_detector.py` | `spot_perception` | Nodo ROS: calcola approach_point frontale per SITTING/STANDING (generalizza laying_human_detector) |
| **N3** | `InjuryDetection.msg` | `spot_msgs` | Messaggio custom: classe, conf, bbox, posizione 3D, segmento anatomico, coordinate UV |
| **N4** | `InjuryDetectionArray.msg` | `spot_msgs` | Array di InjuryDetection |
| **N5** | `download_injury_models.sh` | `scripts/` | Scarica best_yolov8n_roboV3.pt da HuggingFace e skin_burn_2022_8_21.pt da GitHub Releases |

### File modificati

| # | File | Modifica |
|---|------|----------|
| **M1** | `wbc_coordinator.py` | Nuovo stato `EXPOSURE_SCANNING` + `_tick_exposure()`. Lock generalizzato (accetta SITTING/STANDING). Branching dopo LOCKING. Metodi: `_gen_exposure_grid()`, `_compute_exposure_ik_goal()`, `_collect_exposure_detections()`, `_associate_detection()`, `_nms_3d()`, `_finalize_exposure_report()`, `_save_detection_photo()`. Nuovi subscriber: `/human/approach_point`, `/injury/detections`. Nuovi publisher: `/exposure/report`, `/exposure/scan_state` |
| **M2** | `wbc_params.yaml` | Nuovo blocco `exposure_*` (vedi sopra) |
| **M3** | `wbc.launch.py` | Aggiunge nodi `injury_detector` e `human_approach_detector` (condizionati da param `enable_exposure:=true`) |
| **M4** | `setup.py` (spot_control) | Aggiunge entry point: `human_approach_detector`, `injury_detector` (se il nodo sta in spot_control) |
| **M5** | `setup.py` (spot_perception) | Entry point `injury_detector`, `human_approach_detector` |
| **M6** | `CMakeLists.txt` (spot_msgs) | Aggiunge `InjuryDetection.msg`, `InjuryDetectionArray.msg` |
| **M7** | `DESCRIPTION.md` | Nuova fase EXPOSURE_SCANNING, topic, flow |
| **M8** | `CHANGELOG.md` | Entry esposizione body scanning |
| **M9** | `INIT.md` | Aggiornamento current state |

### File invariati

| File | Motivo |
|------|--------|
| `z1_FSM.py` | L'EXPOSURE_SCANNING non usa la Z1 FSM (salta BODY_SCANNING, CHECKING_WORKSPACE, FAST cycle). Il controllo braccio è diretto via ik_goal_mux dal coordinator |
| `wbc_qp_controller.py` | Non si attiva SCAN_SEQ per EXPOSURE, resta in LOOKAT mode o idle |
| `ik_goal_mux.py` | Invariato: riceve goal dal coordinator come sempre |
| `laying_human_detector.py` | Invariato: continua a pubblicare approccio laterale per LYING |
| `posture_classifier.py` | Invariato: già classifica SITTING e STANDING |
| `wbc_spot_navigator.py` | Invariato: usato in APPROACHING come sempre |
| `yolo_skeleton_spot.py` | Invariato: già fornisce keypoint 3D per tutte le posture |
| `wbc_math.py` | Invariato |

## Tempi stimati

| Postura | Punti griglia | Tempo/punto (settle+dwell) | Totale |
|---------|:------------:|:---------------------------:|:------:|
| **SITTING** | ~29 | 1.5s + 3s = 4.5s | **~130s (2 min)** |
| **STANDING** | ~49 | 1.5s + 3s = 4.5s | **~220s (3.7 min)** |

Con early-stop su regioni senza detection il tempo si riduce ulteriormente (non implementato in v1).

## Casi edge e fallback

| Scenario | Comportamento |
|----------|---------------|
| **Posture UNKNOWN con conf > 0.4** | Lock accettato come SITTING → EXPOSURE (fail-safe: meglio scansione inutile che non fare nulla) |
| **Nessun approach_point calcolabile** | Lock ignorato, SEARCHING continua a ruotare |
| **Keypoint insufficienti per griglia** | Salta il segmento (es. braccia non visibili → solo TORSO+HEAD) |
| **Modello YOLO non trovato** | `injury_detector` logga errore, EXPOSURE_SCANNING raccoglie solo foto senza detection |
| **Nessuna detection dopo sweep completo** | Report con summary tutto a zero, Spot torna a IDLE |
| **Spot perde TF durante sweep** | Emergency stop, torna in WAITING_TF |
| **Spazio insufficiente per avvicinarsi** | APPROACHING timeout → IDLE con warning |
| **Dry-run mode** | `injury_detector` pubblica su topic debug, nessun movimento braccio/Spot |
| **Paziente si muove durante sweep** | Detection 3D sono in odom (world-fixed). Se il corpo si sposta, le detection diventano stale — le coordinate parametriche (segmento+UV) sopravvivono meglio delle coordinate odom raw |

## Ordine di implementazione

| Step | Cosa | Dipende da | Complessità |
|:----:|------|:----------:|:-----------:|
| 1 | `InjuryDetection.msg` + `InjuryDetectionArray.msg` in spot_msgs | — | Bassa |
| 2 | `human_approach_detector.py` (nodo approccio frontale) | — | Media |
| 3 | `injury_detector.py` (nodo YOLO ferite+bruciature) | Step 1 | Alta |
| 4 | `wbc_params.yaml` — nuovo blocco `exposure_*` | — | Bassa |
| 5 | `wbc_coordinator.py` — lock generalizzato, EXPOSURE_SCANNING, griglia keypoint, per-point, report | Step 2,4 | Alta |
| 6 | `wbc.launch.py` + `setup.py` (×2) — nuovi nodi nel launch | Step 2,3 | Bassa |
| 7 | `download_injury_models.sh` | — | Bassa |
| 8 | Docs (`DESCRIPTION.md`, `CHANGELOG.md`, `INIT.md`) | Step 5 | Bassa |
| 9 | Test integrato dry-run | Step 6 | Media |

---

# 🔧 FSM Analysis & Fixes (in corso — 3 Giugno 2026)

Analisi fase-per-fase del FSM con fix mirati. Ogni fase viene analizzata, i problemi identificati, e i fix implementati.

| Fase | Stato | Problemi risolti |
|------|:-----:|------------------|
| **SEARCHING** | ✅ | Pitch-based (+10°/+5°/0°), no yaw, 7 arm poses (3 forward 10° tilt + 3 behind + return), step 50cm (14 Giugno) |
| **SEMI_LOCKING** | ✅ | RealSense gate (dwell only if person visible), yaw restore on fail, Orbbec dwell 5s, cooldown 3 ticks, `_end_search(re_enable=True)` |
| **LOCKING** | ✅ | `ik_done` gate, home_lock_z=0.60 → sostituita da prima posa search (8 Giugno). NLF burst multi-frame + blocco PRE_APPROACH (9 Giugno) |
| **PRE_APPROACH** | ✅ | LOOKAT → `/laying_human/body_center`, soglia ESTIMATING/LOCKED ×3 |
| **APPROACHING** | ✅ | Griglia adattiva 2/4 pose + advance X=0.10, `_do_set_state` pulizia, timeout 60s |
| **SCANNING** | ✅ | Global timeout 120s, parametrizzazione `max_workspace_reach`/`ws_ext_goal_tolerance`, body_ready safe skip |

### 📄 Paper aggiornato (3 Giugno 2026)

Sezioni riscritte per riflettere l'implementazione corrente: abstract (no numeri, hybrid lock), introduction (4 principi aggiornati), active\_perception (coarse+refinement, dual-sensor lock, adaptive Cartesian grid), system\_architecture (9 stati FSM, frame tree colorato, giustificazione hardware 2 RGBD vs 360°/Spot built-in). Nuove figure: `fsm.tex` (9 stati), `frame_tree.tex` (albero TF con colori hardware). Sezioni TODO: experiments, results, conclusion.

### Nuovi topic

| Topic | Publisher | Subscriber | Fase |
|-------|-----------|------------|------|
| `/laying_human/body_center` | `laying_human_detector` | `wbc_coordinator` | PRE_APPROACH |
| `/torso_keypoint_conf` | `yolo_torso_tracker` | `wbc_qp_controller` | APPROACHING |

### Nuovi parametri YAML (wbc_params.yaml)

| Parametro | Default | Fase |
|-----------|---------|------|
| `body_center_topic` | `/laying_human/body_center` | PRE_APPROACH |
| `ik_done_topic` | `/ik_done` | PRE_APPROACH |
| `home_lock_z` | 0.60 → sostituito da prima posa search (8 Giugno) | LOCKING |
| `cartesian_x_advance` | 0.10 | APPROACHING |
| `pre_scan_conf_thr` | 0.6 | APPROACHING |
| `approach_timeout` | 60.0 | APPROACHING |

### Parametri modificati (z1_yolo_torso_params.yaml)

| Parametro | Prima | Dopo |
|-----------|-------|------|
| `guidance_min_conf` | 0.3 | 0.5 |

---

# Launch Refactoring (Core + App)

## Obiettivo

Ridurre i 5 terminali attuali a 3, separando i driver hardware (Orbbec, RealSense, Z1) dalla logica applicativa.

```
T1: ros2 launch spot_control teresa_core.launch.py
T2: ros2 launch spot_control teresa_app.launch.py
T3: ros2 run spot_control wbc_keyboard_node
```

## File da creare

| File | Ruolo |
|------|-------|
| `src/spot_control/launch/teresa_core.launch.py` | Core launch: driver + TF statiche + core_ready |
| `src/spot_control/launch/teresa_app.launch.py` | App launch: include i 4 sub-launch applicativi |
| `src/spot_control/spot_control/core_ready.py` | Nodo monitor: controlla driver attivi → pubblica `/core/ready` |

## File da modificare

| File | Modifica |
|------|----------|
| `src/spot_perception/launch/spot_perception.launch.py` | Arg `use_orbbec_driver`; quando false, salta driver e TF, riduce TimerAction |
| `src/spot_control/setup.py` | Entry point `core_ready` |
| `INIT.md` | Aggiornare sezione running con i 3 terminali |

---

# Paper

## Sezioni mancanti

| Sezione | Stato | Cosa fare |
|---------|:-----:|-----------|
| `experiments.tex` | 📝 | Setup sperimentale, griglia posizioni paziente, baseline, metriche |
| `results.tex` | 📝 | Idle time reduction, positioning error, confidence vs velocity, tabella comparativa |
| `conclusion.tex` | 📝 | Summary contributi, limitazioni, future work |
| `figures/fsm.tex` | 🔧 | Aggiornare con 9 stati (8 attuali + nuovo EXPOSURE_SCANNING) |
| `figures/system_block.tex` | 🔧 | Aggiornare con injury_detector, human_approach_detector |
| Compilazione LaTeX | 🔧 | Verificare undefined references |
