# Integrazione NLF in TERESA — 24 Giunti SMPL — Piano di Lavoro

## TL;DR

> **Quick Summary**: Sostituire YOLO11n-pose con NLF (Neural Localizer Fields) in entrambi i nodi di percezione. Pubblicare **24 giunti SMPL** nativi (non più 17 COCO). YOLO in fallback pubblica 17 COCO → 24 SMPL con NaN padding. Default: `nlf`. Selezionabile via `perception_backend`.
>
> **Deliverables**:
> - Modulo costanti SMPL-24 (`sml_pose_indices.py`)
> - Modulo YOLO→24 adapter (NaN padding)
> - Nodo `nlf_skeleton.py` (Orbbec, 24 giunti)
> - Nodo `nlf_torso_tracker.py` (RealSense, 24 giunti)
> - 6 consumer aggiornati ai nuovi indici SMPL
> - 2 nodi YOLO esistenti aggiornati a 24 giunti
> - Parametro `perception_backend` (default: `nlf`) in launch file
> - Mesh SMPL su `/human_pose/smpl_mesh`
>
> **Estimated Effort**: Large (~18 task, 6 consumer da aggiornare)
> **Parallel Execution**: YES — 5 waves
> **Critical Path**: Task 1 → Task 4+5 → Task 8+9+10+11+12+13 → Task 14 → Task 17

---

## Context

### Original Request
L'utente vuole passare da YOLO11n-pose a NLF con **24 giunti SMPL nativi** (non 17 COCO). NLF è già stato testato offline con risultati superiori. YOLO resta come fallback selezionabile. Default: NLF.

### Interview Summary
**Key Decisions**:
- **24 giunti SMPL nativi** (non 17 COCO) — tutti i consumer vanno aggiornati ai nuovi indici
- **YOLO fallback**: pubblica 24 giunti con 17 COCO mappati + 7 NaN
- **Default**: `nlf`
- **SMPL Mesh**: pubblicata su topic separato
- **Backward compatibility**: topic names invariati, formato PoseArray invariato, SOLO il count passa da 17 a 24

### Nuovi indici SMPL-24 (chiave per tutto il piano)

```
# SMPL-24 Joint Index Reference
PELVIS         = 0
HIP_LEFT       = 1    # era COCO 11
HIP_RIGHT      = 2    # era COCO 12
SPINE1         = 3    # NUOVO — vertebra lombare
KNEE_LEFT      = 4    # era COCO 13
KNEE_RIGHT     = 5    # era COCO 14
SPINE2         = 6    # NUOVO — vertebra toracica
ANKLE_LEFT     = 7    # era COCO 15
ANKLE_RIGHT    = 8    # era COCO 16
SPINE3         = 9    # NUOVO — vertebra cervicale
FOOT_LEFT      = 10   # NUOVO
FOOT_RIGHT     = 11   # NUOVO
NECK           = 12   # era COCO 0 (approssimato da nose)
COLLAR_LEFT    = 13   # era COCO 5 (approssimato)
COLLAR_RIGHT   = 14   # era COCO 6 (approssimato)
HEAD           = 15   # NUOVO
SHOULDER_LEFT  = 16   # era COCO 5
SHOULDER_RIGHT = 17   # era COCO 6
ELBOW_LEFT     = 18   # era COCO 7
ELBOW_RIGHT    = 19   # era COCO 8
WRIST_LEFT     = 20   # era COCO 9
WRIST_RIGHT    = 21   # era COCO 10
HAND_LEFT      = 22   # NUOVO
HAND_RIGHT     = 23   # NUOVO
```

### Metis Review — Gap critici aggiornati
- **Gap**: Mapping SMPL→COCO buttava via spine, neck, feet → **RISOLTO**: 24 giunti nativi
- **Gap**: Consumer usano indici hardcodati → **AGGIORNATI**: tutti i consumer usano le nuove costanti SMPL
- **Gap**: YOLO pad 17→24 con NaN mapping → **NUOVO TASK**: modulo `yolo_to_smpl_pad.py`
- **Gap**: Torso scan point (22 float) usa COCO kp5,6,11,12 → **AGGIORNATO**: ora usa SMPL 16,17,1,2

---

## Work Objectives

### Core Objective
Migrare l'intero sistema da 17 giunti COCO a 24 giunti SMPL. NLF pubblica 24 nativi. YOLO pubblica 24 con NaN padding. Tutti i consumer usano i nuovi indici. Default: NLF.

### Concrete Deliverables
- `src/spot_perception/spot_perception/sml_pose_indices.py` — costanti indici SMPL-24
- `src/spot_perception/spot_perception/yolo_to_smpl_pad.py` — adapter YOLO 17→24
- `src/spot_perception/spot_perception/nlf_skeleton.py` — nodo NLF Orbbec (24 giunti)
- `src/z1_vision/z1_vision/nlf_torso_tracker.py` — nodo NLF RealSense (24 giunti)
- 6 file consumer aggiornati (vedi lista sotto)
- 2 file YOLO aggiornati (yolo_skeleton_spot.py, z1_yolo_torso_tracker.py)
- `config/nlf_params.yaml` + `config/nlf_torso_params.yaml`
- `scripts/download_nlf_models.sh`
- 3 launch file aggiornati (default: `nlf`)
- `CHANGELOG.md` + `DESCRIPTION.md` aggiornati

### Definition of Done
- [ ] `perception_backend:=nlf` lancia NLF, pubblica 24 giunti su `/human_pose/points_3d`
- [ ] `perception_backend:=yolo` lancia YOLO, pubblica 24 giunti (17 validi + 7 NaN)
- [ ] `posture_classifier` funziona con 24 giunti e produce LYING/STANDING/SITTING
- [ ] `laying_human_detector` pubblica `approach_point` corretto
- [ ] `exposure_scanner` genera griglia corpo da 24 giunti (con spine, neck, head aggiuntivi)
- [ ] `z1_scan_manager` calcola FAST points da SMPL indices
- [ ] `/torso_scan_point` ha ancora 22 float ma con SMPL keypoint indices
- [ ] `/human_pose/smpl_mesh` pubblica mesh SMPL

### Must Have
- Costanti SMPL-24 condivise in `sml_pose_indices.py` (unica fonte di verità)
- NLF default (`perception_backend` default = `"nlf"`)
- 24 Pose su `/human_pose/points_3d` e `/exposure/body_keypoints`
- YOLO mode produce 24 Pose (7 NaN per spine+neck+head+feet+hands)
- Topic names invariati
- NaN contract: giunti non disponibili = `float('nan')` in `position.x`

### Must NOT Have (Guardrails)
- **NO** magic numbers negli indici — sempre `SMPL.SHOULDER_LEFT`, mai `16`
- **NO** `if len(poses) == 17` rimasto in alcun consumer
- **NO** coordinate frame cambiate
- **NO** QoS modificati
- **NO** topic names nuovi (eccetto `/human_pose/smpl_mesh` che è nuovo)
- **NO** Kalman filter infrastructure modificata (solo `num_joints` → 24)
- **NO** plugin system generico — branch `if` nel launch file
- **NO** refactoring non necessario

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (pytest)
- **Automated tests**: Tests-after
- **Framework**: pytest
- **Approccio**: Unit test per `sml_pose_indices.py` + `yolo_to_smpl_pad.py`; QA agent-executed per integrazione

### QA Policy
Ogni task include QA scenarios. Evidence in `.omo/evidence/task-{N}-*.txt`.

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Foundation — MAX PARALLEL):
├── Task 1: Costanti SMPL-24 (sml_pose_indices.py) [quick]
├── Task 2: YOLO→24 adapter (yolo_to_smpl_pad.py) [quick]
└── Task 3: NLF config + model download [quick]

Wave 2 (Perception nodes — MAX PARALLEL):
├── Task 4: nlf_skeleton.py (NLF Orbbec, 24 giunti) [deep]
├── Task 5: nlf_torso_tracker.py (NLF RealSense, 24 giunti) [deep]
├── Task 6: Update yolo_skeleton_spot.py → 24 giunti [quick]
└── Task 7: Update z1_yolo_torso_tracker.py → 24 giunti [quick]

Wave 3 (Downstream consumers — MAX PARALLEL, 6 task):
├── Task 8:  Update person_tracking.py (24 Kalman) [quick]
├── Task 9:  Update posture_classifier.py (indici SMPL) [quick]
├── Task 10: Update laying_human_detector.py (indici SMPL) [quick]
├── Task 11: Update z1_scan_manager.py (FAST points SMPL) [quick]
├── Task 12: Update exposure_scanner.py (body grid 24) [deep]
└── Task 13: Update body_search_scanner.py (score SMPL) [quick]

Wave 4 (Integration — MAX PARALLEL):
├── Task 14: Launch files + setup.py (default nlf) [quick]
├── Task 15: Mesh SMPL publisher [quick]
└── Task 16: YAML config finalization [quick]

Wave 5 (Docs + tests):
├── Task 17: CHANGELOG.md + DESCRIPTION.md [writing]
└── Task 18: Unit test (sml_pose_indices + yolo_to_smpl_pad) [quick]

Critical Path: T1 → T4+T5 → T8..T13 → T14 → T17
Parallel Speedup: ~65% (Wave 2×4, Wave 3×6, Wave 4×3 in parallelo)
Max Concurrent: 6 (Wave 3)
```

### Agent Dispatch Summary
- **Wave 1**: 3 × `quick`
- **Wave 2**: 2 × `deep` + 2 × `quick`
- **Wave 3**: 5 × `quick` + 1 × `deep`
- **Wave 4**: 3 × `quick`
- **Wave 5**: 1 × `writing` + 1 × `quick`
- **FINAL**: F1→`oracle`, F2→`unspecified-high`, F3→`unspecified-high`, F4→`deep`

- [x] 1. Costanti SMPL-24 (`sml_pose_indices.py`)

  **What to do**:
  - Creare `src/spot_perception/spot_perception/sml_pose_indices.py`
  - Definire costanti per TUTTI i 24 giunti SMPL:
    ```python
    PELVIS = 0; HIP_LEFT = 1; HIP_RIGHT = 2; SPINE1 = 3
    KNEE_LEFT = 4; KNEE_RIGHT = 5; SPINE2 = 6
    ANKLE_LEFT = 7; ANKLE_RIGHT = 8; SPINE3 = 9
    FOOT_LEFT = 10; FOOT_RIGHT = 11
    NECK = 12; COLLAR_LEFT = 13; COLLAR_RIGHT = 14; HEAD = 15
    SHOULDER_LEFT = 16; SHOULDER_RIGHT = 17
    ELBOW_LEFT = 18; ELBOW_RIGHT = 19
    WRIST_LEFT = 20; WRIST_RIGHT = 21
    HAND_LEFT = 22; HAND_RIGHT = 23
    NUM_JOINTS = 24
    ```
  - Liste helper per gruppi anatomici:
    ```python
    TORSO_JOINTS = [SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT,
                    SPINE1, SPINE2, SPINE3, PELVIS, NECK]
    ARM_JOINTS = [SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT,
                  SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT]
    LEG_JOINTS = [HIP_LEFT, KNEE_LEFT, ANKLE_LEFT,
                  HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT]
    SPINE_JOINTS = [SPINE1, SPINE2, SPINE3]     # NUOVI
    HEAD_JOINTS = [NECK, HEAD]                    # NUOVI
    FEET_JOINTS = [FOOT_LEFT, FOOT_RIGHT]         # NUOVI
    NEVER_AVAILABLE_YOLO = [SPINE1, SPINE2, SPINE3, FOOT_LEFT, FOOT_RIGHT,
                             NECK, HEAD, HAND_LEFT, HAND_RIGHT]  # 9 giunti sempre NaN in YOLO
    ```
  - Funzione `is_valid(pose) → bool`: controlla `not math.isnan(pose.position.x)`

  **Must NOT do**:
  - Non usare stringhe o enum — solo int costanti
  - Non creare classi — file piatto con costanti

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 1, parallelo con T2, T3. Blocks: T4-T13. Blocked by: none.

  **References**: Definizione giunti SMPL: https://meshcapade.wiki/SMPL

  **Acceptance Criteria**:
  - [ ] `sml_pose_indices.py` esiste con tutte le 24 costanti
  - [ ] `NUM_JOINTS == 24`
  - [ ] `len(TORSO_JOINTS) == 9`, `len(ARM_JOINTS) == 6`, `len(LEG_JOINTS) == 6`
  - [ ] `len(NEVER_AVAILABLE_YOLO) == 9`

  **QA**: ```python3 -c "from spot_perception.sml_pose_indices import *; assert NUM_JOINTS == 24; print('PASS')"```

  **Commit**: `feat(nlf): add smpl-24 shared constants module`

- [x] 2. YOLO→24 Pad Adapter (`yolo_to_smpl_pad.py`)

  **What to do**:
  - Creare `src/spot_perception/spot_perception/yolo_to_smpl_pad.py`
  - Mappatura COCO-17 → SMPL-24 (per YOLO fallback):
    ```python
    COCO_TO_SMPL = {
        0:  NECK,           # nose → neck (approssimazione)
        1:  None,           # eye_left → nessun SMPL equivalente
        2:  None,           # eye_right → nessun SMPL equivalente
        3:  None,           # ear_left → nessun SMPL equivalente
        4:  None,           # ear_right → nessun SMPL equivalente
        5:  SHOULDER_LEFT,  # shoulder_left
        6:  SHOULDER_RIGHT, # shoulder_right
        7:  ELBOW_LEFT,     # elbow_left
        8:  ELBOW_RIGHT,    # elbow_right
        9:  WRIST_LEFT,     # wrist_left
        10: WRIST_RIGHT,    # wrist_right
        11: HIP_LEFT,       # hip_left
        12: HIP_RIGHT,      # hip_right
        13: KNEE_LEFT,      # knee_left
        14: KNEE_RIGHT,     # knee_right
        15: ANKLE_LEFT,     # ankle_left
        16: ANKLE_RIGHT,    # ankle_right
    }
    ```
  - Funzione `coco_to_smpl_24(coco_poses_17: list[Pose]) → list[Pose]`:
    - Crea array di 24 Pose, tutti inizializzati a `float('nan')`
    - Per ogni COCO index (0-16): se mapping esiste, copia `coco_poses[i]` → `smpl_poses[mapped_index]`
    - Restituisce 24 Pose

  **Must NOT do**: Non modificare il formato Pose — solo riarrangiamento indici + NaN padding

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 1. Blocks: T6, T7. Blocked by: T1.

  **References**: `sml_pose_indices.py` (T1)

  **Acceptance Criteria**:
  - [ ] `coco_to_smpl_24(17_poses_valide)` restituisce 24 Pose
  - [ ] SHOULDER_LEFT (COCO 5) → SMPL 16
  - [ ] HIP_LEFT (COCO 11) → SMPL 1
  - [ ] Spine (3,6,9), feet (10,11), head (15), hands (22,23) sono NaN
  - [ ] Eye/ear COCO (1-4) non mappati da nessuna parte (scartati)

  **QA**: ```python3 -c "from spot_perception.yolo_to_smpl_pad import coco_to_smpl_24; import numpy as np; poses_17 = [type('Pose',(),{'position':type('P',(),{'x':1.0,'y':2.0,'z':3.0})()}) for _ in range(17)]; smpl = coco_to_smpl_24(poses_17); assert len(smpl)==24; assert smpl[16].position.x==1.0; import math; assert math.isnan(smpl[3].position.x); print('PASS')"```

  **Commit**: `feat(nlf): add yolo-17-to-smpl-24 padding adapter`

- [x] 3. NLF Config + Model Download

  **What to do**:
  - Creare `src/spot_perception/config/nlf_params.yaml` con parametri NLF Orbbec
  - Creare `src/z1_vision/config/nlf_torso_params.yaml` con parametri NLF RealSense
  - Creare `scripts/download_nlf_models.sh`
  - Aggiungere `*.torchscript` a `.gitignore`

  **Must NOT do**: Non hardcodare path assoluti

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 1. Blocks: T4, T5. Blocked by: none.

  **Acceptance Criteria**:
  - [ ] `nlf_params.yaml` esiste con parametri `model_path`, `device`, `conf_threshold`, `publish_mesh`
  - [ ] `nlf_torso_params.yaml` esiste con parametri torso-specifici
  - [ ] `download_nlf_models.sh` esiste ed è eseguibile (`chmod +x`)
  - [ ] `.gitignore` include `*.torchscript`
  - [ ] YAML parseable: `python3 -c "import yaml; yaml.safe_load(open('config/nlf_params.yaml'))"` non dà errore

  **QA**: `bash -n scripts/download_nlf_models.sh && python3 -c "import yaml; yaml.safe_load(open('src/spot_perception/config/nlf_params.yaml')); print('OK')"`

  **Commit**: `feat(nlf): add nlf config yamls + model download script`

---

- [x] 4. Nodo NLF Orbbec — `nlf_skeleton.py` (24 giunti nativi)

  **What to do**:
  - Creare `src/spot_perception/spot_perception/nlf_skeleton.py` (~600 linee)
  - **Inference**: NLF su RGB Orbbec. Se NLF non include detector → YOLO detection-only per bounding box, poi NLF su crop.
  - **Output**: `pred['joints3d']` (24,3) pubblicato DIRETTAMENTE (no mapping!)
  - **Pubblicazioni**:
    - `/human_pose/points_3d` — PoseArray, **24** Pose, frame `orbbec_color_optical_frame`
    - `/human_pose/skeleton_markers` — MarkerArray (adattato a 24 giunti)
  - **Multi-person**: riutilizzare `person_tracking.py` con `num_joints=24`
  - **Kalman**: 24 filtri Kalman3D (invece di 17)
  - Usare `from spot_perception.sml_pose_indices import *` per TUTTI gli indici

  **Must NOT do**: Non usare numeri magici per indici — sempre costanti SMPL

  **Recommended Agent Profile**: `deep`

  **Parallelization**: Wave 2. Blocks: T14. Blocked by: T1, T3.

  **References**: `yolo_skeleton_spot.py`, `sml_pose_indices.py` (T1), `nlf_params.yaml` (T3)

  **Acceptance Criteria**:
  - [ ] 24 Pose pubblicati su `/human_pose/points_3d`
  - [ ] `len(kalman_filters) == 24`
  - [ ] Indici SMPL usati ovunque (no magic 5,6,11,12)

  **QA**: `timeout 10 ros2 launch spot_control teresa_perception.launch.py perception_backend:=nlf` → `ros2 topic echo /human_pose/points_3d --once | python3 -c "import sys,json; assert len(json.load(sys.stdin)['poses'])==24; print('PASS')"`

  **Commit**: `feat(nlf): add nlf skeleton node for orbbec (24 smpl joints)`

- [x] 5. Nodo NLF RealSense — `nlf_torso_tracker.py` (24 giunti nativi)

  **What to do**:
  - Creare `src/z1_vision/z1_vision/nlf_torso_tracker.py` (~500 linee)
  - **FSM identico** a `z1_yolo_torso_tracker.py`: IDLE→ESTIMATING→LOCKED
  - **Torso extraction**: da SMPL SHOULDER_LEFT(16), SHOULDER_RIGHT(17), HIP_LEFT(1), HIP_RIGHT(2)
  - **Scan point 22-float**: keypoint indices ora sono SMPL 16,17,1,2 (NON piu' COCO 5,6,11,12)
  - **Pubblicazioni**: stessi topic, ma `/exposure/body_keypoints` ora ha 24 Pose

  **Must NOT do**: Non cambiare formato scan_point (ancora 22 float), non cambiare FSM strings

  **Recommended Agent Profile**: `deep`

  **Parallelization**: Wave 2. Blocks: T14. Blocked by: T1, T3.

  **References**: `z1_yolo_torso_tracker.py`, `sml_pose_indices.py` (T1)

  **Acceptance Criteria**:
  - [ ] 24 Pose su `/exposure/body_keypoints`
  - [ ] Scan point ha ancora 22 float ma con SMPL indices
  - [ ] FSM transitions IDLE→ESTIMATING→LOCKED funzionanti

  **Commit**: `feat(nlf): add nlf torso tracker node for realsense (24 smpl joints)`

- [x] 6. Update `yolo_skeleton_spot.py` → 24 giunti

  **What to do**:
  - Modificare `yolo_skeleton_spot.py`: cambiare `self.num_joints = 17` → `self.num_joints = 24`
  - 17 filtri Kalman → 24 filtri
  - Dopo aver prodotto 17 COCO Pose, chiamare `coco_to_smpl_24()` per ottenere 24 Pose
  - Pubblicare 24 Pose su `/human_pose/points_3d`
  - Importare `from spot_perception.sml_pose_indices import *`

  **Must NOT do**: Non cambiare la logica YOLO interna (inferenza, tracking, target selection)

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 2. Blocked by: T1, T2.

  **Acceptance Criteria**:
  - [ ] YOLO mode pubblica 24 Pose
  - [ ] Spine/feet/head/hands sono NaN (9 giunti)
  - [ ] Shoulder, hip, knee, ankle, elbow, wrist, neck mappati correttamente

  **Commit**: `feat(nlf): update yolo skeleton node to publish 24 smpl joints`

- [x] 7. Update `z1_yolo_torso_tracker.py` → 24 giunti

  **What to do**:
  - Modificare `z1_yolo_torso_tracker.py`: cambiare output `/exposure/body_keypoints` da 17 a 24 Pose
  - Usare `coco_to_smpl_24()` per il padding
  - Scan point keypoint indices → SMPL 16,17,1,2

  **Must NOT do**: Non cambiare la logica di tracking torso

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 2. Blocked by: T1, T2.

  **Acceptance Criteria**:
  - [ ] Pubblica 24 Pose su `/exposure/body_keypoints`
  - [ ] Spine/feet/head/hands sono NaN (9+ giunti)
  - [ ] Scan point keypoint indices usano SMPL 16,17,1,2

  **Commit**: `feat(nlf): update yolo torso tracker to publish 24 smpl joints`

---

- [x] 8. Update `person_tracking.py` → 24 Kalman

  **What to do**:
  - `num_joints = 17` → `from spot_perception.sml_pose_indices import NUM_JOINTS; self.num_joints = NUM_JOINTS`
  - 17 Kalman3D → 24 Kalman3D in `PersonTrack.__init__`
  - Loop `for i in range(self.num_joints)` già generico — funziona con 24 automaticamente
  - Aggiungere per-joint tuning per i nuovi giunti:
    - Spine (3,6,9): Q×0.5, R×0.5 (molto stabili)
    - Head (15): Q×1.5, R×1.5 (come nose)
    - Feet (10,11): Q×0.7, R×0.7 (come torso)
    - Hands (22,23): Q×1.0, R×1.0 (default)
  - `select_target()`: il torso angle ora può usare SPINE1+SPINE3 per un vettore più preciso

  **Must NOT do**: Non cambiare la logica di greedy assignment o hysteresis

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 3 (MAX PARALLEL con T9-T13). Blocked by: T1.

  **Acceptance Criteria**:
  - [ ] `PersonTrack` inizializza 24 Kalman filters
  - [ ] `assign_detections_to_tracks` funziona con 24 giunti
  - [ ] `select_target` funziona con 24 giunti

  **Commit**: `feat(nlf): update person tracking to 24 smpl kalman filters`

- [x] 9. Update `posture_classifier.py` → indici SMPL

  **What to do**:
  - Sostituire TUTTI gli indici hardcodati con costanti SMPL:
    - `points[5]` → `points[SHOULDER_LEFT]`
    - `points[6]` → `points[SHOULDER_RIGHT]`
    - `points[11]` → `points[HIP_LEFT]`
    - `points[12]` → `points[HIP_RIGHT]`
    - `points[13]` → `points[KNEE_LEFT]`
    - `points[14]` → `points[KNEE_RIGHT]`
    - `points[15]` → `points[ANKLE_LEFT]`
    - `points[16]` → `points[ANKLE_RIGHT]`
    - `points[0]` → `points[NECK]` (era nose)
  - `len(valid) / 17.0` → `len(valid) / NUM_JOINTS`
  - **Miglioramento**: torso angle ora può usare `SPINE1` e `SPINE3` per un vettore più preciso:
    ```python
    spine_vec = points[SPINE3] - points[SPINE1]  # vettore colonna
    torso_angle = angle_between(spine_vec, UP_VECTOR)
    ```
  - Importare `from spot_perception.sml_pose_indices import *`

  **Must NOT do**: Non cambiare la logica di classificazione (albero decisionale)

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 3. Blocked by: T1.

  **Acceptance Criteria**:
  - [ ] Nessun numero magico (5,6,11,12,13,14,15,16,17) rimasto
  - [ ] Torso angle calcolato con spine vector
  - [ ] `len(valid) / NUM_JOINTS` invece di `/ 17.0`

  **Commit**: `feat(nlf): update posture classifier to smpl-24 indices`

- [x] 10. Update `laying_human_detector.py` → indici SMPL

  **What to do**:
  - Sostituire indici hardcodati con costanti SMPL
  - `if len(kp) < 17` → `if len(kp) < NUM_JOINTS`
  - Calcolo body axis: ora può usare `HEAD` e `PELVIS` (più preciso di nose→ankles)
  - Calcolo bbox half-width: iterare su `ARM_JOINTS + LEG_JOINTS` invece di range fisso

  **Must NOT do**: Non cambiare la logica dell'approach_point laterale

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 3. Blocked by: T1.

  **Acceptance Criteria**:
  - [ ] `if len(kp) < NUM_JOINTS` (non più 17)
  - [ ] Body axis calcolato da `HEAD` e `PELVIS`
  - [ ] Nessun indice hardcodato 5,6,11,12,0

  **Commit**: `feat(nlf): update laying human detector to smpl-24`

- [x] 11. Update `z1_scan_manager.py` → FAST points da SMPL

  **What to do**:
  - FAST ultrasound points calcolati da keypoints. Indici attuali: kp5, kp6, kp11, kp12 (COCO). Nuovi: `SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT` (SMPL).
  - Ricalcolo punti anatomici (subxifoideo, RUQ, LUQ, sovrapubico) usando gli stessi ratio ma con i nuovi SMPL points
  - Importare `from spot_perception.sml_pose_indices import *`

  **Must NOT do**: Non cambiare la geometria dei punti FAST — solo gli indici sorgente

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 3. Blocked by: T1.

  **Acceptance Criteria**:
  - [ ] FAST points usano `SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT`
  - [ ] Nessun riferimento a COCO kp5, kp6, kp11, kp12
  - [ ] Geometria punti invariata (stessi ratio)

  **Commit**: `feat(nlf): update z1 scan manager to smpl-24 keypoints`

- [x] 12. Update `exposure_scanner.py` → body grid da 24 giunti

  **What to do**:
  - Griglia esposizione corpo attualmente generata da 17 giunti su 7 regioni
  - Con 24 giunti, **migliorare** la griglia:
    - **Torso**: 4 punti → 6 punti (interpolazione SPINE1, SPINE2, SPINE3 aggiuntive)
    - **Testa**: 2 punti → aggiungere punto HEAD centrale
    - **Gambe**: aggiungere punti FOOT_LEFT, FOOT_RIGHT
    - **Braccia**: aggiungere punti HAND_LEFT, HAND_RIGHT
  - Totale punti griglia: 16 → ~22 (più densa)
  - Skeleton refinement: accumula 24 giunti invece di 17
  - Importare `from spot_perception.sml_pose_indices import *`

  **Must NOT do**: Non rimuovere la logica di refinement scheletro o il protocollo per-punto

  **Recommended Agent Profile**: `deep` (reason: modifica algoritmo griglia, ~650 linee da aggiornare)

  **Parallelization**: Wave 3. Blocked by: T1.

  **References**: `exposure_scanner.py` (linee 1-669), `sml_pose_indices.py` (T1)

  **Acceptance Criteria**:
  - [ ] Body grid generata con 22+ punti (vs 16 originali)
  - [ ] Spine, head, feet, hands inclusi nella griglia
  - [ ] Refinement scheletro accumula 24 giunti

  **Commit**: `feat(nlf): update exposure scanner to smpl-24 body grid`

- [x] 13. Update `body_search_scanner.py` → score da SMPL

  **What to do**:
  - Score di qualità per-joint: sostituire COCO indices (kp5,6,11,12) con SMPL equivalents (16,17,1,2)
  - Aggiungere spine joints (SPINE1, SPINE2, SPINE3) al calcolo score per torso detection
  - `n_kp / max_kp` — max_kp ora è 24 invece di 17

  **Must NOT do**: Non cambiare la formula dello score o la logica della scansione

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 3. Blocked by: T1.

  **Acceptance Criteria**:
  - [ ] Score calcolato su SMPL 16,17,1,2 + SPINE1, SPINE2, SPINE3
  - [ ] `max_kp = 24` (non più 17)
  - [ ] Nessun indice COCO residuo

  **Commit**: `feat(nlf): update body search scanner to smpl-24 keypoints`

---

- [x] 14. Launch files + setup.py (default: `nlf`)

  **What to do**:
  - Modificare `spot_perception.launch.py`, `z1_perception.launch.py`, `z1_torso_surface.launch.py`:
    - `default_value='nlf'` (NON piu' 'yolo')
    - Branch condizionale NLF vs YOLO
  - Modificare `spot_perception/setup.py` e `z1_vision/setup.py`:
    - Nuovi entry point: `nlf_skeleton`, `nlf_torso_tracker`
  - Aggiornare `package.xml` con eventuali nuove dipendenze

  **Must NOT do**: Non rimuovere il path YOLO

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 4. Blocks: T17. Blocked by: T4, T5.

  **Acceptance Criteria**:
  - [ ] `perception_backend` default = `"nlf"` in tutti e 3 i launch file
  - [ ] `perception_backend:=nlf` lancia nodi NLF
  - [ ] `perception_backend:=yolo` lancia nodi YOLO (regressione)
  - [ ] `setup.py` ha entry point `nlf_skeleton` e `nlf_torso_tracker`

  **Commit**: `feat(nlf): integrate nlf into launch files with nlf as default`

- [x] 15. Mesh SMPL publisher

  **What to do**:
  - In `nlf_skeleton.py`: pubblicare `pred['vertices3d']` decimati su `/human_pose/smpl_mesh`
  - Decimazione configurabile via parametro YAML
  - Solo se `publish_mesh: true`

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 4. Blocked by: T4.

  **Acceptance Criteria**:
  - [ ] `/human_pose/smpl_mesh` pubblicato solo se `publish_mesh: true`
  - [ ] Vertici decimati (count < 6890)
  - [ ] `header.frame_id` = `orbbec_color_optical_frame`

  **Commit**: NO (incluso in T4)

- [x] 16. YAML config finalization

  **What to do**:
  - Finalizzare `nlf_params.yaml` e `nlf_torso_params.yaml` con TUTTI i parametri
  - Aggiungere `num_joints: 24` ovunque (non più 17)
  - `download_nlf_models.sh` finale

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 4. Blocked by: T3.

  **Acceptance Criteria**:
  - [ ] `nlf_params.yaml` ha tutti i parametri (model_path, device, imgsz, publish_mesh, mesh_decimation, num_joints: 24)
  - [ ] `nlf_torso_params.yaml` ha parametri torso-specifici
  - [ ] `download_nlf_models.sh` scarica `nlf_s.torchscript`
  - [ ] `.gitignore` aggiornato

  **Commit**: NO (incluso in T3)

---

- [x] 17. Aggiornare documentazione

  **What to do**:
  - `CHANGELOG.md`: entry per NLF integration con 24 giunti, default nlf, consumer aggiornati
  - `DESCRIPTION.md`: aggiornare sezione perception pipeline (24 giunti, nuovi topic)

  **Recommended Agent Profile**: `writing`

  **Parallelization**: Wave 5. Blocked by: T14.

  **Acceptance Criteria**:
  - [ ] CHANGELOG.md ha entry "NLF integration — 24 SMPL joints"
  - [ ] DESCRIPTION.md menziona 24 giunti e nuovo topic `/human_pose/smpl_mesh`
  - [ ] Entrambi i file menzionano `perception_backend` parametro

  **Commit**: `docs(nlf): update changelog and architecture for nlf 24-joint migration`

- [x] 18. Unit test: `sml_pose_indices.py` + `yolo_to_smpl_pad.py`

  **What to do**:
  - Creare `src/spot_perception/test/test_sml_pose_indices.py` (6 test)
  - Creare `src/spot_perception/test/test_yolo_to_smpl_pad.py` (5 test)
  - Test: numero giunti, mapping corretto, NaN positions, costanti univoche

  **Recommended Agent Profile**: `quick`

  **Parallelization**: Wave 5. Blocked by: T1, T2.

  **Acceptance Criteria**:
  - [ ] `python3 -m pytest src/spot_perception/test/test_sml_pose_indices.py -v` → 6 passed
  - [ ] `python3 -m pytest src/spot_perception/test/test_yolo_to_smpl_pad.py -v` → 5 passed
  - [ ] Test coprono: NUM_JOINTS==24, mapping 17→24, NaN positions, costanti univoche

  **Commit**: `test(nlf): add unit tests for smpl-24 constants and yolo pad`

---

## Final Verification Wave

- [x] F1. **Plan Compliance Audit** — `oracle`
  Verificare tutti i Must Have e Must NOT Have. Grep per `len(poses) == 17`, `range(17)`, `num_joints = 17`, `range(self.num_joints)` su TUTTI i consumer — se trovati, REJECT. Verificare che `sml_pose_indices.py` sia importato dove servono indici SMPL. Verificare default `nlf` in launch file.

- [x] F2. **Code Quality Review** — `unspecified-high`
  Verificare nuovi file e file modificati. Niente magic numbers (solo costanti SMPL). Niente `as any`, `@ts-ignore`, empty catch, print di debug. Verificare coerenza stile con codebase esistente.

- [x] F3. **Real Manual QA** — `unspecified-high`
  Eseguire TUTTI gli QA scenarios. Verificare:
  1. 24 Pose pubblicati in entrambi i backend
  2. YOLO mode: esattamente 7 NaN nelle posizioni SMPL (spine1, spine2, spine3, foot_L, foot_R, head, hand_L, hand_R — sono 8 NaN in realtà! Verificare)
  3. NLF mode: tutti i 24 giunti popolati (no NaN tranne occlusioni)
  4. Posture classifier funziona con 24 giunti
  5. Laying detector produce approach_point
  6. Exposure scanner griglia corpo con spine/neck
  7. FAST points calcolati correttamente
  8. Launch con `nlf` e `yolo` entrambi funzionanti

- [x] F4. **Scope Fidelity Check** — `deep`
  Per ogni task: verificare 1:1 implementazione vs specifiche. Controllare che nessun file fuori scope sia stato toccato. Verificare che file YOLO originali siano stati SOLO aggiornati a 24 giunti, non refactored. Rilevare cross-task contamination.

---

## Commit Strategy

- **1-3**: `feat(nlf): add smpl-24 constants, yolo-to-smpl pad, config`
- **4-5**: `feat(nlf): add nlf skeleton + torso tracker nodes`
- **6-7**: `feat(nlf): update yolo nodes to publish 24 smpl joints`
- **8-11**: `feat(nlf): update person tracking + posture + laying + scan manager to smpl-24`
- **12-13**: `feat(nlf): update exposure scanner + body search to smpl-24`
- **14-15**: `feat(nlf): integrate launch files + smpl mesh`
- **17-18**: `docs(nlf): update changelog + tests`

---

## Success Criteria

```bash
# NLF mode: 24 giunti
ros2 launch spot_control teresa_perception.launch.py perception_backend:=nlf
timeout 10 bash -c 'ros2 topic echo /human_pose/points_3d --once | python3 -c "
import sys, json; d=json.load(sys.stdin); assert len(d[\"poses\"])==24; print(\"PASS: 24 joints\")
"'

# YOLO fallback: 24 giunti con NaN
ros2 launch spot_control teresa_perception.launch.py perception_backend:=yolo
timeout 10 bash -c 'ros2 topic echo /human_pose/points_3d --once | python3 -c "
import sys, json, math; d=json.load(sys.stdin); assert len(d[\"poses\"])==24
# spine joints (3,6,9) should be NaN in YOLO mode
for i in [3,6,9]: assert math.isnan(d[\"poses\"][i][\"position\"][\"x\"]), f\"SMPL {i} not NaN\"
print(\"PASS: 24 joints, NaN padding ok\")
"'

# Posture classifier works with 24 joints
timeout 10 bash -c 'ros2 topic echo /human_pose/posture --once | grep -qE "LYING|STANDING|SITTING|UNKNOWN" && echo "PASS: posture ok"'
```
