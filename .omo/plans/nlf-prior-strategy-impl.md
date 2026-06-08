# NLF Prior + Active Perception — Implementation Plan

## TL;DR

> **Quick Summary**: Aggiungere 1 frame NLF al LOCKING come prior 24-SMPL. Se NLF arriva entro 10s → fast-path (grid ridotta, PRE_APPROACH immediato, Quality Monitor NLF). Se NLF fallisce → fallback totale al comportamento 6 Giugno 2026. Gate unico: `_nlf_prior_valid()`.
>
> **Deliverables**:
> - `wbc_coordinator.py`: subscriber `/human_pose/points_3d` + `/exposure/nlf_prior`, prior gate, PRE_APPROACH biforcato, Quality Monitor NLF
> - `nlf_skeleton.py`: subscriber `/nlf/trigger`, publisher `/exposure/nlf_prior`, callback single-shot
> - `wbc_qp_controller.py`: `_gen_reduced_grid_from_prior()` per ramo NLF OK
> - `wbc_params.yaml` + `body_search_params.yaml`: nuovi parametri
> - Test: pytest per `_nlf_prior_valid()`, `_check_nlf_delta()`, biforcazione PRE_APPROACH
>
> **Estimated Effort**: Medium-Large (1.5-2 giorni)
> **Parallel Execution**: YES — 2 waves
> **Critical Path**: Task 1 → Task 3 → Task 4 → Task 6

---

## Context

### Original Request
Implementare la strategia "NLF Prior + Active Perception" (`.omo/plans/nlf-prior-strategy.md`): NLF come ground truth al LOCKING, non sostituto di YOLO. Fallback totale se NLF fallisce.

### Interview Summary
**Key Discussions**:
- NLF fallback: binario — se fallisce, TUTTO torna al 6 Giugno 2026 (nessuna ottimizzazione parziale)
- Test: automatici (pytest) + verifica ROS2 sui topic
- File corretti dopo Metis/Oracle: grid in `wbc_qp_controller.py`, non `body_search_scanner.py`
- `nlf_skeleton.py` richiede +50 righe, non +3 (trigger subscriber, prior publisher, callback, timeout)

### Metis Review — 11 Issues Found
**Blockers (fixed in this plan)**:
1. Grid generation relocated: `body_search_scanner.py` → `wbc_qp_controller.py`
2. Coordinator aggiunge subscription a `/human_pose/points_3d`
3. `nlf_skeleton.py` effort revised: +3 → +50 lines
4. PRE_APPROACH fast path: minimum safety gate (1 tick / 1s timeout)
5. Frame transformations specified: `orbbec_color_optical_frame` → `odom` via TF

**Non-blockers (addressed in tasks)**:
6. Two NLF nodes distinguished: Orbbec (prior) vs RealSense (torso tracker)
7. NLF model lazy-load race condition on first trigger
8. Debounce for rapid LOCKING re-entry
9. Parameter declarations in z1_FSM.py for reduced grid
10. Quality Monitor: NLF delta as separate method, not inside _QualityMonitor class
11. Test infrastructure built from scratch (zero existing functional tests)

---

## Work Objectives

### Core Objective
Aggiungere prior NLF al LOCKING con fallback binario: NLF OK → fast-path, NLF fallito → comportamento invariato.

### Concrete Deliverables
- `wbc_coordinator.py`: +2 subscriptions, +1 publisher, +4 metodi, 2 metodi modificati (~120 lines)
- `nlf_skeleton.py`: +1 subscriber, +1 publisher, +1 callback, timeout handling (~50 lines)
- `wbc_qp_controller.py`: +1 metodo `_gen_reduced_grid_from_prior()` (~40 lines)
- `wbc_params.yaml`: +3 parametri NLF
- `body_search_params.yaml`: +5 parametri reduced grid
- `test/`: +3 file di test (coordinator prior logic, NLF skeleton trigger, PRE_APPROACH bifurcation)

### Definition of Done
- [ ] LOCKING triggera NLF, prior 24-SMPL salvato entro 10s o timeout
- [ ] `_nlf_prior_valid()` gate unico controlla tutti i rami condizionali
- [ ] NLF OK: PRE_APPROACH gate 1s/1tick minimo, coherence check non bloccante
- [ ] NLF OK: grid ridotta (~12 pose) in `wbc_qp_controller.py`
- [ ] NLF OK: Quality Monitor delta 15cm/30cm, LOOKAT NLF(70%)+YOLO(30%)
- [ ] NLF fallito: **tutto invariato** — grid attuale, attesa RealSense, YOLO only
- [ ] 3 file di test passano (`colcon test`)
- [ ] Topic `/exposure/nlf_prior` pubblica 24 SMPL su trigger
- [ ] Trasformazione TF `orbbec_color_optical_frame` → `odom` funzionante
- [ ] Debounce: NLF non ri-triggerato se già in corso

### Must Have
- NLF fallito non blocca la missione (timeout 10s → YOLO only)
- **Fallback totale**: nessuna ottimizzazione parziale senza NLF
- YOLO a 15 FPS in tutte le fasi dinamiche (SEARCHING, APPROACHING)
- Prior NLF accessibile via `/exposure/nlf_prior` (PoseArray, 24 joints, frame `odom`)
- Backward compatibility: se NLF non disponibile, sistema = 6 Giugno 2026

### Must NOT Have
- NO NLF in SEARCHING o APPROACHING (troppo lento)
- NO fusione temporale YOLO+NLF (100:1 frame rate disparity)
- NO rimozione dell'active perception (YOLO esplora ancora)
- NO modifica alla matematica del LOOKAT (solo target iniziale migliore)
- NO modifica a `body_search_scanner.py` (è downstream, consuma pose pre-generate)
- NO modifica a RealSense NLF tracker (`nlf_torso_tracker.py`)

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (colcon test, pytest)
- **Automated tests**: Tests-after (test scritti dopo implementazione)
- **Framework**: pytest + colcon test
- **Coverage**: `_nlf_prior_valid()`, `_check_nlf_delta()`, PRE_APPROACH bifurcation, NLF trigger callback

### QA Policy
Ogni task include QA Scenarios eseguibili dall'agente. Per task ROS2: verifica topic, log, timeout. Per task Python puro: pytest.
Evidence saved to `.omo/evidence/task-{N}-{scenario-slug}.{ext}`.

- **API/Backend**: Bash (ros2 topic pub/echo, ros2 node info, colcon test)
- **Library/Module**: Bash (python3 -m pytest)

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — foundation + scaffolding):
├── Task 1: nlf_skeleton.py — trigger + prior publisher
├── Task 2: wbc_params.yaml + body_search_params.yaml — nuovi parametri
└── Task 3: wbc_coordinator.py — subscriber skeleton + TF transform + prior gate

Wave 2 (After Wave 1 — core logic, MAX PARALLEL):
├── Task 4: wbc_coordinator.py — PRE_APPROACH biforcata + safety gate
├── Task 5: wbc_qp_controller.py — reduced grid from prior
├── Task 6: wbc_coordinator.py — Quality Monitor NLF delta + LOOKAT weighting
└── Task 7: Tests — pytest per prior logic, trigger, PRE_APPROACH

Critical Path: Task 1 → Task 3 → Task 4 → Task 6
Parallel Speedup: ~40% faster than sequential
Max Concurrent: 4 (Wave 2)
```

### Dependency Matrix

- **1**: — — 3, 2
- **2**: — — 4, 5, 3
- **3**: 1 — 4, 6, 2
- **4**: 2, 3 — 7, 2
- **5**: 2 — 7, 2
- **6**: 3 — 7, 2
- **7**: 4, 5, 6 — —, 1

### Agent Dispatch Summary

- **Wave 1**: **3 tasks** — T1→`unspecified-high`, T2→`quick`, T3→`deep`
- **Wave 2**: **4 tasks** — T4→`deep`, T5→`unspecified-high`, T6→`deep`, T7→`unspecified-high`

---

## TODOs

- [x] 1. `nlf_skeleton.py` — trigger subscriber + prior publisher + single-shot callback

  **What to do**:
  - Aggiungere subscriber a `/nlf/trigger` (Bool): `create_subscription(Bool, '/nlf/trigger', self._cb_trigger, 10)`
  - Aggiungere publisher `/exposure/nlf_prior` (PoseArray): `create_publisher(PoseArray, '/exposure/nlf_prior', 10)`
  - Implementare `_cb_trigger(msg)`: se `msg.data is True` e il modello è caricato → esegue UNA inferenza sul frame corrente → pubblica 24 SMPL su `/exposure/nlf_prior` in `orbbec_color_optical_frame`
  - Gestire lazy-load race: se `self._nlf_ready == False` al trigger → log warning, non pubblicare
  - Timeout handling: se il modello non è pronto entro 10s dal trigger → log error, non pubblicare
  - Il nodo **continua** lo streaming normale su `/human_pose/points_3d` — il trigger è un'operazione aggiuntiva, non sostitutiva
  - Dichiarare `self._nlf_ready = False` in `__init__`, impostare a `True` dopo `self._build_pipeline()`

  **Must NOT do**:
  - NON cambiare lo streaming continuo esistente
  - NON aggiungere modalità/mode switch (trigger è un'operazione one-shot)
  - NON pubblicare `/exposure/nlf_prior` se modello non caricato

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Modifica a nodo ROS2 esistente con nuova logica di trigger, publisher, e race condition
  - **Skills**: None
  - **Skills Evaluated but Omitted**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3)
  - **Blocks**: Task 3
  - **Blocked By**: None (can start immediately)

  **References**:
  - `src/spot_perception/spot_perception/nlf_skeleton.py:1-50` — imports e struttura nodo esistente
  - `src/spot_perception/spot_perception/nlf_skeleton.py:120-145` — `__init__` con publishers/subscribers esistenti (pattern da seguire)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:330-420` — metodo `_cb_color` e pipeline inferenza (capire dove iniettare il trigger)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:430-517` — `_build_pipeline()` e `_nlf_ready` (capire il lazy-load)

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: Happy path — trigger pubblica prior
    Tool: Bash (ros2 topic)
    Preconditions: nlf_skeleton_node running, modello caricato, camera feed attivo
    Steps:
      1. ros2 topic pub /nlf/trigger std_msgs/msg/Bool "data: true" -1
      2. ros2 topic echo /exposure/nlf_prior --once --timeout 15
      3. Verifica che il messaggio contenga 24 poses
      4. Verifica che almeno 4 giunti torso (indices 6,7,8,14) abbiano posizioni non-NaN
    Expected Result: PoseArray con 24 poses pubblicato entro 10s dal trigger
    Failure Indicators: Nessun messaggio entro 15s, meno di 24 poses, tutti i giunti NaN
    Evidence: .omo/evidence/task-1-trigger-publishes-prior.txt

  Scenario: Failure — trigger con modello non caricato
    Tool: Bash (ros2 topic)
    Preconditions: nlf_skeleton_node running, modello NON caricato (file mancante o stub mode)
    Steps:
      1. ros2 topic pub /nlf/trigger std_msgs/msg/Bool "data: true" -1
      2. ros2 topic echo /exposure/nlf_prior --timeout 5
      3. Verifica che NON venga pubblicato alcun messaggio
    Expected Result: Nessuna pubblicazione su /exposure/nlf_prior
    Failure Indicators: Messaggio pubblicato con modello non caricato (dati invalidi)
    Evidence: .omo/evidence/task-1-trigger-no-model.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-1-trigger-publishes-prior.txt` — output di `ros2 topic echo`
  - [ ] `task-1-trigger-no-model.txt` — output vuoto che conferma nessuna pubblicazione

  **Commit**: YES (groups with 2, 3)
  - Message: `feat(nlf): add /nlf/trigger subscriber and /exposure/nlf_prior publisher for single-shot prior`
  - Files: `src/spot_perception/spot_perception/nlf_skeleton.py`
  - Pre-commit: `colcon build --packages-select spot_perception`

- [x] 2. `wbc_params.yaml` + `body_search_params.yaml` — nuovi parametri NLF

  **What to do**:
  - In `src/spot_control/config/wbc_params.yaml`: aggiungere blocco NLF
    ```yaml
    # NLF prior parameters
    nlf_timeout: 10.0
    nlf_coherence_threshold: 0.15
    nlf_divergence_threshold: 0.30
    nlf_prior_topic: '/exposure/nlf_prior'
    nlf_trigger_topic: '/nlf/trigger'
    ```
  - In `src/z1_vision/config/body_search_params.yaml`: aggiungere blocco reduced grid
    ```yaml
    # NLF reduced scan grid (used only when NLF prior is valid)
    body_scan_reduced_ny: 2
    body_scan_reduced_nx: 2
    body_scan_reduced_wrist_ny: 2
    body_scan_reduced_wrist_nz: 2
    body_scan_reduced_fuse: true
    ```
  - Verificare che i parametri NON confliggano con quelli esistenti (nomi univoci)

  **Must NOT do**:
  - NON modificare parametri esistenti
  - NON rimuovere parametri grid attuali (servono per fallback)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Solo aggiunta di parametri YAML, nessuna logica
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3)
  - **Blocks**: Tasks 4, 5
  - **Blocked By**: None

  **References**:
  - `src/spot_control/config/wbc_params.yaml:1-80` — struttura YAML esistente
  - `src/z1_vision/config/body_search_params.yaml:1-30` — parametri grid esistenti

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: Parametri caricati correttamente
    Tool: Bash (ros2 param)
    Preconditions: workspace built
    Steps:
      1. grep "nlf_timeout" src/spot_control/config/wbc_params.yaml → esiste
      2. grep "body_scan_reduced_ny" src/z1_vision/config/body_search_params.yaml → esiste
      3. colcon build --packages-select spot_control z1_vision → success
    Expected Result: Build success, parametri presenti nei file YAML
    Failure Indicators: Build error, parametri non trovati
    Evidence: .omo/evidence/task-2-params-exist.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-2-params-exist.txt` — output grep + build

  **Commit**: YES (groups with 1, 3)
  - Message: `feat(config): add NLF prior and reduced grid parameters`
  - Files: `src/spot_control/config/wbc_params.yaml`, `src/z1_vision/config/body_search_params.yaml`
  - Pre-commit: `colcon build --packages-select spot_control z1_vision`

- [x] 3. `wbc_coordinator.py` — subscriber skeleton + TF transform + prior gate `_nlf_prior_valid()`

  **What to do**:
  - Aggiungere subscriber a `/exposure/nlf_prior` (PoseArray): `create_subscription(PoseArray, '/exposure/nlf_prior', self._cb_nlf_prior, 10)`
  - Aggiungere subscriber a `/human_pose/points_3d` (PoseArray, già esistente dal NLF streaming): `create_subscription(PoseArray, '/human_pose/points_3d', self._cb_skeleton_stream, 10)`
  - Aggiungere publisher `/nlf/trigger` (Bool): `create_publisher(Bool, '/nlf/trigger', 10)`
  - Aggiungere attributi in `__init__`: `self._nlf_prior = None`, `self._nlf_trigger_time = None`
  - Implementare `_cb_nlf_prior(msg)`: se `_nlf_prior == 'timeout'` → ignora. Altrimenti, trasforma i 24 punti da `orbbec_color_optical_frame` a `odom` via `self._tf_buffer.transformPoseArray()` (o lookupTransform + manual transform se PoseArray non supportato direttamente). Salva in `self._nlf_prior` come `list[ndarray]`.
  - Implementare `_nlf_prior_valid()`:
    ```python
    def _nlf_prior_valid(self) -> bool:
        if self._nlf_prior is None:
            return False
        if self._nlf_prior == 'timeout':
            return False
        if len(self._nlf_prior) != 24:
            return False
        valid_torso = sum(1 for j in [6,7,8,14]
                          if not np.any(np.isnan(self._nlf_prior[j])))
        return valid_torso >= 4
    ```
  - Implementare `_torso_center_from_prior()`: media di SPINE1(6), SPINE2(7), SPINE3(8), PELVIS(14) non-NaN
  - In `_enter_locking()`: aggiungere `self._nlf_prior = None; self._nlf_trigger_time = self.get_clock().now(); self._pub_nlf_trigger.publish(Bool(data=True))`
  - In `_tick_locking()`: aggiungere timeout check (non bloccante) — se `elapsed > 10.0` → `self._nlf_prior = 'timeout'` + log warning
  - Debounce: se già in LOCKING e `_nlf_prior is not None and _nlf_prior != 'timeout'` → non ri-triggerare

  **Must NOT do**:
  - NON rimuovere il codice esistente di LOCKING (5 campioni, ik_done)
  - NON bloccare la transizione LOCKING→PRE_APPROACH se NLF non ancora arrivato
  - NON usare `transformPoseArray` se non esiste nel tf2 Python API — usare `lookupTransform` + loop manuale

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Logica complessa: doppia subscription, TF transform, state machine integration, debounce
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2)
  - **Blocks**: Tasks 4, 6
  - **Blocked By**: Task 1

  **References**:
  - `src/spot_control/spot_control/wbc_coordinator.py:1-80` — imports e struttura classe
  - `src/spot_control/spot_control/wbc_coordinator.py:80-160` — `__init__` con subscriptions/publishers esistenti (pattern)
  - `src/spot_control/spot_control/wbc_coordinator.py:900-960` — `_tick_locking()` esistente (dove iniettare trigger e timeout)
  - `src/spot_control/spot_control/wbc_coordinator.py:1050-1120` — `_tick_pre_approach()` esistente (legacy path da preservare)
  - `src/spot_control/spot_control/wbc_coordinator.py:100-160` — TF buffer esistente (verificare disponibilità)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:370-430` — frame in cui NLF pubblica (`orbbec_color_optical_frame`)

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: Happy path — prior NLF ricevuto e valido
    Tool: Bash (ros2 topic)
    Preconditions: nlf_skeleton_node running, coordinator avviato
    Steps:
      1. Simula LOCKING pubblicando su topic appropriato
      2. ros2 topic pub /exposure/nlf_prior geometry_msgs/msg/PoseArray "{...24 poses valide in odom...}" -1
      3. Verifica che il coordinator logghi "NLF prior received: 24 joints"
      4. Verifica che _nlf_prior_valid() restituisca True (tramite log o test)
    Expected Result: Prior ricevuto, _nlf_prior_valid() == True
    Failure Indicators: Prior ignorato, _nlf_prior_valid() == False con dati validi
    Evidence: .omo/evidence/task-3-prior-received.txt

  Scenario: Failure — timeout NLF 10s
    Tool: Bash (ros2 topic + log)
    Preconditions: coordinator avviato, LOCKING triggerato, /exposure/nlf_prior NON pubblicato
    Steps:
      1. Entra in LOCKING (trigger NLF parte)
      2. Attendi 11 secondi senza pubblicare prior
      3. Verifica log: "NLF timeout — proceeding without prior"
      4. Verifica che _nlf_prior_valid() restituisca False
      5. Verifica che il FSM passi comunque a PRE_APPROACH (non bloccato)
    Expected Result: Timeout dopo 10s, _nlf_prior_valid() == False, FSM prosegue
    Failure Indicators: FSM bloccato in LOCKING, crash, prior 'timeout' non impostato
    Evidence: .omo/evidence/task-3-nlf-timeout.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-3-prior-received.txt` — log del coordinator
  - [ ] `task-3-nlf-timeout.txt` — log del coordinator con timeout

  **Commit**: YES (groups with 1, 2)
  - Message: `feat(wbc): add NLF prior subscription, TF transform, and _nlf_prior_valid() gate`
  - Files: `src/spot_control/spot_control/wbc_coordinator.py`
  - Pre-commit: `colcon build --packages-select spot_control`

---

- [x] 4. `wbc_coordinator.py` — PRE_APPROACH biforcata con safety gate minimo

  **What to do**:
  - Rinominare il metodo esistente `_tick_pre_approach()` in `_tick_pre_approach_legacy()` (codice invariato)
  - Creare nuovo `_tick_pre_approach()` con if/else su `_nlf_prior_valid()`:
    - **Ramo NLF OK**: pubblica LOOKAT goal da `_torso_center_from_prior()`. Avvia un **safety gate di 1s** (10 tick a 10Hz): pubblica il goal, attendi, poi coherence check non bloccante su RealSense (`ESTIMATING/LOCKED`). Se delta < 15cm → log info. Se delta ≥ 15cm → log warning. **In ogni caso dopo 1s → APPROACHING**.
    - **Ramo fallback**: chiama `self._tick_pre_approach_legacy()` — sliding window ≥1/5 tick, timeout 5s, comportamento 6 Giugno 2026 invariato.
  - Aggiungere attributo `self._pre_approach_fast_start = None` per tracciare il safety gate timer

  **Must NOT do**:
  - NON rimuovere `_torso_detected_ticks` buffer (serve al fallback)
  - NON procedere immediatamente a APPROACHING nel ramo NLF OK (minimo 1s safety gate)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Modifica a FSM critico con bifurcazione, preservazione legacy path, safety gate timing
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 5, 6, 7)
  - **Blocks**: Task 7
  - **Blocked By**: Tasks 2, 3

  **References**:
  - `src/spot_control/spot_control/wbc_coordinator.py:1050-1120` — `_tick_pre_approach()` esistente da rinominare in legacy
  - `src/spot_control/spot_control/wbc_coordinator.py:1400-1430` — `_do_set_state()` per transizione PRE_APPROACH→APPROACHING
  - `.omo/plans/nlf-prior-strategy.md:158-180` — pseudocodice PRE_APPROACH biforcata

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: NLF OK — PRE_APPROACH fast path con safety gate 1s
    Tool: Bash (ros2 topic + log)
    Preconditions: coordinator in PRE_APPROACH, _nlf_prior_valid() == True
    Steps:
      1. Verifica che il LOOKAT goal venga pubblicato immediatamente
      2. Attendi 1s — verifica che il FSM NON sia ancora in APPROACHING prima di 1s
      3. Dopo 1s — verifica che il FSM passi a APPROACHING
      4. Verifica log: "RealSense coherent" o "RealSense diverges" (coherence check eseguito)
    Expected Result: Goal pubblicato subito, APPROACHING dopo ≥1s, coherence check eseguito
    Failure Indicators: APPROACHING immediato (<1s), coherence check saltato, crash
    Evidence: .omo/evidence/task-4-fast-path-safety-gate.txt

  Scenario: NLF fallito — PRE_APPROACH legacy invariata
    Tool: Bash (ros2 topic + log)
    Preconditions: coordinator in PRE_APPROACH, _nlf_prior_valid() == False
    Steps:
      1. Verifica che venga chiamato _tick_pre_approach_legacy()
      2. Verifica sliding window ≥1/5 tick attivo
      3. Verifica timeout 5s funzionante
      4. Verifica transizione a APPROACHING dopo ESTIMATING/LOCKED o timeout
    Expected Result: Comportamento identico al 6 Giugno 2026
    Failure Indicators: Fast path attivato erroneamente, sliding window assente
    Evidence: .omo/evidence/task-4-legacy-unchanged.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-4-fast-path-safety-gate.txt` — log con timing safety gate
  - [ ] `task-4-legacy-unchanged.txt` — log con sliding window legacy

  **Commit**: YES (groups with 5, 6)
  - Message: `feat(wbc): bifurcate PRE_APPROACH with NLF fast-path and 1s safety gate`
  - Files: `src/spot_control/spot_control/wbc_coordinator.py`
  - Pre-commit: `colcon build --packages-select spot_control`

- [x] 5. `wbc_qp_controller.py` — `_gen_reduced_grid_from_prior()` per ramo NLF OK

  **What to do**:
  - Aggiungere metodo `_gen_reduced_grid_from_prior(nlf_prior)` nel QP controller:
    - Riceve i 24 SMPL (in `odom` frame, trasformati dal coordinator)
    - Genera ~12 pose ridotte in 2 fasi (invece delle 3 fasi attuali):
      - Fase 1 (home wrist sweep ridotto): `reduced_wrist_ny × reduced_wrist_nz` = 2×2 = 4 pose
      - Fase 2 (arc positions ridotte): `reduced_ny × reduced_nx × (1 + reduced_wrist_ny × reduced_wrist_nz)` = 2×2×(1+1) = 8 pose
    - Usa i 24 SMPL per centrare la griglia sui giunti noti (HEAD, SPINE, PELVIS)
    - Restituisce `scan_poses: list[PoseStamped]`
  - Leggere nuovi parametri da `body_search_params.yaml`: `body_scan_reduced_ny`, `body_scan_reduced_nx`, `body_scan_reduced_wrist_ny`, `body_scan_reduced_wrist_nz`
  - In `_gen_cartesian_scan_grid()`: aggiungere if/else — se `self._coordinator._nlf_prior_valid()` → chiama `_gen_reduced_grid_from_prior()`, altrimenti → grid adattiva esistente invariata
  - Dichiarare i 4 nuovi parametri in `__init__` con `self.declare_parameter()`

  **Must NOT do**:
  - NON rimuovere `_gen_adaptive_grid_from_keypoint_conf()` (serve per fallback)
  - NON modificare `body_search_scanner.py` (consuma pose pre-generate, invariato)
  - NON hardcodare i parametri reduced grid

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Nuovo metodo di generazione griglia in controller esistente, parametri YAML, integrazione con coordinator
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 6, 7)
  - **Blocks**: Task 7
  - **Blocked By**: Task 2

  **References**:
  - `src/spot_control/spot_control/wbc_qp_controller.py:520-600` — `_gen_cartesian_scan_grid()` e grid adattiva esistente
  - `src/spot_control/spot_control/wbc_qp_controller.py:600-700` — `BodySearchScanner` e generazione pose
  - `src/z1_vision/config/body_search_params.yaml` — parametri grid (pattern per i nuovi)
  - `src/spot_perception/spot_perception/sml_pose_indices.py:1-80` — SMPL-24 joint indices (HEAD, SPINE, PELVIS)

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: NLF OK — grid ridotta generata
    Tool: Bash (python3 -c)
    Preconditions: QP controller istanziato, _nlf_prior_valid() == True
    Steps:
      1. Importa wbc_qp_controller e istanzia con prior NLF mock
      2. Chiama _gen_reduced_grid_from_prior(prior_mock)
      3. Verifica che restituisca list[PoseStamped] con ~12 elementi
      4. Verifica che tutte le pose siano nel workspace Z1
    Expected Result: ~12 PoseStamped generate, tutte valide
    Failure Indicators: Lista vuota, pose fuori workspace, eccezione
    Evidence: .omo/evidence/task-5-reduced-grid.txt

  Scenario: NLF fallito — grid adattiva invariata
    Tool: Bash (python3 -c)
    Preconditions: QP controller istanziato, _nlf_prior_valid() == False
    Steps:
      1. Chiama _gen_cartesian_scan_grid()
      2. Verifica che usi _gen_adaptive_grid_from_keypoint_conf()
      3. Verifica che il comportamento sia identico a prima della modifica
    Expected Result: Grid adattiva esistente, nessun cambiamento
    Failure Indicators: Grid ridotta usata erroneamente, crash
    Evidence: .omo/evidence/task-5-adaptive-unchanged.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-5-reduced-grid.txt` — output generazione grid ridotta
  - [ ] `task-5-adaptive-unchanged.txt` — output grid adattiva invariata

  **Commit**: YES (groups with 4, 6)
  - Message: `feat(qp): add _gen_reduced_grid_from_prior() for NLF fast-path`
  - Files: `src/spot_control/spot_control/wbc_qp_controller.py`
  - Pre-commit: `colcon build --packages-select spot_control`

- [x] 6. `wbc_coordinator.py` — Quality Monitor NLF delta + LOOKAT weighting

  **What to do**:
  - Aggiungere metodo `_check_nlf_delta(torso_yolo)` come metodo del coordinator (NON dentro la classe `_QualityMonitor`):
    ```python
    def _check_nlf_delta(self, torso_yolo: np.ndarray) -> tuple:
        if not self._nlf_prior_valid():
            return ('HIGH', None)
        nlf_center = self._torso_center_from_prior()
        delta = np.linalg.norm(torso_yolo[:3] - nlf_center[:3])
        if delta < self._nlf_coherence_threshold:   # 0.15
            return ('HIGH', delta)
        elif delta < self._nlf_divergence_threshold: # 0.30
            return ('MEDIUM', delta)
        else:
            return ('LOW', delta)
    ```
  - Nel punto in cui si calcola il target LOOKAT (durante PRE_APPROACH e APPROACHING):
    ```python
    if self._nlf_prior_valid():
        quality_label, delta = self._check_nlf_delta(torso_yolo)
        if quality_label == 'HIGH':
            target = 0.7 * nlf_target + 0.3 * yolo_target
        elif quality_label == 'MEDIUM':
            target = 0.5 * nlf_target + 0.5 * yolo_target
        else:  # LOW
            target = yolo_target  # YOLO 100%, possibile ri-trigger NLF
    else:
        target = yolo_target  # fallback: comportamento attuale
    ```
  - Leggere `nlf_coherence_threshold` e `nlf_divergence_threshold` da parametri YAML
  - `torso_yolo` proviene dal subscriber `/laying_human/body_center` (già esistente) o da `/human_pose/points_3d` (nuovo subscriber del Task 3)
  - Se `quality_label == 'LOW'` per >30 tick consecutivi → log warning "possible patient movement"

  **Must NOT do**:
  - NON modificare la classe `_QualityMonitor` interna
  - NON cambiare la matematica del LOOKAT (solo il target blending)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Sensor fusion logic, threshold tuning, integrazione con LOOKAT esistente
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 5, 7)
  - **Blocks**: Task 7
  - **Blocked By**: Task 3

  **References**:
  - `src/spot_control/spot_control/wbc_coordinator.py:47-130` — classe `_QualityMonitor` (NON modificare)
  - `src/spot_control/spot_control/wbc_coordinator.py:1050-1120` — LOOKAT goal publishing in PRE_APPROACH
  - `src/spot_control/spot_control/wbc_coordinator.py:800-900` — APPROACHING logic dove il LOOKAT è attivo
  - `.omo/plans/nlf-prior-strategy.md:256-273` — `_nlf_prior_valid()` gate

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: HIGH coherence — LOOKAT blending NLF 70% / YOLO 30%
    Tool: Bash (python3 -c)
    Preconditions: _nlf_prior_valid() == True, torso_yolo a 5cm dal prior NLF
    Steps:
      1. Chiama _check_nlf_delta(torso_yolo_mock)
      2. Verifica che restituisca ('HIGH', delta) con delta ≈ 0.05
      3. Verifica che il target LOOKAT sia 0.7*nlf + 0.3*yolo
    Expected Result: HIGH coherence, blending corretto
    Failure Indicators: Label sbagliata, blending invertito, eccezione
    Evidence: .omo/evidence/task-6-high-coherence.txt

  Scenario: LOW coherence — YOLO 100%, warning loggato
    Tool: Bash (python3 -c)
    Preconditions: _nlf_prior_valid() == True, torso_yolo a 50cm dal prior NLF
    Steps:
      1. Chiama _check_nlf_delta(torso_yolo_mock)
      2. Verifica che restituisca ('LOW', delta) con delta ≈ 0.50
      3. Verifica che il target LOOKAT sia 100% YOLO
    Expected Result: LOW coherence, YOLO only
    Failure Indicators: NLF ancora usato nel blending nonostante divergenza
    Evidence: .omo/evidence/task-6-low-coherence.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-6-high-coherence.txt` — output test HIGH
  - [ ] `task-6-low-coherence.txt` — output test LOW

  **Commit**: YES (groups with 4, 5)
  - Message: `feat(wbc): add NLF quality monitor delta and LOOKAT blending`
  - Files: `src/spot_control/spot_control/wbc_coordinator.py`
  - Pre-commit: `colcon build --packages-select spot_control`

- [x] 7. Test — pytest per prior logic, trigger, PRE_APPROACH bifurcation

  **What to do**:
  - Creare `src/spot_control/test/test_nlf_prior_gate.py`:
    - Test `_nlf_prior_valid()`: None → False, 'timeout' → False, len!=24 → False, <4 valid torso → False, 24 valid → True
    - Test `_nlf_prior_valid()` edge cases: NaN joints, PoseArray vuoto, prior con 12 giunti
    - Test `_torso_center_from_prior()`: 4 giunti validi → media corretta, tutti NaN → zeros(3)
  - Creare `src/spot_control/test/test_pre_approach_bifurcation.py`:
    - Test `_tick_pre_approach()`: NLF valid → fast path (goal pubblicato, safety gate 1s)
    - Test `_tick_pre_approach()`: NLF invalid → legacy path chiamato (`_tick_pre_approach_legacy`)
  - Creare `src/spot_perception/test/test_nlf_trigger.py`:
    - Test `_cb_trigger()`: modello caricato → pubblica prior su `/exposure/nlf_prior`
    - Test `_cb_trigger()`: modello NON caricato → log warning, nessuna pubblicazione
  - Ogni test file usa mock per ROS2 node, subscriber, publisher, TF buffer
  - Usare `unittest.mock` per isolare le dipendenze ROS2

  **Must NOT do**:
  - NON testare l'intero FSM (solo i nuovi metodi)
  - NON richiedere ROS2 running per i test (usare mock)

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Test infrastructure da zero, mocking ROS2, 3 file di test
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 5, 6)
  - **Blocks**: None (last task before Final Verification)
  - **Blocked By**: Tasks 4, 5, 6

  **References**:
  - `src/spot_control/test/` — directory test esistente (pattern)
  - `src/spot_control/spot_control/wbc_coordinator.py:256-273` — `_nlf_prior_valid()` implementazione
  - `src/spot_perception/spot_perception/nlf_skeleton.py:330-420` — `_cb_color` per pattern mock
  - `src/spot_control/spot_control/wbc_coordinator.py:1050-1120` — PRE_APPROACH da testare
  - `.omo/plans/nlf-prior-strategy.md:256-280` — specifica `_nlf_prior_valid()`

  **Acceptance Criteria**:

  **QA Scenarios**:

  ```
  Scenario: All tests pass
    Tool: Bash (colcon test)
    Preconditions: workspace built
    Steps:
      1. colcon test --packages-select spot_control spot_perception
      2. colcon test-result --verbose
      3. Verifica che tutti i test passino (0 failures, 0 errors)
    Expected Result: All tests green
    Failure Indicators: Test failures, import errors, mock setup fallito
    Evidence: .omo/evidence/task-7-all-tests-pass.txt

  Scenario: Edge cases covered
    Tool: Bash (python3 -m pytest -v)
    Preconditions: test files written
    Steps:
      1. python3 -m pytest src/spot_control/test/test_nlf_prior_gate.py -v
      2. Verifica che copra: None, timeout, len!=24, <4 torso, valid
    Expected Result: ≥5 test cases pass
    Failure Indicators: Edge case non testato, test skipping
    Evidence: .omo/evidence/task-7-edge-cases.txt
  ```

  **Evidence to Capture**:
  - [ ] `task-7-all-tests-pass.txt` — output `colcon test-result --verbose`
  - [ ] `task-7-edge-cases.txt` — output `pytest -v` per prior gate

  **Commit**: YES
  - Message: `test: add pytest for NLF prior gate, PRE_APPROACH bifurcation, and trigger callback`
  - Files: `src/spot_control/test/test_nlf_prior_gate.py`, `src/spot_control/test/test_pre_approach_bifurcation.py`, `src/spot_perception/test/test_nlf_trigger.py`
  - Pre-commit: `colcon test --packages-select spot_control spot_perception`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. Verify: `_nlf_prior_valid()` gate exists and controls all branches. PRE_APPROACH has safety gate (≥1s). Grid generation has reduced + legacy paths. `/exposure/nlf_prior` topic publishes on trigger only. Fallback is total (no partial optimizations). Zero lines removed from legacy behavior.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Run `colcon build --packages-select spot_control spot_perception z1_vision`. Check all changed files for: unused imports, `except: pass`, hardcoded magic numbers, missing parameter declarations. Verify all 4 new params declared in `__init__`. Check `_nlf_prior_valid()` is the ONLY gate.
  Output: `Build [PASS/FAIL] | Lint [PASS/FAIL] | Tests [N pass/N fail] | Files [N clean/N issues] | VERDICT`

- [x] F3. **Real Manual QA** — `unspecified-high`
  Start from clean build. Execute EVERY QA scenario from EVERY task. Test cross-task integration: NLF trigger → prior published → coordinator receives → PRE_APPROACH fast path → grid reduced. Test fallback: no prior → legacy PRE_APPROACH → adaptive grid unchanged. Test edge cases: timeout, model not loaded, NaN joints.
  Output: `Scenarios [N/N pass] | Integration [N/N] | Edge Cases [N tested] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff. Verify 1:1 — everything in spec was built, nothing beyond spec was built. Check "Must NOT do" compliance: `body_search_scanner.py` untouched, `_QualityMonitor` internals unchanged, legacy code preserved. Flag unaccounted changes.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

- **1-3 (Wave 1)**: `feat(nlf): add trigger, prior publisher, coordinator subscription, config params` — `nlf_skeleton.py`, `wbc_coordinator.py`, `wbc_params.yaml`, `body_search_params.yaml`, `colcon build`
- **4-6 (Wave 2)**: `feat(wbc): bifurcate PRE_APPROACH, reduced grid, quality monitor NLF delta` — `wbc_coordinator.py`, `wbc_qp_controller.py`, `colcon build`
- **7 (Wave 2)**: `test: add pytest for NLF prior gate, PRE_APPROACH, trigger callback` — `test_nlf_prior_gate.py`, `test_pre_approach_bifurcation.py`, `test_nlf_trigger.py`, `colcon test`

---

## Success Criteria

### Verification Commands
```bash
# Build
colcon build --packages-select spot_control spot_perception z1_vision

# Tests
colcon test --packages-select spot_control spot_perception
colcon test-result --verbose

# NLF trigger (manuale)
ros2 topic pub /nlf/trigger std_msgs/msg/Bool "data: true" -1
ros2 topic echo /exposure/nlf_prior --once

# Parametri
ros2 param get /wbc_coordinator nlf_timeout
ros2 param get /wbc_coordinator nlf_coherence_threshold
```

### Final Checklist
- [ ] All "Must Have" present: fallback totale, YOLO 15 FPS, backward compatibility
- [ ] All "Must NOT Have" absent: no NLF in SEARCHING/APPROACHING, no QualityMonitor class changes
- [ ] `_nlf_prior_valid()` is the single gate for ALL conditional branches
- [ ] PRE_APPROACH fast path has minimum 1s safety gate
- [ ] NLF prior published in `orbbec_color_optical_frame`, transformed to `odom` in coordinator
- [ ] All 3 test files pass (`colcon test`)
- [ ] `body_search_scanner.py` untouched
- [ ] `_tick_pre_approach_legacy()` preserves exact 6 June 2026 behavior
