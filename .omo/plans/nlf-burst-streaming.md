# NLF Burst Streaming — Da One-Shot a Multi-Frame con EMA

## TL;DR

> **Quick Summary**: Trasforma il trigger NLF da one-shot singolo frame a burst multi-frame con accumulo EMA, producendo uno skeleton SMPL raffinato (2 detection valide o timeout 30s), con LOCKING bloccante fino al completamento.
>
> **Deliverables**:
> - `nlf_skeleton.py`: burst state machine (_burst_active, conteggio, auto-finish, publish soppresso)
> - `wbc_coordinator.py`: LOCKING bloccante + timeout 10→30s
> - `nlf_params.yaml` + `wbc_params.yaml`: nuovi parametri burst
> - `spot_perception.launch.py`: condizione mancante su nlf_skeleton_node
>
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 2 waves
> **Critical Path**: Task 1 → Task 3 → Task 4

---

## Context

### Original Request
L'utente vuole che NLF, quando triggerato, esegua un burst di inferenza multi-frame invece di un singolo one-shot. Durante il burst, NLF accumula detection via EMA smoothing (codice già esistente ma irraggiungibile perché `_streaming_paused` non viene mai messo a `False`). Dopo 2 detection valide (o timeout 30s), pubblica lo skeleton raffinato e si auto-ripausa.

### Interview Summary
**Key Discussions**:
- NLF inference: ~7 secondi per frame su CPU — il burst di 2 frame richiede ~14-17s
- LOCKING bloccante: il coordinator aspetta NLF prima di passare a PRE_APPROACH
- "Detection valida": ≥1 persona lying (torso angle > 65°) con SPINE1/2/3/PELVIS non-NaN
- 1 sola detection → pubblica raw (no EMA); 0 detection → PoseArray vuoto
- Nessun publish su `/human_pose/points_3d` durante burst (soppresso via `_burst_active`)
- Nessuna modifica ai consumer, nessuna modifica EMA, nessuna GPU

**Research Findings**:
- `_streaming_paused` inizializzato `True` (line 101), mai messo a `False` in tutto il codice
- EMA smoothing, target selection, multi-person tracking: codice scritto a linee 347-456 ma irraggiungibile
- `_cb_trigger` attuale è one-shot (linee 210-264)
- Coordinator ha già infrastruttura NLF prior completa (`_cb_nlf_prior`, `_nlf_prior_valid`, TF transform)
- `spot_perception.launch.py`: bug — `nlf_skeleton_node` senza `IfCondition` (sempre lanciato)

### Metis Review
**Identified Gaps** (addressed):
- Timing mismatch (burst finisce dopo PRE_APPROACH) → risolto con LOCKING bloccante
- Thread safety su `_streaming_paused` → documentato SingleThreadedExecutor
- Re-entrancy (doppio trigger) → flag `_burst_active`
- `process_every_n_frames=1` crea backlog executor → throttle durante burst
- ByteTrack ID change tra frame → accettato come rischio noto (2 frame, probabilità bassa)

---

## Work Objectives

### Core Objective
Trasformare il trigger NLF da one-shot a burst multi-frame con accumulo EMA, producendo uno skeleton SMPL raffinato a 2 detection valide (o timeout 30s), bloccando LOCKING fino al completamento.

### Concrete Deliverables
- `src/spot_perception/spot_perception/nlf_skeleton.py` — burst state machine
- `src/spot_control/spot_control/wbc_coordinator.py` — LOCKING bloccante
- `src/spot_perception/config/nlf_params.yaml` — parametri burst
- `src/spot_control/config/wbc_params.yaml` — nlf_timeout
- `src/spot_perception/launch/spot_perception.launch.py` — condizione mancante

### Definition of Done
- [ ] `colcon build --packages-select spot_perception spot_control` → SUCCESS
- [ ] `colcon test --packages-select spot_perception spot_control` → ALL PASS
- [ ] Burst produce skeleton raffinato dopo 2 detection valide
- [ ] Burst timeout a 30s con fallback corretto (1→raw, 0→vuoto)
- [ ] `/human_pose/points_3d` non riceve messaggi durante burst attivo
- [ ] `perception_backend:=yolo` non lancia `nlf_skeleton_node`

### Must Have
- `_burst_active` flag per sopprimere publish su `/human_pose/points_3d`
- Conteggio detection valide con stop a 2
- Timer di sicurezza 30s
- `_finish_burst()` che pubblica su `/exposure/nlf_prior`
- LOCKING bloccante: attende `_nlf_prior_valid() OR _nlf_prior == 'timeout'`
- Condizione su `nlf_skeleton_node` nel launch file

### Must NOT Have (Guardrails)
- Modifiche all'algoritmo EMA (alpha, pesatura, outlier rejection)
- Modifiche ai consumer (`posture_classifier`, `laying_human_detector`, `person_tracking`)
- Modifiche a `nlf_torso_tracker.py`
- GPU/CUDA optimization
- Nuovi timer ROS in `nlf_skeleton.py` (resta callback-driven)
- Publish su `/human_pose/points_3d` durante burst attivo
- `Bool(False)` esterno che interrompe burst attivo

---

## Verification Strategy (MANDATORY)

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed.

### Test Decision
- **Infrastructure exists**: YES (pytest in `spot_perception/test/`, `spot_control/test/`)
- **Automated tests**: Tests-after (pytest)
- **Framework**: pytest (colcon test)
- **If TDD**: N/A — tests-after approach

### QA Policy
Every task MUST include agent-executed QA scenarios.
Evidence saved to `.omo/evidence/task-{N}-{scenario-slug}.{ext}`.

- **API/Backend**: Use Bash (ros2 topic pub/echo, colcon test)
- **Library/Module**: Use Bash (python3 -c import, pytest)

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — config + launch fix, MAX PARALLEL):
├── Task 1: nlf_params.yaml + wbc_params.yaml [quick]
└── Task 2: spot_perception.launch.py condition [quick]

Wave 2 (After Wave 1 — core implementation):
├── Task 3: nlf_skeleton.py burst state machine [deep]
└── Task 4: wbc_coordinator.py blocking LOCKING [deep]

Wave FINAL (After ALL tasks — 4 parallel reviews):
├── Task F1: Plan Compliance Audit (oracle)
├── Task F2: Code Quality Review (unspecified-high)
├── Task F3: Real Manual QA (unspecified-high)
└── Task F4: Scope Fidelity Check (deep)
```

Critical Path: Task 1 → Task 3 → Task 4
Parallel Speedup: ~40% vs sequential (Wave 1 parallel, Wave 2 parallel)

### Dependency Matrix

- **1**: - - 3, 4
- **2**: - - (none, independent)
- **3**: 1 - 4
- **4**: 1 - F1-F4

### Agent Dispatch Summary

- **1**: **2** - T1 → `quick`, T2 → `quick`
- **2**: **2** - T3 → `deep`, T4 → `deep`
- **FINAL**: **4** - F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [x] 1. **nlf_params.yaml + wbc_params.yaml** — Aggiungere parametri burst e allineare timeout

  **What to do**:
  - In `src/spot_perception/config/nlf_params.yaml`:
    - Aggiungere `burst_min_detections: 2` (riga dopo `process_every_n_frames`)
    - Aggiungere `burst_timeout_s: 30.0`
    - Aggiungere `burst_throttle_frames: 10` (min frame skip durante burst per evitare backlog executor)
  - In `src/spot_control/config/wbc_params.yaml`:
    - Modificare `nlf_timeout: 10.0` → `nlf_timeout: 30.0`
  - Verificare che `nlf_params.yaml` abbia `process_every_n_frames: 1` (confermato — il throttling avviene in codice durante burst)

  **Must NOT do**:
  - Non toccare altri parametri YAML
  - Non aggiungere sezioni o commenti non richiesti

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: file di configurazione, modifiche puntuali, nessuna logica complessa
  - **Skills**: none
    - Reason: YAML editing non richiede skill specializzate

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Task 2)
  - **Blocks**: Task 3, Task 4
  - **Blocked By**: None (can start immediately)

  **Acceptance Criteria**:
  - [ ] `nlf_params.yaml` contiene `burst_min_detections: 2`
  - [ ] `nlf_params.yaml` contiene `burst_timeout_s: 30.0`
  - [ ] `nlf_params.yaml` contiene `burst_throttle_frames: 10`
  - [ ] `wbc_params.yaml` contiene `nlf_timeout: 30.0`
  - [ ] `git diff --stat` mostra solo i 2 file YAML modificati

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: YAML parameters are valid and loadable
    Tool: Bash (python3)
    Preconditions: colcon build completato
    Steps:
      1. python3 -c "import yaml; yaml.safe_load(open('src/spot_perception/config/nlf_params.yaml'))"
      2. python3 -c "import yaml; yaml.safe_load(open('src/spot_control/config/wbc_params.yaml'))"
      3. grep "burst_min_detections" src/spot_perception/config/nlf_params.yaml
      4. grep "burst_timeout_s" src/spot_perception/config/nlf_params.yaml
      5. grep "nlf_timeout: 30.0" src/spot_control/config/wbc_params.yaml
    Expected Result: All YAML parse without errors. Grep finds all 3 new/updated params.
    Failure Indicators: YAML parse error, missing param, wrong value
    Evidence: .omo/evidence/task-1-yaml-params.txt

  Scenario: Invalid YAML is rejected
    Tool: Bash (python3)
    Preconditions: nlf_params.yaml intentionally malformed (revert after test)
    Steps:
      1. echo "invalid: : yaml" >> src/spot_perception/config/nlf_params.yaml
      2. python3 -c "import yaml; yaml.safe_load(open('src/spot_perception/config/nlf_params.yaml'))" 2>&1
      3. git checkout src/spot_perception/config/nlf_params.yaml
    Expected Result: Step 2 produces YAMLError. Step 3 restores file.
    Evidence: .omo/evidence/task-1-yaml-error.txt
  ```

  **Commit**: YES
  - Message: `feat(nlf): add burst parameters, extend nlf_timeout to 30s`
  - Files: `src/spot_perception/config/nlf_params.yaml`, `src/spot_control/config/wbc_params.yaml`

- [x] 2. **spot_perception.launch.py** — Aggiungere condizione mancante su nlf_skeleton_node

  **What to do**:
  - In `src/spot_perception/launch/spot_perception.launch.py`, al `Node` di `nlf_skeleton` (intorno a linea 136-142):
    - Aggiungere `condition=IfCondition(PythonExpression(['"', perception_backend, '" == "nlf"']))`
  - Il nodo YOLO (`yolo_skeleton_node_orbbec`) ha già `condition=IfCondition(PythonExpression(['"', perception_backend, '" == "yolo"']))` — lascialo invariato
  - Verifica sintassi: il `condition` deve essere un parametro del costruttore `Node()`, non un nodo separato

  **Must NOT do**:
  - Non toccare la condizione del nodo YOLO
  - Non aggiungere condizioni ad altri nodi nel launch file
  - Non modificare la `TimerAction` che controlla il delay di avvio

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: modifica puntuale (1 riga), pattern già presente nel file (YOLO node)
  - **Skills**: none

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Task 1)
  - **Blocks**: None
  - **Blocked By**: None (can start immediately)

  **References**:
  - `src/spot_perception/launch/spot_perception.launch.py:125-135` — YOLO node con `condition=` (pattern da copiare)

  **Acceptance Criteria**:
  - [ ] `nlf_skeleton_node` ha `condition=IfCondition(PythonExpression(['"', perception_backend, '" == "nlf"']))`
  - [ ] `colcon build --packages-select spot_perception` → SUCCESS
  - [ ] Launch con `perception_backend:=yolo` → `ros2 node list | grep nlf_skeleton` restituisce VUOTO
  - [ ] Launch con `perception_backend:=nlf` → `ros2 node list | grep nlf_skeleton` restituisce `/nlf_skeleton`

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: nlf_skeleton_node NOT launched when perception_backend:=yolo
    Tool: Bash (ros2 launch + ros2 node list)
    Preconditions: colcon build completato, ROS2 environment sourced
    Steps:
      1. ros2 launch spot_perception spot_perception.launch.py perception_backend:=yolo use_orbbec_driver:=false &
      2. sleep 8  # wait for TimerAction
      3. ros2 node list | grep nlf_skeleton
      4. kill %1
    Expected Result: Step 3 returns empty (no nlf_skeleton node running)
    Failure Indicators: "nlf_skeleton" appears in node list
    Evidence: .omo/evidence/task-2-yolo-no-nlf.txt

  Scenario: nlf_skeleton_node IS launched when perception_backend:=nlf
    Tool: Bash (ros2 launch + ros2 node list)
    Preconditions: colcon build completato, ROS2 environment sourced
    Steps:
      1. ros2 launch spot_perception spot_perception.launch.py perception_backend:=nlf use_orbbec_driver:=false &
      2. sleep 8
      3. ros2 node list | grep nlf_skeleton
      4. kill %1
    Expected Result: Step 3 returns "/nlf_skeleton"
    Failure Indicators: "nlf_skeleton" not found in node list
    Evidence: .omo/evidence/task-2-nlf-yes-nlf.txt
  ```

  **Commit**: YES (groups with Task 1)
  - Message: `fix(launch): add condition on nlf_skeleton_node for perception_backend`
  - Files: `src/spot_perception/launch/spot_perception.launch.py`

- [x] 3. **nlf_skeleton.py** — Burst state machine: trigger → burst → EMA → auto-finish

  **What to do**:
  - **In `__init__`** (line ~95-170):
    - Aggiungere `self._burst_active = False`
    - Aggiungere `self._burst_detection_count = 0`
    - Aggiungere `self._burst_start_time = None`
    - Leggere nuovi parametri: `burst_min_detections`, `burst_timeout_s`, `burst_throttle_frames`
  - **In `_cb_trigger`** (line 210-264) — REDESIGN:
    - `Bool(False)`: se `_burst_active` → ignora (return). Altrimenti comportamento invariato (set `_streaming_paused = True`)
    - `Bool(True)`: se `_burst_active` → log warning, return (previeni re-entrancy)
    - Altrimenti: `_streaming_paused = False`, `_burst_active = True`, `_burst_detection_count = 0`, `_burst_start_time = time.time()`, reset `_smoothed_kp = {}`
    - **NON eseguire più** l'inferenza one-shot qui — il burst usa `_cb_color`
    - Log: "NLF burst started (target: 2 detections, timeout: 30s)"
  - **In `_cb_color`** (line 347-456) — AGGIUNTE in coda al path di streaming:
    - Dopo il target selection (line ~429), se `_burst_active`:
      - Se target trovato E `_target_id` è valido E i 4 joint torso sono non-NaN:
        - `_burst_detection_count += 1`
        - Log: f"NLF burst: detection {_burst_detection_count}/{burst_min_detections}"
      - Se `_burst_detection_count >= burst_min_detections` → `_finish_burst()`
      - Se `time.time() - _burst_start_time > burst_timeout_s` → `_finish_burst()`
    - **SOPPRIMERE publish**: se `_burst_active`, saltare `_publish_target_pose()` e `_publish_all_markers()` e `_publish_mesh()`
  - **Throttle durante burst**: all'inizio di `_cb_color`, se `_burst_active` e `_frame_count % burst_throttle_frames != 0` → return (per prevenire backlog executor con process_every_n_frames=1)
  - **Nuovo metodo `_finish_burst()`**:
    - `_streaming_paused = True`, `_burst_active = False`
    - Se `_burst_detection_count >= 2`: prendi `_smoothed_kp[_target_id]` (EMA accumulato), costruisci `PoseArray` (24 poses, frame `orbbec_color_optical_frame`, stamp dall'ultimo `_last_color_msg`), pubblica su `pub_nlf_prior` (`/exposure/nlf_prior`)
    - Se `_burst_detection_count == 1`: prendi l'ultima detection raw da `_latest_raw_detection` (da salvare in `_cb_color`), pubblica raw su `pub_nlf_prior`
    - Se `_burst_detection_count == 0`: pubblica `PoseArray` vuoto (0 poses) su `pub_nlf_prior`
    - Log riepilogativo: "NLF burst finished: N detections in T seconds"

  **Must NOT do**:
  - Non modificare l'algoritmo EMA (alpha, pesatura, outlier rejection)
  - Non aggiungere nuovi subscriber o publisher
  - Non aggiungere timer ROS (resta callback-driven: `_cb_color` + `_cb_trigger`)
  - Non toccare `nlf_torso_tracker.py`
  - Non cambiare il formato del `PoseArray` pubblicato su `/exposure/nlf_prior` (24 poses, frame `orbbec_color_optical_frame`)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: redesign di un meccanismo core (one-shot → burst), multiple control flow changes, stato condiviso tra callback, soppressione condizionale publish. Richiede comprensione approfondita del flusso esistente.
  - **Skills**: none
    - Reason: Python ROS2 standard, nessuna libreria esterna
  - **Skills Evaluated but Omitted**:
    - `ros2`: non necessario — modifiche a un singolo nodo, nessuna orchestrazione multi-nodo

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Task 4)
  - **Blocks**: None
  - **Blocked By**: Task 1 (param names)

  **References** (CRITICAL):
  - `src/spot_perception/spot_perception/nlf_skeleton.py:95-101` — `__init__` early-init guards (aggiungere `_burst_*` qui)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:210-264` — `_cb_trigger` one-shot (DA REDESIGNARE per burst)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:249-262` — PoseArray building pattern per `/exposure/nlf_prior` (DA RIUSARE in `_finish_burst`)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:347-456` — `_cb_color` streaming path (aggiungere logica burst in coda)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:356` — `if self._streaming_paused: return` gate (ora si sblocca durante burst)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:372-399` — EMA smoothing (già funzionante, usato così com'è)
  - `src/spot_perception/spot_perception/nlf_skeleton.py:401-429` — target selection + lying detection (usato per conteggio "valid detection")
  - `src/spot_perception/spot_perception/nlf_skeleton.py:118` — `self._process_every` da `process_every_n_frames` (throttle override durante burst)
  - `src/spot_perception/test/test_nlf_trigger.py` — test esistenti per `_cb_trigger` (DA ESTENDERE)

  **Acceptance Criteria**:
  - [ ] `_burst_active` esiste in `__init__` e viene letto/scritto solo in callback ROS
  - [ ] `_cb_trigger(True)` attiva il burst, non esegue più one-shot
  - [ ] `_cb_trigger(False)` viene ignorato durante burst attivo
  - [ ] `_cb_color` sopprime publish su `/human_pose/points_3d` quando `_burst_active`
  - [ ] `_cb_color` incrementa `_burst_detection_count` per ogni detection valida (lying + torso non-NaN)
  - [ ] `_finish_burst()` pubblica su `/exposure/nlf_prior` con header corretto
  - [ ] Burst si ferma dopo `burst_min_detections` detection valide
  - [ ] Burst si ferma dopo `burst_timeout_s` secondi
  - [ ] `colcon build --packages-select spot_perception` → SUCCESS
  - [ ] `colcon test --packages-select spot_perception` → ALL PASS

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Burst publishes refined prior after 2 valid detections (simulated)
    Tool: Bash (python3 unit test via pytest)
    Preconditions: test_nlf_burst.py creato con Mock NLF model
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_burst_two_detections -v
    Expected Result: Test passes. Verify that after 2 mock detections, _finish_burst publishes PoseArray with 24 poses.
    Failure Indicators: Test failure, wrong number of poses published, publish to wrong topic
    Evidence: .omo/evidence/task-3-burst-two-detections.txt

  Scenario: Burst times out at 30s and publishes fallback (0 detections)
    Tool: Bash (pytest)
    Preconditions: test_nlf_burst.py
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_burst_timeout_zero_detections -v
    Expected Result: Test passes. After 30s timeout with 0 valid detections, _finish_burst publishes empty PoseArray (0 poses).
    Evidence: .omo/evidence/task-3-burst-timeout-zero.txt

  Scenario: Burst with 1 detection publishes raw result (no EMA)
    Tool: Bash (pytest)
    Preconditions: test_nlf_burst.py
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_burst_one_detection -v
    Expected Result: Test passes. Raw (non-EMA) single detection published.
    Evidence: .omo/evidence/task-3-burst-one-detection.txt

  Scenario: _cb_trigger(False) ignored during active burst
    Tool: Bash (pytest)
    Preconditions: test_nlf_burst.py
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_ignore_pause_during_burst -v
    Expected Result: _burst_active remains True after receiving Bool(False). Burst continues.
    Evidence: .omo/evidence/task-3-ignore-pause.txt

  Scenario: Double trigger (re-entrancy) is prevented
    Tool: Bash (pytest)
    Preconditions: test_nlf_burst.py
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_double_trigger_prevented -v
    Expected Result: Second trigger during active burst logs warning, does not reset counters.
    Evidence: .omo/evidence/task-3-double-trigger.txt

  Scenario: /human_pose/points_3d receives ZERO messages during burst
    Tool: Bash (pytest)
    Preconditions: test_nlf_burst.py with mock publisher spy
    Steps:
      1. python3 -m pytest src/spot_perception/test/test_nlf_burst.py::test_no_publish_during_burst -v
    Expected Result: pub_pose.publish() is never called while _burst_active is True.
    Evidence: .omo/evidence/task-3-no-publish-burst.txt
  ```

  **Commit**: YES
  - Message: `feat(nlf): burst streaming with EMA accumulation and auto-finish`
  - Files: `src/spot_perception/spot_perception/nlf_skeleton.py`, `src/spot_perception/test/test_nlf_burst.py`

- [x] 4. **wbc_coordinator.py** — LOCKING bloccante: attendi NLF prima di PRE_APPROACH

  **What to do**:
  - In `_tick_locking()` (intorno a linea 849-850 dove avviene la transizione a PRE_APPROACH):
    - **Prima**: `if len(self._search_lock_buffer) >= self._search_lock_samples and self._ik_done_received:`
    - **Dopo**: aggiungere `and (self._nlf_prior_valid() or self._nlf_prior == 'timeout')`
    - Se `_nlf_prior` è ancora `None` (né valido né timeout) → NON transitare, resta in LOCKING
  - Timeout (linea 879): `if elapsed > 10.0:` → `if elapsed > 30.0:`
  - Rimuovere il `Bool(False)` publish su timeout NLF (linee 883-886) — NLF si auto-gestisce l'auto-pausa
    - Tenere solo il log: "NLF timeout (30s) — proceeding without prior"
  - Il metodo `_nlf_prior_valid()` (linee 1332-1341) e `_torso_center_from_prior()` (linee 1343-1346) rimangono invariati

  **Must NOT do**:
  - Non cambiare la logica di raccolta campioni `_search_lock_buffer` (5 campioni)
  - Non cambiare il gate `_ik_done_received`
  - Non modificare `_tick_pre_approach()` o `_tick_pre_approach_legacy()`
  - Non modificare `_cb_nlf_prior` (linee 1281-1327)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: modifica alla macchina a stati FSM (condizione di transizione), impatto su timing dell'intera missione. Richiede comprensione del coordinator lifecycle.
  - **Skills**: none
    - Reason: Python ROS2 standard

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Task 3)
  - **Blocks**: None
  - **Blocked By**: Task 1 (param nlf_timeout)

  **References** (CRITICAL):
  - `src/spot_control/spot_control/wbc_coordinator.py:849-850` — transizione LOCKING → PRE_APPROACH (DA MODIFICARE)
  - `src/spot_control/spot_control/wbc_coordinator.py:876-887` — timeout NLF + Bool(False) publish (DA MODIFICARE)
  - `src/spot_control/spot_control/wbc_coordinator.py:1332-1341` — `_nlf_prior_valid()` (INVARIATO, usato come gate)
  - `src/spot_control/spot_control/wbc_coordinator.py:1281-1327` — `_cb_nlf_prior` (INVARIATO)
  - `src/spot_control/test/test_nlf_prior_gate.py` — test esistenti per `_nlf_prior_valid()` (DA ESTENDERE)

  **Acceptance Criteria**:
  - [ ] LOCKING → PRE_APPROACH richiede `_nlf_prior_valid() OR _nlf_prior == 'timeout'`
  - [ ] `nlf_timeout` nel codice usa 30 secondi (non più 10)
  - [ ] Coordinator NON pubblica `Bool(False)` su `/nlf/trigger` al timeout
  - [ ] Se `_nlf_prior` è `None` e non ancora timeout → LOCKING non transita
  - [ ] `colcon build --packages-select spot_control` → SUCCESS
  - [ ] `colcon test --packages-select spot_control` → ALL PASS

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: LOCKING blocks until NLF prior is valid
    Tool: Bash (pytest)
    Preconditions: test_nlf_prior_gate.py extended
    Steps:
      1. python3 -m pytest src/spot_control/test/test_nlf_prior_gate.py::test_locking_blocks_until_prior_valid -v
    Expected Result: Test verifies that transition condition requires _nlf_prior_valid() OR timeout.
    Evidence: .omo/evidence/task-4-locking-blocks.txt

  Scenario: LOCKING transitions on NLF timeout
    Tool: Bash (pytest)
    Preconditions: test_nlf_prior_gate.py extended
    Steps:
      1. python3 -m pytest src/spot_control/test/test_nlf_prior_gate.py::test_locking_proceeds_on_timeout -v
    Expected Result: When _nlf_prior == 'timeout', transition proceeds even without valid prior.
    Evidence: .omo/evidence/task-4-locking-timeout.txt

  Scenario: Coordinator does NOT publish Bool(False) on NLF timeout
    Tool: Bash (pytest)
    Preconditions: test_nlf_prior_gate.py extended
    Steps:
      1. python3 -m pytest src/spot_control/test/test_nlf_prior_gate.py::test_no_false_publish_on_timeout -v
    Expected Result: Pub to /nlf/trigger is NOT called when timeout fires.
    Evidence: .omo/evidence/task-4-no-false-publish.txt

  Scenario: nlf_timeout reads 30.0 from parameters
    Tool: Bash (grep)
    Preconditions: wbc_params.yaml updated (Task 1)
    Steps:
      1. grep "nlf_timeout" src/spot_control/config/wbc_params.yaml
      2. grep "30.0" src/spot_control/spot_control/wbc_coordinator.py | head -5
    Expected Result: YAML has 30.0. Coordinator code uses parameter or hardcoded 30.
    Evidence: .omo/evidence/task-4-timeout-30.txt
  ```

  **Commit**: YES
  - Message: `feat(wbc): block LOCKING transition until NLF burst completes`
  - Files: `src/spot_control/spot_control/wbc_coordinator.py`, `src/spot_control/test/test_nlf_prior_gate.py`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists. For each "Must NOT Have": search codebase for forbidden patterns. Check evidence files exist in `.omo/evidence/`.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Run `colcon build --packages-select spot_perception spot_control`. Run `colcon test --packages-select spot_perception spot_control`. Review all changed files for: dead code, unused imports, `except: pass`, print debugs.
  Output: `Build [PASS/FAIL] | Lint [PASS/FAIL] | Tests [N pass/N fail] | VERDICT`

- [x] F3. **Real Manual QA** — `unspecified-high`
  Execute EVERY QA scenario from EVERY task. Test cross-task integration. Test edge cases: 0 detections, 1 detection, timeout, re-entrancy, YOLO coexistence.
  Output: `Scenarios [N/N pass] | Integration [N/N] | Edge Cases [N tested] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff. Verify 1:1 — everything in spec was built, nothing beyond spec was built. Check "Must NOT do" compliance.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

- **1**: `feat(nlf): add burst parameters` — nlf_params.yaml, wbc_params.yaml
- **2**: `fix(launch): add condition on nlf_skeleton_node` — spot_perception.launch.py
- **3**: `feat(nlf): burst streaming with EMA accumulation` — nlf_skeleton.py + test_nlf_burst.py
- **4**: `feat(wbc): block LOCKING until NLF burst completes` — wbc_coordinator.py

---

## Success Criteria

### Verification Commands
```bash
colcon build --packages-select spot_perception spot_control  # Expected: SUCCESS
colcon test --packages-select spot_perception spot_control    # Expected: ALL PASS
```

### Final Checklist
- [ ] All "Must Have" present
- [ ] All "Must NOT Have" absent
- [ ] All tests pass
- [ ] Burst produces refined prior on `/exposure/nlf_prior` after 2 valid detections
- [ ] Burst times out at 30s with correct fallback
- [ ] No messages on `/human_pose/points_3d` during active burst
- [ ] `perception_backend:=yolo` does not launch `nlf_skeleton_node`
- [ ] Coordinator blocks LOCKING until NLF prior arrives or times out
