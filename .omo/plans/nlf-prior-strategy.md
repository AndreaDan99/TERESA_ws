# NLF Prior + Active Perception — Piano Completo

## TL;DR

> **Quick Summary**: NLF diventa un ground truth al LOCKING, non un sostituto di YOLO. Una singola inferenza NLF (24 SMPL, ~7s) fornisce un prior di alta qualità che accelera tutte le fasi successive. YOLO mantiene l'active perception a 15 FPS ma parte da un'ancora precisa invece che da zero.
>
> **Deliverables**:
> - NLF trigger al LOCKING → 24 SMPL prior
> - Body scanning grid ridotta (solo se NLF OK): 2 fasi, ~12 pose
> - Fusione YOLO multi-angolo per completare giunti NLF mancanti
> - PRE_APPROACH semplificato (solo se NLF OK): Z1 va diretto, RealSense solo verifica
> - Quality Monitor esteso con tolleranza NLF (15cm/30cm)
> - ⚠️ **Fallback completo**: se NLF fallisce (timeout 10s) → comportamento attuale invariato in TUTTE le fasi
>
> **Estimated Effort**: Medium (~1 giorno)
> **Critical Path**: wbc_coordinator.py → body_search_scanner.py → pre_approach → quality_monitor

---

## Context

### Strategia
NLF (68mm accuracy, 24 SMPL, 0.15 FPS CPU) è troppo lento per sostituire YOLO (15 FPS). Ma un **singolo frame** al LOCKING fornisce un prior di alta qualità — colonna vertebrale, testa, piedi — che YOLO non ha. Questo prior accelera tutte le fasi successive senza violare il principio di active perception: YOLO esplora ancora, ma parte da un'ancora precisa.

### Principio di Fallback (NON NEGOZIABILE)
Se NLF non produce un prior valido entro il timeout (10s), **TUTTE le ottimizzazioni vengono disattivate** e il sistema torna al comportamento esatto del 6 Giugno 2026:

| Fase | Con NLF OK | Senza NLF (fallback) |
|------|-----------|---------------------|
| BODY_SCANNING | Grid ridotta (~12 pose, 2 fasi) | **Grid adattiva attuale** (2-4 pose, invariata) |
| PRE_APPROACH | Immediato (coherence check non bloccante) | **Attesa RealSense ESTIMATING/LOCKED** (sliding window ≥1/5, invariato) |
| Quality Monitor | Delta NLF 15cm/30cm | **Non attivo** (nessun prior da confrontare) |
| LOOKAT target | NLF 70% / YOLO 30% | **YOLO only** (invariato) |

Il fallback **non è un degrado parziale**: è un interruttore binario. O tutto il fast-path NLF è attivo, o niente lo è.

### Perché funziona
- Il paziente è **supino e immobile** — lo scheletro NLF non diventa obsoleto
- La colonna vertebrale (SPINE1→SPINE3) dà un torso angle perfetto
- HEAD e PELVIS danno un body axis più stabile di nose→ankles
- I 24 SMPL nativi abilitano griglie corpo più dense

---

## Work Objectives

### Core Objective
Potenziare l'active perception con un prior NLF di alta qualità al LOCKING, senza sostituire YOLO. Ridurre i tempi delle fasi statiche (body scanning, pre-approach) grazie alla conoscenza pregressa.

### Concrete Deliverables
- `wbc_coordinator.py`: NLF trigger al LOCKING + Quality Monitor esteso
- `body_search_scanner.py`: griglia ridotta a 2 fasi
- `nlf_skeleton.py`: topic `/exposure/nlf_prior` per il prior NLF
- `pre_approach`: semplificato (nessuna attesa, solo verifica)

### Definition of Done
- [ ] LOCKING scatta 1 frame NLF e salva 24 SMPL come prior
- [ ] Se NLF OK: body scanning grid ridotta (2 fasi, ~12 pose) + fusione YOLO multi-angolo
- [ ] Se NLF OK: PRE_APPROACH immediato, RealSense coherence check non bloccante
- [ ] Se NLF OK: Quality Monitor con tolleranza 15cm/30cm, LOOKAT NLF(70%)+YOLO(30%)
- [ ] Se NLF fallisce (timeout 10s): **tutto invariato** — grid attuale, attesa RealSense, YOLO only
- [ ] Il flag `self._nlf_prior is None or 'timeout'` decide TUTTI i rami condizionali

### Must Have
- NLF fallito non blocca la missione (timeout 10s → YOLO only)
- **Fallback totale**: nessuna ottimizzazione parziale senza NLF. La grid NON si riduce, la PRE_APPROACH NON salta l'attesa RealSense
- YOLO a 15 FPS in tutte le fasi dinamiche (SEARCHING, APPROACHING)
- Prior NLF accessibile a tutti i consumer via topic ROS2
- Backward compatibility: se NLF non disponibile, sistema funziona **esattamente** come il 6 Giugno 2026

### Must NOT Have
- NO NLF in SEARCHING o APPROACHING (troppo lento)
- NO fusione temporale YOLO+NLF (100:1 frame rate disparity)
- NO rimozione dell'active perception (YOLO esplora ancora)
- NO modifica alla matematica del LOOKAT (solo target iniziale migliore)

---

## Cambiamenti per Fase FSM

### LOCKING → NLF Prior

**Cosa cambia**: All'ingresso in LOCKING, trigger NLF. Salva 24 SMPL come `self._nlf_prior`.

```python
def _enter_locking(self):
    # ... existing LOCKING entry actions ...
    self._nlf_prior = None
    self._pub_nlf_trigger.publish(Bool(data=True))  # topic /nlf/trigger
    self._nlf_trigger_time = self.get_clock().now()
```

In `_tick_locking`, aggiungere attesa prior:
```python
def _tick_locking(self):
    # ... existing LOCKING logic (5 samples, ik_done) ...
    
    # NLF prior check (non bloccante)
    if self._nlf_prior is None:
        elapsed = (now - self._nlf_trigger_time).nanoseconds * 1e-9
        if elapsed > 10.0:
            self.get_logger().warn('NLF timeout — proceeding without prior')
            self._nlf_prior = 'timeout'  # marker, non None
```

Nuovo callback per ricevere il prior:
```python
def _cb_nlf_prior(self, msg: PoseArray):
    if len(msg.poses) == 24:
        self._nlf_prior = [np.array([p.position.x, p.position.y, p.position.z]) 
                           for p in msg.poses]
        self.get_logger().info(f'NLF prior received: 24 joints')
```

### BODY_SCANNING → Grid ridotta + Fusione (CONDIZIONATO)

**Cosa cambia se NLF OK**: 3 fasi → 2 fasi. La fase 3 (adaptive targeting) sparisce. Ogni posa YOLO accumula keypoint. Alla fine, fusione con NLF prior.

**Cosa NON cambia se NLF fallisce**: la grid adattiva attuale (2-4 pose in base a confidenza keypoint pre-scan) rimane **esattamente invariata**. Nessuna riduzione, nessuna fusione.

```python
def _gen_cartesian_scan_grid(self):
    if self._nlf_prior_valid():  # prior ricevuto entro timeout, non 'timeout'
        return self._gen_reduced_grid_from_prior()
    else:
        # Comportamento 6 Giugno 2026 — invariato
        return self._gen_adaptive_grid_from_keypoint_conf()
```

```yaml
# body_search_params.yaml (usati solo nel ramo NLF OK)
body_scan_reduced_ny: 2       # 4 arc positions (ridotte)
body_scan_reduced_nx: 2
body_scan_reduced_wrist_ny: 2
body_scan_reduced_wrist_nz: 2
# Parametri grid adattiva attuali: INVARIATI (usati nel ramo fallback)
```

**Fusione** (al termine della grid):
```python
def _fuse_skeleton(self, nlf_prior, yolo_accumulated):
    """NLF for available joints, YOLO multi-angle for gaps."""
    fused = [None] * 24
    for i in range(24):
        if nlf_prior[i] is not None and not isnan(nlf_prior[i][0]):
            fused[i] = nlf_prior[i]        # NLF: alta precisione
        elif i in yolo_accumulated:
            fused[i] = yolo_accumulated[i]   # YOLO multi-angolo
    return fused
```

### PRE_APPROACH → Biforcazione NLF / Fallback

**Cosa cambia se NLF OK**: Niente più attesa RealSense. Z1 va diretto alla posa pre-calcolata dal prior NLF. RealSense fa solo coherence check non bloccante (se diverge >15cm: warning ma procede comunque).

**Cosa NON cambia se NLF fallisce**: l'attesa RealSense `ESTIMATING/LOCKED` con sliding window ≥1/5 tick rimane **esattamente invariata** (comportamento 6 Giugno 2026).

```python
def _tick_pre_approach(self):
    if self._nlf_prior_valid():
        # ── FAST PATH: NLF OK ──
        goal = self._torso_center_from_prior()
        self._pub_goal.publish(goal)
        
        # Coherence check (non bloccante)
        if self._torso_tracker_state in ('ESTIMATING', 'LOCKED'):
            delta = self._check_nlf_delta()
            if delta < 0.15:
                self.get_logger().info('RealSense coherent with NLF prior')
            else:
                self.get_logger().warn(f'RealSense diverges from NLF: {delta:.2f}m')
        # In ogni caso: procedi subito
        self._set_state(CoordState.APPROACHING)
    else:
        # ── FALLBACK: comportamento 6 Giugno 2026 invariato ──
        self._tick_pre_approach_legacy()  # sliding window ≥1/5, timeout 5s, etc.
```

Rimosso SOLO nel ramo NLF OK: `_torso_detected_ticks` buffer, timeout 5s, attesa ≥1 tick.
Nel ramo fallback: **tutto invariato**.

### Quality Monitor → Esteso con NLF (CONDIZIONATO)

**Cosa cambia se NLF OK**: Confronta ogni frame YOLO col prior NLF.

```python
def _check_nlf_delta(self, torso_yolo):
    if not self._nlf_prior_valid():
        return 'HIGH', None   # nessun prior → non applicabile
    
    nlf_center = self._torso_center_from_prior()
    delta = np.linalg.norm(torso_yolo - nlf_center)
    
    if delta < 0.15:
        return 'HIGH', delta     # YOLO allineato → LOOKAT: NLF 70% / YOLO 30%
    elif delta < 0.30:
        return 'MEDIUM', delta   # drift lieve → LOOKAT: NLF 50% / YOLO 50%
    else:
        return 'LOW', delta      # divergenza → LOOKAT: YOLO 100%, possibile ri-trigger NLF
```

**Cosa NON cambia se NLF fallisce**: Quality Monitor usa solo YOLO (comportamento attuale invariato). Nessun delta check, nessun peso NLF nel LOOKAT.

---

## File Modificati

| File | Modifica | Righe |
|---|---|---|
| `wbc_coordinator.py` | NLF trigger, prior callback, `_nlf_prior_valid()`, PRE_APPROACH biforcato, Quality Monitor condizionato, BODY_SCANNING condizionato | +100 |
| `body_search_scanner.py` | Metodo `_gen_reduced_grid_from_prior()` per ramo NLF OK; grid adattiva attuale preservata per fallback | +30 |
| `body_search_params.yaml` | Nuovi parametri `body_scan_reduced_*` per ramo NLF OK; parametri attuali invariati | +5 |
| `nlf_skeleton.py` | Topic `/exposure/nlf_prior` (stesso output, topic separato) | +3 |

**NOTA**: il body_search_scanner **NON rimuove** la grid adattiva attuale. Aggiunge un metodo alternativo per il ramo NLF OK. Il codice esistente rimane intatto come fallback.

---

## Non Modificati

- `posture_classifier.py` — già supporta 24 SMPL con fallback ✅
- `laying_human_detector.py` — già usa HEAD/PELVIS ✅
- `exposure_scanner.py` — già genera griglia da 24 giunti ✅
- `z1_scan_manager.py` — già calcola FAST da SMPL indices ✅
- `wbc_qp_controller.py` — LOOKAT invariato, cambia solo target ✅
- `yolo_skeleton_spot.py` — invariato ✅
- **Codice esistente di PRE_APPROACH e BODY_SCANNING** — preservato intatto come ramo fallback ✅

---

## Tempi Risparmiati

| Fase | Prima | Dopo (NLF OK) | Dopo (NLF fallito) | Risparmio (NLF OK) |
|---|---|---|---|---|
| LOCKING | 5 campioni YOLO | +1 frame NLF (~7s) | 5 campioni YOLO (invariato) | −7s (costo una tantum) |
| BODY_SCANNING | 2-4 pose adattive | ~12 pose ridotte + fusione | 2-4 pose adattive (invariato) | N/A (trade-off qualità) |
| PRE_APPROACH | Attesa 0-5s RealSense | 0s immediato | Attesa 0-5s RealSense (invariato) | **5s** |
| APPROACHING | LOOKAT da zero | LOOKAT da prior NLF | LOOKAT da zero (invariato) | Converge più veloce |

**Scenario peggiore (NLF fallito)**: 0 secondi aggiuntivi. Il timeout di 10s per NLF decorre in parallelo alla raccolta dei 5 campioni YOLO e al movimento home del braccio — non è tempo sprecato.

---

## Rischi e Mitigazioni

| Rischio | Mitigazione |
|---|---|
| NLF timeout (10s) | **Fallback totale**: grid adattiva invariata, PRE_APPROACH con attesa RealSense, Quality Monitor YOLO-only. Il sistema torna al comportamento del 6 Giugno 2026. Nessuna ottimizzazione parziale. |
| NLF prior ha giunti NaN (occlusi) | YOLO multi-angolo completa i buchi nel ramo fusione |
| Paziente si muove dopo LOCKING | Quality Monitor rileva delta >30cm → passa a YOLO 100% nel LOOKAT, possibile ri-trigger NLF |
| NLF modello non caricato | Launch file check: se modello assente, `perception_backend:=yolo`. Il flag `_nlf_prior_valid()` restituisce False → fallback totale attivo da subito |
| Bug nel ramo NLF OK | Il ramo fallback è il codice esistente, già testato. Se il nuovo codice ha bug, il sistema degrada al comportamento noto. |

---

## Punto di Decisione Centralizzato

Un unico metodo `_nlf_prior_valid()` decide TUTTI i rami condizionali:

```python
def _nlf_prior_valid(self) -> bool:
    """True se il prior NLF è disponibile e utilizzabile."""
    if self._nlf_prior is None:
        return False           # NLF non ancora triggerato
    if self._nlf_prior == 'timeout':
        return False           # NLF fallito
    if len(self._nlf_prior) != 24:
        return False           # prior corrotto
    # Almeno 4 giunti torso validi (spine + pelvis) per procedere
    valid_torso = sum(1 for j in [6,7,8, 14] 
                      if not np.any(np.isnan(self._nlf_prior[j])))
    return valid_torso >= 4
```

Questo metodo è l'**unico gate** per tutte le ottimizzazioni. Ogni fase che ha un ramo NLF OK / fallback lo chiama:

```
_nlf_prior_valid() == True  → fast path (grid ridotta, PRE_APPROACH immediato, Quality Monitor NLF)
_nlf_prior_valid() == False → comportamento 6 Giugno 2026 invariato
```

**Nessun altro flag, nessuna condizione parziale.**
