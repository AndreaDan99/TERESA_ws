# NLF Prior al LOCKING — Piano di Implementazione

## TL;DR
Aggiungere 1 frame NLF Orbbec al LOCKING come ground truth. Il Quality Monitor lo usa come ancora per validare YOLO durante APPROACHING. LOOKAT parte dal prior invece che da zero. Nessuna modifica ai consumer — già supportano 24 SMPL.

## Cambiamenti (3 file)

### 1. wbc_coordinator.py — LOCKING + NLF trigger
- All'ingresso in LOCKING: pubblicare `/nlf/trigger` (Bool)
- Aspettare `/human_pose/points_3d` con 24 giunti (topic esistente, già pubblicato da nlf_skeleton)
- Salvare come `self._nlf_prior` (24 numpy array)
- Timeout 10s → fallback senza NLF, log warning
- Dopo LOCKING → PRE_APPROACH: LOOKAT target = prior NLF (non più da zero)

### 2. wbc_coordinator.py — Quality Monitor esteso
```python
def _check_nlf_delta(self, torso_yolo):
    if self._nlf_prior is None: return HIGH
    delta = ||torso_yolo - nlf_torso_center||
    if delta < 0.15: return HIGH      # allineato
    elif delta < 0.30: return MEDIUM   # drift lieve
    else: return LOW                   # trigger ri-NLF
```
Il target pubblicato su `/wbc/ee_goal` usa: `0.7 * nlf_prior + 0.3 * yolo` quando HIGH.

### 3. nlf_skeleton.py — nessuna modifica
Già pubblica 24 SMPL su `/human_pose/points_3d`. Il coordinator legge da lì.

## Effetti

| Fase | Prima | Dopo |
|---|---|---|
| LOCKING | Attesa 5 campioni YOLO | + NLF 1 frame (max 10s timeout) |
| PRE_APPROACH | LOOKAT parte da zero | LOOKAT parte da prior NLF |
| APPROACHING | YOLO solo | YOLO + prior NLF (tolleranza 15cm) |
| BODY SEARCH | Grid search da zero | Grid centrata su prior (più veloce) |
| EXPOSURE | 16 punti iterativi | 22 punti da prior + affinamento |

## Non cambia
- SEARCHING, SEMI_LOCKING: solo YOLO
- posture_classifier, laying_detector, exposure_scanner: già pronti
- LOOKAT matematica: invariata, cambia solo il target iniziale
