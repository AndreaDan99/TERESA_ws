# Rimuovere Kalman da nlf_skeleton.py — Mini Piano

## TL;DR
NLF produce già giunti 3D accurati (SOTA 68mm). Il Kalman3D aggiunge solo complessità, latenza e drift tra frame (7s a 0.15 FPS). Sostituire con EMA smoothing (come YOLO).

## Task
- [ ] 1. Rimuovere NLFPersonTrack, Kalman3D, mahalanobis gating, knee validation da nlf_skeleton.py
- [ ] 2. Sostituire con EMA smoothing per-person (come yolo_skeleton_spot.py)
- [ ] 3. Pubblicare direttamente i 24 giunti NLF (con EMA)

## Effetto
- Codice più semplice (~300 righe invece di 700)
- Nessun drift tra frame
- Stessa robustezza (NLF è già preciso)
