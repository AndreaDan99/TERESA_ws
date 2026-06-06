# YOLO Built-in Tracking — Mini Piano

## TL;DR
Sostituire il tracking manuale (Kalman3D + greedy assignment) in `yolo_skeleton_spot.py` con `model.track()` nativo di YOLO (ByteTrack). Mantenere 3D depth projection, posture classification, target selection.

## Task

- [ ] 1. Riscrivere `yolo_skeleton_spot.py`: usare `model.track()` invece di Kalman manuale + PersonTrack. Ridurre ~500 righe a ~250. Mantenere: depth→3D, posture publish, target selection (closest LYING).
- [ ] 2. Rimuovere o semplificare `person_tracking.py` e `kalman_filter.py` (non più necessari per YOLO)

## Commit
refactor: use yolo built-in tracking instead of manual kalman
