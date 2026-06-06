# Fix NLF Frame Skip — Mini Piano

## TL;DR
Aggiungere `process_every_n_frames` a entrambi i nodi NLF per processare 1 frame ogni N. Il Kalman filter copre i frame saltati con `predict()`. Default: 5 (1 FPS con camera a 15 FPS = 3 inferenze/sec di risparmio).

## Task

- [ ] 1. Aggiungere `process_every_n_frames` a nlf_torso_tracker.py + logica skip in _cb_image
- [ ] 2. Aggiungere `process_every_n_frames` a nlf_skeleton.py + logica skip in _cb_color
- [ ] 3. Aggiungere parametro a nlf_torso_params.yaml e nlf_params.yaml

## Commits
- fix(nlf): add frame skip to reduce cpu inference load
