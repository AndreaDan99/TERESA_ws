# Fix YOLO on nlf-24-smpl branch

## TL;DR
Copy the working ByteTrack+EMA refactoring from main, add `coco_to_smpl_24()` at publish step. YOLO internally tracks 17 COCO, publishes 24 SMPL. Consumers already support 24.

## Task
- [ ] 1. Rewrite yolo_skeleton_spot.py: use ByteTrack + EMA like main, publish 24 SMPL via coco_to_smpl_24()
- [ ] 2. Update spot_perception.launch.py: remove stale kalman params, add smpl config reference
- [ ] 3. Verify compile
