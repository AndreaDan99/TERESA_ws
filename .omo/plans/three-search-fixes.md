# 3 Fix — STOP arm, alternate rotation, EE movement

## TASK 1: STOP button stops arm too
- File: `web/teresa_control.html`
- In `onStop()`, add `this.ikEnablePub.publish({data:false});`
- Also reset slider values to home defaults

## TASK 2: Alternate rotation direction
- File: `src/spot_control/spot_control/wbc_coordinator.py`
- In `_tick_search()`, calculate `sign = 1 if (step_idx % 2 == 0) else -1`
- Apply to `t.angular.z = sign * search_max_angular_vel`

## TASK 3: Bigger EE movement
- File: `src/spot_control/spot_control/wbc_qp_controller.py`
- Change offsets to: HOME=[0,0,+0.10], LEFT=[+0.05,-0.15,-0.05], RIGHT=[+0.05,+0.15,-0.05]
