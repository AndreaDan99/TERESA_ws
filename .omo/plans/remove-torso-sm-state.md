# Remove Dead `/torso_sm_state` Subscription

## TL;DR

> **Quick Summary**: Remove the dead `/torso_sm_state` subscription and related dead code from both torso trackers. The old FSM (`z1_FSM_old.py`) published to this topic, but the current FSM publishes to `/z1_fsm/state`. The callbacks only update a debug variable used in RViz marker text — zero functional impact.
>
> **Deliverables**:
> - `z1_yolo_torso_tracker.py` — remove subscription + callback + dead variable + marker text reference
> - `nlf_torso_tracker.py` — remove subscription + callback + dead variable + marker text reference
>
> **Estimated Effort**: Quick
> **Parallel Execution**: YES — 2 tasks, same wave
> **Critical Path**: None (independent tasks)

---

## Context

### Original Request
Remove the dead `/torso_sm_state` topic subscription from both torso trackers. Confirmed via code audit that no node publishes to this topic (old FSM in `Old/` did, current FSM uses `/z1_fsm/state`).

### What We Found
- Both trackers subscribe to `/torso_sm_state` with a 1-line callback that stores `msg.data` into `self.fsm_state_external`
- `fsm_state_external` defaults to `'WAITING'` and is only used in a debug marker string: `f"INT:{self.state} | EXT:{self.fsm_state_external}"`
- No control logic, no state machine, no published data depends on this variable
- **Zero functional impact** — pure dead code

---

## Work Objectives

### Core Objective
Remove the dead `/torso_sm_state` subscription, callback, and the `fsm_state_external` variable from both trackers. Simplify the RViz marker text to not reference the external FSM state.

### Concrete Deliverables
- `src/z1_vision/z1_vision/z1_yolo_torso_tracker.py` — cleaned
- `src/z1_vision/z1_vision/nlf_torso_tracker.py` — cleaned

### Definition of Done
- [ ] `grep -r "torso_sm_state" src/z1_vision/z1_vision/` returns zero matches
- [ ] `grep -r "fsm_state_external" src/z1_vision/z1_vision/` returns zero matches
- [ ] Both files parse without syntax errors

### Must NOT Have
- Do NOT change any other subscriptions, publishers, or control logic
- Do NOT modify the tracker FSM (`self.state` transitions)
- Do NOT touch `z1_FSM.py` or `z1_FSM_old.py`

---

## Verification Strategy

- **Automated tests**: N/A (dead code removal, no logic change)
- **Agent-Executed QA**: Verify files parse + grep confirms removal

---

## Execution Strategy

```
Wave 1 (parallel):
├── Task 1: Clean z1_yolo_torso_tracker.py [quick]
└── Task 2: Clean nlf_torso_tracker.py [quick]
```

---

## TODOs

- [x] 1. Remove `/torso_sm_state` dead code from `z1_yolo_torso_tracker.py`

  **What to do**:
  - Remove the subscription block (lines 164-166): comment + `create_subscription(String, '/torso_sm_state', self.cb_fsm_state, 10)`
  - Remove `self.fsm_state_external = 'WAITING'` from `__init__` (~line 248)
  - Remove the `cb_fsm_state` method (lines 256-257)
  - In the marker text (~line 1053), change from `f"INT:{self.state} | EXT:{self.fsm_state_external}"` to just `f"INT:{self.state}"`

  **Must NOT do**:
  - Do NOT remove any other subscription
  - Do NOT change tracker state logic

  **Recommended Agent Profile**: `quick`
  - Reason: trivial dead code removal, single file, no logic changes

  **Parallelization**: Wave 1 with Task 2 (independent)

  **Acceptance Criteria**:
  - [ ] `grep "torso_sm_state" src/z1_vision/z1_vision/z1_yolo_torso_tracker.py` returns empty
  - [ ] `grep "fsm_state_external" src/z1_vision/z1_vision/z1_yolo_torso_tracker.py` returns empty
  - [ ] `python3 -m py_compile src/z1_vision/z1_vision/z1_yolo_torso_tracker.py` passes

  **QA Scenarios**:
  ```
  Scenario: Verify removal — no dead references remain
    Tool: Bash (grep)
    Steps:
      1. grep -n "torso_sm_state\|fsm_state_external" src/z1_vision/z1_vision/z1_yolo_torso_tracker.py
      2. Check exit code is 1 (no matches)
    Expected Result: No output, exit code 1
    Evidence: .omo/evidence/task-1-grep-clean.txt

  Scenario: File still parses correctly
    Tool: Bash (python3)
    Steps:
      1. python3 -m py_compile src/z1_vision/z1_vision/z1_yolo_torso_tracker.py
    Expected Result: No output, exit code 0
    Evidence: .omo/evidence/task-1-compile.txt
  ```

  **Commit**: YES
  - Message: `refactor(z1_vision): remove dead /torso_sm_state subscription from yolo tracker`
  - Files: `src/z1_vision/z1_vision/z1_yolo_torso_tracker.py`

- [x] 2. Remove `/torso_sm_state` dead code from `nlf_torso_tracker.py`

  **What to do**:
  - Remove the subscription block (lines 183-185): comment + `create_subscription(String, '/torso_sm_state', self._cb_fsm_state, 10)`
  - Remove `self.fsm_state_external = 'WAITING'` from `__init__` (~line 253)
  - Remove the `_cb_fsm_state` method (lines 267-268)
  - In the marker text (~line 1051), change from `f"INT:{self.state} | EXT:{self.fsm_state_external}"` to just `f"INT:{self.state}"`

  **Must NOT do**:
  - Do NOT remove any other subscription
  - Do NOT change tracker state logic

  **Recommended Agent Profile**: `quick`
  - Reason: identical change to Task 1, different file

  **Parallelization**: Wave 1 with Task 1 (independent)

  **Acceptance Criteria**:
  - [ ] `grep "torso_sm_state" src/z1_vision/z1_vision/nlf_torso_tracker.py` returns empty
  - [ ] `grep "fsm_state_external" src/z1_vision/z1_vision/nlf_torso_tracker.py` returns empty
  - [ ] `python3 -m py_compile src/z1_vision/z1_vision/nlf_torso_tracker.py` passes

  **QA Scenarios**:
  ```
  Scenario: Verify removal — no dead references remain
    Tool: Bash (grep)
    Steps:
      1. grep -n "torso_sm_state\|fsm_state_external" src/z1_vision/z1_vision/nlf_torso_tracker.py
      2. Check exit code is 1 (no matches)
    Expected Result: No output, exit code 1
    Evidence: .omo/evidence/task-2-grep-clean.txt

  Scenario: File still parses correctly
    Tool: Bash (python3)
    Steps:
      1. python3 -m py_compile src/z1_vision/z1_vision/nlf_torso_tracker.py
    Expected Result: No output, exit code 0
    Evidence: .omo/evidence/task-2-compile.txt
  ```

  **Commit**: YES
  - Message: `refactor(z1_vision): remove dead /torso_sm_state subscription from nlf tracker`
  - Files: `src/z1_vision/z1_vision/nlf_torso_tracker.py`

---

## Commit Strategy

- **1**: `refactor(z1_vision): remove dead /torso_sm_state subscription from yolo tracker` — z1_yolo_torso_tracker.py
- **2**: `refactor(z1_vision): remove dead /torso_sm_state subscription from nlf tracker` — nlf_torso_tracker.py
- Groups into single commit if preferred: `refactor(z1_vision): remove dead /torso_sm_state subscriptions`

---

## Success Criteria

### Verification Commands
```bash
# Confirm removal
grep -r "torso_sm_state" src/z1_vision/z1_vision/    # Expected: no output
grep -r "fsm_state_external" src/z1_vision/z1_vision/ # Expected: no output

# Confirm both files still parse
python3 -m py_compile src/z1_vision/z1_vision/z1_yolo_torso_tracker.py
python3 -m py_compile src/z1_vision/z1_vision/nlf_torso_tracker.py
```

### Final Checklist
- [ ] Zero references to `torso_sm_state` in active source
- [ ] Zero references to `fsm_state_external` in active source
- [ ] Both files compile without errors
