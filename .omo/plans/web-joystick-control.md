# Web Joystick Control — TERESA Control Panel

## TL;DR

> **Quick Summary**: Add a directional joystick panel (right side) to `teresa_control.html` for manual Spot control via WASD-style arrow buttons. Two modes: Drive (↑↓←→ for cmd_vel) and Body (↑↓ for height, ←→ for pitch). Includes speed +/- buttons and sliders for height/pitch presets (5°/10°/15°).
>
> **Deliverables**:
> - `web/teresa_control.html` — restructured layout with left log panel + right joystick panel
> - D-pad with 5 buttons (↑ ↓ ← → + mode indicator)
> - Mode toggle (Drive ↔ Body)
> - Speed +/- buttons with display
> - Height slider (5/10/15 steps) and pitch slider (5°/10°/15°)
>
> **Estimated Effort**: Medium
> **Parallel Execution**: NO — single file, sequential
> **Critical Path**: CSS layout → HTML structure → JS logic → integration test

---

## Context

### Original Request
Add manual Spot control to the web UI: arrow buttons for driving (WASD), body height/pitch controls, two-mode switch, speed adjust, and slider presets.

### Current State
- `teresa_control.html` has status bar, centered state view, camera panel, body map, bottom button bar
- Keyboard `s`/`r`/`u` already maps to WBC commands, not manual driving
- `wbc_keyboard_controller.py` handles keyboard-based manual control (standalone node)
- Spot accepts: `/my_spot/cmd_vel` (Twist) for driving, `/my_spot/body_pose` (Pose) for height/pitch

### Safety Considerations
- Joystick should only be active when WBC is IDLE (not during autonomous mission)
- Buttons must send zero velocity on release (safety stop)
- Live updates while button held, stop on release

---

## Work Objectives

### Target Layout

```
┌─────────────────────────────────────────────────────────────┐
│  Rosbridge: 🟢 | SpotCore: 🟢 | WBC: IDLE | Start: ---    │
├───────────────────────┬─────────────────────────────────────┤
│                       │          ┌─────┐                    │
│    LOG / Terminal     │          │  ↑  │                    │
│    (scrollabile)      │     ┌────┼─────┼────┐              │
│                       │     │ ←  │ DRV │  → │              │
│  [timestamp] ...      │     └────┼─────┼────┘              │
│  [timestamp] ...      │          │  ↓  │                    │
│                       │          └─────┘                    │
│                       │                                     │
│                       │  [DRIVE ◄► BODY]  SPEED: [-] 0.5 [+]│
│                       │                                     │
│                       │  Height: ───●────────  0.15m        │
│                       │  Pitch:  ──●──────────  10°         │
│                       │          5°  10° 15°                │
├───────────────────────┴─────────────────────────────────────┤
│  [▶START] [↩RETURN] ...                           [STOP]    │
└─────────────────────────────────────────────────────────────┘
```

### Concrete Deliverables
- Restructured `teresa_control.html` with 2-panel layout (left = log, right = joystick)
- D-pad: 5 buttons in cross formation, center shows current mode
- Mode toggle button (Drive ↔ Body)
- Speed display + [-]/[+] buttons
- Two range sliders: Height (0m to 0.20m, 5cm steps) and Pitch (0° to 15°, 5° steps)

### Definition of Done
- [ ] D-pad drives Spot in Drive mode (cmd_vel)
- [ ] D-pad adjusts height/pitch in Body mode (body_pose)
- [ ] Releasing a button sends zero cmd_vel
- [ ] Speed +/- changes velocity multiplier (0.1 to 1.0)
- [ ] Sliders snap to 5cm/5° increments
- [ ] Joystick disabled when WBC ≠ IDLE
- [ ] Existing controls (START, RETURN, camera view, etc.) still work

### Must NOT Have
- Do NOT break existing WBC keyboard controls
- Do NOT interfere with autonomous mission (disable during WBC active states)
- Do NOT change the bottom button bar or camera view
- Do NOT require new ROS2 nodes — publish directly to existing Spot topics

---

## Verification Strategy

- **Automated tests**: N/A (UI change)
- **Agent-Executed QA**: Verify HTML parses, buttons render, JS logic correct via code review

---

## Execution Strategy

```
Wave 1 (sequential — single file):
├── Task 1: Restructure CSS layout — left panel (log) + right panel (joystick)
├── Task 2: Build D-pad HTML + mode toggle + speed controls
├── Task 3: Add height/pitch range sliders
├── Task 4: JavaScript: cmd_vel publisher + body_pose publisher + mode logic + speed logic
├── Task 5: JavaScript: button press/release handlers (mousedown = send, mouseup = stop)
└── Task 6: Integration — verify existing features unbroken, disable during WBC active
```

---

## TODOs

- [x] 1. Restructure CSS layout for two-panel main area

  **What to do**:
  - Change `#state-view` from centered flex to full-width split layout
  - Add `#log-panel` (left, flex: 1) containing the existing log area
  - Add `#joystick-panel` (right, fixed width ~300px) for new controls
  - Style the joystick panel with dark theme matching existing design
  - Make it responsive: on narrow screens, stack vertically

  **Must NOT do**:
  - Do NOT break camera view layout (it replaces main area when open)
  - Do NOT change status bar or button bar CSS

  **Recommended Agent Profile**: `visual-engineering`

  **Parallelization**: Wave 1, sequential (Task 2 depends on this)

  **QA Scenarios**: Verify the two-panel layout renders correctly at different widths

  **Commit**: groups with all web tasks

- [x] 2. Build D-pad HTML structure + mode toggle + speed controls

  **What to do**:
  - Create 5-button D-pad using CSS grid: `grid-template-areas: ". up ." "left center right" ". down ."`
  - Center button shows current mode label ("DRV" or "BDY")
  - Add mode toggle button below D-pad: "[DRIVE ◄► BODY]"
  - Add speed display: `SPEED: [-] 0.5 [+]` with two buttons
  - Buttons must have `data-direction` attributes for JS to read

  **Must NOT do**:
  - Do NOT use `<canvas>` for the D-pad — use CSS-styled buttons
  - Do NOT use images/icons — use Unicode arrows (↑ ↓ ← →)

  **Recommended Agent Profile**: `visual-engineering`

  **Parallelization**: Wave 1, sequential (Task 3 depends on this)

  **QA Scenarios**: Buttons render, mode toggle switches label, speed buttons increment/decrement

  **Commit**: groups with all web tasks

- [x] 3. Add height and pitch range sliders with preset stops

  **What to do**:
  - Height slider: `<input type="range" min="0" max="20" step="5" value="10">` — 0 to 0.20m in 5cm steps
  - Pitch slider: `<input type="range" min="0" max="15" step="5" value="0">` — 0° to 15° in 5° steps
  - Show current value as label next to each slider
  - Style sliders to match dark theme
  - Add preset stop labels below pitch slider: "0° 5° 10° 15°"

  **Must NOT do**:
  - Do NOT make sliders continuous — must snap to 5cm/5° increments via `step` attribute

  **Recommended Agent Profile**: `visual-engineering`

  **Parallelization**: Wave 1, sequential

  **QA Scenarios**: Sliders snap correctly, values displayed update on change

  **Commit**: groups with all web tasks

- [x] 4. JavaScript: publishers + mode logic + speed logic

  **What to do**:
  - Add `this.cmdVelPub` (already exists) and `this.bodyPosePub` (new, `/my_spot/body_pose`, `geometry_msgs/Pose`)
  - Add state: `this.joystickMode = 'drive'` (or 'body')
  - Add state: `this.joystickSpeed = 0.5` (default, range 0.1–1.0)
  - Implement `setJoystickMode(mode)` — toggles between drive and body
  - Implement `adjustSpeed(delta)` — changes speed by ±0.1, clamped to [0.1, 1.0]
  - Implement `sendDriveCommand(direction)` — publishes Twist based on direction and speed:
    - ↑: linear.x = +speed
    - ↓: linear.x = -speed
    - ←: linear.y = +speed
    - →: linear.y = -speed
  - Implement `sendBodyCommand(direction)` — reads slider values, adjusts height/pitch:
    - ↑: height + 0.05 (clamped)
    - ↓: height - 0.05
    - ←: pitch + 5°
    - →: pitch - 5°
  - Implement `stopAll()` — publishes zero Twist

  **Must NOT do**:
  - Do NOT use setInterval for repeated sending — use mousedown/mouseup events

  **Recommended Agent Profile**: `unspecified-high`

  **Parallelization**: Wave 1, sequential (after Tasks 1-3)

  **QA Scenarios**: Verify correct Twist values for each direction, speed changes correctly, body_pose publishes correct quaternion for pitch

  **Commit**: groups with all web tasks

- [x] 5. JavaScript: button press/release event handlers

  **What to do**:
  - Attach `mousedown` to each D-pad button → starts sending command
  - Attach `mouseup` and `mouseleave` → calls `stopAll()` (publish zero Twist)
  - Attach `touchstart`/`touchend` for mobile/tablet support
  - On mode toggle click → switch `joystickMode`, update center button label
  - On speed +/- click → `adjustSpeed(±0.1)`, update speed display
  - On slider change → update displayed value label, apply body pose immediately

  **Safety**:
  - `mouseup` on document (global) → `stopAll()` (catch if user drags outside button)
  - WBC state ≠ IDLE → disable all joystick buttons (gray out, no action)

  **Must NOT do**:
  - Do NOT use keyboard events for joystick (keep existing keyboard shortcuts separate)

  **Recommended Agent Profile**: `unspecified-high`

  **Parallelization**: Wave 1, sequential

  **QA Scenarios**: Hold ↑ → Spot moves forward, release → stops. Toggle mode → buttons change behavior. Speed +/- works.

  **Commit**: groups with all web tasks

- [x] 6. Integration test — verify existing features unbroken

  **What to do**:
  - Verify START/RETURN/STOP buttons still work
  - Verify camera panel still opens/closes
  - Verify body map still works
  - Verify keyboard shortcuts (s/r/u/n/a/c/ESC) still work
  - Verify joystick disables when WBC state ≠ IDLE
  - Verify joystick re-enables when WBC returns to IDLE

  **Must NOT do**:
  - Do NOT change any existing button handlers or keyboard shortcuts

  **Recommended Agent Profile**: `visual-engineering`

  **Parallelization**: Wave 1, sequential (after Tasks 1-5)

  **QA Scenarios**: Full manual test flow — connect, drive with joystick, switch to body mode, adjust height/pitch, verify STOP works

  **Commit**: groups with all web tasks

---

## Commit Strategy

Single commit for all web changes:
```
feat(web): add manual joystick control panel for Spot driving and body pose

- Two-panel layout: left log terminal, right joystick D-pad
- Drive mode: ↑↓←→ for cmd_vel (forward/back/strafe)
- Body mode: ↑↓ for height, ←→ for pitch
- Speed +/- buttons (0.1 to 1.0 m/s)
- Height slider (0–20cm, 5cm steps) and pitch slider (0°–15°, 5° steps)
- Auto-disable during autonomous WBC mission
```

---

## Success Criteria

### Verification Commands
```bash
# Verify HTML valid (no unclosed tags)
python3 -c "
from html.parser import HTMLParser
class V(HTMLParser):
    def handle_starttag(self, t, a): pass
    def handle_endtag(self, t): pass
with open('web/teresa_control.html') as f:
    V().feed(f.read())
print('HTML parses OK')
"

# Verify JS syntax
node -e "
const fs = require('fs');
const html = fs.readFileSync('web/teresa_control.html', 'utf8');
const m = html.match(/<script>\s*\n\s*\(function[\s\S]*?<\/script>/);
if (m) { new (require('vm').Script)(m[0].replace(/<\/?script>/g, '')); console.log('JS OK'); }
"
```

### Final Checklist
- [ ] D-pad renders correctly with arrow buttons
- [ ] Mode toggle switches between DRIVE and BODY
- [ ] Drive mode publishes correct cmd_vel Twists
- [ ] Body mode publishes correct body_pose
- [ ] Release stops all movement
- [ ] Speed +/- works, 0.1 to 1.0 range
- [ ] Sliders snap to 5cm/5° increments
- [ ] Joystick disabled during WBC active states
- [ ] Existing controls unbroken
