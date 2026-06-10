// ─── QP Controller Mode Evolution Slides (May → June 2026) ────────────
// Design system constants and helpers (same as teresa_presentation.js)
const C = {
  TEAL:         "1F7985",
  TEAL_LIGHT:   "80C8CF",
  DARK_NAVY:    "0D3340",
  TEAL_DARK:    "0D6756",
  GREEN:        "2D714C",
  SEAFOAM:      "389896",
  OFF_WHITE:    "E8E8DF",
  WHITE:        "FFFFFF",
};

const SLIDE_W = 10.0;
const SLIDE_H = 5.625;

// ─── Helpers ───────────────────────────────────────────────────────────

function darkBg(slide, circleY) {
  slide.background = { fill: C.TEAL };
  slide.addShape("ellipse", { x:1.20, y:circleY, w:7.60, h:7.60, fill:{color:C.TEAL_LIGHT}, line:{type:"none"} });
}

function lightHeader(slide, title, subtitle) {
  slide.background = { fill: C.WHITE };
  slide.addShape("rect", { x:0.38, y:0.18, w:0.06, h:0.58, fill:{color:C.TEAL}, line:{type:"none"} });
  slide.addText(title, { x:0.55, y:0.18, w:9.10, h:0.60, fontFace:"Trebuchet MS", fontSize:26, bold:true, color:C.TEAL_DARK, valign:"middle" });
  if (subtitle) slide.addText(subtitle, { x:0.55, y:0.85, w:9.00, h:0.28, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:C.SEAFOAM, valign:"middle" });
}

function sectionSlide(prs, num, title, subtitle) {
  const slide = prs.addSlide();
  darkBg(slide, -0.70);
  slide.addText(num, { x:0.55, y:0.25, w:1.20, h:0.65, fontFace:"Trebuchet MS", fontSize:52, bold:true, color:C.WHITE });
  slide.addText(title, { x:0.60, y:0.95, w:8.80, h:0.55, fontFace:"Trebuchet MS", fontSize:26, bold:true, color:C.WHITE });
  if (subtitle) slide.addText(subtitle, { x:0.60, y:1.50, w:8.80, h:0.35, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:C.WHITE });
}

function addCard(slide, x, y, w, h, color, header, bodyLines, fontSize) {
  slide.addShape("rect", { x, y, w, h, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.08 });
  slide.addShape("rect", { x, y, w, h:0.42, fill:{color}, line:{type:"none"} });
  slide.addText(header, { x:x+0.15, y:y+0.02, w:w-0.30, h:0.38, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:C.WHITE, valign:"middle" });
  slide.addText(bodyLines, { x:x+0.15, y:y+0.48, w:w-0.30, h:h-0.56, fontFace:"Calibri", fontSize:fontSize||11, color:C.DARK_NAVY, lineSpacingMultiple:1.2, valign:"top" });
}

// ─── Slide Q1 — QP Controller: Mode Evolution (section 10) ────────────

function slideQp1(prs) {
  // Section divider
  sectionSlide(prs, "10", "SYSTEM EVOLUTION: WHOLE-BODY CONTROL",
    "QP modes \u00B7 PERCEPTUAL_SCAN \u00B7 ACTIVE_SEARCH \u00B7 /wbc/qp_mode topic");

  // Content slide
  const s = prs.addSlide();
  lightHeader(s, "QP Controller: Mode Evolution (May \u2192 June 2026)");

  // Three rows: each has a left (May) and right (June) mini-card
  // Row 1: SEARCH_GRID → ACTIVE_SEARCH
  // Row 2: LOOKAT → LOOKAT (unchanged)
  // Row 3: SCAN_SEQ → PERCEPTUAL_SCAN

  const rowH = 1.10;
  const rowGap = 0.12;
  const cardW = 4.40;
  const startY = 1.25;

  // ── Row 1: SEARCH_GRID → ACTIVE_SEARCH ──
  addCard(s, 0.38, startY, cardW, rowH, C.TEAL,
    "May \u2014 SEARCH_GRID (7 poses)",
    "7 hardcoded FK-reader poses\n" +
    "\u03B4=0.15, virtual target body-X\n" +
    "No orientation logic\n" +
    "IK often found unnatural configs",
    9.5);

  addCard(s, 5.22, startY, cardW, rowH, C.GREEN,
    "June \u2014 ACTIVE_SEARCH (6 symmetric poses)",
    "6 mathematically-generated poses\n" +
    "3 forward X=+0.12 + 3 behind X=-0.15\n" +
    "10\u00B0 downward tilt\n" +
    "compute_ee_orientation() for reachability",
    9.5);

  // ── Row 2: LOOKAT → LOOKAT (unchanged) ──
  addCard(s, 0.38, startY + rowH + rowGap, cardW, rowH, C.TEAL,
    "May \u2014 LOOKAT",
    "\u03C9_des = kp_ang * angle * axis\n" +
    "Damped pseudo-inverse on J_task (3\u00D76)\n" +
    "Null-space joint centering\n" +
    "10 Hz loop",
    9.5);

  addCard(s, 5.22, startY + rowH + rowGap, cardW, rowH, C.GREEN,
    "June \u2014 LOOKAT (unchanged)",
    "\u03C9_des = kp_ang * angle * axis\n" +
    "Damped pseudo-inverse on J_task (3\u00D76)\n" +
    "Null-space joint centering\n" +
    "10 Hz loop \u2014 same algorithm",
    9.5);

  // ── Row 3: SCAN_SEQ → PERCEPTUAL_SCAN ──
  addCard(s, 0.38, startY + 2 * (rowH + rowGap), cardW, rowH, C.TEAL,
    "May \u2014 SCAN_SEQ (11 poses)",
    "11 poses, \u03B4=0.12\n" +
    "BodySearchScanner sequencing\n" +
    "3D fusion across all poses\n" +
    "Complex scanner pipeline",
    9.5);

  addCard(s, 5.22, startY + 2 * (rowH + rowGap), cardW, rowH, C.GREEN,
    "June \u2014 PERCEPTUAL_SCAN (6 fixed poses)",
    "Phase 1: 4 wrist sweep 2\u00D72 @ center\n" +
    "Phase 2: 2 lateral parallax \u00B1Y\n" +
    "Tight offsets (4cm/6cm) or wide (12cm/20cm)\n" +
    "Direct Cartesian grid, no BodySearchScanner",
    9.5);

  // Bottom box — new topic
  s.addShape("rect", { x:0.38, y:4.65, w:9.24, h:0.40, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.06 });
  s.addText(
    "New topic: /wbc/qp_mode (String) \u2014 published on every mode change, displayed in web dashboard",
    { x:0.55, y:4.65, w:8.90, h:0.40, fontFace:"Consolas", fontSize:10, color:C.TEAL_DARK, valign:"middle" }
  );

}

// ─── Slide Q2 — PERCEPTUAL_SCAN: Deep Dive ─────────────────────────────

function slideQp2(prs) {
  const s = prs.addSlide();
  lightHeader(s, "PERCEPTUAL_SCAN \u2014 6-Pose Cartesian Grid");

  // LEFT card — Grid Structure (GREEN)
  addCard(s, 0.38, 1.25, 4.40, 2.60, C.GREEN,
    "Grid Structure",
    "Phase 1: 4 wrist sweep poses (2\u00D72 at torso center)\n" +
    "  wy \u2208 {0,1} \u00D7 wz \u2208 {0,1}\n" +
    "Phase 2: 2 lateral parallax poses (\u00B1Y from center)\n" +
    "Total: 6 fixed poses\n" +
    "Center = mean of NLF torso joints\n" +
    "  (SPINE1-3, PELVIS) when NLF valid,\n" +
    "  else target position",
    10.5);

  // RIGHT card — Adaptive Offsets (TEAL)
  addCard(s, 5.22, 1.25, 4.40, 2.60, C.TEAL,
    "Adaptive Offsets",
    "NLF prior valid \u2192 tight:\n" +
    "  4cm wrist step, 6cm lateral step\n" +
    "YOLO only \u2192 wide:\n" +
    "  12cm wrist step, 20cm lateral step\n" +
    "Grid type published on internal topic\n" +
    "  (/wbc/grid_type)",
    10.5);

  // Bottom box
  s.addShape("rect", { x:0.38, y:4.05, w:9.24, h:0.95, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.06 });
  s.addText(
    "All poses have advance X=0.10m toward patient, Z \u2265 0.44m, workspace clipping. " +
    "No more BodySearchScanner \u2014 direct Cartesian grid.",
    { x:0.55, y:4.08, w:8.90, h:0.40, fontFace:"Calibri", fontSize:11, bold:true, color:C.TEAL_DARK, valign:"middle" }
  );
  s.addText(
    "wbc_qp_controller.py:582-626 \u2014 _gen_cartesian_scan_grid()",
    { x:0.55, y:4.50, w:8.90, h:0.30, fontFace:"Consolas", fontSize:8, italic:true, color:C.SEAFOAM, valign:"middle" }
  );

}

// ─── Slide Q3 — QP Controller: Why the Change? ─────────────────────────

function slideQp3(prs) {
  const s = prs.addSlide();
  lightHeader(s, "QP Controller: Motivation for the Redesign");

  // Four numbered blocks in a 2x2 grid
  const cardW = 4.40;
  const cardH = 1.50;
  const gapX = 0.44;
  const gapY = 0.25;
  const startX = 0.38;
  const startY = 1.25;

  // Row 1
  addCard(s, startX, startY, cardW, cardH, C.TEAL,
    "1. 7\u21926 Poses (ACTIVE_SEARCH)",
    "Fewer poses, mathematically generated (not hardcoded)\n" +
    "Guaranteed reachability via compute_ee_orientation()\n" +
    "10\u00B0 tilt for better torso view\n" +
    "IK finds natural arm configuration",
    10);

  addCard(s, startX + cardW + gapX, startY, cardW, cardH, C.GREEN,
    "2. 11\u21926 Poses (PERCEPTUAL_SCAN)",
    "SCAN_SEQ had 11 poses with complex BodySearchScanner\n" +
    "PERCEPTUAL_SCAN uses 6 well-distributed poses\n" +
    "4-point wrist grid (4cm/12cm step based on NLF prior) + 2-point lateral scan (6cm/20cm step)\n" +
    "Deterministic Cartesian coverage vs random null-space sampling",
    10);

  // Row 2
  addCard(s, startX, startY + cardH + gapY, cardW, cardH, C.SEAFOAM,
    "3. Adaptive Grid",
    "NLF prior enables tight grid (more precision at short range)\n" +
    "YOLO-only uses wide grid (more coverage at long range)\n" +
    "Adapts automatically without parameter tuning\n" +
    "Grid type published for diagnostics",
    10);

  addCard(s, startX + cardW + gapX, startY + cardH + gapY, cardW, cardH, C.TEAL_DARK,
    "4. Real-time Monitoring",
    "/wbc/qp_mode topic enables web dashboard status display\n" +
    "Operators can see current QP mode in real-time:\n" +
    "  ACTIVE_SEARCH / LOOKAT / PERCEPTUAL_SCAN / IDLE\n" +
    "Published on every mode change, one-time event logging",
    10);

}

// ─── Exports ───────────────────────────────────────────────────────────

module.exports = {
  slideQp1: function(prs) { slideQp1(prs); },
  slideQp2: function(prs) { slideQp2(prs); },
  slideQp3: function(prs) { slideQp3(prs); },
};
