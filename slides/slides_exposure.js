// ─── Exposure Scanning ───────────────────────────────────────────────────
// Slide functions for the Exposure Scanning feature (body scan + camera).
// Consumed by the Task 9 assembler.
//
// Sources:
//   DESCRIPTION.md lines 165-233 (exposure phase)
//   DESCRIPTION.md lines 334-335 (exposure_scanner node)
// ────────────────────────────────────────────────────────────────────────

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

// ─── Helpers (same as teresa_presentation.js) ───────────────────────────

function darkBg(slide, circleY) {
  slide.background = { fill: C.TEAL };
  slide.addShape("ellipse", { x: 1.20, y: circleY, w: 7.60, h: 7.60, fill: { color: C.TEAL_LIGHT }, line: { type: "none" } });
}

function lightHeader(slide, title, subtitle) {
  slide.background = { fill: C.WHITE };
  slide.addShape("rect", { x: 0.38, y: 0.18, w: 0.06, h: 0.58, fill: { color: C.TEAL }, line: { type: "none" } });
  slide.addText(title, { x: 0.55, y: 0.18, w: 9.10, h: 0.60, fontFace: "Trebuchet MS", fontSize: 26, bold: true, color: C.TEAL_DARK, valign: "middle" });
  if (subtitle) slide.addText(subtitle, { x: 0.55, y: 0.85, w: 9.00, h: 0.28, fontFace: "Trebuchet MS", fontSize: 11, bold: true, color: C.SEAFOAM, valign: "middle" });
}

function sectionSlide(prs, num, title, subtitle) {
  const slide = prs.addSlide();
  darkBg(slide, -0.70);
  slide.addText(num, { x: 0.55, y: 0.25, w: 1.20, h: 0.65, fontFace: "Trebuchet MS", fontSize: 52, bold: true, color: C.WHITE });
  slide.addText(title, { x: 0.60, y: 0.95, w: 8.80, h: 0.55, fontFace: "Trebuchet MS", fontSize: 26, bold: true, color: C.WHITE });
  if (subtitle) slide.addText(subtitle, { x: 0.60, y: 1.50, w: 8.80, h: 0.35, fontFace: "Trebuchet MS", fontSize: 11, bold: true, color: C.WHITE });
}

function addCard(slide, x, y, w, h, color, header, bodyLines, fontSize) {
  slide.addShape("rect", { x, y, w, h, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.08 });
  slide.addShape("rect", { x, y, w, h: 0.42, fill: { color }, line: { type: "none" } });
  slide.addText(header, { x: x + 0.15, y: y + 0.02, w: w - 0.30, h: 0.38, fontFace: "Trebuchet MS", fontSize: 14, bold: true, color: C.WHITE, valign: "middle" });
  slide.addText(bodyLines, { x: x + 0.15, y: y + 0.48, w: w - 0.30, h: h - 0.56, fontFace: "Calibri", fontSize: fontSize || 11, color: C.DARK_NAVY, lineSpacingMultiple: 1.2, valign: "top" });
}

function addNumberedBlock(slide, x, y, w, h, num, text, color) {
  slide.addShape("rect", { x, y, w, h, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addShape("ellipse", { x: x + 0.08, y: y + 0.06, w: 0.30, h: 0.30, fill: { color: color || C.TEAL }, line: { type: "none" } });
  slide.addText(String(num), { x: x + 0.08, y: y + 0.06, w: 0.30, h: 0.30, fontFace: "Calibri", fontSize: 12, bold: true, color: C.WHITE, align: "center", valign: "middle" });
  slide.addText(text, { x: x + 0.46, y: y + 0.04, w: w - 0.56, h: h - 0.08, fontFace: "Calibri", fontSize: 10, color: C.DARK_NAVY, valign: "middle" });
}

// ─── Slide E1 — Exposure Scanning (section 11) ─────────────────────────

function slideExposure1(prs) {
  // Section divider
  sectionSlide(prs, "11", "SYSTEM EVOLUTION: EXPOSURE & BODY SCANNING", "Full-body exposure grid \u00B7 14 points \u00B7 7 regions \u00B7 Interactive web review");

  // Content slide
  const slide = prs.addSlide();
  lightHeader(slide, "Exposure Scanning");

  // LEFT card — Full-Body Grid (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 3.20,
    C.TEAL,
    "Full-Body Grid",
    [
      "\u2022 14 points across 7 body regions:",
      "    HEAD(2), TORSO(4), LEFT ARM(2),",
      "    RIGHT ARM(2), LEFT LEG(2),",
      "    RIGHT LEG(2), FEET(2)",
      "\u2022 Generated from 17 COCO keypoints",
      "    (Orbbec) transformed to world frame",
      "    via TF",
      "\u2022 NLF prior grid: uses 24 SMPL",
      "    EMA-refined skeleton when available,",
      "    fallback to YOLO keypoints",
    ].join("\n"),
    10.5
  );

  // RIGHT card — exposure_scanner Node (GREEN)
  addCard(
    slide,
    5.22, 1.30, 4.40, 3.20,
    C.GREEN,
    "exposure_scanner Node",
    [
      "\u2022 Dedicated node (650 lines,",
      "    exposure_scanner.py)",
      "\u2022 Same per-point pattern as FAST",
      "    ultrasound",
      "\u2022 Dynamic look-at via",
      "    compute_ee_orientation()",
      "\u2022 Horizontal standoff 0.50m toward",
      "    Spot (not vertical)",
      "\u2022 Head estimation from shoulders",
      "    if nose occluded",
    ].join("\n"),
    10.5
  );

  // Bottom box — progressive refined skeleton
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "Progressive refined skeleton: accumulates /exposure/body_keypoints during dwell, running average \u03B1=0.5, publishes on /exposure/refined_skeleton (0/17 \u2192 17/17)",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9, color: C.TEAL_DARK, valign: "middle" }
  );

}

// ─── Slide E2 — Exposure Pipeline & Web Integration ─────────────────────

function slideExposure2(prs) {
  const slide = prs.addSlide();
  lightHeader(slide, "Exposure: Pipeline & Interactive Web UI");

  // Pipeline flow — numbered blocks
  const blockW = 1.60;
  const blockH = 0.55;
  const gapX = 0.18;
  const arrowW = 0.30;
  const startX = 0.38;
  const startY = 1.30;

  const steps = [
    { num: 1, text: "body_pose(h,p)\nsettle 1.5s" },
    { num: 2, text: "body_ready\n\u2192 IK goal" },
    { num: 3, text: "ik_done\ndwell 2s" },
    { num: 4, text: "running avg kp\n\u2192 next point" },
  ];

  steps.forEach((step, i) => {
    const bx = startX + i * (blockW + gapX + arrowW);
    addNumberedBlock(slide, bx, startY, blockW, blockH, step.num, step.text, C.TEAL);
    // Arrow between blocks
    if (i < steps.length - 1) {
      const ax = bx + blockW + 0.02;
      slide.addText("\u2192", { x: ax, y: startY, w: arrowW, h: blockH, fontFace: "Calibri", fontSize: 16, bold: true, color: C.TEAL, align: "center", valign: "middle" });
    }
  });

  // Right side — Web UI Integration card
  addCard(
    slide,
    5.22, 1.30, 4.40, 3.20,
    C.GREEN,
    "Web UI Integration",
    [
      "\u2022 Grid toggle on RealSense camera",
      "    view ([Grid] button)",
      "\u2022 Color legend bar: 7 region swatches",
      "    HEAD=gold, TORSO=blue, L-ARM=red,",
      "    R-ARM=orange, L-LEG=green,",
      "    R-LEG=light green, FEET=purple",
      "\u2022 Current point: large marker + white",
      "    glow. Visited: small transparent.",
      "    Unvisited: medium semi-transparent.",
      "\u2022 Click-to-revisit: click marker",
      "    \u2192 /exposure/goto_point(id)",
      "    \u2192 Spot repositions arm",
      "\u2022 Body Map panel: toggle via \ud83c\udf10",
      "    or key m. Canvas top-down (X-Y",
      "    world) with progressive skeleton",
      "    17 kp + COCO lines + exposure grid.",
      "    Auto-scaled, auto-fit.",
    ].join("\n"),
    9.5
  );

  // Bottom box — JSON output
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "JSON output: /tmp/exposure_scan_YYYYMMDD_HHMMSS.json with per-region camera pose, surface position, scan data frames",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9, color: C.TEAL_DARK, valign: "middle" }
  );

}

// ─── Slide E3 — Exposure: FSM Integration & Review Mode ─────────────────

function slideExposure3(prs) {
  const slide = prs.addSlide();
  lightHeader(slide, "Exposure: FSM States & Interactive Review");

  // LEFT card — New FSM States (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 3.20,
    C.TEAL,
    "New FSM States",
    [
      "\u2022 WAITING_EXPOSURE: awaiting",
      "    confirmation before scan",
      "\u2022 EXPOSURE_SCANNING: active",
      "    14-point scan in progress",
      "\u2022 EXPOSURE_REVIEW: interactive",
      "    operator review after scan",
      "\u2022 WAITING_FAST: before FAST",
      "    ultrasound phase",
      "\u2022 Manual scan gate:",
      "    manual_scan_gate param (default",
      "    true), confirm via keyboard n or",
      "    web STEP button",
      "\u2022 Toggle MANUAL/AUTO in web UI",
    ].join("\n"),
    10.5
  );

  // RIGHT card — New ROS Topics table (GREEN)
  // Table header
  const colX = [5.22, 6.60, 8.00];
  const colW = [1.38, 1.40, 1.62];
  const rowH = 0.38;
  const tableStartY = 1.30;

  slide.addShape("rect", { x: colX[0], y: tableStartY, w: colW[0] + colW[1] + colW[2], h: rowH, fill: { color: C.GREEN }, line: { type: "none" } });
  slide.addText("Topic", { x: colX[0], y: tableStartY, w: colW[0], h: rowH, fontFace: "Trebuchet MS", fontSize: 10, bold: true, color: C.WHITE, align: "center", valign: "middle" });
  slide.addText("Type", { x: colX[1], y: tableStartY, w: colW[1], h: rowH, fontFace: "Trebuchet MS", fontSize: 10, bold: true, color: C.WHITE, align: "center", valign: "middle" });
  slide.addText("Role", { x: colX[2], y: tableStartY, w: colW[2], h: rowH, fontFace: "Trebuchet MS", fontSize: 10, bold: true, color: C.WHITE, align: "center", valign: "middle" });

  const topics = [
    { topic: "/exposure/grid_markers", type: "MarkerArray", role: "grid overlay" },
    { topic: "/exposure/goto_point",   type: "Int32",       role: "click-to-revisit" },
    { topic: "/exposure/terminate",    type: "Bool",        role: "end exposure" },
    { topic: "/exposure/ready",        type: "Bool",        role: "scan complete" },
    { topic: "/exposure/refined_skeleton", type: "PoseArray", role: "progressive 17-kp" },
    { topic: "/wbc/set_manual_scan_gate", type: "Bool",     role: "manual/auto toggle" },
  ];

  topics.forEach((row, i) => {
    const ry = tableStartY + rowH + i * 0.38;
    const bgColor = i % 2 === 0 ? C.OFF_WHITE : C.WHITE;
    slide.addShape("rect", { x: colX[0], y: ry, w: colW[0] + colW[1] + colW[2], h: 0.38, fill: { color: bgColor }, line: { type: "none" } });
    slide.addText(row.topic, { x: colX[0] + 0.06, y: ry, w: colW[0] - 0.12, h: 0.38, fontFace: "Consolas", fontSize: 7.5, color: C.DARK_NAVY, valign: "middle" });
    slide.addText(row.type,  { x: colX[1] + 0.06, y: ry, w: colW[1] - 0.12, h: 0.38, fontFace: "Consolas", fontSize: 7.5, color: C.DARK_NAVY, valign: "middle" });
    slide.addText(row.role,  { x: colX[2] + 0.06, y: ry, w: colW[2] - 0.12, h: 0.38, fontFace: "Calibri", fontSize: 8, color: C.DARK_NAVY, valign: "middle" });
  });

  // Bottom box — EXPOSURE_REVIEW description
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "EXPOSURE_REVIEW: operator clicks any blue grid point \u2192 Spot returns to optimized body_pose \u2192 arm replays IK trajectory \u2192 camera frames region until next click",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9, color: C.TEAL_DARK, valign: "middle" }
  );

}

// ─── Export ─────────────────────────────────────────────────────────────

module.exports = {
  slideExposure1: slideExposure1,
  slideExposure2: slideExposure2,
  slideExposure3: slideExposure3,
};
