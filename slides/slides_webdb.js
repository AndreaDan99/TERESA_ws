// ─── Web Dashboard — NEW (June 2026) ────────────────────────────────────
// Slide functions for the Web Dashboard feature.
// Consumed by the Task 9 assembler.
//
// Sources:
//   CHANGELOG.md lines 18-22, 38, 43-46 (10 June 2026 — Web Dashboard)
//   INIT.md lines 43-47 (Web Dashboard current state)
// ────────────────────────────────────────────────────────────────────────

const C = {
  TEAL:       "1F7985",
  TEAL_LIGHT: "80C8CF",
  DARK_NAVY:  "0D3340",
  TEAL_DARK:  "0D6756",
  GREEN:      "2D714C",
  SEAFOAM:    "389896",
  OFF_WHITE:  "E8E8DF",
  WHITE:      "FFFFFF",
};

const UNIFE_LOGO = "/Users/andrea/Documents/UNIFE/DOTTORATO/Presentazioni/Unife_logo.png";
const SLIDE_W = 10.0;
const SLIDE_H = 5.625;

// ─── Helpers (copied from teresa_presentation.js) ───────────────────────

function addFooter(slide) {
  slide.addShape("rect", { x: 0, y: 5.17, w: SLIDE_W, h: 0.45, fill: { color: C.DARK_NAVY }, line: { type: "none" } });
  slide.addImage({ path: UNIFE_LOGO, x: 4.46, y: 5.22, w: 1.09, h: 0.35 });
}

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
  addFooter(slide);
}

function addCard(slide, x, y, w, h, color, header, bodyLines, fontSize) {
  slide.addShape("rect", { x, y, w, h, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.08 });
  slide.addShape("rect", { x, y, w, h: 0.42, fill: { color }, line: { type: "none" } });
  slide.addText(header, { x: x + 0.15, y: y + 0.02, w: w - 0.30, h: 0.38, fontFace: "Trebuchet MS", fontSize: 14, bold: true, color: C.WHITE, valign: "middle" });
  slide.addText(bodyLines, { x: x + 0.15, y: y + 0.48, w: w - 0.30, h: h - 0.56, fontFace: "Calibri", fontSize: fontSize || 11, color: C.DARK_NAVY, lineSpacingMultiple: 1.2, valign: "top" });
}

// ─── Slide W1 — Web Dashboard (section 11) ──────────────────────────────

function slideWebDb1(prs) {
  // Section divider
  sectionSlide(prs, "11", "SYSTEM EVOLUTION: WEB INTERFACE", "Component Status Grid · Real-Time Monitoring · rosbridge Integration");

  // Content slide
  const slide = prs.addSlide();
  lightHeader(slide, "Web Dashboard \u2014 NEW (June 2026)");

  // LEFT card — Component Status Grid (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 3.20,
    C.TEAL,
    "Component Status Grid",
    [
      "\u2022 4-card dashboard with colored status dots:",
      "    \u2022 IK (Inverse Kinematics solver status)",
      "    \u2022 Orbbec (Femto Bolt RGB-D camera)",
      "    \u2022 RealSense (D435 end-effector camera)",
      "    \u2022 QP (Whole-Body Controller mode)",
      "\u2022 Status dot colors:",
      "    \u25CF green = active / healthy",
      "    \u25CF yellow = degraded",
      "    \u25CF gray = inactive / unknown",
      "\u2022 One-time event logging: logs only on",
      "    state change \u2014 no 10 Hz spam",
      "\u2022 Works independently of camera panel",
      "    (subscriptions in initTopics(), always active)",
    ].join("\n"),
    10.5
  );

  // RIGHT card — Real-Time Monitoring (GREEN)
  addCard(
    slide,
    5.22, 1.30, 4.40, 3.20,
    C.GREEN,
    "Real-Time Monitoring",
    [
      "\u2022 /wbc/qp_mode topic (String):",
      "    \u2013 Published by QP controller on every",
      "      mode change",
      "    \u2013 Modes: ACTIVE_SEARCH, LOOKAT,",
      "      PERCEPTUAL_SCAN, IDLE",
      "\u2022 /ik_done subscription:",
      "    \u2013 Shows when arm reached target",
      "\u2022 Posture and tracker state subscriptions",
      "\u2022 All status dots update in real-time",
      "    via ROS 2 topic callbacks",
    ].join("\n"),
    10.5
  );

  // Bottom box — implementation detail
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "Implementation: teresa_control.html (+88 lines CSS/HTML/JS). Added to existing web interface without breaking camera panel.",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9.5, color: C.TEAL_DARK, valign: "middle" }
  );

  addFooter(slide);

  // Source annotation (invisible, for traceability)
  // CHANGELOG: 10 June 2026 — Web Dashboard
}

// ─── Slide W2 — Web Dashboard: Architecture & Integration ───────────────

function slideWebDb2(prs) {
  const slide = prs.addSlide();
  lightHeader(slide, "Web Dashboard: Architecture Overview");

  // LEFT card — Subscription Architecture (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 3.20,
    C.TEAL,
    "Subscription Architecture",
    [
      "\u2022 /ik_done \u2192 IK status dot",
      "\u2022 /wbc/qp_mode \u2192 QP mode display",
      "    (ACTIVE_SEARCH / LOOKAT /",
      "     PERCEPTUAL_SCAN / IDLE)",
      "\u2022 Orbbec image topic \u2192 Orbbec status dot",
      "    (checks if frames arriving)",
      "\u2022 RealSense image topic \u2192 RealSense",
      "    status dot",
      "\u2022 Posture topic \u2192 posture classifier",
      "    status",
      "\u2022 Tracker topic \u2192 torso tracker status",
    ].join("\n"),
    10.5
  );

  // RIGHT card — Web Interface Features (GREEN)
  addCard(
    slide,
    5.22, 1.30, 4.40, 3.20,
    C.GREEN,
    "Web Interface Features",
    [
      "\u2022 Pre-existing:",
      "    \u2013 Web joystick panel (D-pad, Drive/Body",
      "      modes, speed control, height/pitch",
      "      sliders, HOME/PARK arm buttons)",
      "    \u2013 Camera view with YOLO overlay",
      "    \u2013 Body Map panel (toggle via \ud83c\udf10 or key m)",
      "\u2022 NEW: Component status grid (always visible)",
      "\u2022 NEW: MANUAL/AUTO toggle for scan gate",
      "\u2022 NEW: Grid toggle button for exposure overlay",
    ].join("\n"),
    10.5
  );

  // Bottom box — integration note
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "All components communicate via rosbridge WebSocket. No backend changes needed \u2014 pure frontend addition.",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9.5, color: C.TEAL_DARK, valign: "middle" }
  );

  addFooter(slide);

  // Source annotation (invisible, for traceability)
  // CHANGELOG: 8-10 June 2026
}

// ─── Export ─────────────────────────────────────────────────────────────

module.exports = {
  slideWebDb1: slideWebDb1,
  slideWebDb2: slideWebDb2,
};
