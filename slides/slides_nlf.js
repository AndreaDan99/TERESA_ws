// ─── NLF Burst Streaming — NEW (June 2026) ─────────────────────────────
// Slide functions for the NLF Burst Streaming + Confidence Gate feature.
// Consumed by the Task 9 assembler.
//
// Sources:
//   CHANGELOG.md lines 50-80  (9 June 2026 — NLF Burst Streaming)
//   DESCRIPTION.md lines 19-31 (perception backends)
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

// ─── Slide N1 — NLF Burst Streaming (section 09) ───────────────────────

function slideNlf1(prs) {
  // Section divider
  sectionSlide(prs, "09", "SYSTEM EVOLUTION: PERCEPTION", "NLF Burst Streaming · Confidence Gate · Dual Backend Architecture");

  // Content slide
  const slide = prs.addSlide();
  lightHeader(slide, "NLF Burst Streaming \u2014 NEW (June 2026)");

  // LEFT card — Burst Mechanism (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 3.20,
    C.TEAL,
    "Burst Mechanism",
    [
      "\u2022 NLF trigger fires at LOCKING state entry",
      "\u2022 Collects burst of multi-frame detections",
      "\u2022 Requires 2 valid detections:",
      "    \u2013 Lying person classification",
      "    \u2013 4+ torso joints non-NaN",
      "\u2022 EMA smoothing across burst frames",
      "\u2022 Refined 24-joint SMPL skeleton published",
      "    on /exposure/nlf_prior",
      "\u2022 Auto-pause after burst completes",
      "\u2022 Timeout: 30s (configurable)",
    ].join("\n"),
    10.5
  );

  // RIGHT card — Dual Backend Architecture (GREEN)
  addCard(
    slide,
    5.22, 1.30, 4.40, 3.20,
    C.GREEN,
    "Dual Backend Architecture",
    [
      "\u2022 YOLO11n-pose (default, ~40 FPS):",
      "    \u2013 17 COCO + 7 NaN \u2192 24 SMPL joints",
      "    \u2013 Active during SEARCHING phase",
      "\u2022 NLF (~2.5 FPS):",
      "    \u2013 Starts in paused mode",
      "    \u2013 Burst triggered at LOCKING",
      "    \u2013 Publishes refined priors",
      "\u2022 Backend selection at launch:",
      "    perception_backend:=yolo|nlf",
    ].join("\n"),
    10.5
  );

  // Bottom box — invariant
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "/human_pose/points_3d always carries 24 SMPL joints regardless of backend",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 10, color: C.TEAL_DARK, valign: "middle" }
  );

  addFooter(slide);

  // Source annotation (invisible, for traceability)
  // CHANGELOG: 9 June 2026 — NLF Burst Streaming
}

// ─── Slide N2 — NLF Confidence Gate ─────────────────────────────────────

function slideNlf2(prs) {
  const slide = prs.addSlide();
  lightHeader(slide, "NLF Confidence Gate \u2014 NEW (June 2026)");

  // Four confidence tier cards in a 2x2 grid
  const cardW = 4.40;
  const cardH = 1.60;
  const gapX = 0.44;
  const gapY = 0.30;
  const startX = 0.38;
  const startY = 1.30;

  // Row 1
  addCard(
    slide,
    startX, startY, cardW, cardH,
    C.TEAL,
    "EXCELLENT (\u22650.80)",
    [
      "\u2022 100% NLF blending",
      "\u2022 Skip positional delta check",
      "\u2022 Published on /exposure/nlf_confidence",
      "    (Float32, mean bbox_score)",
    ].join("\n"),
    10.5
  );

  addCard(
    slide,
    startX + cardW + gapX, startY, cardW, cardH,
    C.GREEN,
    "HIGH",
    [
      "\u2022 70% NLF + 30% YOLO blend",
      "\u2022 NLF prior weighted heavily",
      "\u2022 YOLO provides positional correction",
    ].join("\n"),
    10.5
  );

  // Row 2
  addCard(
    slide,
    startX, startY + cardH + gapY, cardW, cardH,
    C.SEAFOAM,
    "MEDIUM",
    [
      "\u2022 50% NLF + 50% YOLO blend",
      "\u2022 Equal weighting between backends",
      "\u2022 Balanced fusion strategy",
    ].join("\n"),
    10.5
  );

  addCard(
    slide,
    startX + cardW + gapX, startY + cardH + gapY, cardW, cardH,
    C.DARK_NAVY,
    "LOW",
    [
      "\u2022 YOLO only (fallback)",
      "\u2022 NLF prior discarded",
      "\u2022 Pure YOLO keypoint pipeline",
    ].join("\n"),
    10.5
  );

  // Bottom box — publish suppression
  slide.addShape("rect", { x: 0.38, y: 4.65, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "Publish suppression: /human_pose/points_3d suppressed during active NLF burst to avoid YOLO conflict",
    { x: 0.55, y: 4.65, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 9.5, color: C.TEAL_DARK, valign: "middle" }
  );

  addFooter(slide);

  // Source annotation (invisible, for traceability)
  // CHANGELOG: 9 June 2026 — EXCELLENT confidence tier
}

// ─── Export ─────────────────────────────────────────────────────────────

module.exports = {
  slideNlf1: slideNlf1,
  slideNlf2: slideNlf2,
};
