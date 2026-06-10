// ─── LOCKING Phase Evolution — May → June 2026 ──────────────────────────
// Slide functions for the LOCKING gate logic and NLF confidence blending.
// Consumed by the Task 9 assembler.
//
// Sources:
//   DESCRIPTION.md lines 124-134 (LOCKING → PRE_APPROACH)
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

const SLIDE_W = 10.0;
const SLIDE_H = 5.625;

// ─── Helpers (copied from teresa_presentation.js) ───────────────────────

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

// ─── Slide L1 — LOCKING: Old vs New Gate Logic (section 09) ─────────────

function slideLocking1(prs) {
  // Section divider
  sectionSlide(prs, "09", "SYSTEM EVOLUTION: PERCEPTION", "NLF Burst Streaming \u00B7 LOCKING Gate Logic \u00B7 Confidence Blending");

  // Content slide
  const slide = prs.addSlide();
  lightHeader(slide, "LOCKING: Old vs New Gate Logic");

  // LEFT card — May 2026 (TEAL)
  addCard(
    slide,
    0.38, 1.30, 4.40, 2.80,
    C.TEAL,
    "May 2026 \u2014 Simple Lock",
    [
      "\u2022 5 samples collected @10Hz (~0.5s)",
      "\u2022 Arm returns to home position",
      "\u2022 1s Orbbec loss tolerance",
      "\u2022 If Orbbec lost >1s \u2192 resume search",
      "\u2022 No NLF dependency",
      "\u2022 Direct transition to PRE_APPROACH",
      "    after samples + ik_done",
    ].join("\n"),
    10.5
  );

  // RIGHT card — June 2026 (GREEN)
  addCard(
    slide,
    5.22, 1.30, 4.40, 2.80,
    C.GREEN,
    "June 2026 \u2014 NLF-Gated Lock",
    [
      "\u2022 NLF burst trigger at LOCKING entry",
      "\u2022 Gate: 5 samples + ik_done +",
      "    (NLF valid or 30s timeout)",
      "\u2022 YOLO publish suppressed during burst",
      "    (/human_pose/points_3d)",
      "\u2022 Best pitch from refinement applied",
      "    on ALL LOCKING entry paths",
      "\u2022 NLF trigger moved to top of",
      "    _tick_locking() \u2014 no deadlock",
    ].join("\n"),
    10.5
  );

  // Bottom callout
  slide.addShape("rect", { x: 0.38, y: 4.30, w: 9.24, h: 0.40, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "Improvement: Dense 24-joint SMPL skeleton available BEFORE approach begins",
    { x: 0.55, y: 4.30, w: 8.90, h: 0.40, fontFace: "Consolas", fontSize: 10, color: C.TEAL_DARK, valign: "middle" }
  );

}

// ─── Slide L2 — LOCKING: NLF Confidence-Driven Blending ─────────────────

function slideLocking2(prs) {
  const slide = prs.addSlide();
  lightHeader(slide, "LOCKING: NLF Confidence-Driven Blending");

  // Four tier cards in a 2x2 grid (~4.4" x 1.3" each)
  const cardW = 4.40;
  const cardH = 1.30;
  const gapX = 0.44;
  const gapY = 0.25;
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
      "\u2022 Balanced NLF-YOLO fusion",
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

  // Bottom box — recovery
  slide.addShape("rect", { x: 0.38, y: 4.30, w: 9.24, h: 0.75, fill: { color: C.OFF_WHITE }, line: { type: "none" }, rectRadius: 0.06 });
  slide.addText(
    "Recovery: Orbbec loss \u22641s tolerated during LOCKING. Timeout 30s \u2192 fallback to YOLO approach point. Resume search from CURRENT position (no restart).",
    { x: 0.55, y: 4.30, w: 8.90, h: 0.75, fontFace: "Calibri", fontSize: 10.5, color: C.TEAL_DARK, valign: "middle", lineSpacingMultiple: 1.3 }
  );

}

// ─── Export ─────────────────────────────────────────────────────────────

module.exports = {
  slideLocking1: slideLocking1,
  slideLocking2: slideLocking2,
};
