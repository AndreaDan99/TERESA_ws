// ─── SEARCHING Phase Evolution Slides (May → June 2026) ───────────────
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

const UNIFE_LOGO = "/Users/andrea/Documents/UNIFE/DOTTORATO/Presentazioni/Unife_logo.png";
const SLIDE_W = 10.0;
const SLIDE_H = 5.625;

// ─── Helpers ───────────────────────────────────────────────────────────

function addFooter(slide) {
  slide.addShape("rect", { x:0, y:5.17, w:SLIDE_W, h:0.45, fill:{color:C.DARK_NAVY}, line:{type:"none"} });
  slide.addImage({ path:UNIFE_LOGO, x:4.46, y:5.22, w:1.09, h:0.35 });
}

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
  addFooter(slide);
}

function addCard(slide, x, y, w, h, color, header, bodyLines, fontSize) {
  slide.addShape("rect", { x, y, w, h, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.08 });
  slide.addShape("rect", { x, y, w, h:0.42, fill:{color}, line:{type:"none"} });
  slide.addText(header, { x:x+0.15, y:y+0.02, w:w-0.30, h:0.38, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:C.WHITE, valign:"middle" });
  slide.addText(bodyLines, { x:x+0.15, y:y+0.48, w:w-0.30, h:h-0.56, fontFace:"Calibri", fontSize:fontSize||11, color:C.DARK_NAVY, lineSpacingMultiple:1.2, valign:"top" });
}

// ─── SLIDE S1: Old vs New Approach ────────────────────────────────────

function slideSearching1(prs) {
  // Section divider
  sectionSlide(prs, "08", "SYSTEM EVOLUTION: SEARCHING", "From incremental grid to timed open-loop rotation");

  // Content slide
  const s = prs.addSlide();
  lightHeader(s, "SEARCHING: Old vs New Approach", "May 2026 vs June 2026 — two fundamentally different strategies");

  // LEFT card — May 2026
  addCard(s, 0.40, 1.25, 4.40, 2.60, C.TEAL,
    "May 2026 — Incremental Grid Search",
    "18 positions: 6 yaw x 3 pitch\n" +
    "Yaw sequence: 0, +60, -60, +120, -120, +180\n" +
    "Pitch angles: 0, 5, 10 (nose-down)\n" +
    "7 arm exploration poses (FK-reader)\n" +
    "15s dwell per position\n" +
    "TF-dependent: odom to body lookup\n" +
    "Full restart on TF loss",
    10.5);

  // RIGHT card — June 2026
  addCard(s, 5.20, 1.25, 4.40, 2.60, C.GREEN,
    "June 2026 — Timed Open-Loop Rotation",
    "Timed +-30 yaw rotation via cmd_vel\n" +
    "No TF dependency (open-loop timing)\n" +
    "6 symmetric mathematically-generated poses\n" +
    "search_timeout_per_point: 1.2s\n" +
    "20cm step forward after each cycle\n" +
    "10 downward camera tilt\n" +
    "Resume from current position on recovery",
    10.5);

  // Bottom callout box
  s.addShape("rect", { x:0.40, y:4.05, w:9.20, h:0.95, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.06 });
  s.addText("Improvement: ~70% faster search cycle, independent from TF health, 10 downward camera tilt",
    { x:0.60, y:4.10, w:8.80, h:0.40, fontFace:"Calibri", fontSize:12, bold:true, color:C.TEAL_DARK, valign:"middle" });
  s.addText("CHANGELOG: 8 June 2026 — SEARCHING rewrite",
    { x:0.60, y:4.50, w:8.80, h:0.30, fontFace:"Consolas", fontSize:8, italic:true, color:C.SEAFOAM, valign:"middle" });

  addFooter(s);
}

// ─── SLIDE S2: Arm Poses Evolution ────────────────────────────────────

function slideSearching2(prs) {
  const s = prs.addSlide();
  lightHeader(s, "SEARCHING: Arm Poses Evolution", "From 7 FK-reader hardcoded poses to 6 mathematically-generated symmetric poses");

  // LEFT card — May 2026
  addCard(s, 0.40, 1.25, 4.40, 2.80, C.TEAL,
    "7 FK-Reader Hardcoded Poses",
    "Quaternions forced from FK-reader file\n" +
    "7 fixed poses, no orientation logic\n" +
    "No mathematical reachability guarantee\n" +
    "IK often found unnatural configurations\n" +
    "Hardcoded per-joint angles\n" +
    "No camera tilt control\n" +
    "Difficult to maintain or extend",
    10.5);

  // RIGHT card — June 2026
  addCard(s, 5.20, 1.25, 4.40, 2.80, C.GREEN,
    "6 Mathematically-Generated Symmetric Poses",
    "3 forward: X=+0.12, Z=0.53, Y=+-0.20\n" +
    "3 look-behind: X=-0.15, Z=0.53, Y=+-0.20\n" +
    "Orientation via compute_ee_orientation()\n" +
    "X_ee points forward or backward\n" +
    "Y_ee stays near home orientation\n" +
    "10 downward camera tilt\n" +
    "IK finds natural arm configuration",
    10.5);

  // Bottom callout
  s.addShape("rect", { x:0.40, y:4.25, w:9.20, h:0.75, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.06 });
  s.addText("Key change: no hardcoded quaternions, mathematically guaranteed reachability",
    { x:0.60, y:4.28, w:8.80, h:0.35, fontFace:"Calibri", fontSize:12, bold:true, color:C.TEAL_DARK, valign:"middle" });
  s.addText("CHANGELOG: 8-10 June 2026 — unified search poses",
    { x:0.60, y:4.62, w:8.80, h:0.28, fontFace:"Consolas", fontSize:8, italic:true, color:C.SEAFOAM, valign:"middle" });

  addFooter(s);
}

// ─── SLIDE S3: Performance Comparison ─────────────────────────────────

function slideSearching3(prs) {
  const s = prs.addSlide();
  lightHeader(s, "SEARCHING: Performance Comparison", "May 2026 vs June 2026 — side-by-side metrics");

  // Table header row
  const colX = [0.40, 3.20, 6.40];
  const colW = [2.80, 3.20, 3.20];
  const rowH = 0.42;
  const startY = 1.25;

  // Header bar
  s.addShape("rect", { x:colX[0], y:startY, w:colW[0]+colW[1]+colW[2], h:rowH, fill:{color:C.TEAL_DARK}, line:{type:"none"} });
  s.addText("Metric", { x:colX[0], y:startY, w:colW[0], h:rowH, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.WHITE, align:"center", valign:"middle" });
  s.addText("May 2026", { x:colX[1], y:startY, w:colW[1], h:rowH, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.WHITE, align:"center", valign:"middle" });
  s.addText("June 2026", { x:colX[2], y:startY, w:colW[2], h:rowH, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.WHITE, align:"center", valign:"middle" });

  // Table rows
  const rows = [
    { metric: "Search time per cycle", may: "~90s (18 x 5s dwell)", jun: "~7s (2 yaw x 1.2s + poses)" },
    { metric: "TF dependency",        may: "Requires odom to body TF", jun: "None (open-loop timing)" },
    { metric: "Arm poses",            may: "7 hardcoded (FK-reader)", jun: "6 symmetric (mathematical)" },
    { metric: "Pose generation",      may: "FK-reader file",          jun: "compute_ee_orientation()" },
    { metric: "Camera angle",         may: "Horizontal (0 tilt)",     jun: "10 downward tilt" },
    { metric: "Recovery",             may: "Full restart on TF loss", jun: "Resume from current position" },
  ];

  rows.forEach((row, i) => {
    const ry = startY + rowH + i * 0.52;
    const bgColor = i % 2 === 0 ? C.OFF_WHITE : C.WHITE;
    s.addShape("rect", { x:colX[0], y:ry, w:colW[0]+colW[1]+colW[2], h:0.52, fill:{color:bgColor}, line:{type:"none"} });
    s.addText(row.metric, { x:colX[0]+0.10, y:ry, w:colW[0]-0.20, h:0.52, fontFace:"Calibri", fontSize:10.5, bold:true, color:C.TEAL_DARK, valign:"middle" });
    s.addText(row.may,     { x:colX[1]+0.10, y:ry, w:colW[1]-0.20, h:0.52, fontFace:"Calibri", fontSize:10, color:C.DARK_NAVY, valign:"middle" });
    s.addText(row.jun,     { x:colX[2]+0.10, y:ry, w:colW[2]-0.20, h:0.52, fontFace:"Calibri", fontSize:10, color:C.GREEN, valign:"middle" });
  });

  // Source line
  s.addText("CHANGELOG: 8 June 2026 and 10 June 2026",
    { x:0.55, y:5.00, w:9.00, h:0.20, fontFace:"Consolas", fontSize:8, italic:true, color:C.SEAFOAM });

  addFooter(s);
}

// ─── Exports ───────────────────────────────────────────────────────────

module.exports = {
  slideSearching1: function(prs) { slideSearching1(prs); },
  slideSearching2: function(prs) { slideSearching2(prs); },
  slideSearching3: function(prs) { slideSearching3(prs); },
};
