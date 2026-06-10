const PptxGenJS = require("pptxgenjs");
const path = require("path");

// ─── Constants (Andrea's Design System) ───────────────────────────────
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
const OUT = "/Users/andrea/Documents/UNIFE/DOTTORATO/Presentazioni/TERESA/TERESA_comparison.pptx";
const SLIDE_W = 10.0;
const SLIDE_H = 5.625;

// ─── Helpers ───────────────────────────────────────────────────────────

/** Add Unife footer bar to every slide */
function addFooter(slide) {
  slide.addShape("rect", { x:0, y:5.17, w:SLIDE_W, h:0.45, fill:{color:C.DARK_NAVY}, line:{type:"none"} });
  slide.addImage({ path:UNIFE_LOGO, x:4.46, y:5.22, w:1.09, h:0.35 });
}

/** Dark slide background: teal fill + lighter decorative circle */
function darkBg(slide, circleY) {
  slide.background = { fill: C.TEAL };
  slide.addShape("ellipse", { x:1.20, y:circleY, w:7.60, h:7.60, fill:{color:C.TEAL_LIGHT}, line:{type:"none"} });
}

/** Light slide header: left bar + title + subtitle label */
function lightHeader(slide, title, subtitle) {
  slide.background = { fill: C.WHITE };
  slide.addShape("rect", { x:0.38, y:0.18, w:0.06, h:0.58, fill:{color:C.TEAL}, line:{type:"none"} });
  slide.addText(title, { x:0.55, y:0.18, w:9.10, h:0.60, fontFace:"Trebuchet MS", fontSize:26, bold:true, color:C.TEAL_DARK, valign:"middle" });
  if (subtitle) slide.addText(subtitle, { x:0.55, y:0.85, w:9.00, h:0.28, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:C.SEAFOAM, valign:"middle" });
}

/** Section divider slide */
function sectionSlide(prs, num, title, subtitle) {
  const slide = prs.addSlide();
  darkBg(slide, -0.70);
  slide.addText(num, { x:0.55, y:0.25, w:1.20, h:0.65, fontFace:"Trebuchet MS", fontSize:52, bold:true, color:C.WHITE });
  slide.addText(title, { x:0.60, y:0.95, w:8.80, h:0.55, fontFace:"Trebuchet MS", fontSize:26, bold:true, color:C.WHITE });
  if (subtitle) slide.addText(subtitle, { x:0.60, y:1.50, w:8.80, h:0.35, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:C.WHITE });
  addFooter(slide);
}

/** Card component: rounded rect with header bar + body text */
function addCard(slide, x, y, w, h, color, header, bodyLines, fontSize) {
  slide.addShape("rect", { x, y, w, h, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.08 });
  slide.addShape("rect", { x, y, w, h:0.42, fill:{color}, line:{type:"none"} });
  slide.addText(header, { x:x+0.15, y:y+0.02, w:w-0.30, h:0.38, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:C.WHITE, valign:"middle" });
  slide.addText(bodyLines, { x:x+0.15, y:y+0.48, w:w-0.30, h:h-0.56, fontFace:"Calibri", fontSize:fontSize||11, color:C.DARK_NAVY, lineSpacingMultiple:1.2, valign:"top" });
}

/** Phase flow card: colored top bar + list */
function addPhaseFlow(slide, x, y, w, h, color, title, steps) {
  slide.addShape("rect", { x, y, w, h, fill:{color:C.OFF_WHITE}, line:{type:"none"}, rectRadius:0.06 });
  slide.addShape("rect", { x, y, w, h:0.40, fill:{color}, line:{type:"none"} });
  slide.addText(title, { x:x+0.12, y:y+0.02, w:w-0.24, h:0.36, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.WHITE, valign:"middle" });
  slide.addText(steps, { x:x+0.12, y:y+0.46, w:w-0.24, h:h-0.54, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15, valign:"top" });
}

/** Numbered block with colored badge */
function addNumberedBlock(slide, num, color, x, y, title, body, w) {
  slide.addShape("rect", { x, y, w:0.45, h:0.50, fill:{color}, line:{type:"none"} });
  slide.addText(String(num), { x, y, w:0.45, h:0.50, fontFace:"Trebuchet MS", fontSize:20, bold:true, color:C.WHITE, align:"center", valign:"middle" });
  slide.addText(title, { x:x+0.58, y:y-0.02, w:w-0.60, h:0.30, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:color });
  slide.addText(body, { x:x+0.58, y:y+0.28, w:w-0.60, h:0.28, fontFace:"Calibri", fontSize:11, color:C.DARK_NAVY });
}

// ─── Slide Modules ─────────────────────────────────────────────────────
const searching = require("./slides/slides_searching");
const nlf = require("./slides/slides_nlf");
const locking = require("./slides/slides_locking");
const qp = require("./slides/slides_qp");
const exposure = require("./slides/slides_exposure");
const webdb = require("./slides/slides_webdb");

// ─── MAIN ──────────────────────────────────────────────────────────────
function main() {
  const prs = new PptxGenJS();
  prs.defineLayout({ name: "CUSTOM", width: SLIDE_W, height: SLIDE_H });
  prs.layout = "CUSTOM";
  prs.author = "Andrea D'Antona";
  prs.title = "TERESA: System Evolution — May to June 2026";

  // SEARCHING (3 slides)
  searching.slideSearching1(prs);
  searching.slideSearching2(prs);
  searching.slideSearching3(prs);

  // NLF Burst (2 slides)
  nlf.slideNlf1(prs);
  nlf.slideNlf2(prs);

  // LOCKING (2 slides)
  locking.slideLocking1(prs);
  locking.slideLocking2(prs);

  // QP Controller (3 slides)
  qp.slideQp1(prs);
  qp.slideQp2(prs);
  qp.slideQp3(prs);

  // Exposure Scanning (3 slides)
  exposure.slideExposure1(prs);
  exposure.slideExposure2(prs);
  exposure.slideExposure3(prs);

  // Web Dashboard (2 slides)
  webdb.slideWebDb1(prs);
  webdb.slideWebDb2(prs);

  // ─── GENERATE ───────────────────────────────────────────────────────
  prs.writeFile({ fileName: OUT }).then(() => {
    console.log("Comparison slides saved to:", OUT);
  }).catch(err => {
    console.error("Error:", err);
  });
}

main();
