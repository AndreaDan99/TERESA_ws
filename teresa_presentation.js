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
const OUT = "/Users/andrea/Documents/UNIFE/DOTTORATO/Presentazioni/TERESA/TERESA_WBC_v2.pptx";
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

// ─── MAIN ──────────────────────────────────────────────────────────────
function main() {
  const prs = new PptxGenJS();
  prs.defineLayout({ name: "CUSTOM", width: SLIDE_W, height: SLIDE_H });
  prs.layout = "CUSTOM";
  prs.author = "Andrea D'Antona";
  prs.title = "TERESA: Whole-Body Active Perception for Emergency Assessment";

  // ─── SLIDE 1: COVER ───────────────────────────────────────────────
  {
    const s = prs.addSlide();
    darkBg(s, -1.80);
    s.addText("TERESA", { x:0.60, y:0.85, w:8.80, h:0.80, fontFace:"Trebuchet MS", fontSize:38, bold:true, color:C.WHITE });
    s.addText("Trustworthy Emergency Robot for Efficient Support and Assistance", { x:0.60, y:1.60, w:8.80, h:0.45, fontFace:"Trebuchet MS", fontSize:15, bold:true, color:C.WHITE });
    s.addText("Whole-Body Active Perception for Legged Robot-Assisted Emergency Assessment", { x:0.60, y:2.15, w:8.80, h:0.50, fontFace:"Calibri", fontSize:16, italic:true, color:C.WHITE });
    s.addShape("rect", { x:2.80, y:2.80, w:4.40, h:0.02, fill:{color:C.WHITE}, line:{type:"none"} });
    s.addText("Andrea D'Antona  \u00B7  University of Ferrara", { x:0.60, y:2.95, w:8.80, h:0.40, fontFace:"Calibri", fontSize:14, color:C.WHITE });
    s.addText("Spot + Unitree Z1  |  Orbbec Femto Bolt  |  Intel RealSense D435  |  Jetson Orin AGX", { x:0.60, y:3.35, w:8.80, h:0.35, fontFace:"Calibri", fontSize:12, italic:true, color:C.WHITE });
    addFooter(s);
  }

  // ─── SLIDE 2: SECTION 01 — SYSTEM OVERVIEW ─────────────────────────
  sectionSlide(prs, "01", "SYSTEM OVERVIEW", "Spot + Unitree Z1  \u00B7  Two Operational Pipelines  \u00B7  Active Perception Stack");

  // ─── SLIDE 3: TWO PIPELINES & HARDWARE ─────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "TWO PARALLEL PIPELINES", "Hardware integration and operational modes");

    // Left card: Z1 standalone
    addCard(s, 0.40, 1.25, 4.40, 2.15, C.TEAL, "Z1 STANDALONE \u2014 FAST SCANNING",
      "Unitree Z1 arm + RealSense D435\n\n" +
      "3 terminals: hardware \u2192 perception \u2192 control\n" +
      "YOLO torso tracker on RealSense\n" +
      "Cartesian impedance control for ultrasound\n" +
      "Pinocchio IK \u2192 JointTrajectoryController\n" +
      "FAST protocol with 5 anatomical points",
      10.5);

    // Right card: Spot + Z1 WBC
    addCard(s, 5.20, 1.25, 4.40, 2.15, C.GREEN, "SPOT + Z1 \u2014 FULL WBC PIPELINE",
      "Boston Dynamics Spot + Unitree Z1 arm\n" +
      "Orbbec Femto Bolt (body) + RealSense D435 (EE)\n" +
      "5 terminals: core \u2192 perception \u2192 control \u2192 WBC \u2192 keyboard\n" +
      "Hybrid search 360\u00B0 + Whole-Body Active Perception\n" +
      "Coordinator FSM: 7 states, 10 Hz\n" +
      "Body pose optimization per FAST point",
      10.5);

    // Hardware strip
    s.addShape("rect", { x:0.40, y:3.60, w:9.20, h:1.05, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("HARDWARE PLATFORM", { x:0.65, y:3.63, w:8.70, h:0.30, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL_DARK });
    const hw = [
      { label:"Spot", desc:"Boston Dynamics quadruped \u00B7 cmd_vel + body_pose control" },
      { label:"Unitree Z1", desc:"6-DOF arm \u00B7 1 kg payload \u00B7 torque + position control" },
      { label:"Orbbec Femto Bolt", desc:"Body-mounted RGB-D \u00B7 1280\u00D7720 @15fps \u00B7 YOLO skeleton" },
      { label:"RealSense D435", desc:"End-effector stereo depth \u00B7 848\u00D7480 @30fps \u00B7 YOLO torso" },
      { label:"Jetson Orin AGX", desc:"On-board GPU \u00B7 AI inference \u00B7 ROS 2 Jazzy \u00B7 SpotCore" },
    ];
    hw.forEach((h, i) => {
      const hx = 0.55 + i * 1.85;
      s.addText(h.label, { x:hx, y:3.98, w:1.75, h:0.24, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:C.TEAL });
      s.addText(h.desc, { x:hx, y:4.20, w:1.75, h:0.40, fontFace:"Calibri", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });
    });

    addFooter(s);
  }

  // ─── SLIDE 4: FRAME TREE ───────────────────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "FRAME TREE", "TF architecture: odom \u2192 body \u2192 world \u2192 link00..link06 \u2192 camera");

    const frameText = [
      "my_spot/odom                      \u2190 world-fixed odometry (spot_ros2 on SpotCore)",
      "   \u2514\u2500\u2500 my_spot/body              \u2190 Spot body frame (dynamic: moves with body_pose)",
      "            \u251C\u2500\u2500 orbbec_link          \u2190 static TF (0.30, 0, 0.15)",
      "            \u2502        \u2514\u2500\u2500 orbbec_color_optical_frame  \u2190 static TF",
      "            \u2514\u2500\u2500 world                 \u2190 static TF (z1_mount_x, 0, z1_mount_z) = Z1 base",
      "                      \u2514\u2500\u2500 link00              \u2190 Z1 URDF root (robot_state_publisher, fixed joint)",
      "                            \u2514\u2500\u2500 link01 ... link06  \u2190 Z1 arm chain",
      "                                  \u2514\u2500\u2500 camera_link        \u2190 static TF (0, 0, 0.05)",
      "                                        \u2514\u2500\u2500 camera_color_optical_frame  \u2190 RealSense driver" ];

    s.addText(frameText.join("\n"), { x:0.50, y:1.25, w:9.00, h:2.80, fontFace:"Consolas", fontSize:10, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    // Key points
    const kp = [
      "\u25B6 my_spot/odom is world-fixed \u2014 does NOT move with Spot",
      "\u25B6 my_spot/body moves with Spot (height, pitch, yaw from body_pose)",
      "\u25B6 world is the Z1 kinematic model root \u2014 child of body via static TF",
      "\u25B6 link00 = 'world' in the IK solver \u2014 coincident (fixed joint, zero offset)",
      "\u25B6 4 static TFs published by teresa_core.launch.py",
    ];
    s.addText(kp.join("\n"), { x:0.50, y:4.15, w:9.00, h:0.95, fontFace:"Calibri", fontSize:10.5, bold:true, color:C.TEAL_DARK, lineSpacingMultiple:1.25 });
    addFooter(s);
  }

  // ─── SLIDE 5: OPERATIONAL FLOW + FSM ───────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "OPERATIONAL FLOW & COORDINATOR FSM", "5 terminals \u00B7 7 states \u00B7 tf_monitor gate");

    // 5 terminal blocks
    const terms = [
      { t:"T1", label:"Core", desc:"Orbbec + RealSense + Z1 drivers\n4 static TFs + tf_monitor", color:C.TEAL },
      { t:"T2", label:"Perception", desc:"Orbbec YOLO posture classifier\nRealSense YOLO torso tracker", color:C.GREEN },
      { t:"T3", label:"Z1 Control", desc:"IK solver (Pinocchio) + ik_goal_mux\nz1_FSM + controller switch", color:C.SEAFOAM },
      { t:"T4", label:"WBC", desc:"QP Controller + Coordinator\nSpot Navigator", color:C.TEAL_DARK },
      { t:"T5", label:"Keyboard", desc:"s=start  r=return\nESC=emergency stop", color:C.DARK_NAVY },
    ];
    terms.forEach((tm, i) => {
      const tx = 0.30 + i * 1.90;
      s.addShape("rect", { x:tx, y:1.25, w:1.78, h:1.50, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
      s.addShape("rect", { x:tx, y:1.25, w:1.78, h:0.35, fill:{color:tm.color}, line:{type:"none"} });
      s.addText(tm.t, { x:tx+0.06, y:1.25, w:0.32, h:0.35, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:C.WHITE, valign:"middle", align:"center" });
      s.addText(tm.label, { x:tx+0.40, y:1.25, w:1.30, h:0.35, fontFace:"Trebuchet MS", fontSize:10, bold:true, color:C.WHITE, valign:"middle" });
      s.addText(tm.desc, { x:tx+0.12, y:1.65, w:1.54, h:1.02, fontFace:"Calibri", fontSize:10, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });
    });

    // FSM diagram (text)
    const fsm = [
      "WAITING_TF  \u2192  IDLE  (/wbc/tf_ready)",
      "    \u2192 keyboard 's'",
      "SEARCHING  \u2194  SEMI_LOCKING",
      "    \u2193  (2 sensors in parallel)",
      "LOCKING  (5 samples + arm home)",
      "    \u2193",
      "PRE_APPROACH  (5\u00D7 RealSense LOCKED)",
      "    \u2193",
      "APPROACHING  (soft\u2192hard handoff)",
      "    \u2193",
      "SCANNING  (all 5 FAST points)",
      "    \u2193",
      "HOMING \u2192 IDLE" ];

    s.addShape("rect", { x:0.30, y:2.92, w:9.40, h:2.20, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("COORDINATOR FSM (10 Hz)", { x:0.50, y:2.95, w:4.00, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL_DARK });
    s.addText(fsm.join("\n"), { x:0.50, y:3.28, w:4.50, h:1.75, fontFace:"Consolas", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.0 });
    s.addText(
      "tf_monitor: checks 8 TF chains + 3 topics every 2s\n" +
      "Publishes /wbc/tf_ready \u2192 gates all operations\n" +
      "If TF degrades \u2192 coordinator returns to WAITING_TF\n" +
      "Keyboard blocks 's' until TF ready",
      { x:5.20, y:3.28, w:4.30, h:1.75, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.35 });

    addFooter(s);
  }

  // ─── SLIDE 6: SECTION 02 — HYBRID SEARCH ───────────────────────────
  sectionSlide(prs, "02", "HYBRID SEARCH 360\u00B0", "Phase 1: Coordinated Spot body poses + Arm null-space exploration \u00B7 Two sensors in parallel");

  // ─── SLIDE 7: SEARCHING — SPOT + ARM ───────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "SEARCHING — COORDINATED SPOT + ARM", "Fase 1: ricerca ibrida a 360\u00B0 con due sensori in parallelo");

    addCard(s, 0.40, 1.25, 4.40, 2.30, C.TEAL, "SPOT \u2014 INCREMENTAL SEARCH",
      "18 positions: 6 yaw \u00D7 3 pitch = full 360\u00B0 coverage\n" +
      "Yaw sequence: 0\u00B0, +60\u00B0, \u221260\u00B0, +120\u00B0, \u2212120\u00B0, +180\u00B0\n" +
      "Pitch angles: 0\u00B0, 5\u00B0, 10\u00B0 (nose-down)\n" +
      "Height: nominal (0m), no longer lowered\n" +
      "15s dwell per position, overlap 10\u00B0\n" +
      "\u2192 Spot orients the Orbbec toward the ground",
      10.5);

    addCard(s, 5.20, 1.25, 4.40, 2.30, C.SEAFOAM, "ARM \u2014 QP SEARCH_GRID MODE",
      "7 exploration poses from null-space\n" +
      "Virtual target: body-X forward (no real target)\n" +
      "\u03B4 = 0.15 rad (\u22489\u00B0) \u2014 safe exploration\n" +
      "Safe joint limits to avoid:\n" +
      "  \u2022 Collision with Spot body\n" +
      "  \u2022 Touching the ground\n" +
      "Infinite loop: 2s data collection per pose\n" +
      "\u2192 FK recalculated when Spot rotates",
      10.5);

    // Sensors box
    s.addShape("rect", { x:0.40, y:3.72, w:9.20, h:1.30, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("TWO SENSORS IN PARALLEL", { x:0.65, y:3.75, w:8.70, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL_DARK });
    s.addText(
      "Orbbec Femto Bolt (on Spot body)\n" +
      "\u2192 YOLO11 skeleton \u2192 posture classifier \u2192 approach_point in odom",
      { x:0.65, y:4.08, w:4.20, h:0.42, fontFace:"Calibri", fontSize:10.5, color:C.TEAL, bold:true });
    s.addText(
      "RealSense D435 (on Z1 wrist)\n" +
      "\u2192 YOLO torso tracker \u2192 3D torso position",
      { x:5.20, y:4.08, w:4.20, h:0.42, fontFace:"Calibri", fontSize:10.5, color:C.GREEN, bold:true });
    s.addText(
      "Both pipelines run simultaneously.  Orbbec provides posture + approach distance.  RealSense provides precise torso localization for semi-lock.",
      { x:0.65, y:4.60, w:8.70, h:0.30, fontFace:"Calibri", fontSize:10, italic:true, color:C.DARK_NAVY });

    addFooter(s);
  }

  // ─── SLIDE 8: QP SEARCH_GRID MODE ──────────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "QP CONTROLLER \u2014 SEARCH_GRID MODE", "How the 7 exploration poses are generated from the null-space");

    s.addText(
      "The SEARCH_GRID mode explores the arm null-space without a real target. It uses a virtual \"look-at\" direction (body X-axis forward) to generate pose variations that preserve the same end-effector orientation while exploring different joint configurations.",
      { x:0.50, y:1.22, w:9.00, h:0.58, fontFace:"Calibri", fontSize:12, color:C.DARK_NAVY, lineSpacingMultiple:1.3 });

    // Generation steps
    const steps = [
      { n:1, c:C.TEAL,      t:"Compute FK",      b:"Forward kinematics at current joint configuration q_current" },
      { n:2, c:C.GREEN,     t:"Jacobian",         b:"Compute angular Jacobian J_task (3\u00D76) at current configuration" },
      { n:3, c:C.SEAFOAM,   t:"Null-Space Projector", b:"N = I \u2212 J_task\u207A \u00B7 J_task  \u2192  3 null-space directions via SVD" },
      { n:4, c:C.TEAL_DARK, t:"Generate Poses",   b:"7 poses: 1 home + 6 (\u00B1\u03B4 along each of 3 basis vectors), \u03B4=0.15" },
      { n:5, c:C.DARK_NAVY, t:"Safety Check",     b:"Clip joint angles to safe limits (avoid Spot, avoid ground)" },
      { n:6, c:C.TEAL,      t:"FK + Loop",        b:"Forward kinematics on q + \u03B4\u00B7v  \u2192  cycles at 10 Hz in infinite loop" },
    ];
    steps.forEach((st, i) => {
      const sy = 1.95 + i * 0.50;
      s.addShape("rect", { x:0.40, y:sy, w:0.40, h:0.40, fill:{color:st.c}, line:{type:"none"} });
      s.addText(String(st.n), { x:0.40, y:sy, w:0.40, h:0.40, fontFace:"Trebuchet MS", fontSize:16, bold:true, color:C.WHITE, align:"center", valign:"middle" });
      s.addText(st.t, { x:0.95, y:sy-0.02, w:1.80, h:0.22, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:st.c });
      s.addText(st.b, { x:0.95, y:sy+0.18, w:8.40, h:0.22, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY });
    });

    addFooter(s);
  }

  // ─── SLIDE 9: HYBRID LOCK ──────────────────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "HYBRID LOCK \u2014 TWO SENSORS, TWO LOCK TYPES", "Orbbec full lock + RealSense semi-lock \u00B7 Confidence gates \u00B7 Resilient recovery");

    // Full lock card
    addCard(s, 0.40, 1.25, 4.40, 3.10, C.TEAL,
      "FULL LOCK \u2014 ORBBEC DIRECT",
      "Posture = LYING  AND  confidence \u2265 70%\n" +
      "\u2192 approach_point available from Orbbec\n\n" +
      "LOCKING state:\n" +
      "\u2022 Arm returns to home position\n" +
      "\u2022 5 samples collected @10Hz (\u22480.5s)\n" +
      "\u2022 Average \u2192 fix target in odom\n" +
      "\u2022 Tolerance 1s if Orbbec loses LYING\n" +
      "\u2022 If Orbbec lost >1s \u2192 resume\n" +
      "   search from CURRENT position",
      10.5);

    // Semi-lock card
    addCard(s, 5.20, 1.25, 4.40, 3.10, C.GREEN,
      "SEMI-LOCK \u2014 REALSENSE GUIDES SPOT",
      "RealSense torso tracker = LOCKED\n" +
      "torso 3D position available\n\n" +
      "SEMI_LOCKING state:\n" +
      "\u2022 Coordinator computes optimal (yaw, pitch)\n" +
      "   to point Orbbec at the torso\n" +
      "\u2022 Spot rotates and tilts toward torso\n" +
      "\u2022 Arm FROZEN (QP paused): 3s clean window\n" +
      "\u2022 If Orbbec confirms \u2192 LOCKING\n" +
      "\u2022 If timeout 3s or RealSense loses torso\n" +
      "   \u2192 resume SEARCHING from current position",
      10.5);

    addFooter(s);
  }

  // ─── SLIDE 10: SECTION 03 — WBC APPROACH ───────────────────────────
  sectionSlide(prs, "03", "WBC APPROACH", "Phases 2\u20133: Arm-only Look-at Control \u00B7 Anticipatory Body Scanning \u00B7 FAST Point Generation");

  // ─── SLIDE 11: PRE_APPROACH — LOOKAT MODE ──────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "PRE_APPROACH \u2014 LOOKAT MODE", "Arm-only WBC: orientation error \u2192 damped pseudo-inverse \u2192 IK goal at 10 Hz");

    const mathLines = [
      "1. Orientation error:   \u03C9_des = kp_ang \u00B7 \u03B8 \u00B7 axis",
      "      \u03B8 = angle between X_ee and target direction",
      "      axis = rotation axis (cross product direction)",
      "",
      "2. Damped pseudo-inverse on angular Jacobian J_task (3\u00D76):",
      "      J\u207A = J\u1D40 (J\u00B7J\u1D40 + \u03BB\u00B2I)\u207B\u00B9",
      "      q\u0307_task = J\u207A \u00B7 \u03C9_des",
      "",
      "3. Null-space joint centering:",
      "      N = I \u2212 J\u207A\u00B7J  (null-space projector)",
      "      q\u0307_null = N \u00B7 k_null \u00B7 (q_mid \u2212 q_current)",
      "      q\u0307 = q\u0307_task + q\u0307_null",
      "",
      "4. FK prediction + workspace clip \u2192 publish IK goal",
      "      Loop at 10 Hz: orientation adapts in real-time" ];

    s.addText(mathLines.join("\n"), { x:0.50, y:1.22, w:5.40, h:3.70, fontFace:"Consolas", fontSize:10, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    // Right side: properties box
    s.addShape("rect", { x:6.20, y:1.22, w:3.50, h:3.70, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("KEY PROPERTIES", { x:6.35, y:1.28, w:3.20, h:0.30, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.TEAL });
    const props = [
      { c:C.TEAL,      t:"Arm-only",      d:"No J_base dependency. Spot is stationary." },
      { c:C.GREEN,     t:"10 Hz loop",    d:"IK goal updated continuously for smooth tracking." },
      { c:C.SEAFOAM,   t:"Joint centering", d:"Null-space projection keeps joints near mid-range." },
      { c:C.TEAL_DARK, t:"Workspace-safe", d:"FK prediction ensures goals stay within reach." },
      { c:C.DARK_NAVY, t:"No Spot motion", d:"Spot is straight and still (height=0, pitch=0)." },
    ];
    props.forEach((p, i) => {
      const py = 1.72 + i * 0.62;
      s.addShape("rect", { x:6.35, y:py, w:0.08, h:0.48, fill:{color:p.c}, line:{type:"none"} });
      s.addText(p.t, { x:6.52, y:py-0.02, w:3.05, h:0.24, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:p.c });
      s.addText(p.d, { x:6.52, y:py+0.22, w:3.05, h:0.26, fontFace:"Calibri", fontSize:10, color:C.DARK_NAVY });
    });

    addFooter(s);
  }

  // ─── SLIDE 12: APPROACHING — NAVIGATOR + SCAN_SEQ ──────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "APPROACHING \u2014 NAVIGATOR + QP SCAN_SEQ", "Fase 3: Spot navigates to patient while arm performs anticipatory body scan");

    // Navigator card
    addCard(s, 0.40, 1.25, 4.40, 1.85, C.TEAL,
      "SPOT NAVIGATOR \u2014 SIMPLIFIED APPROACH",
      "Receives goal in odom frame\n" +
      "\u2192 Transform to body frame (1 TF hop)\n" +
      "\u2192 Rotate to face goal\n" +
      "\u2192 Drive forward (P-controller)\n" +
      "\u2192 Stop at handoff distance\n\n" +
      "Independent from QP \u2014 Spot never\n"+
      "controlled by the QP controller.",
      10.5);

    // SCAN_SEQ card
    addCard(s, 5.20, 1.25, 4.40, 1.85, C.SEAFOAM,
      "QP CONTROLLER \u2014 SCAN_SEQ MODE",
      "Generates 11 null-space poses:\n" +
      "  1 home + 6 axes \u00B1\u03B4 + 4 diagonals\n" +
      "  \u03B4 = 0.12 rad (\u22487\u00B0) \u2014 reduced movement\n\n" +
      "BodySearchScanner:\n" +
      "  SEND_IK \u2192 wait ik_done\n" +
      "  \u2192 collect detection data 4s\n" +
      "  (min 5 frames, early stop score\u22650.95)\n" +
      "  \u2192 next pose",
      10.5);

    // Handoff box
    s.addShape("rect", { x:0.40, y:3.28, w:9.20, h:1.28, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("HANDOFF STRATEGY", { x:0.65, y:3.32, w:8.70, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL_DARK });
    s.addText(
      "Soft handoff (20cm): Spot pauses if scan not finished \u2192 scanner completes all poses \u2192 navigator resumes\n" +
      "Hard handoff (5cm + FAST points published): APPROACHING \u2192 SCANNING  \u2192  WBC disabled, Spot lowers (-0.15m)\n" +
      "Navigator is paused/resumed via /wbc/spot_control = False/True",
      { x:0.65, y:3.65, w:8.70, h:0.80, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.3 });

    addFooter(s);
  }

  // ─── SLIDE 13: SCAN_SEQ — FAST POINTS ──────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "SCAN_SEQ \u2014 FAST POINT GENERATION", "How 11 null-space poses become 5 anatomical FAST ultrasound targets");

    // Left: pipeline steps
    s.addText("PIPELINE (during APPROACHING)", { x:0.50, y:1.22, w:5.40, h:0.30, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL });
    const pipe = [
      { n:"1", c:C.TEAL,      t:"Grid Generation",    b:"SVD of N = I \u2212 J\u207A\u00B7J \u2192 3 orthonormal directions.\n11 poses = 1 home + 6 axial \u00B1\u03B4 + 4 diagonals (\u03B4=0.12).\nEach pose generated via FK(q + \u03B4\u00B7v)." },
      { n:"2", c:C.GREEN,     t:"BodySearchScanner",  b:"For each pose: send IK goal \u2192 wait ik_done \u2192 collect\nYOLO detection data for 4s (min 5 frames).\nEarly stop if detection score \u2265 0.95." },
      { n:"3", c:C.SEAFOAM,   t:"3D Fusion",          b:"Fuse 3D torso estimates from all poses.\nOutlier rejection at 0.15m threshold.\nProduces robust torso center + keypoints in link00." },
      { n:"4", c:C.TEAL_DARK, t:"FAST Point Calculation", b:"Compute 5 anatomical FAST points from fused\nkeypoints using fixed offset ratios from torso." },
    ];
    pipe.forEach((st, i) => {
      const py = 1.60 + i * 0.85;
      s.addShape("rect", { x:0.40, y:py, w:0.36, h:0.36, fill:{color:st.c}, line:{type:"none"} });
      s.addText(st.n, { x:0.40, y:py, w:0.36, h:0.36, fontFace:"Trebuchet MS", fontSize:14, bold:true, color:C.WHITE, align:"center", valign:"middle" });
      s.addText(st.t, { x:0.88, y:py-0.02, w:4.80, h:0.20, fontFace:"Trebuchet MS", fontSize:11, bold:true, color:st.c });
      s.addText(st.b, { x:0.88, y:py+0.18, w:4.80, h:0.60, fontFace:"Calibri", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });
    });

    // Right: FAST points diagram
    s.addShape("rect", { x:5.70, y:1.22, w:4.00, h:3.70, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("5 FAST POINTS", { x:5.85, y:1.28, w:3.70, h:0.28, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.TEAL });
    const pts = [
      { idx:"\u2460", name:"Center Hub",      desc:"torso_center (0,0,0)\nimpedance contact" },
      { idx:"\u2461", name:"Subxiphoid",     desc:"shoulder_mid +\n0.25\u00D7body_length" },
      { idx:"\u2462", name:"RUQ",            desc:"+0.40 body \u2212 0.50\nshoulder_width lateral" },
      { idx:"\u2463", name:"LUQ",            desc:"+0.35 body + 0.60\nshoulder_width lateral" },
      { idx:"\u2464", name:"Suprapubic",     desc:"hip_mid \u2212 0.15\n\u00D7body_length" },
    ];
    pts.forEach((pt, i) => {
      const py = 1.72 + i * 0.62;
      s.addText(pt.idx, { x:5.85, y:py, w:0.35, h:0.50, fontFace:"Calibri", fontSize:16, color:C.TEAL });
      s.addText(pt.name, { x:6.20, y:py-0.02, w:1.60, h:0.24, fontFace:"Trebuchet MS", fontSize:10.5, bold:true, color:C.DARK_NAVY });
      s.addText(pt.desc, { x:7.80, y:py-0.02, w:1.75, h:0.52, fontFace:"Calibri", fontSize:9, color:C.SEAFOAM });
    });

    addFooter(s);
  }

  // ─── SLIDE 14: SECTION 04 — SCANNING ───────────────────────────────
  sectionSlide(prs, "04", "BODY RECONFIGURATION & SCANNING", "Phases 4\u20135: Optimized body pose per FAST point \u00B7 Workspace extension \u00B7 Per-point workspace validation");

  // ─── SLIDE 15: BODY POSE OPTIMIZATION ───────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "BODY POSE OPTIMIZATION PER FAST POINT", "Grid search (h, p) per punto \u00B7 Sweet spot Z1 \u00B7 Minimize arm extension");

    s.addText(
      "Instead of keeping Spot at a fixed handoff height for all 5 FAST points, the coordinator pre-computes the optimal body posture (height + pitch) that brings each target closest to the center of the Z1 workspace.",
      { x:0.50, y:1.22, w:9.00, h:0.55, fontFace:"Calibri", fontSize:12, color:C.DARK_NAVY, lineSpacingMultiple:1.3 });

    // Grid search explanation
    s.addShape("rect", { x:0.40, y:1.90, w:5.60, h:2.85, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("GRID SEARCH (offline, per FAST point)", { x:0.60, y:1.95, w:5.20, h:0.30, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL });

    const gsLines = [
      "Primary grid: 3 heights \u00D7 4 pitches = 12 combinations",
      "  heights = [\u22120.20, \u22120.18, \u22120.15] m",
      "  pitches = [0\u00B0, 5\u00B0, 10\u00B0, 15\u00B0]  (yaw always fixed)",
      "",
      "For each (h, p) combination:",
      "  1. Simulate link00 position in odom for this body posture",
      "  2. Transform FAST target from odom \u2192 link00",
      "  3. Score = \u2212\u2016target_in_link00 \u2212 sweet_spot\u2016",
      "",
      "Sweet spot Z1 = [0.35, 0, 0.30] in link00 frame",
      "Select (h*, p*) that minimizes distance from sweet spot.",
      "",
      "After grid search: apply body_pose(h*[idx], p*[idx])",
      "Settle 1.5s \u2192 transform target \u2192 publish body_ready" ];

    s.addText(gsLines.join("\n"), { x:0.60, y:2.30, w:5.20, h:2.38, fontFace:"Consolas", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });

    // Right: constraints box
    s.addShape("rect", { x:6.30, y:1.90, w:3.40, h:1.30, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("CONSTRAINTS", { x:6.45, y:1.95, w:3.10, h:0.28, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.GREEN });
    s.addText(
      "Height: [\u22120.20, \u22120.15] m\n" +
      "Pitch: [0\u00B0, 15\u00B0]\n" +
      "Yaw: never changed\n" +
      "All targets pre-computed in odom\n" +
      "(world-fixed, invariant to Spot)",
      { x:6.45, y:2.28, w:3.10, h:0.82, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    // Right: WS_EXTENSION box
    s.addShape("rect", { x:6.30, y:3.40, w:3.40, h:1.35, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("WS_EXTENSION FALLBACK", { x:6.45, y:3.45, w:3.10, h:0.28, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.TEAL_DARK });
    s.addText(
      "If target still unreachable after (h,p):\n" +
      "4D grid: 3\u00D74\u00D75\u00D75 = 300 combos\n" +
      "  (h, p, dx, dy)\n" +
      "Navigator drives Spot \u00B1dx,dy\n" +
      "Timeout 5s \u2192 body_pose applied\n" +
      "\u2192 settle 1.5s \u2192 body_ready",
      { x:6.45, y:3.78, w:3.10, h:0.87, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    addFooter(s);
  }

  // ─── SLIDE 16: FAST CYCLE — PER-POINT WORKSPACE CHECK ──────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "FAST CYCLE \u2014 PER-POINT CHECKING_WORKSPACE", "Handshake protocol \u00B7 Skip logic for unreachable points \u00B7 Coordinator-FSM synchronization");

    // Flow diagram
    s.addShape("rect", { x:0.40, y:1.22, w:9.20, h:2.30, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("PER-POINT FLOW (for each of 5 FAST points)", { x:0.60, y:1.27, w:8.80, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL });

    const flowText = [
      "SCAN_PRELIFT",
      "  \u2193  FSM publishes: /z1/next_point_idx = idx",
      "SCAN_PAUSE",
      "  \u2193  waits for /wbc/body_ready = True from coordinator",
      "CHECKING_WORKSPACE",
      "  \u251C\u2500\u2500 target OK \u2192 APPROACHING \u2192 WAIT_IK_DONE \u2192 SCAN_PRELIFT (next point)",
      "  \u2514\u2500\u2500 was_clipped:",
      "        \u251C\u2500\u2500 idx=0 \u2192 proceed with clipped target (saves center_approach_pose)",
      "        \u2514\u2500\u2500 idx>0 \u2192 SKIP point, advance to next, publish next_point_idx, return to SCAN_PAUSE",
      "",
      "After all 5 points: Spot returns to handoff_height (-0.15m), FSM \u2192 HOMING \u2192 WAITING" ];

    s.addText(flowText.join("\n"), { x:0.60, y:1.60, w:8.80, h:2.00, fontFace:"Consolas", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });

    // Detailed explanation
    s.addText("COORDINATOR \u2194 FSM HANDSHAKE", { x:0.50, y:3.80, w:9.00, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL_DARK });

    const handshake = [
      { c:C.TEAL,      t:"body_ready",       d:"Coordinator publishes after applying body_pose(h*, p*) for point idx. Includes WS_EXTENSION drive if needed." },
      { c:C.GREEN,     t:"next_point_idx",   d:"FSM signals the coordinator which FAST point it's about to visit next. Coordinator pre-positions Spot accordingly." },
      { c:C.SEAFOAM,   t:"approach_target",  d:"Coordinator publishes the target in link00 frame (transformed from odom via live TF) for the FSM to use." },
      { c:C.TEAL_DARK, t:"center_approach_pose", d:"Saved at point 0 (Hub). Subsequent points (1-4) compute targets as center + offset. No live tracker needed." },
    ];
    handshake.forEach((h, i) => {
      const hy = 4.15 + i * 0.26;
      s.addShape("rect", { x:0.50, y:hy, w:0.06, h:0.22, fill:{color:h.c}, line:{type:"none"} });
      s.addText(h.t, { x:0.68, y:hy-0.02, w:2.00, h:0.22, fontFace:"Consolas", fontSize:9.5, bold:true, color:h.c });
      s.addText(h.d, { x:2.75, y:hy-0.02, w:6.70, h:0.22, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY });
    });

    addFooter(s);
  }

  // ─── SLIDE 17: Z1 FSM ──────────────────────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "Z1 FSM \u2014 STATE MACHINE FLOW", "Arm-side state machine: from homing to FAST cycle with per-point validation");

    s.addShape("rect", { x:0.40, y:1.22, w:5.80, h:3.68, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("Z1 FSM STATES", { x:0.60, y:1.27, w:5.40, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:C.TEAL });

    const fsmText = [
      "HOMING                      \u2192 joint_trajectory \u2192 home pose",
      "  \u2193",
      "WAITING                     \u2192 waits for WBC signal + FAST points",
      "  \u2193  /z1/fast_ready = True",
      "BODY_SCANNING               \u2192 SKIPPED (QP did it in APPROACHING)",
      "  \u2193",
      "CHECKING_WORKSPACE  (idx=0) \u2192 live tracker, validate reachability",
      "  \u2193  target OK / was_clipped(idx=0 proceeds)",
      "APPROACHING              \u2192 IK goal \u2192 JTC",
      "  \u2193  ik_done",
      "WAIT_IK_DONE             \u2192 impedance skipped (use_impedance=false)",
      "  \u2193",
      "SCAN_PRELIFT \u2192 pub next_point_idx=1 \u2192 SCAN_PAUSE",
      "  \u2193  /wbc/body_ready",
      "CHECKING_WORKSPACE (idx>0)    \u2192 target = center + offset",
      "  \u2193  was_clipped? idx>0 skips to next point",
      "APPROACHING \u2192 ... \u2192 (repeat for idx 1-4)",
      "  \u2193  after idx=4: next_point_idx=-1",
      "HOMING \u2192 WAITING             \u2192 mission complete" ];

    s.addText(fsmText.join("\n"), { x:0.60, y:1.60, w:5.40, h:3.22, fontFace:"Consolas", fontSize:9, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });

    // Right side notes
    s.addShape("rect", { x:6.50, y:1.22, w:3.20, h:3.68, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
    s.addText("KEY DETAILS", { x:6.65, y:1.28, w:2.90, h:0.28, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.GREEN });
    s.addText(
      "BODY_SCANNING skipped\n" +
      "because QP already scanned\n" +
      "during APPROACHING phase.\n\n" +
      "CHECKING_WORKSPACE\n" +
      "executed for EVERY point\n" +
      "(not just center hub).\n\n" +
      "Skip logic:\n" +
      "idx=0: always proceed\n" +
      "  (saves center pose)\n" +
      "idx>0: skip if was_clipped\n" +
      "  (advances to next point)\n\n" +
      "No impedance contact:\n" +
      "skip_impedance=true\n" +
      "Positioning only.",
      { x:6.65, y:1.62, w:2.90, h:3.15, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    addFooter(s);
  }

  // ─── SLIDE 18: SECTION 05 — QP CONTROLLER ───────────────────────────
  sectionSlide(prs, "05", "QP CONTROLLER", "Arm-only WBC \u00B7 3 operational modes \u00B7 Damped pseudo-inverse \u00B7 Null-space projection");

  // ─── SLIDE 19: QP CONTROLLER — ARCHITECTURE ────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "QP CONTROLLER \u2014 3 MODES, ARM-ONLY", "SEARCH_GRID \u00B7 LOOKAT \u00B7 SCAN_SEQ  \u00B7  Spot NEVER controlled by the QP");

    // 3 mode cards
    const modes = [
      { color:C.TEAL,      title:"SEARCH_GRID", phase:"Phase 1 \u2014 SEARCHING",
        bullets:"7 poses from null-space\n\u03B4 = 0.15 rad, safe joint limits\nVirtual target: body X forward\nInfinite loop, 2s per pose\nFK recalculated on Spot rotation\nPaused during SEMI_LOCKING\nExits to home during LOCKING" },
      { color:C.GREEN,     title:"LOOKAT", phase:"Phase 2 \u2014 PRE_APPROACH",
        bullets:"\u03C9_des from X_ee \u2192 target error\nDamped pinv on J_task (3\u00D76, angular)\nNull-space joint centering\nN @ k_null \u00B7 (q_mid \u2212 q)\nFK prediction + workspace clip\nPublishes IK goal at 10 Hz" },
      { color:C.SEAFOAM,   title:"SCAN_SEQ", phase:"Phase 3 \u2014 APPROACHING",
        bullets:"11 null-space poses (\u03B4=0.12)\n1 home + 6 axes + 4 diagonals\nBodySearchScanner sequencing\nFusion: outlier rejection 0.15m\nPublishes 5 FAST points\nSpot moved by navigator, not QP" },
    ];
    modes.forEach((m, i) => {
      const mx = 0.35 + i * 3.15;
      addCard(s, mx, 1.22, 3.00, 3.70, m.color, m.title + "  \u2014  " + m.phase, m.bullets, 10.5);
    });

    addFooter(s);
  }

  // ─── SLIDE 20: MATH — DAMPED PINV + NULL-SPACE ─────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "DAMPED PSEUDO-INVERSE & NULL-SPACE PROJECTION", "The mathematical core: arm-only look-at without holistic Jacobian");

    s.addText(
      "The QP controller uses a damped pseudo-inverse on the angular Jacobian (J_task, 3\u00D76) to solve the look-at task, with null-space projection for joint centering and grid pose generation.",
      { x:0.50, y:1.22, w:9.00, h:0.48, fontFace:"Calibri", fontSize:11.5, color:C.DARK_NAVY, lineSpacingMultiple:1.25 });

    // Math blocks
    const mathBlocks = [
      { color:C.TEAL,      title:"Damped Pseudo-Inverse",
        lines:[
          "J_task (3\u00D76) = angular Jacobian at current config",
          "J\u207A = J\u1D40 (J\u00B7J\u1D40 + \u03BB\u00B2I)\u207B\u00B9",
          "q\u0307_task = J\u207A \u00B7 \u03C9_des",
          "",
          "\u03C9_des = kp_ang \u00B7 \u03B8 \u00B7 axis",
          "  \u03B8 = angle(X_ee, target_dir)",
          "  axis = X_ee \u00D7 target_dir / \u2016\u00B7\u2016",
          "",
          "\u03BB = \u03BB_min + (\u03BB_max\u2212\u03BB_min) / (m + \u03B5)",
          "  m = \u221Adet(J\u00B7J\u1D40)  [Yoshikawa manipulability]",
          "Damping increases near singularities." ] },
      { color:C.GREEN,     title:"Null-Space Projector",
        lines:[
          "N = I \u2212 J\u207A\u00B7J  \u2208 \u211D\u2076\u00D7\u2076",
          "",
          "Joint centering:",
          "q\u0307_null = N @ k_null \u00B7 (q_mid \u2212 q)",
          "q\u0307 = q\u0307_task + q\u0307_null",
          "",
          "Grid pose generation:",
          "U, \u03A3, V\u1D40 = SVD(N)",
          "\u2192 3 orthonormal null-space directions",
          "",
          "Pose = FK( q + \u03B4\u00B7basis_vector )",
          "Preserves look-at orientation by construction" ] },
    ];
    mathBlocks.forEach((mb, i) => {
      const mbx = 0.35 + i * 4.80;
      s.addShape("rect", { x:mbx, y:1.82, w:4.55, h:3.10, fill:{color:C.OFF_WHITE}, line:{type:"none"} });
      s.addShape("rect", { x:mbx, y:1.82, w:4.55, h:0.35, fill:{color:mb.color}, line:{type:"none"} });
      s.addText(mb.title, { x:mbx+0.15, y:1.84, w:4.25, h:0.31, fontFace:"Trebuchet MS", fontSize:12, bold:true, color:C.WHITE, valign:"middle" });
      s.addText(mb.lines.join("\n"), { x:mbx+0.15, y:2.25, w:4.25, h:2.58, fontFace:"Consolas", fontSize:9.5, color:C.DARK_NAVY, lineSpacingMultiple:1.15 });
    });

    addFooter(s);
  }

  // ─── SLIDE 21: SECTION 06 — KEY INNOVATIONS ────────────────────────
  sectionSlide(prs, "06", "KEY INNOVATIONS", "Active perception pillars \u00B7 QualityMonitor \u00B7 System resilience \u00B7 Multi-controller");

  // ─── SLIDE 22: FOUR PILLARS ─────────────────────────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "FOUR PILLARS OF WHOLE-BODY ACTIVE PERCEPTION", "How TERESA maximizes perceptual quality through coordinated body reconfiguration");

    const pillars = [
      { n:"1", c:C.TEAL,      t:"Confidence-Gated Active Search",
        b:"Hybrid lock strategy with two sensors in parallel. Orbbec provides full lock at conf\u226570%. RealSense semi-lock guides Spot to improve Orbbec viewpoint. Tolerant recovery: 1s Orbbec loss during lock, 3s window for semi-lock. Resume search from current position \u2014 never restart from zero." },
      { n:"2", c:C.GREEN,     t:"Anticipatory Body Scanning with WBC",
        b:"Arm performs multi-view body scan DURING navigation (SCAN_SEQ), not after. 11 null-space poses explore the torso from different angles while Spot approaches. 3D fusion across poses produces robust FAST points before reaching the patient. Eliminates post-handoff waiting time." },
      { n:"3", c:C.SEAFOAM,   t:"Perception-Quality-Aware Control",
        b:"QualityMonitor tracks target uncertainty in odom. Target mean from first 3 observations, updated only on improved confidence. Quality metric grows linearly without fresh data. Uncertainty feeds into behavior modulation \u2014 higher uncertainty triggers more conservative actions." },
      { n:"4", c:C.TEAL_DARK, t:"Pre-Planned Body Reconfiguration",
        b:"Grid search optimization per FAST point finds optimal (height, pitch) minimizing arm extension toward sweet spot. Mathematical simulation without moving the robot. WS_EXTENSION fallback: 4D grid search (h,p,dx,dy) with navigator drive. Spot repositions BEFORE each point." },
    ];
    pillars.forEach((p, i) => {
      const py = 1.22 + i * 0.96;
      s.addShape("rect", { x:0.35, y:py, w:0.50, h:0.50, fill:{color:p.c}, line:{type:"none"} });
      s.addText(p.n, { x:0.35, y:py, w:0.50, h:0.50, fontFace:"Trebuchet MS", fontSize:20, bold:true, color:C.WHITE, align:"center", valign:"middle" });
      s.addText(p.t, { x:1.00, y:py-0.02, w:3.50, h:0.28, fontFace:"Trebuchet MS", fontSize:13, bold:true, color:p.c });
      s.addText(p.b, { x:1.00, y:py+0.26, w:8.80, h:0.62, fontFace:"Calibri", fontSize:10.5, color:C.DARK_NAVY, lineSpacingMultiple:1.2 });
    });

    addFooter(s);
  }

  // ─── SLIDE 23: QUALITYMONITOR + INFRASTRUCTURE ──────────────────────
  {
    const s = prs.addSlide();
    lightHeader(s, "QUALITYMONITOR & SYSTEM INFRASTRUCTURE", "Quality-driven behavior \u00B7 TF monitoring \u00B7 Multi-controller safety");

    addCard(s, 0.40, 1.22, 4.40, 1.80, C.TEAL,
      "QUALITYMONITOR",
      "Target = mean of first quality_buf_size=3 observations (in odom)\n" +
      "Update: only if posture_confidence > best_conf + confidence_margin (0.10)\n" +
      "Quality[m] = max_q \u00B7 (1 \u2212 posture_confidence)\n" +
      "Grows linearly without fresh data\n" +
      "Published on /wbc/target_uncertainty\n\n" +
      "Higher uncertainty \u2192 more conservative behavior",
      10);

    addCard(s, 5.20, 1.22, 4.40, 1.80, C.GREEN,
      "TF_MONITOR \u2014 HEALTH GATE",
      "Checks 8 TF chains every 2s:\n" +
      "  odom\u2192body, body\u2192world, world\u2192link00,\n" +
      "  body\u2192orbbec_link, orbbec\u2192optical,\n" +
      "  world\u2192link06, link06\u2192camera_link,\n" +
      "  camera_link\u2192camera_optical\n\n" +
      "+ 3 hardware topics: joint_states, Orbbec, RealSense\n" +
      "Publishes /wbc/tf_ready = True/False continuously\n" +
      "TF loss \u2192 coordinator returns to WAITING_TF",
      10);

    addCard(s, 0.40, 3.18, 4.40, 1.78, C.SEAFOAM,
      "MULTI-CONTROLLER Z1",
      "Alternates between two ROS 2 controllers:\n\n" +
      "Joint Trajectory Controller (JTC):\n" +
      "  Position control for homing, approaching\n" +
      "  Default safety controller\n\n" +
      "Torque Controller:\n" +
      "  Effort control for impedance during contact\n" +
      "  safe_controller_switch: /to_torque, /to_jtc\n" +
      "  JTC is always the safety default",
      10);

    addCard(s, 5.20, 3.18, 4.40, 1.78, C.TEAL_DARK,
      "RESILIENCE FEATURES",
      "Dry-run mode: all outputs go to debug topics\n" +
      "  \u2192 /wbc/ik_goal_pose_debug, /wbc/cmd_vel_debug\n\n" +
      "ESC emergency stop: keyboard node publishes\n" +
      "  /wbc/restart=False + cmd_vel=0\n\n" +
      "Timeout fallbacks:\n" +
      "  fast_ready_timeout: 10s (standalone Z1)\n" +
      "  body_ready_timeout: 3s (coordinator)\n" +
      "  navigator timeout: 5s\n\n" +
      "All TF lookups throttled with descriptive errors",
      10);

    addFooter(s);
  }

  // ─── SLIDE 24: THANK YOU ───────────────────────────────────────────
  {
    const s = prs.addSlide();
    darkBg(s, -1.25);
    s.addText("THANK YOU", { x:0.60, y:0.70, w:8.80, h:0.80, fontFace:"Trebuchet MS", fontSize:36, bold:true, color:C.WHITE });
    s.addShape("rect", { x:2.80, y:1.60, w:4.40, h:0.02, fill:{color:C.WHITE}, line:{type:"none"} });
    s.addText("TERESA: Whole-Body Active Perception for Emergency Assessment", { x:0.60, y:1.75, w:8.80, h:0.50, fontFace:"Calibri", fontSize:15, italic:true, color:C.WHITE });
    s.addText("Andrea D'Antona  \u2014  andrea.dantona@unife.it", { x:0.60, y:2.30, w:8.80, h:0.40, fontFace:"Calibri", fontSize:14, color:C.WHITE });
    s.addText("University of Ferrara  \u00B7  Department of Engineering", { x:0.60, y:2.65, w:8.80, h:0.35, fontFace:"Calibri", fontSize:13, italic:true, color:C.WHITE });
    s.addText("Medical Robotics and Automation research group", { x:0.60, y:2.95, w:8.80, h:0.35, fontFace:"Calibri", fontSize:13, italic:true, color:C.WHITE });
    s.addText("Questions?", { x:0.60, y:3.55, w:8.80, h:0.45, fontFace:"Trebuchet MS", fontSize:20, bold:true, color:C.WHITE });
    addFooter(s);
  }

  // ─── GENERATE ───────────────────────────────────────────────────────
  prs.writeFile({ fileName: OUT }).then(() => {
    console.log("Presentation saved to:", OUT);
  }).catch(err => {
    console.error("Error:", err);
  });
}

main();
