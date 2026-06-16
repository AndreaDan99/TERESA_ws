"""Build report.html for the Orin deployment from the collected metrics + media.
Run from the orin_deploy/ root: python3 scripts/build_report.py"""
import os, json, glob, html

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(p):
    try:
        with open(os.path.join(ROOT, p)) as f:
            return json.load(f)
    except Exception:
        return None


def imgs(pattern, n, every=1):
    fs = sorted(glob.glob(os.path.join(ROOT, pattern)))[::every][:n]
    return [os.path.relpath(f, ROOT) for f in fs]


nlf_s = load("results/nlf_s_full/metrics.json")
nlf_l = load("results/nlf_l_full/metrics.json")
wound = {m: load(f"results/wound/{m}/metrics.json") for m in ("gdino", "owlv2", "clipseg")}

CSS = """
:root{--bg:#0f1117;--card:#1a1d27;--ink:#e8eaf0;--mut:#9aa3b2;--acc:#5b8def;--ok:#3ecf8e;--warn:#f5a623;--line:#2a2e3a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 -apple-system,Segoe UI,Roboto,Helvetica,Arial}
.wrap{max-width:1100px;margin:0 auto;padding:32px 20px 80px}
h1{font-size:30px;margin:0 0 4px}h2{font-size:22px;margin:38px 0 10px;border-bottom:1px solid var(--line);padding-bottom:8px}
h3{font-size:17px;margin:22px 0 8px;color:var(--acc)}
.sub{color:var(--mut);margin:0 0 18px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px 20px;margin:16px 0}
.verdict{font-size:17px;font-weight:600}.ok{color:var(--ok)}.warn{color:var(--warn)}
table{border-collapse:collapse;width:100%;margin:10px 0;font-size:14px}
th,td{padding:8px 11px;text-align:left;border-bottom:1px solid var(--line)}th{color:var(--mut);font-weight:600}
td.n,th.n{text-align:right;font-variant-numeric:tabular-nums}
tr.best td{background:rgba(62,207,142,.08)}
video{width:100%;border-radius:10px;border:1px solid var(--line);background:#000}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
.grid2 img,.grid3 img{width:100%;border-radius:8px;border:1px solid var(--line)}
.cap{color:var(--mut);font-size:13px;margin:6px 0 0}
.pill{display:inline-block;padding:2px 9px;border-radius:20px;font-size:12px;background:#23283a;color:var(--mut);margin-right:6px}
code{background:#23283a;padding:1px 6px;border-radius:5px;font-size:13px}
ul{margin:8px 0}li{margin:4px 0}
"""


def metric_rows(rows, headers, best_idx=None):
    h = "".join(f"<th class='n'>{html.escape(x)}</th>" if i else f"<th>{html.escape(x)}</th>"
                for i, x in enumerate(headers))
    body = ""
    for ri, r in enumerate(rows):
        cls = " class='best'" if best_idx == ri else ""
        cells = "".join(f"<td class='n'>{html.escape(str(x))}</td>" if i else f"<td>{html.escape(str(x))}</td>"
                        for i, x in enumerate(r))
        body += f"<tr{cls}>{cells}</tr>"
    return f"<table><tr>{h}</tr>{body}</table>"


P = []
P.append(f"<!doctype html><html><head><meta charset='utf-8'><title>Orin Deployment Report</title><style>{CSS}</style></head><body><div class='wrap'>")
P.append("<h1>Real-Time Perception on the Jetson Orin</h1>")
P.append("<p class='sub'>NLF 3D body pose + open-vocabulary wound detection, deployed and benchmarked on the Spot robot's "
         "NVIDIA Jetson AGX Orin (JetPack 5.1.2, CUDA 11.4). Generated 2026-06-15.</p>")

P.append("<div class='card'><span class='pill'>Jetson AGX Orin</span><span class='pill'>L4T R35.4.1 / JP 5.1.2</span>"
         "<span class='pill'>CUDA 11.4</span><span class='pill'>torch 2.0.0 (dustynv)</span><span class='pill'>Docker / nvidia runtime</span>"
         "<p class='verdict ok'>✓ NLF runs near real-time on the Orin (17.3 fps small / 13.6 fps large inference).<br>"
         "✓ Wound detection (GDINO / OWLv2 / CLIPSeg) runs at ~1–3 fps — within the ≤1 fps triage budget.</p></div>")

# ---- NLF ----
P.append("<h2>1 · NLF — 3D body pose &amp; SMPL mesh</h2>")
P.append("<p class='sub'>Self-contained TorchScript (own detector, per-joint uncertainty). Input video: a person doing "
         "upper-body manipulation at a lab table (720p, 1908 frames). Det-rate matches the desktop reference (88.8%).</p>")
if nlf_s and nlf_l:
    rows = [
        ["nlf_s (small)", nlf_s["infer_fps"], nlf_s["e2e_fps"], nlf_s["infer_ms_per_frame_mean"],
         f'{nlf_s["det_rate"]*100:.1f}%', nlf_s["gpu_mem_mb"]],
        ["nlf_l (large)", nlf_l["infer_fps"], nlf_l["e2e_fps"], nlf_l["infer_ms_per_frame_mean"],
         f'{nlf_l["det_rate"]*100:.1f}%', nlf_l["gpu_mem_mb"]],
    ]
    P.append(metric_rows(rows, ["model", "infer fps", "e2e fps", "ms/frame", "det rate", "GPU MB"], best_idx=0))
    P.append("<p class='cap'>infer fps = GPU model throughput (batch 8); e2e includes CPU-side overlay drawing + mp4 writing "
             "(not present in a real pose consumer). Both models near real-time; small is the deployment pick.</p>")
P.append("<h3>Small vs Large — side-by-side overlay</h3>")
P.append("<video controls preload='metadata' src='viz/nlf_small_vs_large.mp4'></video>")
P.append("<p class='cap'>Left: nlf_s · Right: nlf_l. Green = SMPL mesh vertices, orange/blue = 24-joint skeleton. "
         "The lower body is extrapolated (table occludes the legs) — expected SMPL behaviour.</p>")
ns = imgs("results/nlf_s_full/samples/f00400.jpg", 1) + imgs("results/nlf_l_full/samples/f00400.jpg", 1)
if len(ns) == 2:
    P.append("<div class='grid2'>" + "".join(f"<div><img src='{i}'><p class='cap'>{['nlf_s','nlf_l'][k]} · frame 400</p></div>"
             for k, i in enumerate(ns)) + "</div>")

# ---- NLF on the mannequin (Bob) ----
bob_s = load("results/nlf_bob_s/metrics.json")
bob_l = load("results/nlf_bob_l/metrics.json")
P.append("<h3>Same model on the mannequin (&ldquo;Bob&rdquo;, the wound clip)</h3>")
P.append("<p class='sub'>NLF body detection on the supine medical mannequin — the body-pose input for the wound "
         "scenario (960×540, 201 frames, camera panning over the body).</p>")
if bob_s and bob_l:
    rows = [["nlf_s (small)", bob_s["infer_fps"], f'{bob_s["det_rate"]*100:.0f}%', bob_s["gpu_mem_mb"]],
            ["nlf_l (large)", bob_l["infer_fps"], f'{bob_l["det_rate"]*100:.0f}%', bob_l["gpu_mem_mb"]]]
    P.append(metric_rows(rows, ["model", "infer fps", "det rate", "GPU MB"], best_idx=1))
    P.append("<p class='cap'>Large detects the lying mannequin better (63% vs 52%); the rest of the clip is body-part "
             "close-ups with no full body in view. The 63% matches the desktop NLF result (62.5%).</p>")
P.append("<video controls preload='metadata' src='viz/nlf_bob_small_vs_large.mp4'></video>")
P.append("<p class='cap'>Left: nlf_s · Right: nlf_l, on Bob's video.</p>")
b200 = imgs("results/nlf_bob_l/samples/f00200.jpg", 1)
if b200:
    P.append(f"<img style='width:100%;border-radius:8px;border:1px solid var(--line)' src='{b200[0]}'>"
             "<p class='cap'>nlf_l · full-body frame — clean SMPL skeleton + 1024-vertex mesh on the supine mannequin.</p>")

# ---- Wound ----
P.append("<h2>2 · Open-vocabulary wound detection</h2>")
P.append("<p class='sub'>Zero-shot detection on a medical-training mannequin (no wound training data). Prompted with "
         "<code>wound. laceration. cut. bleeding wound. surgical incision. …</code> All methods are pure HuggingFace "
         "transformers — no custom CUDA builds. 80 frames @2 fps.</p>")
wl = [(m, wound[m]) for m in ("gdino", "owlv2", "clipseg") if wound[m]]
if wl:
    rows = [[m, w["model"].split("/")[-1], f'{w["detection_rate"]*100:.0f}%', w["fps"], round(w["ms_per_frame"]),
             w["peak_vram_mb"], w["mean_dets_per_frame"]] for m, w in wl]
    P.append(metric_rows(rows, ["method", "weights", "det rate", "fps", "ms/frame", "VRAM MB", "dets/frame"], best_idx=0))
    P.append("<p class='cap'>GDINO = highest recall (leads triage); OWLv2 = cleaner boxes; CLIPSeg = fastest + gives masks. "
             "All clear the ≤1 fps deployment bar.</p>")
P.append("<h3>Three methods — side-by-side</h3>")
P.append("<video controls preload='metadata' src='viz/wound_methods_grid.mp4'></video>")
P.append("<p class='cap'>Left→right: Grounding DINO · OWLv2 · CLIPSeg.</p>")
hero = []
for m in ("gdino", "owlv2", "clipseg"):
    f = f"results/wound/{m}/overlays/frame_00026.jpg"
    if os.path.exists(os.path.join(ROOT, f)):
        hero.append((m, f))
if hero:
    P.append("<h3>Hero frame (clearest forearm laceration + abdominal/stoma)</h3>")
    P.append("<div class='grid3'>" + "".join(f"<div><img src='{f}'><p class='cap'>{m}</p></div>" for m, f in hero) + "</div>")

# ---- Pose detection + posture ----
yolo_p = load("results/pose/bob_yolo/metrics.json")
mp_p = load("results/pose/bob_mediapipe/metrics.json")
def _dist(m):
    d = m.get("posture_distribution", {}) if m else {}
    return ", ".join(f"{k} {v*100:.0f}%" for k, v in sorted(d.items(), key=lambda x: -x[1]))
P.append("<h2>3 · 2D skeleton pose + posture (lying / sitting / standing)</h2>")
P.append("<p class='sub'>The dedicated keypoint detectors from wound_det, run on Bob with a geometric posture "
         "classifier on top (ported verbatim). Both deployed backends agree the mannequin is <b>LYING</b> "
         "— matching the desktop study exactly (YOLO det-rate 0.2875 and posture lying 91.4% are identical).</p>")
prows = []
if yolo_p:
    prows.append(["YOLO11x-pose (COCO-17)", f'{yolo_p["detection_rate"]*100:.0f}%', yolo_p["fps"],
                  yolo_p["dominant_posture"].upper(), _dist(yolo_p)])
if mp_p:
    prows.append(["MediaPipe BlazePose (33)", f'{mp_p["detection_rate"]*100:.0f}%', mp_p["fps"],
                  mp_p["dominant_posture"].upper(), _dist(mp_p)])
prows.append(["ViTPose++ (HF)", "n/a", "n/a", "NOT PORTABLE",
              "needs transformers ≥4.48 / py≥3.9; JP5 image is py3.8"])
P.append(metric_rows(prows, ["backend", "det rate", "fps", "posture", "distribution / note"]))
P.append("<video controls preload='metadata' src='viz/pose_yolo_vs_mediapipe.mp4'></video>")
P.append("<p class='cap'>Left: YOLO11x-pose (COCO-17) · Right: MediaPipe BlazePose (33 landmarks). "
         "Posture label is burned bottom-left of each frame.</p>")
ph = []
for b in ("yolo", "mediapipe"):
    f = f"results/pose/bob_{b}/samples/frame_00076.jpg"
    if os.path.exists(os.path.join(ROOT, f)):
        ph.append((b, f))
if ph:
    P.append("<div class='grid2'>" + "".join(
        f"<div><img src='{f}'><p class='cap'>{b} · full-body frame</p></div>" for b, f in ph) + "</div>")

# ---- Engineering ----
P.append("<h2>4 · Key engineering findings</h2>")
P.append("<div class='card'><ul>"
         "<li><b>Substrate:</b> the existing <code>teresa_ws</code> image ships torch 2.12+cu130 with <code>cuda.is_available()=False</code> "
         "(wrong CUDA for this device). Used <code>dustynv/l4t-pytorch:r35.4.1</code> (torch 2.0.0 + torchvision, CUDA 11.4) instead — GPU works.</li>"
         "<li><b>Disk:</b> the eMMC <code>/</code> is 91% full (5 GB), but a 1 TB NVMe <code>/ssd</code> has 805 GB free and already hosts Docker — all work lives on <code>/ssd</code>.</li>"
         "<li><b>NLF model version:</b> the <code>0.3.2</code> TorchScript needs torch ≥2.4 (<code>aten::get_autocast_dtype</code>) — unloadable on JP5. The <code>v0.2.0</code> export loads on torch 2.0.</li>"
         "<li><b>The deadlock:</b> NLF's first inference ran on GPU but every subsequent call hung. Fix: disable the TorchScript "
         "profiling executor + fuser (<code>torch._C._jit_set_profiling_executor(False)</code> …) — a known Jetson fuser hazard.</li>"
         "<li><b>Wound API skew:</b> desktop scripts target transformers 5.x; the JP5 image is py3.8 → transformers 4.44.2, so OWLv2/GDINO "
         "post-processing was adapted (<code>post_process_object_detection</code>, <code>box_threshold</code>) and <code>scipy</code> added for OWLv2.</li>"
         "<li><b>Pose deps:</b> plain <code>pip install ultralytics</code>/<code>mediapipe</code> pulls a generic <code>opencv-python</code> that "
         "breaks the Jetson-native cv2 (the <code>_registerMatType</code> crash). Fixed by installing both <code>--no-deps</code> + minimal deps; "
         "MediaPipe lives in its own isolated image (<code>andrea_mp</code>) to keep its protobuf&lt;4 away from transformers.</li>"
         "</ul></div>")
P.append("<p class='sub'>Full logs: <code>orin_deploy/explore.md</code> (algorithms) · <code>orin_deploy/troubleshooting.md</code> (bugs &amp; fixes). "
         "Scripts &amp; raw metrics under <code>orin_deploy/scripts/</code> and <code>orin_deploy/results/</code>.</p>")
P.append("</div></body></html>")

out = os.path.join(ROOT, "report.html")
with open(out, "w") as f:
    f.write("\n".join(P))
print("wrote", out)
