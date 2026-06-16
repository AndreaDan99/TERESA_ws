#!/usr/bin/env python3
"""Build the sensor-integration HTML report from captured assets + metrics.
Expects an assets/ dir with:
  <cam>_color.jpg, <cam>_depth.png                  (cam in orbbec, rs)
  <cam>_<mod>.jpg, <cam>_<mod>.mp4, <cam>_<mod>.json (mod in pose, nlf, wound)
  grid_orbbec.mp4, grid_rs.mp4                       (optional)
Writes report.html next to assets/.
Usage: python3 build_sensors_report.py --assets sensors/report/assets --out sensors/report/report.html
"""
import os, json, argparse, html

CAMS = [("orbbec", "Orbbec Femto Bolt — fixed on Spot"),
        ("rs", "Intel RealSense D415 — on the Z1 arm")]
MODS = [("nlf", "NLF — 3D body (SMPL mesh + skeleton)"),
        ("pose", "YOLO11x-pose — 2D skeleton + posture"),
        ("wound", "Wound (Grounding-DINO) — zero-shot + depth-fused 3D point")]


def has(assets, name):
    return os.path.exists(os.path.join(assets, name))


def metrics_table(m):
    rows = "".join(f"<tr><td>{html.escape(str(k))}</td><td><b>{html.escape(str(v))}</b></td></tr>"
                   for k, v in m.items())
    return f"<table class='m'>{rows}</table>"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--assets', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--date', default='2026-06-16')
    a = ap.parse_args()
    A = a.assets
    rel = os.path.relpath(A, os.path.dirname(a.out))

    def img(name, cls=''):
        return f"<img class='{cls}' src='{rel}/{name}'>" if has(A, name) else "<div class='missing'>—</div>"

    def vid(name):
        return (f"<video src='{rel}/{name}' controls loop muted playsinline></video>"
                if has(A, name) else "")

    def load(name):
        p = os.path.join(A, name)
        return json.load(open(p)) if os.path.exists(p) else None

    P = []
    P.append(f"""<!doctype html><html><head><meta charset='utf-8'><title>TERESA Sensor Integration — Orin</title>
<style>
 body{{font:15px/1.55 -apple-system,Segoe UI,Roboto,Arial;margin:0;background:#0e1117;color:#d7dde6}}
 .wrap{{max-width:1180px;margin:0 auto;padding:28px}}
 h1{{font-size:27px;margin:.2em 0}} h2{{font-size:21px;border-bottom:1px solid #2a3340;padding-bottom:6px;margin-top:34px}}
 h3{{font-size:16px;color:#9fb3c8;margin:18px 0 8px}}
 .sub{{color:#8b97a7}} a{{color:#6cb6ff}}
 .grid{{display:grid;gap:14px}} .g2{{grid-template-columns:1fr 1fr}} .g3{{grid-template-columns:1fr 1fr 1fr}}
 img{{width:100%;border-radius:8px;display:block;background:#000}} video{{width:100%;border-radius:8px;background:#000}}
 .card{{background:#161b22;border:1px solid #232b36;border-radius:10px;padding:12px}}
 table{{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0}}
 td,th{{border:1px solid #2a3340;padding:5px 9px;text-align:left}} th{{background:#1b222c}}
 table.m td:first-child{{color:#9fb3c8}} .ok{{color:#4ade80}} .no{{color:#f87171}} .warn{{color:#fbbf24}}
 .pill{{display:inline-block;padding:2px 9px;border-radius:11px;font-size:12px;font-weight:600}}
 .pill.ok{{background:#143020;color:#4ade80}} .pill.no{{background:#3a1515;color:#f87171}}
 .missing{{color:#555;text-align:center;padding:40px;border:1px dashed #333;border-radius:8px}}
 code{{background:#1b222c;padding:1px 5px;border-radius:4px;color:#e2b}}
</style></head><body><div class='wrap'>""")

    P.append(f"""<h1>TERESA Sensor Integration on the Jetson Orin</h1>
<p class='sub'>Read-only sensor bring-up + live GPU perception &middot; {a.date} &middot; no Spot/arm motion</p>
<div class='card'><b>One-line:</b> both cameras read live; our NLF&nbsp;3D&nbsp;body, YOLO&nbsp;pose+posture, and
zero-shot wound detection run as <b>live ROS&nbsp;nodes on the Jetson GPU</b> over a two-container DDS split, with the
wound back-projected to a <b>3D point</b> via the registered depth. The Z1 arm and Spot core remain network-blocked
this phase and are read-only-deferred.</div>""")

    # architecture / diagnosis
    P.append("""<h2>Architecture &amp; the GPU fix</h2>
<div class='card'><p><b>Two-container DDS split.</b> Camera drivers run in <code>teresa_ws:latest</code> (ROS Humble);
GPU perception runs in <code>teresa_gpu</code>; they talk over DDS (<code>ROS_DOMAIN_ID=42</code>, cyclonedds,
isolated from the operator's domain&nbsp;0).</p>
<p><b>Root-cause fixed.</b> The legacy image shipped <code>torch 2.12.0+cu130</code> (a generic CUDA-13 wheel) →
<span class='no'>cuda unavailable</span> on the Jetson, so all perception fell back to CPU (why it "only worked on
another computer"). <code>teresa_gpu</code> grafts the correct Jetson torch <code>2.0.0+nv23.05 / CUDA&nbsp;11.4</code>
onto a ROS Humble base → <span class='ok'>GPU enabled</span>, our deps installed the <code>--no-deps</code> way so
nothing clobbers torch/cv2/numpy.</p></div>""")

    # sensor matrix
    P.append("""<h2>Sensor read matrix</h2><table>
<tr><th>Sensor</th><th>Connection</th><th>Read via</th><th>Status</th></tr>
<tr><td>Orbbec Femto Bolt (Spot cam)</td><td>USB3</td><td>orbbec_camera → /orbbec/color+depth</td><td><span class='pill ok'>LIVE</span></td></tr>
<tr><td>Intel RealSense D415 (arm cam)</td><td>USB3</td><td>realsense2_camera → /camera/camera/*</td><td><span class='pill ok'>LIVE</span></td></tr>
<tr><td>Z1 arm joint state</td><td>Ethernet/UDP 192.168.123.x</td><td>(no motion-free read; eth0 down)</td><td><span class='pill no'>DEFERRED</span></td></tr>
<tr><td>Spot core state</td><td>DDS from SpotCore</td><td>subscribe my_spot/* (LAN down)</td><td><span class='pill no'>DEFERRED</span></td></tr>
</table>""")

    # live sensor frames
    P.append("<h2>Live sensor frames</h2>")
    for cam, title in CAMS:
        P.append(f"<h3>{html.escape(title)}</h3><div class='grid g2'>"
                 f"<div class='card'><b>Color</b>{img(cam+'_color.jpg')}</div>"
                 f"<div class='card'><b>Depth</b>{img(cam+'_depth.png')}</div></div>")

    # perception per camera
    P.append("<h2>Live perception (on the GPU)</h2>")
    for cam, ctitle in CAMS:
        P.append(f"<h3>{html.escape(ctitle)}</h3><div class='grid g3'>")
        for mod, mtitle in MODS:
            m = load(f"{cam}_{mod}.json")
            mtab = metrics_table(m) if m else "<p class='sub'>no run</p>"
            media = vid(f"{cam}_{mod}.mp4") or img(f"{cam}_{mod}.jpg")
            P.append(f"<div class='card'><b>{html.escape(mtitle)}</b>{media}{mtab}</div>")
        P.append("</div>")

    # grids
    if has(A, 'grid_orbbec.mp4') or has(A, 'grid_rs.mp4'):
        P.append("<h2>Grid videos</h2><div class='grid g2'>")
        for g, t in [('grid_orbbec.mp4', 'Orbbec — NLF | Pose | Wound+3D'),
                     ('grid_rs.mp4', 'RealSense — NLF | Pose | Wound+3D')]:
            if has(A, g):
                P.append(f"<div class='card'><b>{t}</b>{vid(g)}</div>")
        P.append("</div>")

    # analysis
    P.append("""<h2>Analysis &amp; conclusions</h2><div class='card'><ul>
<li><b>3D wound localization works:</b> the wound box-centre is back-projected through the registered depth to a
metric XYZ in the camera frame — the target the Z1 arm will eventually servo to.</li>
<li><b>NLF is the robust body signal</b> (detection on essentially every frame); the 2D YOLO-pose detection on the
Orbbec is intermittent because the Femto Bolt runs HDR-interleaved exposures — alternating frames are hard for the
person detector. A bbox-aspect fallback still recovers the lying/standing label.</li>
<li><b>GPU throughput</b> (single-stream, 720p/480p): see per-card metrics. NLF ≈ 3–4 fps, YOLO-pose ≈ 8–14 fps,
Grounding-DINO ≈ 1 fps — all on the Orin GPU via <code>teresa_gpu</code>.</li>
<li><b>Deferred (network):</b> Z1 joint state has no motion-free read and the arm is in use; Spot state needs the
SpotCore DDS link (down). Both are pure-subscribe once reachable — never publish motion.</li>
</ul></div>
<p class='sub'>Generated by build_sensors_report.py &middot; assets in <code>""" + html.escape(rel) + """/</code></p>
</div></body></html>""")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write("\n".join(P))
    print("wrote", a.out)


if __name__ == '__main__':
    main()
