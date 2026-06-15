#!/usr/bin/env python3
"""
annotate_cover.py — Interactive TERESA system screenshot annotation.
Step 1: Click centers → saves annotations.json
Step 2: Run with --generate → produces annotated PDF
"""

import sys, json
from pathlib import Path
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.image import imread

INPUT  = Path(__file__).parent.parent / "TERESA_RAL/figures/system_overview.png"
OUTPUT = Path(__file__).parent.parent / "TERESA_RAL/figures/system_overview_annotated.png"
CONFIG = Path(__file__).parent.parent / "TERESA_RAL/figures/annotations.json"

LABELS = [
    ("RealSense\nD415",     "#ff8844"),
    ("Orbbec\nFemto",       "#ff66aa"),
    ("Burn\nScar",          "#ffffff"),
]


def collect_annotations():
    img = imread(str(INPUT))
    h, w = img.shape[:2]
    dpi = 100
    fig, ax = plt.subplots(figsize=(w/dpi, h/dpi), dpi=dpi)
    ax.imshow(img)  # pixel coords: (0,0)=top-left
    ax.set_title("Click center of each element:\n"
                 + " -> ".join(l[0].replace("\n"," ") for l in LABELS),
                 fontsize=10)
    ax.axis("off")
    fig.subplots_adjust(0,0,1,1)

    clicks = []
    for label, color in LABELS:
        pts = plt.ginput(1, timeout=0)
        if not pts:
            break
        x, y = pts[0]
        ax.plot(x, y, "o", color=color, markersize=14, markeredgewidth=2,
                markerfacecolor="none")
        ax.text(x+15, y, label.replace("\n"," "), color=color, fontsize=9,
                fontweight="bold", va="center")
        fig.canvas.draw()
        clicks.append({"label": label, "x": x, "y": y, "color": color})

    plt.close()
    if clicks:
        CONFIG.write_text(json.dumps(clicks, indent=2))
        print(f"Saved {len(clicks)} points to {CONFIG}")
        print("Run: python3 scripts/annotate_cover.py --generate")
    return clicks


def generate_annotated():
    if not CONFIG.exists():
        print("No annotations.json found. Run without --generate first.")
        sys.exit(1)

    clicks = json.loads(CONFIG.read_text())
    # Only draw labels present in current LABELS list
    label_names = {l[0] for l in LABELS}
    clicks = [c for c in clicks if c["label"] in label_names]
    img = imread(str(INPUT))
    h, w = img.shape[:2]
    dpi = 200
    fig, ax = plt.subplots(figsize=(w/dpi, h/dpi), dpi=dpi)
    ax.imshow(img)  # pixel coords: (0,0)=top-left
    ax.axis("off")
    fig.subplots_adjust(0,0,1,1)

    # Find RealSense and Scar points for exposure arrow
    rs_point = next((c for c in clicks if "RealSense" in c["label"]), None)
    scar_point = next((c for c in clicks if "Scar" in c["label"]), None)

    for c in clicks:
        color = c["color"]
        cx, cy = c["x"], c["y"]

        # Skip dashed box for remaining labels (just label + arrow)
        skip_box = True
        bw, bh = 90, 60

        if "Scar" in c["label"]:
            bw, bh = 60, 40

        if not skip_box:
            rect = mpatches.FancyBboxPatch(
                (cx-bw/2, cy-bh/2), bw, bh,
                boxstyle=mpatches.BoxStyle("round,pad=4"),
                edgecolor=color, facecolor="none", linewidth=2.5, linestyle="--")
            ax.add_patch(rect)

        lx = cx + bw/2 + 40
        if "Orbbec" in c["label"]:
            lx += 210
        ax.annotate("", xy=(cx+bw/2, cy), xytext=(lx, cy),
                    arrowprops=dict(arrowstyle="->", color=color, lw=2))
        ax.text(lx+8, cy, c["label"], fontsize=11, fontweight="bold",
                color=color, va="center")

    # Exposure arrow: RealSense → Scar
    if rs_point and scar_point:
        ax.annotate("", xy=(scar_point["x"], scar_point["y"]),
                    xytext=(rs_point["x"], rs_point["y"]),
                    arrowprops=dict(arrowstyle="->", color="#ff8844", lw=3,
                                    linestyle="dashed", connectionstyle="arc3,rad=0.2"))
        # Label at midpoint
        mx = (rs_point["x"] + scar_point["x"]) / 2
        my = (rs_point["y"] + scar_point["y"]) / 2
        ax.text(mx+10, my-10, "EXPOSURE", fontsize=11, fontweight="bold",
                color="#ff8844", va="center", rotation=0)

    fig.savefig(str(OUTPUT), dpi=dpi, bbox_inches="tight", pad_inches=0.1)
    print(f"Saved: {OUTPUT}")
    plt.close()


if __name__ == "__main__":
    if "--generate" in sys.argv:
        generate_annotated()
    else:
        print("Click center of each element in the popup window.")
        print("Then run: python3 scripts/annotate_cover.py --generate")
        collect_annotations()
