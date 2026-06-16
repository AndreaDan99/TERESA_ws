#!/usr/bin/env python3
"""Compose several mp4s into a labelled grid mp4 + a first-frame montage JPG, using cv2 only
(no ffmpeg on the Orin). Shorter clips hold their last frame so all cells stay in sync.

Usage:
  python3 make_grid_cv2.py --out grid.mp4 --montage grid.jpg --cols 3 --cell 480x270 \
      "NLF:/work/cap/orbbec_nlf.mp4" "Pose:/work/cap/orbbec_pose.mp4" "Wound3D:/work/cap/orbbec_wound.mp4"
"""
import argparse, os
import numpy as np, cv2


def load_video(path):
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
    cap.release()
    return frames


def label(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(img, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cells', nargs='+', help='LABEL:path.mp4')
    ap.add_argument('--out', required=True)
    ap.add_argument('--montage', default='')
    ap.add_argument('--cols', type=int, default=3)
    ap.add_argument('--cell', default='480x270')
    ap.add_argument('--fps', type=float, default=8.0)
    a = ap.parse_args()
    cw, ch = [int(x) for x in a.cell.split('x')]

    labels, paths = [], []
    for c in a.cells:
        lab, p = c.split(':', 1)
        if os.path.exists(p):
            labels.append(lab); paths.append(p)
        else:
            print("skip missing", p)
    vids = [load_video(p) for p in paths]
    vids = [v for v in vids if v]  # drop empty
    if not vids:
        print("no usable videos"); return
    maxlen = max(len(v) for v in vids)
    cols = min(a.cols, len(vids))
    rows = (len(vids) + cols - 1) // cols
    GW, GH = cols * cw, rows * ch

    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (GW, GH))
    montage = None
    for t in range(maxlen):
        canvas = np.zeros((GH, GW, 3), np.uint8)
        for i, v in enumerate(vids):
            f = v[min(t, len(v) - 1)]
            cell = label(cv2.resize(f, (cw, ch)), labels[i])
            r, c = divmod(i, cols)
            canvas[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw] = cell
        vw.write(canvas)
        if t == 0:
            montage = canvas.copy()
    vw.release()
    print("wrote", a.out, f"{GW}x{GH}", maxlen, "frames")
    if a.montage and montage is not None:
        cv2.imwrite(a.montage, montage)
        print("wrote", a.montage)


if __name__ == '__main__':
    main()
