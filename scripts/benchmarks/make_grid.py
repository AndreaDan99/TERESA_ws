"""Compose labeled overlay mp4s into a grid video via ffmpeg.
Inputs already have captions burned in (bench scripts draw them), so we just scale+stack.

Usage:
  python3 make_grid.py --out grid.mp4 --layout hstack --cell 640 360 \
      --videos a.mp4 b.mp4
  layouts: hstack (1xN), vstack (Nx1), 2x2 (exactly 4)
"""
import argparse, subprocess, sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--layout", default="hstack", choices=["hstack", "vstack", "2x2"])
    ap.add_argument("--cell", nargs=2, type=int, default=[640, 360])
    ap.add_argument("--fps", type=int, default=15)
    a = ap.parse_args()
    W, H = a.cell
    W -= W % 2; H -= H % 2   # yuv420p needs even dimensions
    n = len(a.videos)

    cmd = ["ffmpeg", "-y"]
    for v in a.videos:
        cmd += ["-i", v]
    fc = []
    for i in range(n):
        fc.append(f"[{i}:v]scale={W}:{H}:force_original_aspect_ratio=decrease,"
                  f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,setsar=1,fps={a.fps},format=yuv420p[v{i}]")
    labels = "".join(f"[v{i}]" for i in range(n))
    if a.layout == "hstack":
        fc.append(f"{labels}hstack=inputs={n}[out]")
    elif a.layout == "vstack":
        fc.append(f"{labels}vstack=inputs={n}[out]")
    else:  # 2x2
        assert n == 4, "2x2 needs exactly 4 videos"
        fc.append(f"{labels}xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[out]")
    cmd += ["-filter_complex", ";".join(fc), "-map", "[out]",
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", a.out]
    print("running:", " ".join(cmd[:8]), "...", flush=True)
    r = subprocess.run(cmd, stderr=subprocess.PIPE)
    if r.returncode != 0:
        sys.stderr.write(r.stderr.decode()[-1500:]); sys.exit(r.returncode)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
